/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Om Trivedi and Brijesh Vaghasiya
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
/*!
 * \file     as7050.c
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include "nrf_delay.h"

#include "as7050_app_manager.h"
#include "as7050_interface.h"
#include "as7050_chiplib.h"
#include "as7050_std_include.h"
#include "bio_hrm_a0_typedefs.h"
#include "bio_spo2_a0_typedefs.h"
#include "bio_spo2_a0.h"
#include "vital_signs_accelerometer.h"
#include "gsr_app_typedefs.h"
#include "spo2.h"

#include "error_codes.h"
#include "common.h"
#include "hardware.h"
#include "nrf_log.h"
#include "as7050.h"
#include "export.h"

/******************************************************************************
 *                                  DEFINITIONS                               *
 ******************************************************************************/
#define ACC_SAMPLE_PERIOD_US 100000
#define SPO2_MODE            1
#define HRM_MODE             0
#define ECG_MODE             1
#define ECGNPPG_MODE         1
#define GSR_APP              0
#define AS7050_CHANNEL_CNT   9
#define ECGNSPO2_MODE        1

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
uint16_t ir_max = 25;
uint16_t ir_min = 25;
uint16_t red_max = 25;
uint16_t red_min = 0;

/******************************************************************************
 *                                 LOCALS                                 *
 ******************************************************************************/

static volatile uint8_t g_keep_running = 1;

/* Bitmap of sampled channels */
static volatile uint16_t g_fifo_map;

/* Sample period of PPG signals */
static volatile double g_ppg_sample_period_s;

/* Sample period of ECG signals */
static volatile double g_ecg_sample_period_s;

static const char *g_p_channel_names[AS7050_CHANNEL_CNT] = {
    "PPG1", "PPG2", "PPG3", "PPG4", "PPG5", "PPG6", "PPG7", "PPG8", "ECG",
};

static volatile uint32_t g_sample_cnt[AS7050_CHANNEL_CNT];


/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/**
 * @brief:  Define a callback function for the Chip Library. This function will be
            called by the Chip Library when new measurement data is available.
 *
 * @param[in] err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param
 *
 * @param[out] void 
 */
void as7050_callback(err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param)
{
    err_code_t result = ERR_SUCCESS;
    uint32_t ready_for_execution = 0;

    int channel_index;
    as7050_channel_flags_t channel_flag;
    double channel_sample_period;
    uint32_t samples[48];
    uint16_t sample_cnt;

    int sample_index;
    /* Data buffer size depends on the sample rate of the AS7050 and
       and the accelerometer.
       Sample rate for HRM is 20Hz. */
    uint8_t num_acc_data = 20;
    vs_acc_data_t acc_data[20];

    /* This is an unused argument, mark it to avoid failures on static code analysis. */
    M_UNUSED_PARAM(p_cb_param);

    /* Handle the error argument. */
    if (ERR_SUCCESS != error) {
        result = error;
    }
       
    /* Transfer the ADC data and the AGC status from the Chip Library and
       the accelerometer data to the Application Manager. */
    if (ERR_SUCCESS == result) {
        /* Chip Library and Application Manager use different structures for AGC status information.
           This conversion code assumes that single-channel AGC is used. */

        /* FOR SPO2 */
        bio_agc_status_t bio_agc_statuses[2];
        bio_agc_statuses[0].led_change = p_agc_status->led_change[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].led_current = p_agc_status->led_current[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].pd_offset_change = p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].pd_offset_current = p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[1].led_change = p_agc_status->led_change[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].led_current = p_agc_status->led_current[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].pd_offset_change = p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].pd_offset_current = p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_B];

        result = as7050_appmgr_set_input(p_data, data_num, (as7050_appmgr_chip_status_t){}, bio_agc_statuses, 2,
                                         NULL, 0, &ready_for_execution);
    }

    if (ERR_SUCCESS == result) {
        /* The Application Manager indicates if it is ready for execution
           every time new input data is provided. This can be used in a real
           implementation to efficiently signal an Application Manager
           process to execute when it is ready for execution.
           Set the 'ready for execution' flag. */
        if (ready_for_execution && !g_ready_for_execution) {
            g_ready_for_execution = 1;
        }
    } else {
        /* handle errors from Accelerometer and Application Manager */
    }
}

/**
 * @brief: Function to check communication of AS7050 
 *
 * @param[in] void
 *
 * @param[out] void 
 */
uint8_t as7050_check(void)
{
    uint8_t result;
    result = as7050_initialize(as7050_callback, NULL, NULL);
    return result;
}

/**
 * @brief: Function to Turn off internal LDO(VDDA) of AS7050 
 *
 * @param[in] void
 *
 * @param[out] void 
 */
void internal_ldo_off(void)
{
    uint8_t value;
    read_reg_i2c(AS7050_REGADDR_CONTROL,&value);
    nrf_delay_ms(1);
    as7050_ifce_write_register(AS7050_REGADDR_CONTROL,0);
}

/******************************************************************************
 *                               Main                                         *
 ******************************************************************************/

/**
 * @brief: Function to Initialize AS7050 
 *
 * @param[in] void
 *
 * @param[out] void 
 */

void as7050_init(void)
{

    uint32_t err_code;
    err_code_t result = ERR_SUCCESS;

#if 1
/******************************************************************************
 *                              Spo2                                         *
 ******************************************************************************/
#if SPO2_MODE

  if(4 == sensor_mode)
  {
      /* Initialization

         (1) Initialize the Chip Library
         This function must be called first.

         First parameter is the pointer to the callback function.
         Second parameter is the pointer to an application specific parameter,
         which will be transmitted with every callback. This parameter is optional.
         Third parameter is the pointer to an interface description. The
         Chip Library forwards this to ::as7050_osal_initialize. See the
         documentation of the Chip Library. This parameter is mandatory in
         conjunction with the AS7050 EVK board. */
      result = as7050_initialize(as7050_callback, NULL, NULL);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_initialize: %d\n", result);
      }

      /* (2) Initialize the Application Manager. */
      result = as7050_appmgr_initialize();
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_initialize: %d\n", result);
      }

      /* Configuration of the Chip Library
         *** example: SPO2, with AGC
         *** The configuration below works together with the AS7050 EVK board.

         (1) Set the configuration register
         See as7050_typedefs.h: enum as7050_reg_group_ids
         See the datasheet of the sensor. */

      /* configuration of AFE group */
      const as7050_config_afe_t afe_config = {
          .reg_vals.afe_dac0l = 0,
          .reg_vals.afe_dac1l = 0,
          .reg_vals.afe_dach = 0,//0,
          .reg_vals.afe_cfga = 0,//0,
          .reg_vals.afe_cfgb = 0,//0,
          .reg_vals.afe_gsr = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
      }

      /* configuration of AMP group */
      const as7050_config_amp_t amp_config = {
          .reg_vals.ecg_amp_cfga = 0,
          .reg_vals.ecg_amp_cfgb = 0x23,
          .reg_vals.ecg_amp_cfgc = 0x30,
          .reg_vals.ecg_amp_cfge = 0x01,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AMP, (uint8_t *)amp_config.reg_buffer, sizeof(as7050_config_amp_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AMP: %d\n", result);
      }

      /* configuration of AOC group */
      const as7050_config_aoc_t aoc_config = {
          .reg_vals.aoc_ios_ppg1 = 0xFF,
          .reg_vals.aoc_ios_ppg2 = 0xFF,
          .reg_vals.aoc_ios_ppg3 = 0xFF,
          .reg_vals.aoc_ios_ppg4 = 0xFF,
          .reg_vals.aoc_ios_ppg5 = 0xFF,
          .reg_vals.aoc_ios_ppg6 = 0xFF,
          .reg_vals.aoc_ios_ppg7 = 0xFF,
          .reg_vals.aoc_ios_ppg8 = 0xFF,
          .reg_vals.aoc_ppg_thh = 0xB9,
          .reg_vals.aoc_ppg_thl = 0x82,
          .reg_vals.aoc_ppg_cfg = 0,
          .reg_vals.aoc_ios_ecg = 0,//0xFF,
          .reg_vals.aoc_ecg_thh = 0xB9,
          .reg_vals.aoc_ecg_thl = 0x82,
          .reg_vals.aoc_ecg_cfg = 0,
          .reg_vals.aoc_ios_ledoff = 0xFF,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AOC, (uint8_t *)aoc_config.reg_buffer, sizeof(as7050_config_aoc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AOC: %d\n", result);
      }

      /* configuration of CTRL group */
      const as7050_config_ctrl_t ctrl_config = {
          .reg_vals.control = 1,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
      }

      /* configuration of ECG group */
      const as7050_config_ecg_t ecg_config = {
          .reg_vals.ecg_source = 0,
          .reg_vals.ecg_mod_cfga = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
      }

      /* configuration of FIFO group */
      const as7050_config_fifo_t fifo_config = {
          .reg_vals.fifo_ctrl = 0,
          .reg_vals.fifo_threshold = 0x02,//0x14,//0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
      }

      /* configuration of GPIO group */
      const as7050_config_gpio_t gpio_config = {
          .reg_vals.gpio1_cfg = 0,
          .reg_vals.gpio2_cfg = 0,
          .reg_vals.gpio1_cfgb = 0,
          .reg_vals.gpio2_cfgb = 0,
          .reg_vals.gpio_io = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
      }

      /* configuration of IIR group */
      const as7050_config_iir_t iir_config = {
          .reg_vals.iir_cfg = 0x05,//0,//5,
          /* iir_coeff_data_sos will not be initialized here */
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_IIR, (uint8_t *)iir_config.reg_buffer, sizeof(as7050_config_iir_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_IIR: %d\n", result);
      }

      /* configuration of LED group */
      const as7050_config_led_t led_config = {
          .reg_vals.lowvds_wait = 0,
          .reg_vals.led1_ictrl = 0x2A,//0x2A,
          .reg_vals.led2_ictrl = 0x15,//0x15,
          .reg_vals.led3_ictrl = 0x19,//0x15,
          .reg_vals.led4_ictrl = 0x12,//0x2A,
          .reg_vals.led5_ictrl = 0,
          .reg_vals.led6_ictrl = 0,
          .reg_vals.led7_ictrl = 0,
          .reg_vals.led8_ictrl = 0,
          .reg_vals.led_init = 0x1E,
          .reg_vals.led_ppg1 = 0x06,//0x02,//0x06,//0x24,//0x06(For RED LED),//0x09(For IR LED),
          .reg_vals.led_ppg2 = 0x09,//0x01,//0x09,//0x01,
          .reg_vals.led_ppg3 = 0,
          .reg_vals.led_ppg4 = 0,
          .reg_vals.led_ppg5 = 0,
          .reg_vals.led_ppg6 = 0,
          .reg_vals.led_ppg7 = 0,
          .reg_vals.led_ppg8 = 0,
          .reg_vals.led_tia = 0,
          .reg_vals.led_mode = 15,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
      }

      /* configuration of PD group */
      const as7050_config_pd_t pd_config = {
          .reg_vals.pdsel_cfg = 0,
          .reg_vals.pd_ppg1 = 0x01,//0,//0,//1,
          .reg_vals.pd_ppg2 = 0x01,//0x3F,//0,//0,//0,//0x01,//1,
          .reg_vals.pd_ppg3 = 0x01,//0x3F,//0,
          .reg_vals.pd_ppg4 = 0,
          .reg_vals.pd_ppg5 = 0,
          .reg_vals.pd_ppg6 = 0,
          .reg_vals.pd_ppg7 = 0,
          .reg_vals.pd_ppg8 = 0,
          .reg_vals.pd_tia = 0,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
      }

      /* configuration of PPG group */
      const as7050_config_ppg_t ppg_config = {
          .reg_vals.ppg_mod_cfga = 0xC6,
          .reg_vals.ppg_mod_cfgb = 0,
          .reg_vals.ppg_mod_cfgc = 0x07,
          .reg_vals.ppg_mod_cfgd = 0x04,
          .reg_vals.ppg_mod_cfge = 0x0F,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
      }

      /* configuration of REF group */
      const as7050_config_ref_t ref_config = {
          .reg_vals.ref_cfga = 0xAC,
          .reg_vals.ref_cfgb = 0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_REF, (uint8_t *)ref_config.reg_buffer, sizeof(as7050_config_ref_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_REF: %d\n", result);
      }

      /* configuration of SEQ group */
      const as7050_config_seq_t seq_config = {
          .reg_vals.cgb_cfg = 0x07,
          .reg_vals.seq_sample = 0x24,//0x64,//0x54,//0x64,//0x24,//0x64,
          .reg_vals.seq_ppga = 0x02,//0x00,
          .reg_vals.seq_ppgb = 0x01,//0x04,//0x01
          .reg_vals.seq_mode = 0x80,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);
      }

      /* configuration of SINC group */
      const as7050_config_sinc_t sinc_config = {
          .reg_vals.sinc_ppg_cfga = 0x84,//0x8D,
          .reg_vals.sinc_ppg_cfgb = 0x03,
          .reg_vals.sinc_ppg_cfgc = 0x00,//0x05,
          .reg_vals.sinc_ecg_cfga = 0x00,
          .reg_vals.sinc_ecg_cfgb = 0x01,
          .reg_vals.sinc_ecg_cfgc = 0x00,
          .reg_vals.ovs_cfg = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SINC, (uint8_t *)sinc_config.reg_buffer, sizeof(as7050_config_sinc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SINC: %d\n", result);
      }

      // configuration of STANDBY group
      const as7050_config_standby_t standby_config = {
          .reg_vals.standby_cfga = 0x35,//0x00,
          .reg_vals.standby_cfgb = 0x01,//0x00,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                    sizeof(as7050_config_standby_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
      }

      /* configuration of TIA group */
      const as7050_config_tia_t tia_config = {
          .reg_vals.pd_offset_cfg = 4,
          .reg_vals.tia_cfga = 0,
          .reg_vals.tia_cfgb = 0,
          .reg_vals.tia_cfgc = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
      }

      /* (2) Set automatic gain control (AGC)
         See as7050_typedefs.h: as7050_agc_config_t */
      const as7050_agc_config_t agc_config = {
          /* Set the AGC mode - it presents the used algorithm
              *** For this example the mode is set to dual channel PPG.
             See as7050_typedefs.h: as7050_agc_mode_t */
          .mode = AS7050_AGC_MODE_PPG_TWO_CHANNEL,
          /* Set channel group configuration */
          .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_PPG_1,
          .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_PPG_2,
          /* Set minimum and maximum for channel group A.
             For dual channel PPG mode the channel A LED current is fixed to current_min */
          .current_max[AS7050_CHANNEL_GROUP_A] = 35,//25,//51,
          .current_min[AS7050_CHANNEL_GROUP_A] = 35,//25,//33,
          /* Set minimum and maximum for channel group B. */
          .current_max[AS7050_CHANNEL_GROUP_B] = 37,//34,
          .current_min[AS7050_CHANNEL_GROUP_B] = 8,//8,//20, 
          /* Set minium and maximum threshold of the ADC signals. */
          .threshold_max = 400000,
          .threshold_min = 110000,
          /* Set number of samples to average for calculating the signal's mean. */
          .sample_cnt = 4,//4,//10,//20,
          /* Set interval in milliseconds to reset temporary AGC parameters like
             min/max peak signals. */
          .reset_interval = 4000,
      };
      result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_set_agc_config: %d\n", result);
      }

      /* Configuration of the Application Manager

         (1) Set the routing of the ADC input data to the Vital Signs applications
         *** For that example the following ADC Channels are used for SpO2:
             red led --> PPG1
             ir led --> PPG2
             ambient light --> PPG3 */
      as7050_appmgr_channel_id_t spo2_channels[BIO_SPO2_A0_SIGNAL_NUM] = {
          [BIO_SPO2_A0_SIGNAL_PPG_RED] = AS7050_CHANNEL_PPG_1,
          [BIO_SPO2_A0_SIGNAL_PPG_IR] = AS7050_CHANNEL_PPG_2,
          [BIO_SPO2_A0_SIGNAL_AMBIENT] = AS7050_CHANNEL_PPG_3,
      };
      result = as7050_appmgr_set_signal_routing(AS7050_APPMGR_APP_ID_SPO2_A0, spo2_channels,BIO_SPO2_A0_SIGNAL_NUM);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_set_signal_routing: %d\n", result);
      }

      /* (2) Enable the application(s)
             For this example the application 'SPO2' is enabled. */
      result = as7050_appmgr_enable_apps(AS7050_APPMGR_APP_FLAG_SPO2_A0);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_enable_apps: %d\n", result);
      }

      /* (3) Configure SPO2 application
         Note: The SpO2 calibration parameters are optical stack dependent.
         0, 2625, 10910, 1620, 1620 */
      bio_spo2_a0_configuration_t spo2_config = {
          .a = 0,
          .b = 2843,
          .c = 11313,
          .dc_comp_red = 1916,
          .dc_comp_ir = 1916,
      };

      result = as7050_appmgr_configure_app(AS7050_APPMGR_APP_ID_SPO2_A0, &spo2_config, sizeof(spo2_config));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_configure_app: %d\n", result);
      }

      /* (4) Get the measurement info from the Chip Library */
      as7050_meas_config_t meas_config;
      result = as7050_get_measurement_config(&meas_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_get_measurement_config: %d\n", result);
      }

      /* (5) Set the Application Manager to processing state
         The following information is provided to the Application Manager
         - Chip Library measurement configuration
         - Accelerometer sample period (Accelerometer is not used in this example, set to 1 since 0 is an invalid value)
         - Channels for which AGC is enabled
         After calling this function, the Application Manager is ready to receive measurement data and to process it.
         While in processing state, the configuration of the Application Manager and its applications can not be changed.
         To leave the processing state, as7050_appmgr_stop_processing needs to be called.*/
      result = as7050_appmgr_start_processing(meas_config, 1, agc_config.channel, 2);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_start_processing: %d\n", result);
      }

      /* Start measurement */

      /* (1) Start the measurement on the Chip Library */
      result = as7050_start_measurement();
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_start_measurement: %d\n", result);
      }
    }

  #endif


  #if ECG_MODE
  if(3 == sensor_mode)
  {
      /* Initialization

         (1) Initialize the Chip Library
         This function must be called first.

         First parameter is the pointer to the callback function.
         Second parameter is the pointer to an application specific parameter,
         which will be transmitted with every callback. This parameter is optional.
         Third parameter is the pointer to an interface description. The
         Chip Library forwards this to ::as7050_osal_initialize. See the
         documentation of the Chip Library. This parameter is mandatory in
         conjunction with the AS7050 EVK board. */
      result = as7050_initialize(as7050_callback, NULL, NULL);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_initialize: %d\n", result);
      }

      /* Configuration of the Chip Library
         *** example: ECG 200 Hz, PPG Single Channel 100 Hz, AOC
         *** The configuration below works together with the AS7050 EVK board.

         (1) Set the configuration register
         See as7050_typedefs.h: enum as7050_reg_group_ids
         See the datasheet of the sensor. */

      const as7050_config_afe_t afe_config = {
          .reg_vals.afe_dac0l = 0x00,
          .reg_vals.afe_dac1l = 0x00,
          .reg_vals.afe_dach = 0x00,
          .reg_vals.afe_cfga = 0x00,
          .reg_vals.afe_cfgb = 0x00,
          .reg_vals.afe_gsr = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
      }

      /* configuration of AMP group */
      const as7050_config_amp_t amp_config = {
          .reg_vals.ecg_amp_cfga = 0x81,//0x81,//0x82,
          .reg_vals.ecg_amp_cfgb = 0xAB,//0x4B,
          .reg_vals.ecg_amp_cfgc = 0xB4,//0xB4,//0xBC,//0x84,//0x9C,//0xBC,//0xA4,//0xB4,
          .reg_vals.ecg_amp_cfge = 0x44,//0x47,//0x44,//0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AMP, (uint8_t *)amp_config.reg_buffer, sizeof(as7050_config_amp_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AMP: %d\n", result);
      }

      /* configuration of AOC group */
      const as7050_config_aoc_t aoc_config = {
          .reg_vals.aoc_ios_ppg1 = 0xFF,
          .reg_vals.aoc_ios_ppg2 = 0xFF,
          .reg_vals.aoc_ios_ppg3 = 0xFF,
          .reg_vals.aoc_ios_ppg4 = 0xFF,
          .reg_vals.aoc_ios_ppg5 = 0xFF,
          .reg_vals.aoc_ios_ppg6 = 0xFF,
          .reg_vals.aoc_ios_ppg7 = 0xFF,
          .reg_vals.aoc_ios_ppg8 = 0xFF,
          .reg_vals.aoc_ppg_thh = 0xB9,
          .reg_vals.aoc_ppg_thl = 0x82,
          .reg_vals.aoc_ppg_cfg = 0x00,
          .reg_vals.aoc_ios_ecg = 0xFF,
          .reg_vals.aoc_ecg_thh = 0xB9,
          .reg_vals.aoc_ecg_thl = 0x82,
          .reg_vals.aoc_ecg_cfg = 0x00,
          .reg_vals.aoc_ios_ledoff = 0xFF,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AOC, (uint8_t *)aoc_config.reg_buffer, sizeof(as7050_config_aoc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AOC: %d\n", result);
      }

      /* configuration of CTRL group */
      const as7050_config_ctrl_t ctrl_config = {
          .reg_vals.control = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
      }

      /* configuration of ECG group */
      const as7050_config_ecg_t ecg_config = {
          .reg_vals.ecg_source = 0x50,
          .reg_vals.ecg_mod_cfga = 0x90,//0x80,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
      }

      /* configuration of FIFO group */
      const as7050_config_fifo_t fifo_config = {
          .reg_vals.fifo_ctrl = 0x00,
          .reg_vals.fifo_threshold = 0x15,//0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
      }

      /* configuration of GPIO group */
      const as7050_config_gpio_t gpio_config = {
          .reg_vals.gpio1_cfg = 0x00,
          .reg_vals.gpio2_cfg = 0x00,
          .reg_vals.gpio1_cfgb = 0x00,
          .reg_vals.gpio2_cfgb = 0x00,
          .reg_vals.gpio_io = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
      }

      /* configuration of IIR group */
      const as7050_config_iir_t iir_config = {
          .reg_vals.iir_cfg = 0x05,
          /* iir_coeff_data_sos will not be initialized here */
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_IIR, (uint8_t *)iir_config.reg_buffer, sizeof(as7050_config_iir_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_IIR: %d\n", result);
      }

      /* configuration of LED group */
      const as7050_config_led_t led_config = {
          .reg_vals.lowvds_wait = 0x00,
          .reg_vals.led1_ictrl = 0x00,//0x03,
          .reg_vals.led2_ictrl = 0x00,//0x03,
          .reg_vals.led3_ictrl = 0x00,
          .reg_vals.led4_ictrl = 0x00,
          .reg_vals.led5_ictrl = 0x00,
          .reg_vals.led6_ictrl = 0x00,
          .reg_vals.led7_ictrl = 0x00,
          .reg_vals.led8_ictrl = 0x00,
          .reg_vals.led_init = 0x00,//0x0A,
          .reg_vals.led_ppg1 = 0x00,//0x03,
          .reg_vals.led_ppg2 = 0x00,
          .reg_vals.led_ppg3 = 0x00,
          .reg_vals.led_ppg4 = 0x00,
          .reg_vals.led_ppg5 = 0x00,
          .reg_vals.led_ppg6 = 0x00,
          .reg_vals.led_ppg7 = 0x00,
          .reg_vals.led_ppg8 = 0x00,
          .reg_vals.led_tia = 0x00,
          .reg_vals.led_mode = 0x00,//0x0F,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
      }

      const as7050_config_pd_t pd_config = {
          .reg_vals.pdsel_cfg = 0x00,
          .reg_vals.pd_ppg1 = 0x00,//0x02,
          .reg_vals.pd_ppg2 = 0x00,
          .reg_vals.pd_ppg3 = 0x00,
          .reg_vals.pd_ppg4 = 0x00,
          .reg_vals.pd_ppg5 = 0x00,
          .reg_vals.pd_ppg6 = 0x00,
          .reg_vals.pd_ppg7 = 0x00,
          .reg_vals.pd_ppg8 = 0x00,
          .reg_vals.pd_tia = 0x00,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
      }

      const as7050_config_ppg_t ppg_config = {
          .reg_vals.ppg_mod_cfga = 0x04,
          .reg_vals.ppg_mod_cfgb = 0x00,
          .reg_vals.ppg_mod_cfgc = 0x00,
          .reg_vals.ppg_mod_cfgd = 0x00,
          .reg_vals.ppg_mod_cfge = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
      }

      const as7050_config_ref_t ref_config = {
          .reg_vals.ref_cfga = 0xD4,//0xFC,
          .reg_vals.ref_cfgb = 0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_REF, (uint8_t *)ref_config.reg_buffer, sizeof(as7050_config_ref_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_REF: %d\n", result);
      }


      const as7050_config_seq_t seq_config = {
          .reg_vals.cgb_cfg = 0x07,
          .reg_vals.seq_sample = 0x04,//0x54,//0x54,
          .reg_vals.seq_ppga = 0x00,
          .reg_vals.seq_ppgb = 0x00,
          .reg_vals.seq_mode = 0x20,//0xA0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);
      }

      const as7050_config_sinc_t sinc_config = {
          .reg_vals.sinc_ppg_cfga = 0x84,
          .reg_vals.sinc_ppg_cfgb = 0x03,
          .reg_vals.sinc_ppg_cfgc = 0x00,
          .reg_vals.sinc_ecg_cfga = 0x84,
          .reg_vals.sinc_ecg_cfgb = 0x03,
          .reg_vals.sinc_ecg_cfgc = 0x00,
          .reg_vals.ovs_cfg = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SINC, (uint8_t *)sinc_config.reg_buffer, sizeof(as7050_config_sinc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SINC: %d\n", result);
      }

      const as7050_config_standby_t standby_config = {
          .reg_vals.standby_cfga = 0x00,
          .reg_vals.standby_cfgb = 0x00,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                    sizeof(as7050_config_standby_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
      }

      /* configuration of TIA group */
      const as7050_config_tia_t tia_config = {
          .reg_vals.pd_offset_cfg = 0x04,
          .reg_vals.tia_cfga = 0x00,
          .reg_vals.tia_cfgb = 0x00,
          .reg_vals.tia_cfgc = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
      }

      /* (2) Set automatic gain control (AGC)
         See as7050_typedefs.h: as7050_agc_config_t */
      const as7050_agc_config_t agc_config = {
          /* Set the AGC mode - it presents the used algorithm
             *** For this example the mode is set to single channel PPG.
             See as7050_typedefs.h: as7050_agc_mode_t */
          .mode = AS7050_AGC_MODE_DISABLED,

          /* Set channel group configuration */
          .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_DISABLED,
          .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_DISABLED,
          /* Set minimum and maximum for channel group A. */
          .current_max[AS7050_CHANNEL_GROUP_A] = 0,
          .current_min[AS7050_CHANNEL_GROUP_A] = 0,
          /* Set minimum and maximum for channel group B. */
          .current_max[AS7050_CHANNEL_GROUP_B] = 0,
          .current_min[AS7050_CHANNEL_GROUP_B] = 0,
          /* Set minium and maximum threshold of the ADC signals. */
          .threshold_max = 0,
          .threshold_min = 0,
          /* Set number of samples to average for calculating the signal's mean. */
          .sample_cnt = 0,
          /* Set interval in milliseconds to reset temporary AGC parameters like
             min/max peak signals. */
          .reset_interval = 0,
      };


      result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_set_agc_config: %d\n", result);
      }

      /* (1) Get FIFO map and sample periods from Chip Library measurement config */
      as7050_meas_config_t meas_config;
      result = as7050_get_measurement_config(&meas_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_get_measurement_config: %d\n", result);
      }
      g_fifo_map = meas_config.fifo_map;
      g_ppg_sample_period_s = (double)meas_config.ppg_sample_period_us / 1000 / 1000;
      g_ecg_sample_period_s = (double)meas_config.ecg_sample_period_us / 1000 / 1000;

      /* Start measurement */

      /* (1) Start the measurement on the Chip Library */
      result = as7050_start_measurement();
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_start_measurement: %d\n", result);
      }

  }
#endif
#endif
#if ECGNPPG_MODE
/******************************************************************************
 *                              ECG & PPG                                         *
 ******************************************************************************/
  if(1 == sensor_mode)
  {

      /* Initialization

        (1) Initialize the Chip Library
        This function must be called first.

        First parameter is the pointer to the callback function.
        Second parameter is the pointer to an application specific parameter,
        which will be transmitted with every callback. This parameter is optional.
        Third parameter is the pointer to an interface description. The
        Chip Library forwards this to ::as7050_osal_initialize. See the
        documentation of the Chip Library. This parameter is mandatory in
        conjunction with the AS7050 EVK board.*/
      result = as7050_initialize(as7050_callback, NULL, NULL);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_initialize: %d\n", result);
      }

      /* Configuration of the Chip Library
         *** example: ECG 200 Hz, PPG Single Channel 100 Hz, AOC
         *** The configuration below works together with the AS7050 EVK board.

         (1) Set the configuration register
         See as7050_typedefs.h: enum as7050_reg_group_ids
         See the datasheet of the sensor. */


      const as7050_config_afe_t afe_config = {
          .reg_vals.afe_dac0l = 0x00,
          .reg_vals.afe_dac1l = 0x00,
          .reg_vals.afe_dach = 0x00,
          .reg_vals.afe_cfga = 0x00,
          .reg_vals.afe_cfgb = 0x00,
          .reg_vals.afe_gsr = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
      }

      /* configuration of AMP group */
      const as7050_config_amp_t amp_config = {
          .reg_vals.ecg_amp_cfga = 0x81,
          .reg_vals.ecg_amp_cfgb = 0xAB,
          .reg_vals.ecg_amp_cfgc = 0xB4,
          .reg_vals.ecg_amp_cfge = 0x44,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AMP, (uint8_t *)amp_config.reg_buffer, sizeof(as7050_config_amp_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AMP: %d\n", result);
      }

      /* configuration of AOC group */
      const as7050_config_aoc_t aoc_config = {
          .reg_vals.aoc_ios_ppg1 = 0xFF,
          .reg_vals.aoc_ios_ppg2 = 0xFF,
          .reg_vals.aoc_ios_ppg3 = 0xFF,
          .reg_vals.aoc_ios_ppg4 = 0xFF,
          .reg_vals.aoc_ios_ppg5 = 0xFF,
          .reg_vals.aoc_ios_ppg6 = 0xFF,
          .reg_vals.aoc_ios_ppg7 = 0xFF,
          .reg_vals.aoc_ios_ppg8 = 0xFF,
          .reg_vals.aoc_ppg_thh = 0xB9,
          .reg_vals.aoc_ppg_thl = 0x82,
          .reg_vals.aoc_ios_ecg = 0xFF,
          .reg_vals.aoc_ecg_thh = 0xB9,
          .reg_vals.aoc_ecg_thl = 0x82,
          .reg_vals.aoc_ecg_cfg = 0x00,
          .reg_vals.aoc_ios_ledoff = 0xFF,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AOC, (uint8_t *)aoc_config.reg_buffer, sizeof(as7050_config_aoc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AOC: %d\n", result);
      }

      /* configuration of CTRL group */
      const as7050_config_ctrl_t ctrl_config = {
          .reg_vals.control = 0x01,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
      }

      /* configuration of ECG group */
      const as7050_config_ecg_t ecg_config = {
          .reg_vals.ecg_source = 0x50,
          .reg_vals.ecg_mod_cfga = 0x90,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
      }

      /* configuration of FIFO group */
      const as7050_config_fifo_t fifo_config = {
          .reg_vals.fifo_ctrl = 0x00,
          .reg_vals.fifo_threshold = 0x15,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
      }

      /* configuration of GPIO group */
      const as7050_config_gpio_t gpio_config = {
          .reg_vals.gpio1_cfg = 0x00,
          .reg_vals.gpio2_cfg = 0x00,
          .reg_vals.gpio1_cfgb = 0x00,
          .reg_vals.gpio2_cfgb = 0x00,
          .reg_vals.gpio_io = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
      }

      /* configuration of IIR group */
      const as7050_config_iir_t iir_config = {
          .reg_vals.iir_cfg = 0x05,
          /* iir_coeff_data_sos will not be initialized here */
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_IIR, (uint8_t *)iir_config.reg_buffer, sizeof(as7050_config_iir_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_IIR: %d\n", result);
      }

      /* configuration of LED group */
      const as7050_config_led_t led_config = {
          .reg_vals.lowvds_wait = 0x00,
          .reg_vals.led1_ictrl = 0x03,
          .reg_vals.led2_ictrl = 0x03,
          .reg_vals.led3_ictrl = 0x03,
          .reg_vals.led4_ictrl = 0x03,
          .reg_vals.led5_ictrl = 0x00,
          .reg_vals.led6_ictrl = 0x00,
          .reg_vals.led7_ictrl = 0x00,//03glucose
          .reg_vals.led8_ictrl = 0x00,
          .reg_vals.led_init = 0x0A,
          .reg_vals.led_ppg1 = 0x06,//0x03,0x06,0x48(glucose)
          .reg_vals.led_ppg2 = 0x00,
          .reg_vals.led_ppg3 = 0x00,
          .reg_vals.led_ppg4 = 0x00,
          .reg_vals.led_ppg5 = 0x00,
          .reg_vals.led_ppg6 = 0x00,
          .reg_vals.led_ppg7 = 0x00,
          .reg_vals.led_ppg8 = 0x00,
          .reg_vals.led_tia = 0x00,
          .reg_vals.led_mode = 0x0F,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
      }

      const as7050_config_pd_t pd_config = {
          .reg_vals.pdsel_cfg = 0x00,
          .reg_vals.pd_ppg1 = 0x02,//0x01,
          .reg_vals.pd_ppg2 = 0x00,
          .reg_vals.pd_ppg3 = 0x00,
          .reg_vals.pd_ppg4 = 0x00,
          .reg_vals.pd_ppg5 = 0x00,
          .reg_vals.pd_ppg6 = 0x00,
          .reg_vals.pd_ppg7 = 0x00,
          .reg_vals.pd_ppg8 = 0x00,
          .reg_vals.pd_tia = 0x00,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
      }

      const as7050_config_ppg_t ppg_config = {
          .reg_vals.ppg_mod_cfga = 0xC7,
          .reg_vals.ppg_mod_cfgb = 0x00,
          .reg_vals.ppg_mod_cfgc = 0x07,
          .reg_vals.ppg_mod_cfgd = 0x04,
          .reg_vals.ppg_mod_cfge = 0x1F,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
      }

      const as7050_config_ref_t ref_config = {
          .reg_vals.ref_cfga = 0xFC,
          .reg_vals.ref_cfgb = 0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_REF, (uint8_t *)ref_config.reg_buffer, sizeof(as7050_config_ref_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_REF: %d\n", result);
      }

      const as7050_config_seq_t seq_config = {
          .reg_vals.cgb_cfg = 0x07,
          .reg_vals.seq_sample = 0x42,//0x64,//0x54,
          .reg_vals.seq_ppga = 0x00,
          .reg_vals.seq_ppgb = 0x00,
          .reg_vals.seq_mode = 0xA0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);
      }

      const as7050_config_sinc_t sinc_config = {
          .reg_vals.sinc_ppg_cfga = 0x84,
          .reg_vals.sinc_ppg_cfgb = 0x03,
          .reg_vals.sinc_ppg_cfgc = 0x00,
          .reg_vals.sinc_ecg_cfga = 0x84,
          .reg_vals.sinc_ecg_cfgb = 0x03,
          .reg_vals.sinc_ecg_cfgc = 0x00,
          .reg_vals.ovs_cfg = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SINC, (uint8_t *)sinc_config.reg_buffer, sizeof(as7050_config_sinc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SINC: %d\n", result);
      }

      const as7050_config_standby_t standby_config = {
          .reg_vals.standby_cfga = 0x00,
          .reg_vals.standby_cfgb = 0x00,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                    sizeof(as7050_config_standby_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
      }

      /* configuration of TIA group */
      const as7050_config_tia_t tia_config = {
          .reg_vals.pd_offset_cfg = 0x04,
          .reg_vals.tia_cfga = 0x00,
          .reg_vals.tia_cfgb = 0x00,
          .reg_vals.tia_cfgc = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
      }

      const as7050_agc_config_t agc_config = {
          .mode = AS7050_AGC_MODE_PPG_ONE_CHANNEL,

          /* Set channel group configuration */
          .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_PPG_1,
          .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_DISABLED,
          /* Set minimum and maximum for channel group A. */
          .current_max[AS7050_CHANNEL_GROUP_A] = 51,//10,
          .current_min[AS7050_CHANNEL_GROUP_A] =33,//2,
          /* Set minimum and maximum for channel group B. */
          .current_max[AS7050_CHANNEL_GROUP_B] = 0,
          .current_min[AS7050_CHANNEL_GROUP_B] = 0,
          /* Set minium and maximum threshold of the ADC signals. */
          .threshold_max = 400000,
          .threshold_min = 200000,
          /* Set number of samples to average for calculating the signal's mean. */
          .sample_cnt = 20,
          /* Set interval in milliseconds to reset temporary AGC parameters like
             min/max peak signals. */
          .reset_interval = 4000,
      };

      result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_set_agc_config: %d\n", result);
      }

      /* (1) Get FIFO map and sample periods from Chip Library measurement config */
      as7050_meas_config_t meas_config;
      result = as7050_get_measurement_config(&meas_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_get_measurement_config: %d\n", result);
      }
      g_fifo_map = meas_config.fifo_map;
      g_ppg_sample_period_s = (double)meas_config.ppg_sample_period_us / 1000 / 1000;
      g_ecg_sample_period_s = (double)meas_config.ecg_sample_period_us / 1000 / 1000;

      /* Start measurement */

      /* (1) Start the measurement on the Chip Library */
      result = as7050_start_measurement();
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_start_measurement: %d\n", result);
      }

  }
#endif

#if ECGNSPO2_MODE
/******************************************************************************
 *                              ECG & SpO2                                         *
 ******************************************************************************/
  if(2 == sensor_mode)
  {
      /* Initialization
         (1) Initialize the Chip Library
         This function must be called first.

         First parameter is the pointer to the callback function.
         Second parameter is the pointer to an application specific parameter,
         which will be transmitted with every callback. This parameter is optional.
         Third parameter is the pointer to an interface description. The
         Chip Library forwards this to ::as7050_osal_initialize. See the
         documentation of the Chip Library. This parameter is mandatory in
         conjunction with the AS7050 EVK board. */
      result = as7050_initialize(as7050_callback, NULL, NULL);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_initialize: %d\n", result);
      }

      /* (2) Initialize the Application Manager. */
      result = as7050_appmgr_initialize();
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_initialize: %d\n", result);
      }

      /* Configuration of the Chip Library
         *** example: SPO2, with AGC
         *** The configuration below works together with the AS7050 EVK board.

         (1) Set the configuration register
         See as7050_typedefs.h: enum as7050_reg_group_ids
         See the datasheet of the sensor. */


      const as7050_config_afe_t afe_config = {
          .reg_vals.afe_dac0l = 0,
          .reg_vals.afe_dac1l = 0,
          .reg_vals.afe_dach = 0,//0,
          .reg_vals.afe_cfga = 0,//0,
          .reg_vals.afe_cfgb = 0,//0,
          .reg_vals.afe_gsr = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
      }

      /* configuration of AMP group */
      const as7050_config_amp_t amp_config = {
          .reg_vals.ecg_amp_cfga = 0x81,
          .reg_vals.ecg_amp_cfgb = 0xAB,
          .reg_vals.ecg_amp_cfgc = 0xB4,
          .reg_vals.ecg_amp_cfge = 0x44,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AMP, (uint8_t *)amp_config.reg_buffer, sizeof(as7050_config_amp_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AMP: %d\n", result);
      }

      /* configuration of AOC group */
      const as7050_config_aoc_t aoc_config = {
          .reg_vals.aoc_ios_ppg1 = 0xFF,
          .reg_vals.aoc_ios_ppg2 = 0xFF,
          .reg_vals.aoc_ios_ppg3 = 0xFF,
          .reg_vals.aoc_ios_ppg4 = 0xFF,
          .reg_vals.aoc_ios_ppg5 = 0xFF,
          .reg_vals.aoc_ios_ppg6 = 0xFF,
          .reg_vals.aoc_ios_ppg7 = 0xFF,
          .reg_vals.aoc_ios_ppg8 = 0xFF,
          .reg_vals.aoc_ppg_thh = 0xB9,
          .reg_vals.aoc_ppg_thl = 0x82,
          .reg_vals.aoc_ppg_cfg = 0,
          .reg_vals.aoc_ios_ecg = 0xFF,
          .reg_vals.aoc_ecg_thh = 0xB9,
          .reg_vals.aoc_ecg_thl = 0x82,
          .reg_vals.aoc_ecg_cfg = 0x00,
          .reg_vals.aoc_ios_ledoff = 0xFF,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_AOC, (uint8_t *)aoc_config.reg_buffer, sizeof(as7050_config_aoc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_AOC: %d\n", result);
      }

      /* configuration of CTRL group */
      const as7050_config_ctrl_t ctrl_config = {
          .reg_vals.control = 1,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
      }

      /* configuration of ECG group */
      const as7050_config_ecg_t ecg_config = {
          .reg_vals.ecg_source = 0x50,
          .reg_vals.ecg_mod_cfga = 0x90,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
      }

      /* configuration of FIFO group */
      const as7050_config_fifo_t fifo_config = {
          .reg_vals.fifo_ctrl = 0x10,
          .reg_vals.fifo_threshold = 0x15,//0x14,//0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
      }

      /* configuration of GPIO group */
      const as7050_config_gpio_t gpio_config = {
          .reg_vals.gpio1_cfg = 0,
          .reg_vals.gpio2_cfg = 0,
          .reg_vals.gpio1_cfgb = 0,
          .reg_vals.gpio2_cfgb = 0,
          .reg_vals.gpio_io = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
      }

      /* configuration of IIR group */
      const as7050_config_iir_t iir_config = {
          .reg_vals.iir_cfg = 0x05,//0,//5,
          /* iir_coeff_data_sos will not be initialized here */
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_IIR, (uint8_t *)iir_config.reg_buffer, sizeof(as7050_config_iir_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_IIR: %d\n", result);
      }

      /* configuration of LED group */
      const as7050_config_led_t led_config = {
          .reg_vals.lowvds_wait = 0,
          .reg_vals.led1_ictrl = 0x2A,//0x2A,
          .reg_vals.led2_ictrl = 0x15,//0x15,
          .reg_vals.led3_ictrl = 0x19,//0x15,
          .reg_vals.led4_ictrl = 0x12,//0x2A,
          .reg_vals.led5_ictrl = 0,
          .reg_vals.led6_ictrl = 0,
          .reg_vals.led7_ictrl = 0,
          .reg_vals.led8_ictrl = 0,
          .reg_vals.led_init = 0x1E,
          .reg_vals.led_ppg1 = 0x06,//0x02,//0x06,//0x24,//0x06(For RED LED),//0x09(For IR LED),
          .reg_vals.led_ppg2 = 0x09,//0x01,//0x09,//0x01,
          .reg_vals.led_ppg3 = 0,
          .reg_vals.led_ppg4 = 0,
          .reg_vals.led_ppg5 = 0,
          .reg_vals.led_ppg6 = 0,
          .reg_vals.led_ppg7 = 0,
          .reg_vals.led_ppg8 = 0,
          .reg_vals.led_tia = 0,
          .reg_vals.led_mode = 15,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
      }

      const as7050_config_pd_t pd_config = {
          .reg_vals.pdsel_cfg = 0,
          .reg_vals.pd_ppg1 = 0x01,//0,//0,//1,
          .reg_vals.pd_ppg2 = 0x01,//0x3F,//0,//0,//0,//0x01,//1,
          .reg_vals.pd_ppg3 = 0x01,//0x3F,//0,
          .reg_vals.pd_ppg4 = 0,
          .reg_vals.pd_ppg5 = 0,
          .reg_vals.pd_ppg6 = 0,
          .reg_vals.pd_ppg7 = 0,
          .reg_vals.pd_ppg8 = 0,
          .reg_vals.pd_tia = 0,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
      }

      const as7050_config_ppg_t ppg_config = {
          .reg_vals.ppg_mod_cfga = 0xC6,
          .reg_vals.ppg_mod_cfgb = 0,
          .reg_vals.ppg_mod_cfgc = 0x07,
          .reg_vals.ppg_mod_cfgd = 0x04,
          .reg_vals.ppg_mod_cfge = 0x0F,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
      }

      const as7050_config_ref_t ref_config = {
          .reg_vals.ref_cfga = 0xFC,
          .reg_vals.ref_cfgb = 0x02,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_REF, (uint8_t *)ref_config.reg_buffer, sizeof(as7050_config_ref_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_REF: %d\n", result);
      }

      const as7050_config_seq_t seq_config = {
          .reg_vals.cgb_cfg = 0x07,
          .reg_vals.seq_sample = 0x64,//0x64,//0x54,//0x64,//0x24,//0x64,
          .reg_vals.seq_ppga = 0x02,//0x00,
          .reg_vals.seq_ppgb = 0x01,//0x04,//0x01
          .reg_vals.seq_mode = 0xA0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);

      }

      const as7050_config_sinc_t sinc_config = {
          .reg_vals.sinc_ppg_cfga = 0x84,//0x8D,
          .reg_vals.sinc_ppg_cfgb = 0x03,
          .reg_vals.sinc_ppg_cfgc = 0x00,//0x05,
          .reg_vals.sinc_ecg_cfga = 0x84,
          .reg_vals.sinc_ecg_cfgb = 0x03,
          .reg_vals.sinc_ecg_cfgc = 0x00,
          .reg_vals.ovs_cfg = 0x00,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_SINC, (uint8_t *)sinc_config.reg_buffer, sizeof(as7050_config_sinc_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_SINC: %d\n", result);
      }

      const as7050_config_standby_t standby_config = {
          .reg_vals.standby_cfga = 0x00,//0x35,
          .reg_vals.standby_cfgb = 0x00,//0x01,
      };
      result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                    sizeof(as7050_config_standby_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
      }

      /* configuration of TIA group */
      const as7050_config_tia_t tia_config = {
          .reg_vals.pd_offset_cfg = 4,
          .reg_vals.tia_cfga = 0,
          .reg_vals.tia_cfgb = 0,
          .reg_vals.tia_cfgc = 0,
      };
      result =
          as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
      if (ERR_SUCCESS != result) {
          printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
      }


      /* (2) Set automatic gain control (AGC)
         See as7050_typedefs.h: as7050_agc_config_t */
      const as7050_agc_config_t agc_config = {
          /* Set the AGC mode - it presents the used algorithm
             *** For this example the mode is set to dual channel PPG.
             See as7050_typedefs.h: as7050_agc_mode_t */
          .mode = AS7050_AGC_MODE_PPG_TWO_CHANNEL,
          /* Set channel group configuration */
          .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_PPG_1,
          .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_PPG_2,
          /* Set minimum and maximum for channel group A.
             For dual channel PPG mode the channel A LED current is fixed to current_min */
          .current_max[AS7050_CHANNEL_GROUP_A] = 35,//ir_max,//35
          .current_min[AS7050_CHANNEL_GROUP_A] = 25,//ir_min,//35
          /* Set minimum and maximum for channel group B. */
          .current_max[AS7050_CHANNEL_GROUP_B] = 35,//red_max,//35
          .current_min[AS7050_CHANNEL_GROUP_B] = 0,//red_min, 
          /* Set minium and maximum threshold of the ADC signals. */
          .threshold_max = 400000,
          .threshold_min = 110000,
          /* Set number of samples to average for calculating the signal's mean. */
          .sample_cnt = 20,//20,//4,//10,//20,
          /* Set interval in milliseconds to reset temporary AGC parameters like
             min/max peak signals. */
          .reset_interval = 4000,
      };
      result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_set_agc_config: %d\n", result);
      }

      /* Configuration of the Application Manager

         (1) Set the routing of the ADC input data to the Vital Signs applications
         *** For that example the following ADC Channels are used for SpO2:
             red led --> PPG1
             ir led --> PPG2
             ambient light --> PPG3 */
      as7050_appmgr_channel_id_t spo2_channels[BIO_SPO2_A0_SIGNAL_NUM] = {
          [BIO_SPO2_A0_SIGNAL_PPG_RED] = AS7050_CHANNEL_PPG_1,
          [BIO_SPO2_A0_SIGNAL_PPG_IR] = AS7050_CHANNEL_PPG_2,
          [BIO_SPO2_A0_SIGNAL_AMBIENT] = AS7050_CHANNEL_PPG_3,
      };
      result = as7050_appmgr_set_signal_routing(AS7050_APPMGR_APP_ID_SPO2_A0, spo2_channels,BIO_SPO2_A0_SIGNAL_NUM);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_set_signal_routing: %d\n", result);
      }

      /* (2) Enable the application(s)
         *** For this example the application 'SPO2' is enabled. */
      result = as7050_appmgr_enable_apps(AS7050_APPMGR_APP_FLAG_SPO2_A0);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_enable_apps: %d\n", result);
      }

      /* (3) Configure SPO2 application
         Note: The SpO2 calibration parameters are optical stack dependent.
        0, 2625, 10910, 1620, 1620 */
          bio_spo2_a0_configuration_t spo2_config = {
          .a = 0,
          .b = 2843,
          .c = 11313,
          .dc_comp_red = 1916,
          .dc_comp_ir = 1916,
      };


      //{0, 2625, 10910, 1620, 1620};
      result = as7050_appmgr_configure_app(AS7050_APPMGR_APP_ID_SPO2_A0, &spo2_config, sizeof(spo2_config));
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_configure_app: %d\n", result);
      }

      /* (4) Get the measurement info from the Chip Library */
      as7050_meas_config_t meas_config;
      result = as7050_get_measurement_config(&meas_config);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_get_measurement_config: %d\n", result);
      }

      /*(5) Set the Application Manager to processing state
         The following information is provided to the Application Manager
         - Chip Library measurement configuration
         - Accelerometer sample period (Accelerometer is not used in this example, set to 1 since 0 is an invalid value)
         - Channels for which AGC is enabled
         After calling this function, the Application Manager is ready to receive measurement data and to process it.
         While in processing state, the configuration of the Application Manager and its applications can not be changed.
         To leave the processing state, as7050_appmgr_stop_processing needs to be called. */
      result = as7050_appmgr_start_processing(meas_config, 1, agc_config.channel, 2);
      if (ERR_SUCCESS != result) {
          printf("Error on as7050_appmgr_start_processing: %d\n", result);
      }

      /* Start measurement */

      /* (1) Start the measurement on the Chip Library */
      //result = as7050_start_measurement();
      //if (ERR_SUCCESS != result) {
      //    printf("Error on as7050_start_measurement: %d\n", result);
      //}
  }
 
}


#endif

/*
  END OF FILE
*/ 

