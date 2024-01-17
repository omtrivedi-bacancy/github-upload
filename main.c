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
 * \file     main.c
 */

 /******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
/* nrf Driver Includes */
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_nus.h"

/* nrf Log and Delay Includes */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

/* Peripheral Includes */
#include "BMA400.h"
#include "spo2.h"
#include "hardware.h"
#include "export.h"
#include "common.h"
#include "bio_spo2_a0.h"
#include "as7050.h"
#include "as7050_chiplib.h"
#include "as7050_interface.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "sts4x.h"
#include <nrfx_qspi.h>
#include "nrf_power.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                100                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define NOTIF_BUTTON_ID                 0 
#define APP_ADV_DURATION                3000                                    /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SAMPLES_IN_BUFFER               5
#define MOV_AVE_ARR_SIZE                5U
/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
int spo2_count = 0;
uint8_t sensor_mode = 0;
uint8_t g_ready_for_execution = 0;
uint8_t data_collected = 0;
uint16_t length;
uint16_t ble_length;
uint32_t app_data_available;
uint8_t scheduler_enable = 0;
int minutes_interval = DEFAULT_MIN_INT;
int sec_interval = DEFAULT_SEC_INT;
uint8_t flash_write_enable = 0;
uint8_t ble_connected = 0;
uint8_t flash_data_exist = 0;
uint8_t flash_data_collected = 0;
uint32_t ecg_end_address = 0;
uint32_t ppg_end_address = 0;
bool flash_copy_done = 0;
int flash_index = 0;
bool qspi_init_flag = 0;
uint8_t as7050_power = 0;

/* BLE Packet Objects */
spo2AlgorithmOutput_t spo2_output;
export_step stepdata;
export_fall falldata;
export_temp tempdata;
export_spo2 spo2data;
export_ppg ppgdata;
export_scheduler schedulerdata;
export_bat batdata;


/******************************************************************************
 *                                  LOCALS                                  *
 ******************************************************************************/
static uint32_t m_adc_evt_counter;
static bool m_is_notification_mode    = false;
static uint16_t   m_tx_buf[20];    /* < TX buffer.> */
static uint8_t    m_rx_buf[21];    /* < RX buffer.> */                      
ble_gap_addr_t mac_id[6];
uint8_t  mac_add[16];

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
APP_TIMER_DEF(m_timer);                                                         /**< Minutes Interval Timer Instance. */
APP_TIMER_DEF(s_timer);                                                         /**< Seconds Interval Timer Instance. */
APP_TIMER_DEF(p_timer);                                                         /**< Periodic Timer Instance. */
APP_TIMER_DEF(f_timer);                                                         /**< Flash Read Timer Instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


/* NUS UUID for service(s) used in your application. */
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/
/**
 * @brief:   Send current steps to BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_current_steps(void);

/**
 * @brief:   Function to calculate Moving average
 *
 * @param[in] uint16_t new_data
 *
 * @param[out] uint16_t    
 */
static uint16_t moving_avg(uint16_t new_data);

/**
 @brief Function for starting scheduler.
*/
static void scheduler_init(void);

 /**
 @brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds);

/**
 * @brief:   Function to send battery data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_bat_percent_to_ble(void);

/**
 * @brief:   Function to shutdown device or enter into deep sleep mode
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void device_shutdown(void);

/**
 * @brief:   Function to send flash data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void ReadFlashdataforBLE(void);

/**
 * @brief:   Function to send temperature data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_body_temp(void);

/**
 * @brief:   Function to send scheduler status through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_device_status(void);

/**
 * @brief:   Function to get MAC Address of Device
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void get_mac_id(void);

/**
 * @brief:   Function to check Temperature sensor communication
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void temp_check(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

/**@brief Function for primary scheduler timer start
 */
void primary_schedular_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(minutes_interval), NULL);//APP_TIMER_TICKS(1,800,000) = 30 min
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for secondary scheduler timer start
 */
void secondary_scheduler_timer_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(s_timer,APP_TIMER_TICKS(sec_interval), NULL);
    APP_ERROR_CHECK(err_code); 
}

/**@brief Function for primary scheduler timeout
 */
static void primary_schedular_timeout(void)
{
    printf("\ntimeout seconds interval = %d\n",sec_interval);
    as7050_power_on();
    sensor_mode = 2;
    as7050_init();
    secondary_scheduler_timer_start();
}

/**@brief Function for secondary scheduler timeout
 */
static void secondary_scheduler_timeout(void)
{
   if(1 == as7050_power)
   {
      as7050_stop_measurement();
      as7050_shutdown();
      as7050_power_off();
   }
   primary_schedular_timer_start();
}


/**@brief Function for periodic update start
 */
static void periodic_update_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(p_timer, APP_TIMER_TICKS(15000), NULL); //APP_TIMER_TICKS(1,800,000) = 30 min
    APP_ERROR_CHECK(err_code);                                         //Periodic Timer Start on Connect
}

/**@brief Function for periodic update stop
 */
static void periodic_update_stop(void)
{
    app_timer_stop(p_timer);//Periodic Timer Stop on Disconnect
}

/**@brief Function for periodic update timeout
 */
static void periodic_update_timeout(void)
{
    send_body_temp();
    send_bat_percent_to_ble();
}

/**@brief Function for flash update timeout
 */
static void flash_update_timeout(void)
{
  if(1 == flash_data_exist)
  {
    ReadFlashdataforBLE();
    if (flash_index < total_data_collected) {
      memcpy(mixdata.ecg_vals, ecg_vals, 22 * sizeof(&flashdata.ecg_vals[flash_index]));
      memcpy(mixdata.ppg_vals, ppg_vals, 22 * sizeof(&flashdata.ppg_vals[flash_index]));
      flash_index += 22;
      flash_copy_done = 1;
    } else {
      printf("\nFlash Data Sent");
      app_timer_stop(f_timer);
      flash_data_exist = 0;
      flash_index = 0;
      total_data_collected = 0;
      qspi_erase();
      qspi_deinit();
      flash_power_off();
      qspi_init_flag = 0;
      scheduler_init();
      scheduler_enable = 1;
    }
  }
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    /* Initialize timer module. */
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    /* Create timers. */

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_SINGLE_SHOT,(void *)primary_schedular_timeout);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&s_timer, APP_TIMER_MODE_SINGLE_SHOT,(void *)secondary_scheduler_timeout);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&p_timer, APP_TIMER_MODE_REPEATED,(void *)periodic_update_timeout);
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&f_timer, APP_TIMER_MODE_REPEATED,(void *)flash_update_timeout);
    APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    get_mac_id();
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)mac_add,
                                          strlen(mac_add));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);


}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/


/**@brief Function for scheduler init.
 */
static void scheduler_init(void)
{
    primary_schedular_timer_start(); 
    printf("\nScheduler Start with minutes_interval = %d\n",minutes_interval);
}

/**@brief Function for stop scheduler mode.
 */
static void scheduler_deinit(void)
{
    printf("\nScheduler Stop\n");
    if(1 == as7050_power)
    {
      as7050_stop_measurement();
      as7050_shutdown();
    }
    app_timer_stop(m_timer);
    app_timer_stop(s_timer);
}

/**@brief Function to receive ble commands
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
  if (p_evt->type == BLE_NUS_EVT_RX_DATA) 
  {
    uint32_t err_code;
    NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    NRF_LOG_INFO("%s", p_evt->params.rx_data.p_data);

    for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) 
    {
      do {
        m_tx_buf[i] = p_evt->params.rx_data.p_data[i];

        if (m_tx_buf[i] == '0') {
          as7050_stop_measurement();
          internal_ldo_off();
          as7050_shutdown();
          as7050_power_off();
          //NRF_LOG_INFO("Received stop!");
        }

        if (m_tx_buf[i] == '1') {
          as7050_power_on();
          as7050_init();
          as7050_start_measurement();
          //NRF_LOG_INFO("Received start!");
        }

        if (m_tx_buf[i] == '2') {
          NRF_LOG_DEBUG("Starting Firmware Update...");
          err_code = sd_power_gpregret_clr(0, 0xffffffff);
          if (err_code != NRF_SUCCESS)
            return;

          err_code = sd_power_gpregret_set(0, 0xb1);
          if (err_code != NRF_SUCCESS)
            return;

          NVIC_SystemReset();
        }

        if (m_tx_buf[i] == '3') {
          NRF_LOG_DEBUG("Resetting the device...");
          NVIC_SystemReset();
        }

        if (m_tx_buf[i] == '4') {
          printf("\nECG&SPO2 mode");
          sensor_mode = 2;
        }

        if (m_tx_buf[i] == '5') {
          printf("\nECG mode");
          sensor_mode = 3;
        }

        if (m_tx_buf[i] == '6') {
          printf("\nECG&PPG mode");
          sensor_mode = 1;
        }

        if (m_tx_buf[i] == '7') {
          NRF_LOG_DEBUG("SPO2 mode");
          sensor_mode = 4;
        }

        if (m_tx_buf[i] == '8') {
          /* Reset Step Count to Zero */
          reset_step_count();
          send_current_steps();
        }

        if (m_tx_buf[i] == 'F') {
          if (1 == flash_data_exist) {
            NRF_LOG_INFO("Sending Saved data first");
            scheduler_deinit();
            scheduler_enable = 0;
            err_code = app_timer_start(f_timer, APP_TIMER_TICKS(80), NULL); /* APP_TIMER_TICKS(1,800,000) = 30 min */
            APP_ERROR_CHECK(err_code);
          }
        }

        if (m_tx_buf[i] == 'T') {
          if (0 == scheduler_enable) {
            NRF_LOG_INFO("Scheduler mode START");
            scheduler_enable = 1;
            minutes_interval = 1000 * 60 * p_evt->params.rx_data.p_data[1]; /* minutes to milliseconds */
            sec_interval = 1000 * p_evt->params.rx_data.p_data[2];          /* seconds to milliseconds */
            NRF_LOG_INFO("minutes_interval %d", minutes_interval);
            NRF_LOG_INFO("sec_interval %d", sec_interval);
            scheduler_init();
          }
        }

        //if (m_tx_buf[i] == 'I') {
        //    NRF_LOG_INFO("IR current config");
        //    if((p_evt->params.rx_data.p_data[1] < 50) && (p_evt->params.rx_data.p_data[2] < 50))
        //    {
        //      ir_max = p_evt->params.rx_data.p_data[1]; 
        //      ir_min =  p_evt->params.rx_data.p_data[2];
        //    }
        //    else
        //    {
        //      ir_max = 25;
        //      ir_min = 25;
        //    }
        //    NRF_LOG_INFO("ir_max %d", ir_max);
        //    NRF_LOG_INFO("ir_min %d", ir_min);
        //  }

        //  if (m_tx_buf[i] == 'R') {
        //    NRF_LOG_INFO("Red current config");
        //    if((p_evt->params.rx_data.p_data[1] < 50) && (p_evt->params.rx_data.p_data[2] < 50))
        //    {
        //       red_max = p_evt->params.rx_data.p_data[1];
        //       red_min = p_evt->params.rx_data.p_data[2];
        //    }
        //    else
        //    {
        //       red_max = 25;
        //       red_min = 0;
        //    }
 
        //    NRF_LOG_INFO("red_max %d", red_max);
        //    NRF_LOG_INFO("red_min %d", red_min);
        //  }

        if (m_tx_buf[i] == 'S') {
          NRF_LOG_INFO("Scheduler mode STOP");
          scheduler_deinit();
          scheduler_enable = 0;
        }

        if (m_tx_buf[i] == 'B') {
          NRF_LOG_INFO("Checking Battery Percent...");
          send_bat_percent_to_ble();
          send_body_temp();
          send_current_steps();
        }

        if (m_tx_buf[i] == 'N') {
          NRF_LOG_INFO("Checking Device status...");
          send_device_status();
        }

        if (m_tx_buf[i] == 'W') {
          length = sizeof(FW_VERSION);
          ble_nus_data_send(&m_nus, FW_VERSION, &length, m_conn_handle);
        }

        if (m_tx_buf[i] == 'C') {
        if (0 == scheduler_enable){
          as7050_power_on();
          NRF_LOG_INFO("Checking All Peripherals Communication....");
          err_code = bma400_init();
          if (!err_code) 
          {
            length = sizeof(ACCEL_SUCCESS);
            ble_nus_data_send(&m_nus, ACCEL_SUCCESS, &length, m_conn_handle);
          } else {
            length = sizeof(ACCEL_ERROR);
            ble_nus_data_send(&m_nus, ACCEL_ERROR, &length, m_conn_handle);
          }
          err_code = as7050_check();
          if (!err_code) 
          {
            length = sizeof(AS_SUCCESS);
            ble_nus_data_send(&m_nus, AS_SUCCESS, &length, m_conn_handle);
          } else {
            length = sizeof(AS_ERROR);
            ble_nus_data_send(&m_nus, AS_ERROR, &length, m_conn_handle);
          }
          as7050_power_off();
          temp_check();
          err_code = qspi_check();
          if (!err_code) 
          {
            length = sizeof(F_SUCCESS);
            ble_nus_data_send(&m_nus, F_SUCCESS, &length, m_conn_handle);
          } else {
            length = sizeof(F_ERROR);
            ble_nus_data_send(&m_nus, F_ERROR, &length, m_conn_handle);
          }
         }
        }
      } while (err_code == NRF_ERROR_BUSY);
    }
  }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_nus_init_t     nus_init;
    /* Initialize Queued Write Module. */
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;
    twi_deinit();
    /* Prepare wakeup buttons. */
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt) 
    {
      case BLE_ADV_EVT_FAST:
        break;

      case BLE_ADV_EVT_IDLE:
        if (1 == scheduler_enable) {
          ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
          APP_ERROR_CHECK(err_code);
        }
        if (0 == scheduler_enable) {
          sleep_mode_enter();
        }
        break;

      default:
        break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id) 
    {
      case BLE_GAP_EVT_DISCONNECTED:
        printf("\nDisconnected.\n");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        ble_connected = 0;
        periodic_update_stop();
        if (1 == scheduler_enable) {
          flash_write_enable = 1;
          if (0 == qspi_init_flag) {
            flash_power_on();
            qspi_init();
            qspi_init_flag = 1;
          }
        }
        if (0 == scheduler_enable) {
          as7050_power_off();
        }
        break;

      case BLE_GAP_EVT_CONNECTED:
        printf("\nConnected.\n");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        ble_connected = 1;
        periodic_update_start();
        if (1 == scheduler_enable) 
        {
          flash_write_enable = 0;
        }
        break;

      case BLE_GAP_EVT_PHY_UPDATE_REQUEST: 
      {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
      } break;

      case BLE_GATTC_EVT_TIMEOUT:
        /* Disconnect on GATT Client timeout event. */
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_GATTS_EVT_TIMEOUT:
        /* Disconnect on GATT Server timeout event.  */
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

      case BLE_GAP_EVT_ADV_SET_TERMINATED:
        if (scheduler_enable == 0) 
        {
          if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT) {
            /* Go to system-off mode (this function will not return; wakeup will cause a reset). */
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
          }
        }
        break;
      default:
        /* No implementation needed. */
        break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    /* Configure the BLE stack using the default settings.
       Fetch the start address of the application RAM. */
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    /* Enable BLE stack. */
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    /* Register a handler for BLE events. */
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    /* Security parameters to be used for all security procedures. */
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event) 
    {
      case BSP_EVENT_SLEEP:
        if (0 == scheduler_enable) {
          sleep_mode_enter();
        }
        break; /* BSP_EVENT_SLEEP */

      case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE) {
          APP_ERROR_CHECK(err_code);
        }
        break; /* BSP_EVENT_DISCONNECT */

      case BSP_EVENT_WHITELIST_OFF:
        if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
          err_code = ble_advertising_restart_without_whitelist(&m_advertising);
          if (err_code != NRF_ERROR_INVALID_STATE) {
            APP_ERROR_CHECK(err_code);
          }
        }
      case BSP_EVENT_RESET:
        NVIC_SystemReset();

        break; /* BSP_EVENT_KEY_0 */

      default:
        break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    ble_advdata_manuf_data_t  man_data;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);


}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_init(bool* p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,BSP_BUTTON_ACTION_RELEASE,BSP_EVENT_WAKEUP);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0,BSP_BUTTON_ACTION_LONG_PUSH,BSP_EVENT_RESET);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}



/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
   nrf_pwr_mgmt_run();
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        /* Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event */
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief:  Function to get Spo2 output
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void spo2_calculation(void)
{
    uint8_t result;

    /* (1) Execute the Application Manager if flag 'ready for execution'
           was set by the Chip Library's callback function */
    if (g_ready_for_execution) {
      result = as7050_appmgr_execute(&app_data_available);
      if (ERR_SUCCESS != result) {
      }
      g_ready_for_execution = FALSE;
    } else {
      app_data_available = FALSE;
    }

    /* (2) Get the output data from the Vital Signs applications */
    if (AS7050_APPMGR_APP_FLAG_SPO2_A0 & app_data_available) {
      /* The SPO2 application has new output data available. */
      uint16_t output_size = sizeof(spo2AlgorithmOutput_t);
      result = as7050_appmgr_get_output(AS7050_APPMGR_APP_ID_SPO2_A0, &spo2_output, &output_size);
      if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_get_output: %d\n", result);
      }

      if (!spo2_output.status) /* Signal Valid Data */
      {
        spo2_count++;

        length = sizeof(spo2data);
        spo2data.spo2val = moving_avg(spo2_output.spo2);
        spo2data.spo2_quality = 100;

        if (spo2data.spo2val < 9000) 
        {
          spo2data.spo2val = spo2data.spo2val + 2000; /* Chest error correction */
        }
        if (spo2data.spo2val > MAX_SPO2_OUTPUT) 
        {
          spo2data.spo2val = MAX_SPO2_OUTPUT;
        }

        printf("\nspo2 = %d  %d  %d", spo2data.spo2val, spo2_output.signalQuality, spo2_output.heartrate);

        if (spo2_count > 5) {
          ble_nus_data_send(&m_nus, &spo2data, &length, m_conn_handle);
          spo2_count = 0;
        }
      } else {
        /* Signal invalid data */
        printf("\nLoop  - invalid SPO2 result, wait for more data");
      }
    }
}


/**
 * @brief:   Callback Function to saadc
 *
 * @param[in] nrf_drv_saadc_evt_t const * p_event
 *
 * @param[out] void    
 */
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        //NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);
        printf("ADC event number: %d   \n", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
            printf(" \n %d \n", p_event->data.done.p_buffer[i]);
        }
        m_adc_evt_counter++;
    }
}

/**
 * @brief:   Function to initialize saadc
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(4, &channel_config);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief:   Function to uninitialize saadc
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void saadc_uninit(void)
{
    nrf_drv_saadc_channel_uninit(NRF_SAADC_INPUT_AIN4);
    nrf_drv_saadc_uninit(); 
}

/**
 * @brief:   Interrupt handler for Fall detection
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void fall_interrupt_handler(void)
{
    printf("\nFall detected\n");
    falldata.fall_vals = 1;
    length = sizeof(falldata);
    ble_nus_data_send(&m_nus, &falldata, &length, m_conn_handle);
    falldata.fall_vals = 0;
}

/**
 * @brief:   Send current steps to BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_current_steps(void)
{
    int8_t rslt;
    uint32_t step_count = 0;
    uint8_t act_int;
    struct bma400_dev bma;

    rslt = bma400_get_steps_counted(&step_count, &act_int, &bma);

    stepdata.step_vals = step_count;
    length = sizeof(stepdata);
    ble_nus_data_send(&m_nus, &stepdata, &length, m_conn_handle);
}
/**
 * @brief:   Interrupt handler for Step detection
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void step_interrupt_handler(void)
{
    int8_t rslt;
    uint32_t step_count = 0;
    uint8_t act_int;
    uint16_t int_status;
    struct bma400_dev bma;

    rslt = bma400_get_steps_counted(&step_count, &act_int, &bma);

    switch (act_int) {
    case BMA400_STILL_ACT:
      stepdata.activity = 0; //Still
      break;
    case BMA400_WALK_ACT:
      stepdata.activity = 1; //Walking
      break;
    case BMA400_RUN_ACT:
      stepdata.activity = 2; //Running
      break;
    }
    if (step_count != 0) 
    {
      stepdata.step_vals = step_count;
      length = sizeof(stepdata);
      ble_nus_data_send(&m_nus, &stepdata, &length, m_conn_handle);
    }
}

/**
 * @brief:   Function to send battery data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */

static void send_bat_percent_to_ble(void)
{
    saadc_init();
    #if BATTERY_MONITOR
    nrf_saadc_value_t adc_val;
    uint16_t bat_per;
    static uint16_t bat_val; 
    uint16_t length;
    nrfx_saadc_sample_convert(4, &adc_val);
    NRF_LOG_INFO("Sample value Read: %d", adc_val);

    if (adc_val >= 568) {
      bat_per = 100;
    } else if (adc_val >= 539 && adc_val < 568) {
      bat_per = 80;
    } else if (adc_val >= 510 && adc_val < 539) {
      bat_per = 60;
    } else if (adc_val >= 481 && adc_val < 510) {
      bat_per = 40;
    } else if (adc_val >= 452 && adc_val < 481) {
      bat_per = 20;
    } else if (adc_val >= 423 && adc_val < 452) {
      bat_per = 10;
    } else {
      bat_per = 0;
    }
    batdata.bat_val = bat_per;
    length = sizeof(batdata);
    ble_nus_data_send(&m_nus, &batdata, &length, m_conn_handle);

    #endif
    saadc_uninit();
    if (adc_val <= 411) {
      if (1 == scheduler_enable) {
        scheduler_deinit();
        scheduler_enable = 0;
      }
      device_shutdown();
    }
}


/**
 * @brief:   Function to send flash data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void sendFlashBLE(void)
{
    ble_length = sizeof(mixdata);
    if(1 == ble_connected)
    {
      if (1 == flash_copy_done) 
      {
        ble_nus_data_send(&m_nus, &mixdata, &ble_length, m_conn_handle);
        flash_copy_done = 0;
      }
    }
}
/**
 * @brief:   Function to send flash data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void ReadFlashdataforBLE(void)
{
    if (0 == flash_write_enable) 
    {
      qspi_page_read(ecg_start_address, 1);
    }
}


/**
 * @brief:   Function to send sensor data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */

static void sendDatatoBLE(void)
{
    uint16_t err_code;
    if(1 == ble_connected)
    {
        if(data_collected == 1)
        {
          if(sensor_mode == 2)
          {
            do
            {
                ble_length = sizeof(mixdata);
                err_code = ble_nus_data_send(&m_nus,&mixdata,&ble_length,m_conn_handle);
            } while (err_code == NRF_ERROR_BUSY);
            data_collected = 0;
          }
          if(sensor_mode == 3)
          {
            do
            {
                ble_length = sizeof(ecgdata);
                err_code = ble_nus_data_send(&m_nus,&ecgdata,&ble_length,m_conn_handle);
            } while (err_code == NRF_ERROR_BUSY);
            data_collected = 0;
          }
        }
        spo2_calculation();
    }
}


/**
 * @brief:   Function to shutdown device or enter into deep sleep mode
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void device_shutdown(void)
{
    printf("\n Battery Low, Entering Deep Sleep Mode\n");
    sleep_mode_enter();
}

/**
 * @brief:   Function to assign msg_ids to data structure
 *
 * @param[in] void
 *
 * @param[out] void    
 */

static void assign_msg_ids(void)
{
    mixdata.ecgnppg_msg_id = 10;
    stepdata.step_msg_id = 11;
    tempdata.temp_msg_id = 12;
    spo2data.spo2_msg_id = 13;
    ecgdata.ecg_msg_id = 14;
    ppgdata.ppg_msg_id = 15;
    batdata.bat_msg_id = 16;
    falldata.fall_msg_id = 17;
    schedulerdata.sch_msg_id = 18;
}

/**
 * @brief:   Function to send temperature data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void temp_check(void)
{
    int temp_value = 0;
    uint16_t length;
    sts4x_measure_high_precision(&temp_value);
    if (temp_value > 100000) 
    {
      length = sizeof(TEMP_ERROR);
      ble_nus_data_send(&m_nus,TEMP_ERROR,&length,m_conn_handle);
    } 
    else 
    {
      length = sizeof(TEMP_SUCCESS);
      ble_nus_data_send(&m_nus,TEMP_SUCCESS,&length,m_conn_handle);
    }
}

/**
 * @brief:   Function to calculate Moving average
 *
 * @param[in] uint16_t new_data
 *
 * @param[out] uint16_t    
 */
static uint16_t moving_avg(uint16_t new_data)
{
    static uint16_t DATA[MOV_AVE_ARR_SIZE];
    static uint8_t old_data_index_1 = 0;
    static uint32_t sum = 0;
   
     /* Subtract oldest value from sum and add new value to sum  */
    sum = sum - DATA[old_data_index_1] + new_data;

    /* Overwrite new value in place of oldest value */
    DATA[old_data_index_1] = new_data;

    /* Increment index. But remember, this is a circular buffer. */
    if (old_data_index_1 >= (MOV_AVE_ARR_SIZE - 1U))
    {
        old_data_index_1 = 0U;
    }
    else
    {
        old_data_index_1++;
    }
    return (uint16_t)(sum / MOV_AVE_ARR_SIZE);
}

/**
 * @brief:   Function to send temperature data through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_body_temp(void)
{
    int temp_value;
    uint16_t length;
    ret_code_t err_code;
    err_code = sts4x_measure_high_precision(&temp_value);
    if (err_code) 
    {
      printf("Error executing sts4x_measure_high_precision(): %i\n",err_code);
    } 
    else 
    {
      printf("Temperature: %f °C\n", temp_value / 1000.0f);
      tempdata.temp_vals = (uint16_t)(temp_value / 1000.0f);
      length = sizeof(tempdata);
      err_code = ble_nus_data_send(&m_nus,&tempdata,&length,m_conn_handle);
    }
}

/**
 * @brief:   Function to send scheduler status through BLE
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void send_device_status(void)
{
    uint16_t length;
    if (0 == scheduler_enable) 
    {
      schedulerdata.sch_val = 0;
    }
    if (1 == scheduler_enable) 
    {
      schedulerdata.sch_val = 1;
    }
    length = sizeof(schedulerdata);
    ble_nus_data_send(&m_nus, &schedulerdata, &length, m_conn_handle);
}

/**
 * @brief:   Function to get MAC Address of device
 *
 * @param[in] void
 *
 * @param[out] void    
 */
static void get_mac_id(void)
{
    sd_ble_gap_addr_get(mac_id);
    sprintf((char *)mac_add, "ACC_%x%x%x%x%x%x", mac_id->addr[5], mac_id->addr[4], mac_id->addr[3], mac_id->addr[2], mac_id->addr[1], mac_id->addr[0]);
} 



/**
   Main function
 */
int main(void)
{
    bool erase_bonds;
    /* Intialize */
    printf("\nFW Version: %s\n",FW_VERSION);
    battery_mon_en();
    /* Enable Power Suppy for Accelerometer. */
    accel_power_on();
    /* Enable Power Suppy for Temperature. */
    temp_power_on();
    /* Peripherals Initialize. */
    hardware_init();
    #if ACCEL_INIT
    /* Start Accelerometer. */
    bma400_start();
    #endif
    assign_msg_ids();
    #if BLE_ENABLE
    timers_init();
    buttons_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    /* Initialize Advertising. */
    advertising_init();
    conn_params_init();
    peer_manager_init();
    /* Start BLE Advertising. */
    advertising_start(erase_bonds);
    #endif
    for (;;)
    {
      idle_state_handle();
      #if AS7050_INIT
      sendDatatoBLE();
      #endif
      //sendFlashBLE();
    }
}


/**
 * @}
 */