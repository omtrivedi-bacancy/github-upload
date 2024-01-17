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
 * \file     bio_spo2_a0.c
 * \brief   Define
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stddef.h>
#include <string.h>

#include "bio_spo2_a0.h"
#include "bio_common.h"
#include "as7050_typedefs.h"
#include "error_codes.h"
#include "spo2.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#define SPO2_SAMPLES_PER_SECOND 20
#define MICROS 1000000
#define MAX_INPUT_BUFFER 2

enum STATES { STATE_INIT_DONE = 0x01, STATE_CNFG_DONE = 0x02, STATE_READY_FOR_DATA = 0x04 };


struct spo2_config {
    uint8_t states;
    uint32_t sample_period_us;
    uint16_t average_factor;

    volatile uint8_t write_index;
    volatile uint8_t read_index;
    volatile spo2AlgorithmInput_t input[MAX_INPUT_BUFFER];
    uint16_t num_samples[MAX_INPUT_BUFFER];

    uint16_t raw_samples_count;
    uint32_t ambient_light_avg;
    uint32_t ppg_ir_avg;
    uint32_t ppg_red_avg;

    spo2_app_result_t spo2_output;
    uint8_t output_available;

    spo2AlgorithmCalibration_t calib_params;
};


/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

static struct spo2_config g_spo2_config = {0};

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

 
/*!
 * \brief Initializes the bio app.
 *
 * \retval ::ERR_SUCCESS Initialized successfully.
 */
err_code_t bio_spo2_a0_initialize(void)
{
    err_code_t result;

    result = bio_spo2_a0_shutdown();

    if (ERR_SUCCESS == result) {
        g_spo2_config.states = STATE_INIT_DONE;
    }

    return result;
}


/*!
 * \brief Configures the bio app.
 *
 * This function can only be called when the bio app is initialized and not in a processing session.
 *
 * \param[in] p_config Pointer to the configuration structure, see ::bio_spo2_a0_configuration_t. It must not be NULL.
 * \param[in] size     Size of the configuration structure. It must be `sizeof(bio_spo2_a0_configuration_t)`.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_POINTER    Invalid pointer to configuration structure.
 * \retval ::ERR_SIZE       Size does not match expected size.
 * \retval ::ERR_PERMISSION Not initialized.
 */
err_code_t bio_spo2_a0_configure(const void *p_config, uint8_t size)
{
  if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    }

    M_CHECK_NULL_POINTER(p_config);

    memcpy(&g_spo2_config.calib_params, p_config, size);

    return ERR_SUCCESS;
}


/*!
 * \brief Provides the sensor signals and accelerometer sample periods to the bio app and starts a processing session.
 *
 * This function can only be called when the bio app is initialized.
 *
 * \param[in] p_signal_sample_periods_us Pointer to an array containing the sample periods of each signal in
 *                                       microseconds. There must be ::BIO_SPO2_A0_SIGNAL_NUM items in the array,
 *                                       ordered according to ::bio_spo2_a0_signal. This value must not be NULL. This
 *                                       app requires that all signals are samples with an identical non-zero sample
 *                                       period.
 * \param[in] signal_num                 The number of sensor signals provided to the bio app. It must be
 *                                       ::BIO_SPO2_A0_SIGNAL_NUM.
 * \param[in] acc_sample_period_us       The sample period of the accelerometer in microseconds. This value is ignored
 *                                       by this bio app as it does not use accelerometer data.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid sample periods or invalid signal count provided.
 * \retval ::ERR_POINTER    p_signal_sample_periods_us is NULL.
 * \retval ::ERR_PERMISSION Not initialized.
 */
err_code_t bio_spo2_a0_start(const uint32_t *p_signal_sample_periods_us, uint8_t signal_num,
                             uint32_t acc_sample_period_us)
{

    int samples_per_second;
    spo2AlgorithmOutput_t output;

    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    }
    if (0 == p_signal_sample_periods_us) {
        return ERR_ARGUMENT;
    }
    if (BIO_SPO2_A0_SIGNAL_NUM != signal_num){
        return ERR_ARGUMENT;
    }

    g_spo2_config.sample_period_us = (uint32_t)&p_signal_sample_periods_us;
    g_spo2_config.states |= STATE_CNFG_DONE;

    err_code_t result = ERR_SUCCESS;


    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_CNFG_DONE & g_spo2_config.states)) {
        return ERR_CONFIG;
    }

    samples_per_second = MICROS / g_spo2_config.sample_period_us;
    if (samples_per_second > SPO2_SAMPLES_PER_SECOND) {
        g_spo2_config.average_factor = samples_per_second / SPO2_SAMPLES_PER_SECOND;
    } else {
        g_spo2_config.average_factor = 1;
    }

    if (ERR_SUCCESS == result) {
        if (SPO2_ALGORITHM_OK != spo2Initialise(SPO2_AFE_DRIVER_TYPE_UNKNOWN, &output)) {
            result = ERR_CONFIG;
        }
    }

    if (ERR_SUCCESS == result) {
        if (SPO2_ALGORITHM_OK != spo2UpdateCalibration(&g_spo2_config.calib_params)) {
            result = ERR_CONFIG;
        }
    }

    g_spo2_config.write_index = 0;
    g_spo2_config.read_index = 0;
    g_spo2_config.num_samples[0] = 0;
    g_spo2_config.num_samples[1] = 0;

    g_spo2_config.raw_samples_count = 0;
    g_spo2_config.ambient_light_avg = 0;
    g_spo2_config.ppg_ir_avg = 0;
    g_spo2_config.ppg_red_avg = 0;

    g_spo2_config.output_available = FALSE;

    if (ERR_SUCCESS == result) {
        g_spo2_config.states |= STATE_READY_FOR_DATA;
    } else {
        g_spo2_config.states &= ~STATE_READY_FOR_DATA;
    }

    return result;
}

/*!
 * @brief Commits the cyclic measurement data to the SPO2 module
 * *
 * @param[in] uint32_t *p_ambient_light_samples, uint32_t *p_ir_samples,
                                      uint32_t *p_red_samples, uint16_t num_samples, spo2_app_agc_states_t agc_state,
                                      uint8_t pd_on_offset_current, uint8_t *p_ready_for_execution
 *
 * @param[out] err_code_t 
 */
err_code_t spo2_app_append_input_data(uint32_t *p_ambient_light_samples, uint32_t *p_ir_samples,
                                      uint32_t *p_red_samples, uint16_t num_samples, spo2_app_agc_states_t agc_state,
                                      uint8_t pd_on_offset_current, uint8_t *p_ready_for_execution)
{
    uint16_t i;

    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_spo2_config.states)) {
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_ambient_light_samples);
    M_CHECK_NULL_POINTER(p_ir_samples);
    M_CHECK_NULL_POINTER(p_red_samples);
    M_CHECK_NULL_POINTER(p_ready_for_execution);

    if (0 == num_samples) {
        return ERR_ARGUMENT;
    }

    *p_ready_for_execution = FALSE;

    if (g_spo2_config.num_samples[g_spo2_config.write_index] >= SPO2_ADC_VALUES_PER_SECOND) {
        return ERR_OVERFLOW;
    }
    //printf("num_samples = %d\n",num_samples);
    for (i = 0; i < num_samples; i++) {
        
        g_spo2_config.ambient_light_avg += p_ambient_light_samples[i];
        g_spo2_config.ppg_ir_avg += p_ir_samples[i];
        g_spo2_config.ppg_red_avg += p_red_samples[i];
        g_spo2_config.raw_samples_count++;
        //printf("g_spo2_config.ppg_ir_avg =%d\n",g_spo2_config.ppg_ir_avg);

        if (g_spo2_config.raw_samples_count >= g_spo2_config.average_factor) {

            g_spo2_config.ambient_light_avg /= g_spo2_config.raw_samples_count;
            g_spo2_config.ppg_ir_avg /= g_spo2_config.raw_samples_count;
            g_spo2_config.ppg_red_avg /= g_spo2_config.raw_samples_count;

            spo2LedData_t ir_data;
            spo2LedData_t r_data;
            ir_data.absValue = g_spo2_config.ppg_ir_avg;
            // ir_data.acValue = 0;
            ir_data.ledChange = agc_state;
            r_data.absValue = g_spo2_config.ppg_red_avg;
            // r_data.acValue = 0;
            r_data.ledChange = agc_state;

            g_spo2_config.input[g_spo2_config.write_index]
                .ambient[g_spo2_config.num_samples[g_spo2_config.write_index]] = g_spo2_config.ambient_light_avg;
            g_spo2_config.input[g_spo2_config.write_index]
                .infrared[g_spo2_config.num_samples[g_spo2_config.write_index]] = ir_data;
            g_spo2_config.input[g_spo2_config.write_index].red[g_spo2_config.num_samples[g_spo2_config.write_index]] =
                r_data;
            g_spo2_config.input[g_spo2_config.write_index]
                .pdOffset[g_spo2_config.num_samples[g_spo2_config.write_index]] = pd_on_offset_current;
            g_spo2_config.num_samples[g_spo2_config.write_index]++;

            g_spo2_config.raw_samples_count = 0;
            g_spo2_config.ambient_light_avg = 0;
            g_spo2_config.ppg_ir_avg = 0;
            g_spo2_config.ppg_red_avg = 0;

            if (g_spo2_config.num_samples[g_spo2_config.write_index] >= SPO2_ADC_VALUES_PER_SECOND) {
                if (g_spo2_config.write_index == g_spo2_config.read_index) {
                    g_spo2_config.write_index = !g_spo2_config.write_index;
                    *p_ready_for_execution = TRUE;
                } else {
                    return ERR_OVERFLOW;
                }
            }
        }
    }

    return ERR_SUCCESS;
}


/*!
 * @brief Provides measurement data to the bio app.
 * *
 * @param[in] bio_signal_samples_type_t signal_samples_type,
                                  const bio_signal_samples_t *p_signal_samples,
                                 const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                                 const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                                 bio_execution_status_t *p_result
 *
 * @param[out] err_code_t 
 */
err_code_t bio_spo2_a0_set_input(bio_signal_samples_type_t signal_samples_type,
                                  const bio_signal_samples_t *p_signal_samples,
                                 const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                                 const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                                 bio_execution_status_t *p_result)
{
    bio_agc_change_t agc_state;
    uint16_t num_samples = 0;

    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_spo2_config.states)) {
        return ERR_CONFIG;
    }
    if (BIO_SIGNAL_SAMPLES_TYPE_U16 != signal_samples_type){
        return ERR_CONFIG;
    }
    if (BIO_SPO2_A0_SIGNAL_NUM != signal_num){
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_signal_samples);
    M_CHECK_NULL_POINTER(p_result);

    *p_result = BIO_EXECUTION_STATUS_NOT_EXECUTABLE;
    num_samples = p_signal_samples[BIO_SPO2_A0_SIGNAL_NUM].count;
    if (g_spo2_config.num_samples[g_spo2_config.write_index] >= SPO2_ADC_VALUES_PER_SECOND) {
        return ERR_OVERFLOW;
    }
    //for (uint8_t signal_idx = 0; signal_idx < signal_num; signal_idx++) {
    if (BIO_SIGNAL_SAMPLES_TYPE_U16 == signal_samples_type){
        memcpy(&g_spo2_config.ppg_red_avg,p_signal_samples[BIO_SPO2_A0_SIGNAL_PPG_RED].samples.p_u16,sizeof(uint16_t));
        memcpy(&g_spo2_config.ppg_ir_avg,p_signal_samples[BIO_SPO2_A0_SIGNAL_PPG_IR].samples.p_u16,sizeof(uint16_t));
        memcpy(&g_spo2_config.ambient_light_avg,p_signal_samples[BIO_SPO2_A0_SIGNAL_AMBIENT].samples.p_u16,sizeof(uint16_t));
      }

        g_spo2_config.raw_samples_count++;

        if (g_spo2_config.raw_samples_count >= g_spo2_config.average_factor) {

            g_spo2_config.ambient_light_avg /= g_spo2_config.raw_samples_count;
            g_spo2_config.ppg_ir_avg /= g_spo2_config.raw_samples_count;
            g_spo2_config.ppg_red_avg /= g_spo2_config.raw_samples_count;

            spo2LedData_t ir_data;
            spo2LedData_t r_data;
            ir_data.absValue = g_spo2_config.ppg_ir_avg;
            // ir_data.acValue = 0;
            ir_data.ledChange = pp_agc_statuses[BIO_SPO2_A0_SIGNAL_PPG_IR];
            r_data.absValue = g_spo2_config.ppg_red_avg;
            // r_data.acValue = 0;
            r_data.ledChange = pp_agc_statuses[BIO_SPO2_A0_SIGNAL_PPG_RED];

            g_spo2_config.input[g_spo2_config.write_index]
                .ambient[g_spo2_config.num_samples[g_spo2_config.write_index]] = g_spo2_config.ambient_light_avg;
            g_spo2_config.input[g_spo2_config.write_index]
                .infrared[g_spo2_config.num_samples[g_spo2_config.write_index]] = ir_data;
            g_spo2_config.input[g_spo2_config.write_index]
                .red[g_spo2_config.num_samples[g_spo2_config.write_index]] = r_data;
            g_spo2_config.input[g_spo2_config.write_index]
                .pdOffset[g_spo2_config.num_samples[g_spo2_config.write_index]] = pp_agc_statuses;//pd_on_offset_current;
            g_spo2_config.num_samples[g_spo2_config.write_index]++;

            g_spo2_config.raw_samples_count = 0;
            g_spo2_config.ambient_light_avg = 0;
            g_spo2_config.ppg_ir_avg = 0;
            g_spo2_config.ppg_red_avg = 0;

            if (g_spo2_config.num_samples[g_spo2_config.write_index] >= SPO2_ADC_VALUES_PER_SECOND) {
                if (g_spo2_config.write_index == g_spo2_config.read_index) {
                    g_spo2_config.write_index = !g_spo2_config.write_index;
                    *p_result = BIO_EXECUTION_STATUS_EXECUTABLE;
                } else {
                    return ERR_OVERFLOW;
                }
            }
            //}
            
        
    }

    return ERR_SUCCESS;
    
}


/*!
 * @brief Processes the data previously provided to the bio app.
 * *
 * @param[in] bio_output_status_t *p_result
 *
 * @param[out] err_code_t 
 */
 err_code_t bio_spo2_a0_execute(bio_output_status_t *p_result)
{
    err_code_t result;

    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_spo2_config.states)) {
        return ERR_CONFIG;
    } else if (g_spo2_config.write_index == g_spo2_config.read_index) {
        return ERR_NO_DATA;
    }

    M_CHECK_NULL_POINTER(p_result);

    spo2Input((spo2AlgorithmInput_t *)&g_spo2_config.input[g_spo2_config.read_index]);
    if (spo2Calculate((spo2AlgorithmOutput_t *)&g_spo2_config.spo2_output)) {
        result = ERR_CONFIG;
        *p_result = BIO_OUTPUT_STATUS_DATA_UNAVAILABLE;
    } else {
        *p_result = BIO_OUTPUT_STATUS_DATA_AVAILABLE;
        g_spo2_config.output_available = TRUE;
        result = ERR_SUCCESS;
    }

    g_spo2_config.num_samples[g_spo2_config.read_index] = 0;
    g_spo2_config.read_index = !g_spo2_config.read_index;

    return result;

}

/*!
 * @brief Copies output data generated by the app to a buffer provided by the caller.
 *
 * This function can be called once after ::bio_spo2_a0_execute indicated via the p_result argument that output data is
 * available. Afterwards, this function can be called again when a subsequent call of ::bio_spo2_a0_execute indicdated
 * that output data is available. The bio app must be initialized and in a processing session when calling this
 * function.
  * *
 * @param[in] void *p_dest, uint16_t *p_size
 *
 * @param[out] err_code_t 

 */
err_code_t bio_spo2_a0_get_output(void *p_dest, uint16_t *p_size)
{
   if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_spo2_config.states)) {
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_dest);
    M_CHECK_NULL_POINTER(p_size);

    if (!g_spo2_config.output_available) {
        return ERR_NO_DATA;
    }

    g_spo2_config.output_available = FALSE;
    memcpy(p_dest, &g_spo2_config.spo2_output,*p_size);

    return ERR_SUCCESS; 
}

/*!
 * @brief Readout the calculated SPO2 data
  * *
 * @param[in] spo2_app_result_t *p_result
 *
 * @param[out] err_code_t 
 */
err_code_t spo2_app_get_output_data(spo2_app_result_t *p_result)
{
    if (0 == (STATE_INIT_DONE & g_spo2_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_spo2_config.states)) {
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_result);

    if (!g_spo2_config.output_available) {
        return ERR_NO_DATA;
    }

    g_spo2_config.output_available = FALSE;
    memcpy(p_result, &g_spo2_config.spo2_output, sizeof(spo2_app_result_t));

    return ERR_SUCCESS;
}

/*!
 * @brief Stops the current processing session of the bio app, allowing it to be reconfigured.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * @retval ::ERR_SUCCESS    Stopping successful.
 * @retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_spo2_a0_stop(void)
{
     if (STATE_INIT_DONE == g_spo2_config.states) {
        return ERR_PERMISSION;
    }

    g_spo2_config.states = STATE_CNFG_DONE;

    return ERR_SUCCESS;
}


/*!
 * @brief De-initializes the bio app.
  * *
 * @param[in] void
 *
 * @param[out] err_code_t 
 */
err_code_t bio_spo2_a0_shutdown(void)
{
    memset((void *)&g_spo2_config, 0, sizeof(g_spo2_config));

    return ERR_SUCCESS;
}




