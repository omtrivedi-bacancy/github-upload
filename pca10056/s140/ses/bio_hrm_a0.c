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
 * \file     bio_hrm_a0.c
 * \brief   Define
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stddef.h>
#include <string.h>

#include "bio_hrm_a0.h"
#include "bio_common.h"
#include "as7050_typedefs.h"
#include "error_codes.h"
#include "heartrate.h"
#include "prv.h"


/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

 #define MICROS 1000000
#define MILLIS 1000

#define DEFAULT_CROSSTALK_LEVEL 0

#define MAX_INPUT_BUFFER 2

/* Define the maximum variability allowed relative to HRM-result */
#define MAX_PRV_PERCENT 20

#define MIN_PRV_SAMPLES_PER_SEC 100

#define MIN_ACC_SAMPLE_PERIOS_US 50000
#define MAX_ACC_SAMPLE_PERIOS_US 100000

enum STATES { STATE_INIT_DONE = 0x01, STATE_CNFG_DONE = 0x02, STATE_READY_FOR_DATA = 0x04 };

typedef struct hrm_input {
    uint16_t accelerometer_counter;
    uint16_t input_data[HRM_APP_INPUT_BUF_NUM];
    uint16_t input_data_num;
    uint8_t state_changes[HRM_APP_INPUT_BUF_NUM];
    hrAlgorithmInput alg_input;
} hrm_input_t;

struct hrm_config {
    uint8_t states;
    uint8_t enable_prv;
    uint16_t average_factor;
    uint16_t input_data_max;
    volatile uint8_t write_index;
    volatile uint8_t read_index;
    volatile hrm_input_t input[MAX_INPUT_BUFFER];
    uint32_t ppg_sample_period_us;
    uint32_t acc_sample_period_us;
    hrm_app_result_t result;
    uint8_t result_available;
};

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
 static struct hrm_config g_hrm_config = {0};

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
err_code_t bio_hrm_a0_initialize(void)
{
    err_code_t result;

    result = bio_hrm_a0_shutdown();

    if (ERR_SUCCESS == result) {
        g_hrm_config.states = STATE_INIT_DONE;
    }

    return result;
}

/*!
 * \brief Configures the bio app.
 *
 * This function can only be called when the bio app is initialized and not in a processing session.
 *
 * \param[in] p_config Pointer to the configuration structure, see ::bio_hrm_a0_configuration_t. It must not be NULL.
 * \param[in] size     Size of the configuration structure. It must be `sizeof(bio_hrm_a0_configuration_t)`.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_POINTER    Invalid pointer to configuration structure.
 * \retval ::ERR_SIZE       Size does not match expected size.
 * \retval ::ERR_PERMISSION Not initialized.
 */

err_code_t bio_hrm_a0_configure(const void *p_config, uint8_t size)
{
    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    }

    /* convert integer to boolean */
    g_hrm_config.enable_prv = 1;//!!enable_prv;
    return ERR_SUCCESS;
}

/*!
 * \brief Provides the sensor signals and accelerometer sample periods to the bio app and starts a processing session.
 *
 * This function can only be called when the bio app is initialized.
 *
 * \param[in] p_signal_sample_periods_us Pointer to an array containing the sample periods of each signal in
 *                                       microseconds. There must be ::BIO_HRM_A0_SIGNAL_NUM items in the array,
 *                                       ordered according to ::bio_hrm_a0_signal. This value must not be NULL. The
 *                                       sample period pointed to must not be zero.
 * \param[in] signal_num                 The number of sensor signals provided to the bio app. It must be
 *                                       ::BIO_HRM_A0_SIGNAL_NUM.
 * \param[in] acc_sample_period_us       The sample period of the accelerometer in microseconds. It must not be zero.
 *
 * \htmlonly
 *     p_signal_sample_
 *     _periods_us
 *          │
 *          └─►┌──────────┐   ┌──►┌───────────────────┐
 *         PPG │          ├───┘   │ PPG Sample Period │
 *             └──────────┘       └───────────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid sample periods or invalid signal count provided.
 * \retval ::ERR_POINTER    p_signal_sample_periods_us is NULL.
 * \retval ::ERR_PERMISSION Not initialized.
 */

err_code_t bio_hrm_a0_start(const uint32_t *p_signal_sample_periods_us, uint8_t signal_num,
                            uint32_t acc_sample_period_us)
{
    err_code_t result = ERR_SUCCESS;
    int samples_per_second;
    uint8_t afe_driver_type;
    hrAlgorithmOutput output;
    const uint32_t min_sample_period = MICROS / HRM_APP_INPUT_BUF_NUM;
    const uint32_t max_sample_period = 100000; /* 10hz */

    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    }
    if ((min_sample_period > *p_signal_sample_periods_us) && (max_sample_period < *p_signal_sample_periods_us)) {
        return ERR_ARGUMENT;
    }

    if ((MIN_ACC_SAMPLE_PERIOS_US > acc_sample_period_us) || (MAX_ACC_SAMPLE_PERIOS_US < acc_sample_period_us)) {
        return ERR_ARGUMENT;
    }

    g_hrm_config.acc_sample_period_us = acc_sample_period_us;
    g_hrm_config.ppg_sample_period_us = (uint32_t)p_signal_sample_periods_us;
    g_hrm_config.states |= STATE_CNFG_DONE;
    
  

    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_CNFG_DONE & g_hrm_config.states)) {
        return ERR_CONFIG;
    }

    samples_per_second = MICROS / g_hrm_config.ppg_sample_period_us;
    if (samples_per_second > HR_ADC_VALUES_PER_SECOND) {
        g_hrm_config.average_factor = samples_per_second / HR_ADC_VALUES_PER_SECOND;
    } else {
        g_hrm_config.average_factor = 1;
    }
    g_hrm_config.input_data_max = HR_ADC_VALUES_PER_SECOND * g_hrm_config.average_factor;
    if (HRM_APP_INPUT_BUF_NUM < g_hrm_config.input_data_max) {
        result = ERR_SIZE;
    }

    if (ERR_SUCCESS == result) {
        afe_driver_type = (samples_per_second > HR_ADC_VALUES_PER_SECOND) ? HR_AFE_DRIVER_TYPE_LOW_POWER
                                                                          : HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER;
        if (HEARTRATE_ALGORITHM_OK != heartrateInitialise(afe_driver_type,
                                                          (MICROS / g_hrm_config.acc_sample_period_us) * MILLIS,
                                                          DEFAULT_CROSSTALK_LEVEL, &output)) {
            result = ERR_CONFIG;
        }
    }

    if (ERR_SUCCESS == result && g_hrm_config.enable_prv) {
        afe_driver_type = (samples_per_second > HR_ADC_VALUES_PER_SECOND) ? HR_AFE_DRIVER_TYPE_NORMAL_POWER
                                                                          : HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER;
        if (MIN_PRV_SAMPLES_PER_SEC > samples_per_second) {
            result = ERR_CONFIG;
        } else if (PRV_ALGORITHM_OK != prvInitialise(afe_driver_type, samples_per_second)) {
            result = ERR_CONFIG;
        }
    }

    g_hrm_config.write_index = 0;
    g_hrm_config.read_index = 0;

    g_hrm_config.input[0].accelerometer_counter = 0;
    g_hrm_config.input[0].input_data_num = 0;
    g_hrm_config.input[0].alg_input.numAccSamples = 0;
    g_hrm_config.input[1].accelerometer_counter = 0;
    g_hrm_config.input[1].input_data_num = 0;
    g_hrm_config.input[1].alg_input.numAccSamples = 0;

    g_hrm_config.result_available = FALSE;

    if (ERR_SUCCESS == result) {
        g_hrm_config.states |= STATE_READY_FOR_DATA;
    } else {
        g_hrm_config.states &= ~STATE_READY_FOR_DATA;
    }

    return result;
}

/*!
 * \brief Provides measurement data to the bio app.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * \param[in] signal_samples_type The data type of sensor signals. It must be ::BIO_HRM_A0_SIGNAL_DATA_TYPE.
 * \param[in] signal_num          The number of sensor signals provided to the bio app. It must be
 *                                ::BIO_HRM_A0_SIGNAL_NUM.
 * \param[in] p_signal_samples    Pointer to an array of ::bio_signal_samples_t. There must be ::BIO_HRM_A0_SIGNAL_NUM
 *                                items in the array, ordered according to ::bio_hrm_a0_signal.
 * \param[in] pp_agc_statuses     Pointer to an array containing pointers to the AGC status of a signal. The items of
 *                                the pointer array are ordered according to ::bio_hrm_a0_signal. If no AGC status is
 *                                available for a channel, the corresponding pointer must point to NULL. The number of
 *                                items in the pointer array must be ::BIO_HRM_A0_SIGNAL_NUM. This argument is ignored
 *                                by this app as it does not use AGC status information.
 * \param[in] p_acc_samples       Pointer to an array containing accelerometer samples. This value must not be NULL if
 *                                acc_sample_num is greater than zero.
 * \param[in] acc_sample_num      The number of accelerometer samples contained in the array pointed to by
 *                                p_acc_samples.
 * \param[out] p_result           The value pointed to by this argument is updated with information whether the bio app
 *                                is ready for execution. This value must not be NULL.
 *
 * \htmlonly
 *     p_signal_                                  pp_agc_
 *     samples                                    statuses
 *        │               p_u16                      │
 *        └─►┌──────────┐   ┌──►┌──────────────┐     └─►┌──────────┐   ┌──►┌────────────┐
 *           │   data ──────┘   │ PPG Sample 0 │    PPG │          ├───┘   │ PPG        │
 *       PPG │ ──────── │       ├──────────────┤        └──────────┘       │ AGC Status │
 *           │ count: 2 │       │ PPG Sample 1 │                           └────────────┘
 *           └──────────┘       └──────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Data accepted.
 * \retval ::ERR_ARGUMENT   Invalid signal sample type or invalid signal count provided.
 * \retval ::ERR_POINTER    At least one pointer argument is NULL which must not be NULL.
 * \retval ::ERR_OVERFLOW   Cannot store the provided data due to exhausted internal buffers.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_hrm_a0_set_input(bio_signal_samples_type_t signal_samples_type,
                                const bio_signal_samples_t *p_signal_samples,
                                const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                                const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                                bio_execution_status_t *p_result)
{
    err_code_t result = ERR_SUCCESS;
    uint16_t copy_num;
    volatile hrm_input_t *p_input;
    hrm_app_agc_states_t agc_state;
    uint8_t i;

    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_hrm_config.states)) {
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_signal_samples);
    M_CHECK_NULL_POINTER(p_result);
    if (0 < acc_sample_num) {
        M_CHECK_NULL_POINTER(p_acc_samples);
    }

    if (0 == signal_num) {
        return ERR_ARGUMENT;
    }

    *p_result = FALSE;

    /* copy all data to internal buffer */
    while (0 < signal_num) {
        p_input = &g_hrm_config.input[g_hrm_config.write_index];
        copy_num = ((g_hrm_config.input_data_max - p_input->input_data_num) >= signal_num)
                       ? signal_num
                       : g_hrm_config.input_data_max - p_input->input_data_num;

        memcpy((uint8_t *)&p_input->input_data[p_input->input_data_num], p_signal_samples, copy_num * sizeof(uint16_t));
        memset((void *)&(p_input->state_changes[p_input->input_data_num]), agc_state, copy_num);
        signal_num -= copy_num;
        p_signal_samples += copy_num;
        p_input->input_data_num += copy_num;

        /* save agc status and accelerometer data on every interrupt */
        if ((0 < acc_sample_num) && (0 < (HR_ACC_MAX_VALUES_PER_SECOND - p_input->alg_input.numAccSamples))) {
            for (i = 0; i < acc_sample_num; i++) {
                p_input->alg_input.accX[p_input->alg_input.numAccSamples] = p_acc_samples[i].x;
                p_input->alg_input.accY[p_input->alg_input.numAccSamples] = p_acc_samples[i].y;
                p_input->alg_input.accZ[p_input->alg_input.numAccSamples] = p_acc_samples[i].z;
                p_input->alg_input.numAccSamples++;
            }
        }

        if (g_hrm_config.input_data_max == p_input->input_data_num) {
            if (g_hrm_config.write_index == g_hrm_config.read_index) {
                g_hrm_config.write_index = !g_hrm_config.write_index;
                *p_result = TRUE;
                continue; /* start size calculation again */
            } else {
                return ERR_OVERFLOW;
            }
        }
    }

    return result;
}


/*!
 * \brief Processes the data previously provided to the bio app.
 *
 * This function can be called once after ::bio_hrm_a0_set_input indicated via the p_result argument that the bio app
 * is executable. Afterwards, this function can be called again when a subsequent call of ::bio_hrm_a0_set_input
 * indicated that the bio app is executable. The bio app must be initialized and in a processing session when calling
 * this function.
 *
 * \param[out] p_result The value pointed to by this argument is updated with information whether the bio app has data
 *                      available for output. This value must not be NULL.
 *
 * \retval ::ERR_SUCCESS    Processing successful.
 * \retval ::ERR_POINTER    p_result is NULL.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_hrm_a0_execute(bio_output_status_t *p_result)
{
    err_code_t result;
    uint32_t i, j;
    uint32_t average;
    hrAlgorithmOutput hrm_output;
    prvAlgorithmOutput prv_output;
    uint32_t hrm_ppi_ms;
    uint16_t hrm_ppi_ms_max_variability;
    volatile hrm_input_t *p_input = &g_hrm_config.input[g_hrm_config.read_index];

    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_hrm_config.states)) {
        return ERR_CONFIG;
    //} else if (g_hrm_config.input_data_max > p_input->input_data_num) {
    } else if (g_hrm_config.input_data_max < p_input->input_data_num) {
        return ERR_NO_DATA;
    }

    M_CHECK_NULL_POINTER(p_result);

    /* average data and copy to algorithm input buffer */
    for (i = 0; i < g_hrm_config.input_data_max / g_hrm_config.average_factor; i++) {
        average = 0;
        for (j = 0; j < g_hrm_config.average_factor; j++) {
            average += p_input->input_data[(g_hrm_config.average_factor * i) + j];
        }
        p_input->alg_input.adcValue[i] = average / g_hrm_config.average_factor;
       // printf("hrm adc ,g_hrm_config.average_factor,average  %x  %x  %x \n",p_input->alg_input.adcValue[i],g_hrm_config.average_factor,average);
    }

    *p_result = FALSE;

    /* hand in input immediately to the algorithm */
    heartrateInput((hrAlgorithmInput *)&(p_input->alg_input), 0, 0, 0);
    //result = heartrateCalculate(0, &hrm_output);
    
    if (heartrateCalculate(0, &hrm_output)) {
        result = ERR_POINTER;
    } else {
        g_hrm_config.result.heartrate = hrm_output.heartrate;
        g_hrm_config.result.signal_quality = hrm_output.signalQuality;
        g_hrm_config.result.motion_frequency = hrm_output.motionFreq;
        g_hrm_config.result.prv_ms_num = 0;
        g_hrm_config.result_available = TRUE;
        *p_result = TRUE;
        result = ERR_SUCCESS;
    }
   // printf("hrm output heart_rate ,motion_frequency,signal_quality  %x  %x  %x \n",hrm_output.heartrate,hrm_output.motionFreq,hrm_output.signalQuality);
    
    //if (ERR_SUCCESS == result && g_hrm_config.enable_prv) {
    if (1) {

        //printf("p_input->input_data_num && g_hrm_config.result.prv_ms_num   %x   %x %x\n",p_input->input_data_num && g_hrm_config.result.prv_ms_num,g_hrm_config.result.prv_ms_num,(p_input->input_data_num && g_hrm_config.result.prv_ms_num));
        /* convert HRM heartrate (0.1bpm) to PPI (ms) */ 
        hrm_ppi_ms = (uint32_t)(1000 * 600) / g_hrm_config.result.heartrate;

        /* calculate the maximum variability (relative to HRM-result) */
        hrm_ppi_ms_max_variability = (uint16_t)((hrm_ppi_ms * MAX_PRV_PERCENT) / 100);
        

        for (i = 0; i < p_input->input_data_num && g_hrm_config.result.prv_ms_num < PRV_DATA_NUM; i++) {
            if (PRV_ALGORITHM_VALUE_AVAILABLE == prvCalculate(p_input->input_data[i], &prv_output)) {
                /* if the PRV result is outside the allowed range, then exclude this result */
                if ((prv_output.peakToPeakIntervalMs < (hrm_ppi_ms + hrm_ppi_ms_max_variability)) &&
                    (prv_output.peakToPeakIntervalMs > (hrm_ppi_ms - hrm_ppi_ms_max_variability))) {
                    g_hrm_config.result.prv_ms[g_hrm_config.result.prv_ms_num++] = prv_output.peakToPeakIntervalMs;
                }
            }
        }
    }

    p_input->alg_input.numAccSamples = 0;
    p_input->accelerometer_counter = 0;
    p_input->input_data_num = 0;
    g_hrm_config.read_index = !g_hrm_config.read_index;

    return result;
}

/*!
 * \brief Copies output data generated by the app to a buffer provided by the caller.
 *
 * This function can be called once after ::bio_hrm_a0_execute indicated via the p_result argument that output data is
 * available. Afterwards, this function can be called again when a subsequent call of ::bio_hrm_a0_execute indicdated
 * that output data is available. The bio app must be initialized and in a processing session when calling this
 * function.
 *
 * The size of the output data is sizeof(bio_hrm_a0_output_t).
 *
 * \param[out] p_dest   Points to the start of the buffer where the output data shall be copied to. This value must not
 *                      be NULL.
 * \param[inout] p_size Points to a variable containing the size of the buffer where the output data shall be copied to.
 *                      After copying, the value of this variable is updated with the actual size of the copied data.
 *                      This value must not be NULL.
 *
 * \retval ::ERR_SUCCESS    Copying successful.
 * \retval ::ERR_POINTER    At least one pointer argument is NULL which must not be NULL.
 * \retval ::ERR_SIZE       The size of the buffer is not sufficient.
 * \retval ::ERR_NO_DATA    No output data is available for copying.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */

err_code_t bio_hrm_a0_get_output(void *p_dest, uint16_t *p_size)
{
    if (0 == (STATE_INIT_DONE & g_hrm_config.states)) {
        return ERR_PERMISSION;
    } else if (0 == (STATE_READY_FOR_DATA & g_hrm_config.states)) {
        return ERR_CONFIG;
    }

    M_CHECK_NULL_POINTER(p_dest);
    g_hrm_config.result_available = TRUE;

    if (!g_hrm_config.result_available) {
        return ERR_NO_DATA;
    }
      
    g_hrm_config.result_available = FALSE;
    
    memcpy(p_dest, &g_hrm_config.result, *p_size); 
    return ERR_SUCCESS;
}

/*!
 * \brief Stops the current processing session of the bio app, allowing it to be reconfigured.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * \retval ::ERR_SUCCESS    Stopping successful.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_hrm_a0_stop(void)
{
    if (STATE_INIT_DONE == g_hrm_config.states) {
        return ERR_PERMISSION;
    }

    g_hrm_config.states = STATE_CNFG_DONE;

    return ERR_SUCCESS;
}


/*!
 * \brief De-initializes the bio app.
 *
 * \retval ::ERR_SUCCESS De-initialization successful.
 */
err_code_t bio_hrm_a0_shutdown(void)
{
    memset((void *)&g_hrm_config, 0, sizeof(g_hrm_config));

    return ERR_SUCCESS;
}
