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
 * \file     bio_hrm_a0.h
 * \brief   Define
 */

#ifndef __BIO_HRM_A0_H__
#define __BIO_HRM_A0_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file       bio_hrm_a0.h
 * \authors    ARIT, PKRN
 * \copyright  ams
 * \addtogroup bio_hrm_a0_group
 *
 * \brief This module provides an interface to the ams HRM algorithm which conforms to the Generic Bio App Handler
 *        Interface Specification.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "bio_hrm_a0_typedefs.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#ifndef HRM_APP_INPUT_BUF_NUM
/*!
 * \brief The internal buffer is sized to support a sampling frequency of 200 Hz, assuming that ::bio_hrm_a0_execute is
 *        called at least every second when the bio app is executable. To accommodate for timing accuracies, the actual
 *        size of the buffer can support a sampling frequency of up to 250 Hz.
 */
#define HRM_APP_INPUT_BUF_NUM 250
#endif

#ifndef TRUE
/*! Alias for boolean true */
#define TRUE 1
#endif

#ifndef FALSE
/*! Alias for boolean false */
#define FALSE 0
#endif

/*! Number of maximum found PRV data */
#define PRV_DATA_NUM 5

/*! Current state changes list */
typedef enum {
    HRM_APP_AGC_STATE_UNCHANGED = 0, /*!< AGC was not changed */
    HRM_APP_AGC_STATE_INCREASED = 1, /*!< AGC was increased */
    HRM_APP_AGC_STATE_DECREASED = 2, /*!< AGC was decreased */
    HRM_APP_AGC_STATE_UNKNOWN = 3    /*!< AGC state is unknown */
} hrm_app_agc_states_t;

/*! Output data of HRM algorithm */
typedef struct {
    uint16_t heartrate;            /*!< Heart rate [units=0.1bpm]  [815==>81.5bpm] */
    uint8_t signal_quality;        /*!< The quality of the heartrate signal (0 - best quality) */
    uint8_t motion_frequency;      /*!< The detected motion-frequency [units=bpm] (0 - no motion) */
    uint16_t prv_ms[PRV_DATA_NUM]; /*!< Buffer with PRV data (unit: ms)*/
    uint8_t prv_ms_num;            /*!< Number of used PRV data */
} hrm_app_result_t;

/*! Accelerometer data */
typedef struct {
    int16_t x; /*!< x coordinate */
    int16_t y; /*!< y coordinate */
    int16_t z; /*!< z coordinate */
} acc_data_t;
/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Initializes the bio app.
 *
 * \retval ::ERR_SUCCESS Initialized successfully.
 */
err_code_t bio_hrm_a0_initialize(void);

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
err_code_t bio_hrm_a0_configure(const void *p_config, uint8_t size);

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
                            uint32_t acc_sample_period_us);

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
                                bio_execution_status_t *p_result);

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
err_code_t bio_hrm_a0_execute(bio_output_status_t *p_result);

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
err_code_t bio_hrm_a0_get_output(void *p_dest, uint16_t *p_size);

/*!
 * \brief Stops the current processing session of the bio app, allowing it to be reconfigured.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * \retval ::ERR_SUCCESS    Stopping successful.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_hrm_a0_stop(void);

/*!
 * \brief De-initializes the bio app.
 *
 * \retval ::ERR_SUCCESS De-initialization successful.
 */
err_code_t bio_hrm_a0_shutdown(void);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __BIO_HRM_A0_H__ */
