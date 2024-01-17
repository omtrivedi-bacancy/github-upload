/******************************************************************************
 * Copyright by ams AG                                                        *
 * All rights are reserved.                                                   *
 *                                                                            *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING      *
 * THE SOFTWARE.                                                              *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS          *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,      *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT           *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,      *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY      *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT        *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
 ******************************************************************************/

#ifndef __GSR_APP_H__
#define __GSR_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file       gsr_app.h
 * \authors    ARIT
 * \copyright  ams
 * \addtogroup gsr_app_group Galvanic Skin Resistance
 *
 * \brief This application provides Galvanic Skin Resistance (GSR) measurements.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "gsr_app_typedefs.h"

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Initializes the GSR app.
 *
 * \retval ::ERR_SUCCESS Initialized successfully.
 */
err_code_t gsr_app_initialize(void);

/*!
 * \brief Configures the GSR app.
 *
 * This function can only be called when the GSR app is initialized and not in a processing session.
 *
 * \param[in] p_config Pointer to the configuration structure, see ::gsr_app_configuration_t. It must not be NULL.
 * \param[in] size     Size of the configuration structure. It must be `sizeof(gsr_app_configuration_t)`.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_POINTER    Invalid pointer to configuration structure.
 * \retval ::ERR_SIZE       Size does not match expected size.
 * \retval ::ERR_PERMISSION Not initialized.
 */
err_code_t gsr_app_configure(const void *p_config, uint8_t size);

/*!
 * \brief Provides the sensor signals and accelerometer sample periods to the GSR app and starts a processing session.
 *
 * This function can only be called when the GSR app is initialized.
 *
 * \param[in] p_signal_sample_periods_us Pointer to an array containing the sample periods of each signal in
 *                                       microseconds. There must be ::GSR_APP_SIGNAL_NUM items in the array,
 *                                       ordered according to ::bio_hrm_a0_signal. This value must not be NULL. The
 *                                       sample period pointed to must not be zero.
 * \param[in] signal_num                 The number of sensor signals provided to the GSR app. It must be
 *                                       ::GSR_APP_SIGNAL_NUM.
 * \param[in] num_dac_samples            Describes how much DAC samples of value 0 and value 1 will alternate.
 *
 * \htmlonly
 *     p_signal_sample_
 *     _periods_us
 *          │
 *          └─►┌──────────┐   ┌──►┌───────────────────┐
 *         ECG │          ├───┘   │ ECG Sample Period │
 *             └──────────┘       └───────────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid sample periods or invalid signal count provided or num_dac_samples is out of range.
 * \retval ::ERR_POINTER    p_signal_sample_periods_us is NULL.
 * \retval ::ERR_PERMISSION Not initialized.
 */
err_code_t gsr_app_start(const uint32_t *p_signal_sample_periods_us, uint8_t signal_num, uint8_t num_dac_samples);

/*!
 * \brief Provides measurement data to the GSR app.
 *
 * This function can only be called when the GSR app is initialized and in a processing session.
 *
 * \param[in] signal_samples_type The data type of sensor signals. It must be ::GSR_APP_SIGNAL_DATA_TYPE.
 * \param[in] signal_num          The number of sensor signals provided to the GSR app. It must be
 *                                ::GSR_APP_SIGNAL_NUM.
 * \param[in] p_signal_samples    Pointer to an array of ::bio_signal_samples_t. There must be ::GSR_APP_SIGNAL_NUM
 *                                items in the array, ordered according to ::gsr_app_signal.
 * \param[in] pp_agc_statuses     Pointer to an array containing pointers to the AGC status of a signal. The items of
 *                                the pointer array are ordered according to ::gsr_app_signal. If no AGC status is
 *                                available for a channel, the corresponding pointer must point to NULL. The number of
 *                                items in the pointer array must be ::GSR_APP_SIGNAL_NUM. This argument is ignored
 *                                by this app as it does not use AGC status information. This value is ignored by this
 *  	                          GSR app.
 * \param[in] p_acc_samples       Pointer to an array containing accelerometer samples. This value must not be NULL
 *                                if acc_sample_num is greater than zero. This value is ignored by this GSR app as
 *                                it does not use accelerometer data.
 * \param[in] acc_sample_num      The number of accelerometer samples contained in the array pointed to by
 *                                p_acc_samples. This value is ignored by this GSR app as it does not use accelerometer
 *                                data.
 * \param[out] p_result           The value pointed to by this argument is updated with information whether the GSR app
 *                                is ready for execution. This value must not be NULL.
 *
 * \htmlonly
 *     p_signal_                                  pp_agc_
 *     samples                                    statuses
 *        │               p_u16                      │
 *        └─►┌──────────┐   ┌──►┌──────────────┐     └─►┌──────────┐   ┌──►┌────────────┐
 *           │   data ──────┘   │ ECG Sample 0 │    ECG │          ├───┘   │ ECG        │
 *       ECG │ ──────── │       ├──────────────┤        └──────────┘       │ AGC Status │
 *           │ count: 2 │       │ ECG Sample 1 │                           └────────────┘
 *           └──────────┘       └──────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Data accepted.
 * \retval ::ERR_ARGUMENT   Invalid signal sample type or invalid signal count provided.
 * \retval ::ERR_POINTER    At least one pointer argument is NULL which must not be NULL.
 * \retval ::ERR_OVERFLOW   Cannot store the provided data due to exhausted internal buffers.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t gsr_app_set_input(bio_signal_samples_type_t signal_samples_type,
                             const bio_signal_samples_t *p_signal_samples,
                             const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                             const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                             bio_execution_status_t *p_result);

/*!
 * \brief Processes the data previously provided to the GSR app.
 *
 * This function can be called once after ::gsr_app_set_input indicated via the p_result argument that the GSR app
 * is executable. Afterwards, this function can be called again when a subsequent call of ::gsr_app_set_input
 * indicated that the GSR app is executable. The GSR app must be initialized and in a processing session when calling
 * this function.
 *
 * \param[out] p_result The value pointed to by this argument is updated with information whether the GSR app has data
 *                      available for output. This value must not be NULL.
 *
 * \retval ::ERR_SUCCESS    Processing successful.
 * \retval ::ERR_POINTER    p_result is NULL.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t gsr_app_execute(bio_output_status_t *p_result);

/*!
 * \brief Copies output data generated by the app to a buffer provided by the caller.
 *
 * This function can be called once after ::gsr_app_execute indicated via the p_result argument that output data is
 * available. Afterwards, this function can be called again when a subsequent call of ::gsr_app_execute indicdated
 * that output data is available. The GSR app must be initialized and in a processing session when calling this
 * function.
 *
 * The size of the output data is sizeof(gsr_app_output_t).
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
err_code_t gsr_app_get_output(void *p_dest, uint16_t *p_size);

/*!
 * \brief Stops the current processing session of the GSR app, allowing it to be reconfigured.
 *
 * This function can only be called when the GSR app is initialized and in a processing session.
 *
 * \retval ::ERR_SUCCESS    Stopping successful.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t gsr_app_stop(void);

/*!
 * \brief De-initializes the GSR app.
 *
 * \retval ::ERR_SUCCESS De-initialization successful.
 */
err_code_t gsr_app_shutdown(void);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __GSR_APP_H__ */
