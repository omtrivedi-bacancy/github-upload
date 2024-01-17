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

#ifndef __AS7050_APP_MANAGER_H__
#define __AS7050_APP_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file       as7050_app_manager.h
 * \authors    PKRN
 * \copyright  ams
 * \addtogroup AS7050_APPMGR Application Manager
 *
 * \brief The Application Manager routes measurement data to Vital Signs Applications and provides a unified interface
 * for application configuration and obtaining resulting output data.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#include "as7050_app_manager_typedefs.h"
#include "bio_common.h"
#include "error_codes.h"

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Initializes the Application Manager.
 *
 * The Application Manager transitions to Configuration state after initialization.
 *
 * \retval ::ERR_SUCCESS    Initialized successfully.
 */
err_code_t as7050_appmgr_initialize(void);

/*!
 * \brief Sets the enabled Vital Signs Applications.
 *
 * This function can only be called when the Application Manager is in Configuration state.
 *
 * \param[in] enabled_apps Flags of enabled Vital Signs Applications, see \ref AS7050_APPMGR_APP_FLAG. A Vital Signs
 *                         Application is enabled when the corresponding bit is set and disabled when the bit is not
 *                         set. At least one app must be enabled. ::AS7050_APPMGR_APP_FLAG_HRM_A0 and
 *                         ::AS7050_APPMGR_APP_FLAG_SPO2_A0 cannot be enabled at the same time.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid selection of application.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_enable_apps(uint32_t enabled_apps);

/*!
 * \brief Sets the signal routing for a Vital Signs Application.
 *
 * Each Vital Signs Application has an ordered list of signals assigned, which can be found in \ref AS7050_APPMGR_APPS.
 * When ::as7050_appmgr_set_input is called, the Application Manager extracts the samples of each signal of each enabled
 * Vital Signs Application. In order to extract samples from the provided sensor FIFO data, the Application Manager
 * requires a mapping between sensor channels and Vital Signs Application signals. This mapping is provided to the
 * Application Manager for each Vital Signs Application using this function.
 *
 * No signal routing is required for ::AS7050_APPMGR_APP_ID_RAW.
 *
 * This function can only be called when the Application Manager is in Configuration state.
 *
 * \param[in] app          Identifier of the Vital Signs Application for which the signal routing shall be set.
 * \param[in] p_channels   Pointer to the start of an array containing the channel identifiers used for each signal of
 *                         the given Vital Signs Application, in the order specified in the subsections of \ref
 *                         AS7050_APPMGR_APPS. Can be NULL if channels_num is zero.
 * \param[in] channels_num Number of items contained in the p_channels array. This number must be equal to the signal
 *                         count of the given Vital Signs Application, which can be found in the subsections of \ref
 *                         AS7050_APPMGR_APPS.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid application identifier or invalid channel identifiers.
 * \retval ::ERR_SIZE       Mismatching number of channels.
 * \retval ::ERR_POINTER    Invalid pointer argument value.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_set_signal_routing(as7050_appmgr_app_id_t app, const as7050_appmgr_channel_id_t *p_channels,
                                            uint8_t channels_num);

/*!
 * \brief Configures a Vital Signs Application.
 *
 * This function can only be called when the Application Manager is in Configuration state.
 *
 * \param[in] app      Identifier of the Vital Signs Application to configure.
 * \param[in] p_config Pointer to the configuration structure for the given Vital Signs Application, which can found in
 *                     the subsections of \ref AS7050_APPMGR_APPS.
 * \param[in] size     Size of the configuration structure.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid application identifier or mismatching configuration structure size.
 * \retval ::ERR_POINTER    Invalid pointer argument value.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_configure_app(as7050_appmgr_app_id_t app, const void *p_config, uint8_t size);

/*!
 * \brief Starts processing.
 *
 * This function can only be called when the Application Manager is in Configuration state. The Application Manager
 * transitions to Processing state when this function executes successfully.
 *
 * \param[in] measurement_config   Measurement configuration used to acquire the data that will be provided to the
 *                                 Application Manager. This value is typically obtained from the Chip Library.
 * \param[in] acc_sample_period_us Sample period of the accelerometer in microseconds.
 * \param[in] p_agc_mappings       Pointer to the start of an array containing the identifiers of the channels
 *                                 controlled by Automatic Gain Control (AGC). When providing AGC status information via
 *                                 ::as7050_appmgr_set_input, the status information items must be ordered identically.
 *                                 For example, if p_agc_mappings[0] contains the identifer of the first channel, then
 *                                 the AGC status information for the first channel must be provided to
 *                                 ::as7050_appmgr_set_input via the first item of the AGC status information array.
 *                                 Can be NULL if agc_mappings_num is zero.
 * \param[in] agc_mappings_num     Number of items in the p_agc_mappings array.
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_CONFIG     Measurement configuration incompatible with application configurations.
 * \retval ::ERR_ARGUMENT   Invalid accelerometer sample period.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_start_processing(as7050_meas_config_t measurement_config, uint32_t acc_sample_period_us,
                                          const as7050_appmgr_channel_id_t *p_agc_mappings, uint8_t agc_mappings_num);

/*!
 * \brief Provides measurement data to the Application Manager.
 *
 * This function can only be called when the Application Manager is in Processing state.
 *
 * \param[in]  p_fifo_data           Pointer to the start of the sensor FIFO data. This data is typically obtained from
 *                                   the Chip Library.
 * \param[in]  fifo_data_size        Size of the FIFO data.
 * \param[in]  chip_status           Chip status. This value is typically obtained from the Chip Library.
 * \param[in]  p_agc_statuses        Pointer to the start of an array containing Automatic Gain Control (AGC) status
 *                                   information for each channel with enabled AGC. The order of the items must be
 *                                   identical to the order of items in the AGC mapping array provided to
 *                                   ::as7050_appmgr_start_processing. Can be NULL if agc_statuses_num is zero.
 * \param[in]  agc_statuses_num      Number of AGC status information items. Must be equivalent to the number of AGC
 *                                   mappings provided to ::as7050_appmgr_start_processing.
 * \param[in]  p_acc_samples         Pointer to the start of an array containing accelerometer samples.
 * \param[in]  acc_samples_num       Number of accelerometer samples.
 * \param[out] p_ready_for_execution Flags of Vital Signs Applications that indicated that they are ready for execution,
 *                                   see \ref AS7050_APPMGR_APP_FLAG. An app is ready for execution when the
 *                                   corresponding bit is set.
 *
 * \retval ::ERR_SUCCESS    Measurement data accepted.
 * \retval ::ERR_SIZE       Too many samples in the FIFO data or invalid number of AGC status information items.
 * \retval ::ERR_OVERFLOW   Signals of an app have sample counts which are too different to be handled.
 * \retval ::ERR_POINTER    Invalid pointer argument value.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_set_input(const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                                   as7050_appmgr_chip_status_t chip_status, const bio_agc_status_t *p_agc_statuses,
                                   uint8_t agc_statuses_num, const vs_acc_data_t *p_acc_samples,
                                   uint16_t acc_samples_num, uint32_t *p_ready_for_execution);

/*!
 * \brief Informs the Application Manager that an external event occurred.
 *
 * The function counts how many times it has been called and provides the count to applications on the next invocation
 * of ::as7050_appmgr_set_input. This function can only be called when the Application Manager is in Processing
 * state.
 *
 * \retval ::ERR_SUCCESS    Information accepted.
 * \retval ::ERR_OVERFLOW   Too many events occurred since last ::as7050_appmgr_set_input call.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_set_ext_event_occurred(void);

/*!
 * \brief Executes enabled Vital Signs Applications.
 *
 * This function can only be called when the Application Manager is in Processing state.
 *
 * \param[out] p_data_available Flags of Vital Signs Applications that indicated that their execution generated output
 *                              data, see \ref AS7050_APPMGR_APP_FLAG. An app has output data available when the
 *                              corresponding bit is set.
 *
 * \retval ::ERR_SUCCESS    Execution successful.
 * \retval ::ERR_POINTER    Invalid pointer argument value.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_execute(uint32_t *p_data_available);

/*!
 * \brief Writes output of a Vital Signs Application to a buffer provided by the caller.
 *
 * This function can only be called when the Application Manager is in Processing state.
 *
 * \param[in]    app    Identifier of the Vital Signs Application to get output from.
 * \param[out]   p_dest Pointer to the buffer where the output shall be written to. The output is application-specific
 *                      and is described in the subsections of \ref AS7050_APPMGR_APPS.
 * \param[inout] p_size Pointer to the amount of memory allocated for the output data. The function updates the value
 *                      pointed to with the actual size of the written output data.
 *
 * \retval ::ERR_SUCCESS    Output data write successful.
 * \retval ::ERR_ARGUMENT   Invalid app identifier or disabled application.
 * \retval ::ERR_POINTER    Invalid pointer argument value.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_get_output(as7050_appmgr_app_id_t app, void *p_dest, uint16_t *p_size);

/*!
 * \brief Stops processing.
 *
 * This function can only be called when the Application Manager is not in Uninitialized state.
 *
 * \retval ::ERR_SUCCESS    Stop successful.
 * \retval ::ERR_PERMISSION Invalid state.
 */
err_code_t as7050_appmgr_stop_processing(void);

/*!
 * \brief De-initializes the Application Manager.
 *
 * \retval ::ERR_SUCCESS De-initialization successful.
 */
err_code_t as7050_appmgr_shutdown(void);

/*!
 * \brief Gets the version of the Application Manager.
 *
 * \param[out] p_version Pointer to the location where the version shall be written to.
 *
 * \retval ::ERR_SUCCESS Version write successful.
 * \retval ::ERR_POINTER Invalid pointer argument value.
 */
err_code_t as7050_appmgr_get_version(as7050_appmgr_version_t *p_version);

#ifdef __cplusplus
}
#endif // __cplusplus

/*! @} */

#endif /* __AS7050_APP_MANAGER_H__ */
