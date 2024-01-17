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

#ifndef __AS7050_CHIP_LIB_H__
#define __AS7050_CHIP_LIB_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file      as7050_chiplib.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup chiplib_group
 *
 * \brief This is the chip library for ams vital signs chip AS7050.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_agc.h"
#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#if defined POSIX
#define CHIPLIB_DECLDIR
#else
#if defined CHIPLIB_DLL_EXPORT
#define CHIPLIB_DECLDIR __declspec(dllexport)
#elif defined CHIPLIB_DLL_IMPORT
#define CHIPLIB_DECLDIR __declspec(dllimport)
#else
#define CHIPLIB_DECLDIR
#endif
#endif

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/
extern err_code_t time_ms_tick(uint32_t *p_tick_ms);
/*!
 * \brief Initializes the library and the device.
 *
 * Following tasks will be done here:
 * - Initialize hardware abstraction layer.
 * - Reset chip.
 *
 * \note This function must be called at first, otherwise all other functions return with error code.
 *
 * \param[in] p_callback            Pointer to the callback function, see as7050_callback_t
 * \param[in] p_cb_param            Optional pointer to an application parameter, which will be transmitted with every
 *                                  callback.
 * \param[in] p_interface_descr     Chiplib forwards this interface description to ::as7050_osal_initialize.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_ARGUMENT           An argument is invalid.
 * \retval ::ERR_IDENTIFICATION     The specified sensor was not found.
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor.
 */
err_code_t CHIPLIB_DECLDIR as7050_initialize(const as7050_callback_t p_callback, const void *p_cb_param,
                                             const char *p_interface_descr);

/*!
 * \brief Stops all internal actions and power down the device.
 *
 * Following tasks will be done here:
 * - Stops measurement, if running.
 * - Power down the sensor device.
 * - Shutdown the hardware abstraction layer.
 * - Block calling of all other functions, but initialize.
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 * \retval ::ERR_DATA_TRANSFER  Communication error to sensor.
 */
err_code_t CHIPLIB_DECLDIR as7050_shutdown(void);

/*!
 * \brief Write a register group
 *
 * This function configures the sensor directly via register formated structure.
 *
 * \param[in] id                    Identification number of an item, see ::as7050_reg_group_ids.
 * \param[in] p_data                Pointer to the register data, like ::as7050_config_led_t
 * \param[in] size                  Sets the size of register data in bytes.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_ARGUMENT           Register group id is not supported.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_SIZE               Size of the data buffer is wrong
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor.
 */
err_code_t CHIPLIB_DECLDIR as7050_set_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t size);

/*!
 * \brief Read a register group
 *
 * Reads the actual register data into a register group structure.
 *
 * \param[in] id                    Identification number of an item, see ::as7050_reg_group_ids.
 * \param[out] p_data               Pointer, where the data of the register group can be saved.
 * \param[inout] p_size             IN: Maximum buffer size, OUT: Size in byte of the register group data
 *                                  The maximum size is defined in ::AS7050_MAX_GROUP_SIZE
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_ARGUMENT           Register group id is not supported.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_SIZE               Size of the data buffer is wrong
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor
 */
err_code_t CHIPLIB_DECLDIR as7050_get_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t *p_size);

/*!
 * \brief Write register
 *
 * This function sets the value of a single I2C-register.
 *
 * \param[in] reg_addr              Register address of the sensor
 * \param[in] reg_val               New register value, which shall be written to the register address
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor
 */
err_code_t CHIPLIB_DECLDIR as7050_write_register(uint8_t reg_addr, uint8_t reg_val);

/*!
 * \brief Read register
 *
 * This function gets the value of a single I2C-register.
 *
 * \param[in] reg_addr              Register address of the sensor
 * \param[out] p_reg_val            Actual register value, which was read from the register address
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor
 */
err_code_t CHIPLIB_DECLDIR as7050_read_register(uint8_t reg_addr, uint8_t *p_reg_val);

/*!
 * \brief Read the actual measurement configuration, which was set by the register groups
 *
 * \note The measurement configuration can be changed after set of new register blocks or writting to single registers.
 * Therefore, read it back after finished configuration to get the actual values.
 *
 * \param[out] p_meas_config   Actual measurement configuration, See ::as7050_meas_config_t
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor
 */
err_code_t CHIPLIB_DECLDIR as7050_get_measurement_config(as7050_meas_config_t *p_meas_config);

/*!
 * \brief Sets the configuration for auto-gain-control (AGC)
 *
 * \param[in] p_agc_config          Pointer to the AGC-configuration structure, see ::as7050_agc_config_t
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 */
err_code_t CHIPLIB_DECLDIR as7050_set_agc_config(const as7050_agc_config_t *p_agc_config);

/*!
 * \brief Gets the configuration for auto-gain-control (AGC)
 *
 * \param[out] p_agc_config         Pointer to the AGC-configuration structure, see ::as7050_agc_config_t
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 */
err_code_t CHIPLIB_DECLDIR as7050_get_agc_config(as7050_agc_config_t *p_agc_config);

/*!
 * \brief Extract the requested samples from FIFO data, which was received by the ::as7050_callback_t
 *
 * \param[in] chan                      Requested channel. See ::as7050_channel_flags_t.
 * \param[in] p_fifo_data               FIFO data
 * \param[in] fifo_data_num             Number of FIFO elements
 * \param[out] p_chan_data              Pointer to buffer, where the channel data can be saved
 * \param[inout] p_chan_data_num        Input: Number of provided chan_data elements,
 *                                      Output: Number of used chan_data elements
 *
 * \retval ::ERR_SUCCESS                Function returns without error.
 * \retval ::ERR_PERMISSION             Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_ARGUMENT               adc_map or req_adc is wrong
 * \retval ::ERR_POINTER                Detected NULL pointer for data
 * \retval ::ERR_CONFIG                 FIFO is not configured
 */
err_code_t CHIPLIB_DECLDIR as7050_extract_samples(as7050_channel_flags_t chan, const uint8_t *p_fifo_data,
                                                  uint16_t fifo_data_num, uint32_t *p_chan_data,
                                                  uint16_t *p_chan_data_num);

/*!
 * \brief Starts a measurement.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor.
 * \retval ::ERR_CONFIG             PPG or ECG are not enabled, AOC and AGC are enabled in parallel
 */
err_code_t CHIPLIB_DECLDIR as7050_start_measurement(void);

/*!
 * \brief Stops a measurement.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked, call ::as7050_initialize at first.
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor.
 */
err_code_t CHIPLIB_DECLDIR as7050_stop_measurement(void);

/*!
 * \brief Requests the version information
 *
 * \param[out] p_version            Pointer to memory, where the version information can be saved. See
 *                                  ::as7050_version_t
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 */
err_code_t CHIPLIB_DECLDIR as7050_get_version(as7050_version_t *p_version);

/*!
 * \brief Calculates the DAC reference value which is used for GSR measurement
 *
 * \attention This function is only needed for GSR measurement and uses the Chip Library in a special mode because
 * multiple measurements with different configurations are performed.
 *
 * Internally, DAC 0 and 1 are measured multiple times and are used to calculate the DAC reference value. For both DAC 0
 * and 1 a separate measurement with a different configuration is performed. Depending on sample rate and the number of
 * samples to be measured per DAC, this calculation can take more than one second. This function does not block while
 * the calculation is in progress, but needs to be called repeatedly until ::DAC_REF_STATUS_RUNNING is no longer
 * returned via the p_status argument.
 *
 * Requirements:
 * * ECG channel must be activated. PPG channels must be disabled.
 * * GSR registers must be configured.
 *
 * \param[in]  control     Control byte. See ::dac_ref_controls
 * \param[in]  average_cnt Minimum number of measured samples per DAC
 * \param[out] p_status    Pointer to variable to which the current calculation status is written. See ::dac_ref_status
 * \param[out] p_dac_ref   Pointer to variable to which the resulting DAC reference value is written.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the library is blocked.
 * \retval ::ERR_ARGUMENT           Provided input parameters are invalid.
 * \retval ::ERR_POINTER            NULL pointer detected.
 * \retval ::ERR_DATA_TRANSFER      Communication error to sensor.
 */
err_code_t CHIPLIB_DECLDIR as7050_calculate_dac_reference_value(dac_ref_control_t control, uint16_t average_cnt,
                                                                dac_ref_status_t *p_status, uint32_t *p_dac_ref);

#ifdef __cplusplus
}
#endif // __cplusplus

/*! @} */

#endif /* __AS7050_CHIP_LIB_H__ */
