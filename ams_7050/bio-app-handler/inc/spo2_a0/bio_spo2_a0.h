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

#ifndef __BIO_SPO2_A0_H__
#define __BIO_SPO2_A0_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file       bio_spo2_a0.h
 * \authors    ARIT, PKRN
 * \copyright  ams
 * \addtogroup bio_spo2_a0_group
 *
 * \brief This module provides an interface to the ams SpO2 algorithm which conforms to the Generic Bio App Handler
 *        Interface Specification.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "bio_spo2_a0_typedefs.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#ifndef SPO2_APP_INPUT_BUF_NUM
/*!
 * \brief The internal buffer is sized to support a sampling frequency of 200 Hz, assuming that ::bio_spo2_a0_execute is
 *        called at least every second when the bio app is executable.
 */
#define SPO2_APP_INPUT_BUF_NUM 200
#endif

/*! Calibration values for SpO2 algorithm correction.
    a,b,c are coefficients of quadratic function used to correct SpO2 algorithm results */
typedef struct {
    uint16_t a;         /*!< quadratic correction coefficient 1 */
    uint16_t b;         /*!< quadratic correction coefficient 2 */
    uint16_t c;         /*!< quadratic correction coefficient 3 */
    uint16_t dcCompRed; /*!< Red LED compensation */
    uint16_t dcCompIR;  /*!< IR LED compensation */
} spo2_calibration_t;

/*! Current state changes list */
typedef enum {
    SPO2_APP_AGC_STATE_UNCHANGED = 0, /*!< AGC was not changed */
    SPO2_APP_AGC_STATE_INCREASED = 1, /*!< AGC was increased */
    SPO2_APP_AGC_STATE_DECREASED = 2, /*!< AGC was decreased */
    SPO2_APP_AGC_STATE_UNKNOWN = 3    /*!< AGC state is unknown */
} spo2_app_agc_states_t;

/*! Output data of SPO2 algorithm */
typedef struct {
    uint8_t status;        /*!< 0 for valid SpO2, 1 for no result */
    uint8_t signalQuality; /*!< quality of the signal/result */
    uint16_t spo2;         /*!< SpO2 percent * 100 [9812==>98.12%] */
    uint16_t heartrate;    /*!< heartrate bpm * 10 [815==>81.5bpm] */
    uint16_t pi;           /*!< PI=Perfusion-Index * 100 [123==>1.23%] */
    uint16_t averageR;     /*!< average-R value * 10000 [4376==>0.4376] */
    uint16_t acRed;        /*!< AC for Red-LED */
    uint16_t dcRed;        /*!< DC for Red-LED */
    uint16_t acInfrared;   /*!< AC for Infrared-LED */
    uint16_t dcInfrared;   /*!< DC for Infrared-LED */
} spo2_app_result_t;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Initializes the bio app.
 *
 * \retval ::ERR_SUCCESS Initialized successfully.
 */
err_code_t bio_spo2_a0_initialize(void);

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
err_code_t bio_spo2_a0_configure(const void *p_config, uint8_t size);

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
 * \htmlonly
 *     p_signal_sample_
 *     _periods_us
 *          │
 *          └─►┌──────────┐   ┌──►┌───────────────────────┐
 *     PPG Red │          ├───┘   │ PPG Red Sample Period │
 *             ├──────────┤       └───────────────────────┘
 *      PPG IR │          ├──────►┌───────────────────────┐
 *             ├──────────┤       │ PPG IR Sample Period  │
 *     Ambient │          ├───┐   └───────────────────────┘
 *             └──────────┘   └──►┌───────────────────────┐
 *                                │ Ambient Sample Period │
 *                                └───────────────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Updated successfully.
 * \retval ::ERR_ARGUMENT   Invalid sample periods or invalid signal count provided.
 * \retval ::ERR_POINTER    p_signal_sample_periods_us is NULL.
 * \retval ::ERR_PERMISSION Not initialized.
 */
err_code_t bio_spo2_a0_start(const uint32_t *p_signal_sample_periods_us, uint8_t signal_num,
                             uint32_t acc_sample_period_us);

/*!
 * \brief Provides measurement data to the bio app.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * \param[in] signal_samples_type The data type of sensor signals. It must be ::BIO_SPO2_A0_SIGNAL_DATA_TYPE.
 * \param[in] signal_num          The number of sensor signals provided to the bio app. It must be
 *                                ::BIO_SPO2_A0_SIGNAL_NUM.
 * \param[in] p_signal_samples    Pointer to an array of ::bio_signal_samples_t. There must be ::BIO_SPO2_A0_SIGNAL_NUM
 *                                items in the array, ordered according to ::bio_spo2_a0_signal. For each signal, the
 *                                same number of samples needs to be provided.
 * \param[in] pp_agc_statuses     Pointer to an array containing pointers to the AGC status of a signal. The items of
 *                                the pointer array are ordered according to ::bio_spo2_a0_signal. If no AGC status is
 *                                available for a channel, the corresponding pointer must point to NULL. The number of
 *                                items in the pointer array must be ::BIO_SPO2_A0_SIGNAL_NUM.
 * \param[in] p_acc_samples       Pointer to an array containing accelerometer samples. This value is ignored by this
 *                                bio app as it does not use accelerometer data.
 * \param[in] acc_sample_num      The number of accelerometer samples contained in the array pointed to by
 *                                p_acc_samples. This value is ignored by this bio app as it does not use accelerometer
 *                                data.
 * \param[out] p_result           The value pointed to by this argument is updated with information whether the bio app
 *                                is ready for execution. This value must not be NULL.
 *
 * \htmlonly
 *       p_signal_                                          pp_agc_
 *       samples                                            statuses
 *          │               p_u16                              │
 *          └─►┌──────────┐   ┌──►┌──────────────────┐         └─►┌──────────┐   ┌──►┌────────────┐
 *             │   data ──────┘   │ PPG Red Sample 0 │    PPG Red │          ├───┘   │ PPG Red    │
 *     PPG Red │ ──────── │       ├──────────────────┤            ├──────────┤       │ AGC Status │
 *             │ count: 2 │       │ PPG Red Sample 1 │     PPG IR │          ├───┐   └────────────┘
 *             ├──────────┤ p_u16 └──────────────────┘            ├──────────┤   └──►┌────────────┐
 *             │   data ─────────►┌──────────────────┐    Ambient │   NULL   │       │ PPG IR     │
 *      PPG IR │ ──────── │       │ PPG IR Sample 0  │            └──────────┘       │ AGC Status │
 *             │ count: 2 │       ├──────────────────┤                               └────────────┘
 *             ├──────────┤ p_u16 │ PPG IR Sample 1  │
 *             │   data ──────┐   └──────────────────┘
 *     Ambient │ ──────── │   └──►┌──────────────────┐
 *             │ count: 2 │       │ Ambient Sample 0 │
 *             └──────────┘       ├──────────────────┤
 *                                │ Ambient Sample 1 │
 *                                └──────────────────┘
 * \endhtmlonly
 *
 * \retval ::ERR_SUCCESS    Data accepted.
 * \retval ::ERR_ARGUMENT   Invalid signal sample type or invalid signal count provided.
 * \retval ::ERR_POINTER    At least one pointer argument is NULL which must not be NULL.
 * \retval ::ERR_OVERFLOW   Cannot store the provided data due to exhausted internal buffers.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_spo2_a0_set_input(bio_signal_samples_type_t signal_samples_type,
                                 const bio_signal_samples_t *p_signal_samples,
                                 const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                                 const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                                 bio_execution_status_t *p_result);

/*!
 * \brief Processes the data previously provided to the bio app.
 *
 * This function can be called once after ::bio_spo2_a0_set_input indicated via the p_result argument that the bio app
 * is executable. Afterwards, this function can be called again when a subsequent call of ::bio_spo2_a0_set_input
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
err_code_t bio_spo2_a0_execute(bio_output_status_t *p_result);

/*!
 * \brief Copies output data generated by the app to a buffer provided by the caller.
 *
 * This function can be called once after ::bio_spo2_a0_execute indicated via the p_result argument that output data is
 * available. Afterwards, this function can be called again when a subsequent call of ::bio_spo2_a0_execute indicdated
 * that output data is available. The bio app must be initialized and in a processing session when calling this
 * function.
 *
 * The size of the output data is sizeof(bio_spo2_a0_output_t).
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
err_code_t bio_spo2_a0_get_output(void *p_dest, uint16_t *p_size);

/*!
 * \brief Stops the current processing session of the bio app, allowing it to be reconfigured.
 *
 * This function can only be called when the bio app is initialized and in a processing session.
 *
 * \retval ::ERR_SUCCESS    Stopping successful.
 * \retval ::ERR_PERMISSION Not initialized or not in a processing session.
 */
err_code_t bio_spo2_a0_stop(void);

/*!
 * \brief De-initializes the bio app.
 *
 * \retval ::ERR_SUCCESS De-initialization successful.
 */
err_code_t bio_spo2_a0_shutdown(void);


/*!
 * \brief Commits the cyclic measurement data to the SPO2 module
 *
 * \param[in] p_ambient_light_samples   Measured ambient light data
 * \param[in] p_ir_samples              Measured ADC channel data
 * \param[in] p_red_samples             Measured ADC channel data
 * \param[in] num_samples               Number of the measured samples
 * \param[in] agc_state                 Indicates if LED current and/or PD offset was touched by AGC.
 *                                      See ::spo2_app_agc_states_t
 * \param[in] pd_on_offset_current      Current for PD offset
 * \param[out] p_ready_for_execution    Returns TRUE if enough data are available for calculation of new SPO2 data
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the function is blocked, library must be initialized and configured first.
 * \retval ::ERR_ARGUMENT           Parameter channel_data_num is zero.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_OVERFLOW           Too much data. Data must be calculated at first (see: ::spo2_app_calculate)!
 * \retval ::ERR_CONFIG             Configuration is missing. Call ::spo2_app_configure_sample_period and
 *                                  ::spo2_app_prepare_calculation before.
 */
err_code_t spo2_app_append_input_data(uint32_t *p_ambient_light_samples, uint32_t *p_ir_samples,
                                      uint32_t *p_red_samples, uint16_t num_samples, spo2_app_agc_states_t agc_state,
                                      uint8_t pd_on_offset_current, uint8_t *p_ready_for_execution);

/*!
 * \brief Readout the calculated SPO2 data
 *
 * \param[out] p_result             Pointer, where the calculated SPO2 results will be saved
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Access to the function is blocked, library must be initialized and configured first.
 * \retval ::ERR_POINTER            Detected NULL pointer for data
 * \retval ::ERR_NO_DATA            No calculated data available
 * \retval ::ERR_CONFIG             Configuration is missing. Call ::spo2_app_configure_sample_period and
 *                                  ::spo2_app_prepare_calculation before.
 */
err_code_t spo2_app_get_output_data(spo2_app_result_t *p_result);

#ifdef __cplusplus
}
#endif // __cplusplus

/*! @} */

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __BIO_SPO2_A0_H__ */
