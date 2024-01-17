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

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_app_manager.h"
#include "as7050_app_manager_version.h"
#include "as7050_extract.h"
#include "bio_hrm_a0.h"
#include "bio_spo2_a0.h"
#include "gsr_app.h"
#include "raw_app.h"
#include "std_inc.h"
#include "nrf_log.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*!
 * \brief Maximum total number of samples that can be provided to a bio app via a single call of
 *        ::as7050_appmgr_set_input.
 *
 * This is the total count of samples across all signals used by a bio app. If the channels mapped to the signals used
 * by the app contain more samples, the ::as7050_appmgr_set_input call fails.
 */
#define AS7050_APPMGR_MAX_APP_SAMPLE_NUM (64 * 3)

/*! Maximum number of signals that can be used by a bio app. */
#define AS7050_APPMGR_MAX_APP_SIGNAL_NUM 3

/*! Indicates that AGC is not enabled for a given channel. */
#define AS7050_APPMGR_AGC_MAPPING_DISABLED UINT8_MAX

/*! Lower bound of the chip-specific range of valid channel identifiers. */
#define AS7050_APPMGR_CHANNEL_ID_FIRST AS7050_CHANNEL_PPG_1

/*! Upper bound of the chip-specific range of valid channel identifiers. */
#define AS7050_APPMGR_CHANNEL_ID_LAST AS7050_CHANNEL_TIA

/*! Chip-specific size of an acquired sample in bits. */
#define AS7050_APPMGR_SAMPLE_BIT_SIZE 20

/*!
 * \brief The identifier of the first app whose functions are called entirely via the ::g_app_table. There is special
 *        handling for ::AS7050_APPMGR_APP_ID_RAW, which is located at index 0 in ::g_app_table.
 */
#define AS7050_APPMGR_APP_ID_FIRST_AFTER_RAW AS7050_APPMGR_APP_ID_HRM_A0

/*! States of the Application Manager. */
typedef enum {
    AS7050_APPMGR_STATE_UNINITIALIZED = 0, /*!< Uninitialized state. */
    AS7050_APPMGR_STATE_CONFIGURATION,     /*!< Configuration state. */
    AS7050_APPMGR_STATE_PROCESSING         /*!< Processing state. */
} as7050_appmgr_state_t;

/*! Chip-specific type which is used to store a sample internally in the Application Manager. */
typedef uint32_t as7050_appmgr_sample_t;

/*! Holds a single excess sample of a signal. */
typedef struct {
    uint8_t has_data : 1;                                        /*!< Is non-zero when an excess sample is stored. */
    as7050_appmgr_sample_t data : AS7050_APPMGR_SAMPLE_BIT_SIZE; /*!< Sample data storage. */
} as7050_appmgr_excess_sample_t;

/*! Definition of a Vital Signs Application. */
typedef struct {
    err_code_t (*p_initialize)(void);                           /*!< Function called by ::as7050_appmgr_initialize. */
    err_code_t (*p_shutdown)(void);                             /*!< Function called by ::as7050_appmgr_shutdown. */
    err_code_t (*p_configure)(const void *, uint8_t);           /*!< Function called by
                                                                     ::as7050_appmgr_configure_app. */
    err_code_t (*p_start)(const uint32_t *, uint8_t, uint32_t); /*!< Function called by
                                                                     ::as7050_appmgr_start_processing. */
    err_code_t (*p_stop)(void);                                 /*!< Function called by
                                                                     ::as7050_appmgr_stop_processing. */
    err_code_t (*p_set_input)(bio_signal_samples_type_t, const bio_signal_samples_t *, const bio_agc_status_t *const *,
                              uint8_t, const vs_acc_data_t *, uint8_t,
                              bio_execution_status_t *); /*!< Function called by ::as7050_appmgr_set_input. */
    err_code_t (*p_execute)(bio_output_status_t *);      /*!< Function called by ::as7050_appmgr_execute. */
    err_code_t (*p_get_output)(void *, uint16_t *);      /*!< Function called by ::as7050_appmgr_get_output. */
    as7050_appmgr_channel_id_t *p_signal_mapping;        /*!< Pointer to the begin of the array containing the signal
                                                              mapping. The size of the array must be ::signals_num. */
    uint8_t *p_agc_mapping;                              /*!< Pointer to the begin of the array containing the index of
                                                              the AGC status for each channel. This index is used to
                                                              look up the corresponding AGC status in the p_agc_statuses
                                                              array provided to ::as7050_appmgr_set_input. When AGC is
                                                              disabled for a channel, this array contains
                                                              ::AS7050_APPMGR_AGC_MAPPING_DISABLED at the corresponding
                                                              index. The size of this array must be ::signals_num. */
    as7050_appmgr_excess_sample_t *p_excess_samples;     /*!< Pointer to the begin of the array containing the excess
                                                              sample storage. The size of the array must be
                                                              ::signals_num. */
    bio_signal_samples_type_t data_type;                 /*!< Signal data type expected by the app. */
    uint8_t signals_num;                                 /*!< Number of signals used by the app. */
    uint32_t excluded_apps;                              /*!< Flags of apps that cannot be enabled concurrently with
                                                              this app, see \ref AS7050_APPMGR_APP_FLAG.*/
} as7050_appmgr_app_definition_t;

typedef union {
    uint16_t u16[AS7050_APPMGR_MAX_APP_SAMPLE_NUM];
    uint32_t u32[AS7050_APPMGR_MAX_APP_SAMPLE_NUM];
} as7050_appmgr_converted_samples_buffer_t;

/*! Returns whether the corresponding \ref AS7050_APPMGR_APP_FLAG flag of an ::as7050_appmgr_app_id_t is set. */
#define M_APP_FLAG_SET(bitfield, app) ((bitfield)&M_AS7050_APPMGR_APP_FLAG((app)))

/*! Returns whether the app with a given ::as7050_appmgr_app_id_t is enabled. */
#define M_APP_IS_ENABLED(app) M_APP_FLAG_SET(g_enabled_apps, (app))

/*! Checks whether the app with a given ::as7050_appmgr_app_id_t is enabled. */
#define M_CHECK_APP_IS_ENABLED(app)                                                                                    \
    do {                                                                                                               \
        if (!M_APP_IS_ENABLED(app)) {                                                                                  \
            return ERR_ARGUMENT;                                                                                       \
        }                                                                                                              \
    } while (0)

/*! Checks whether an ::as7050_appmgr_app_id_t is valid. */
#define M_CHECK_APP_ID(app) M_CHECK_ARGUMENT_LOWER((app), AS7050_APPMGR_APP_ID_NUM)

/*! Checks whether the Application Manager is in the given state. */
#define M_CHECK_STATE(expected)                                                                                        \
    do {                                                                                                               \
        if (g_state != (expected)) {                                                                                   \
            return ERR_PERMISSION;                                                                                     \
        }                                                                                                              \
    } while (0)

/*! Checks whether the Application Manager is not in the given state. */
#define M_CHECK_STATE_NOT(expected)                                                                                    \
    do {                                                                                                               \
        if ((expected) == g_state) {                                                                                   \
            return ERR_PERMISSION;                                                                                     \
        }                                                                                                              \
    } while (0)

/*! Checks whether a value evaluates to true and returns ::ERR_CONFIG otherwise. */
#define M_CHECK_IS_TRUE_CONFIG(value)                                                                                  \
    do {                                                                                                               \
        if (!(value)) {                                                                                                \
            return ERR_CONFIG;                                                                                         \
        }                                                                                                              \
    } while (0)

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

/*! State of the Application Manager. */
static volatile as7050_appmgr_state_t g_state = AS7050_APPMGR_STATE_UNINITIALIZED;

/*! Bitfield storing which apps are enabled. (see \ref AS7050_APPMGR_APP_FLAG) */
static uint32_t g_enabled_apps;

/*! Signal mapping configuration for the HRM app. */
static as7050_appmgr_channel_id_t g_signal_mapping_hrm[BIO_HRM_A0_SIGNAL_NUM];

/*! Signal mapping configuration for the SpO2 app. */
static as7050_appmgr_channel_id_t g_signal_mapping_spo2[BIO_SPO2_A0_SIGNAL_NUM];

/*! Signal mapping configuration for the GSR app. */
static as7050_appmgr_channel_id_t g_signal_mapping_gsr[GSR_APP_SIGNAL_NUM];

/*! AGC mapping for the HRM app. */
static uint8_t g_agc_mapping_hrm[BIO_HRM_A0_SIGNAL_NUM];

/*! AGC mapping for the SpO2 app. */
static uint8_t g_agc_mapping_spo2[BIO_SPO2_A0_SIGNAL_NUM];

/*! Excess sample storage for the SpO2 app. */
static as7050_appmgr_excess_sample_t g_excess_samples_spo2[BIO_SPO2_A0_SIGNAL_NUM];

/*! Bitfield storing which apps are ready for execution. (see \ref AS7050_APPMGR_APP_FLAG) */
static volatile uint32_t g_executable_apps;

/*! Number of external events that need to be provided to the applications. */
static volatile uint8_t g_pending_ext_event_count;

/*! Definition of the support Vital Signs Applications. */
static const as7050_appmgr_app_definition_t g_app_table[AS7050_APPMGR_APP_ID_NUM] = {
    [AS7050_APPMGR_APP_ID_RAW] =
        {
            .p_initialize = raw_app_initialize,
            .p_shutdown = raw_app_shutdown,
            .p_configure = raw_app_configure,
            .p_start = NULL,
            .p_stop = raw_app_stop,
            .p_set_input = NULL,
            .p_execute = raw_app_execute,
            .p_get_output = raw_app_get_output,
            .p_signal_mapping = NULL,
            .p_agc_mapping = NULL,
            .p_excess_samples = NULL,
            .data_type = BIO_SIGNAL_SAMPLES_TYPE_U16,
            .signals_num = 0,//2
            .excluded_apps = 0,
        },
    [AS7050_APPMGR_APP_ID_HRM_A0] =
        {
            .p_initialize = bio_hrm_a0_initialize,
            .p_shutdown = bio_hrm_a0_shutdown,
            .p_configure = bio_hrm_a0_configure,
            .p_start = bio_hrm_a0_start,
            .p_stop = bio_hrm_a0_stop,
            .p_set_input = bio_hrm_a0_set_input,
            .p_execute = bio_hrm_a0_execute,
            .p_get_output = bio_hrm_a0_get_output,
            .p_signal_mapping = g_signal_mapping_hrm,
            .p_agc_mapping = g_agc_mapping_hrm,
            .p_excess_samples = NULL,
            .data_type = BIO_SIGNAL_SAMPLES_TYPE_U16,
            .signals_num = BIO_HRM_A0_SIGNAL_NUM,
            .excluded_apps = AS7050_APPMGR_APP_FLAG_SPO2_A0,
        },
    [AS7050_APPMGR_APP_ID_SPO2_A0] =
        {
            .p_initialize = bio_spo2_a0_initialize,
            .p_shutdown = bio_spo2_a0_shutdown,
            .p_configure = bio_spo2_a0_configure,
            .p_start = bio_spo2_a0_start,
            .p_stop = NULL,
            .p_set_input = bio_spo2_a0_set_input,
            .p_execute = bio_spo2_a0_execute,
            .p_get_output = bio_spo2_a0_get_output,
            .p_signal_mapping = g_signal_mapping_spo2,
            .p_agc_mapping = g_agc_mapping_spo2,
            .p_excess_samples = g_excess_samples_spo2,
            .data_type = BIO_SIGNAL_SAMPLES_TYPE_U16,
            .signals_num = BIO_SPO2_A0_SIGNAL_NUM,
            .excluded_apps = AS7050_APPMGR_APP_FLAG_HRM_A0,
        },
      //[AS7050_APPMGR_APP_ID_SPO2_A0] =
      //  {
      //      .p_initialize = bio_spo2_a0_initialize,
      //      .p_shutdown = bio_spo2_a0_shutdown,
      //      .p_configure = (void *)spo2_app_configure,
      //      .p_start =NULL,
      //      .p_stop = NULL,
      //      .p_set_input = (void *)spo2_app_append_input_data,
      //      .p_execute = spo2_app_execute,
      //      .p_get_output = (void *)spo2_app_get_output_data,
      //      .p_signal_mapping = g_signal_mapping_spo2,
      //      .p_agc_mapping = g_agc_mapping_spo2,
      //      .p_excess_samples = g_excess_samples_spo2,
      //      .data_type = BIO_SIGNAL_SAMPLES_TYPE_U16,
      //      .signals_num = BIO_SPO2_A0_SIGNAL_NUM,
      //      .excluded_apps = AS7050_APPMGR_APP_FLAG_HRM_A0,
      //  },
    [AS7050_APPMGR_APP_ID_GSR] =
        {
            .p_initialize = gsr_app_initialize,
            .p_shutdown = gsr_app_shutdown,
            .p_configure = gsr_app_configure,
            .p_start = NULL,
            .p_stop = gsr_app_stop,
            .p_set_input = gsr_app_set_input,
            .p_execute = gsr_app_execute,
            .p_get_output = gsr_app_get_output,
            .p_signal_mapping = g_signal_mapping_gsr,
            .p_agc_mapping = NULL,
            .p_excess_samples = NULL,
            .data_type = BIO_SIGNAL_SAMPLES_TYPE_U32,
            .signals_num = GSR_APP_SIGNAL_NUM,
            .excluded_apps = 0,
        },
};

/*! Chip-specific measurement configuration. */
static as7050_meas_config_t g_measurement_config;

/******************************************************************************
 *                              LOCAL FUNCTIONS                               *
 ******************************************************************************/

/*!
 * \brief Chip-specific function called when the Application Manager enters state ::AS7050_APPMGR_STATE_PROCESSING. It
 *        shall store required information from the measurement configuration which is needed by other chip-specific
 *        functions that are called while the Application Manager is entering state ::AS7050_APPMGR_STATE_PROCESSING or
 *        is in state ::AS7050_APPMGR_STATE_PROCESSING.
 *
 * This function is called before any other chip-specific function while the Application Manager is entering state
 * ::AS7050_APPMGR_STATE_PROCESSING or is in state ::AS7050_APPMGR_STATE_PROCESSING.
 */
static err_code_t process_measurement_config(as7050_meas_config_t measurement_config)
{
    memcpy(&g_measurement_config, &measurement_config, sizeof(g_measurement_config));

    return ERR_SUCCESS;
}

/*!
 * \brief Chip-specific function called when the Application Manager leaves ::AS7050_APPMGR_STATE_PROCESSING.
 */
static err_code_t clear_measurement_config(void)
{
    memset(&g_measurement_config, 0, sizeof(g_measurement_config));

    return ERR_SUCCESS;
}

/*!
 * \brief Chip-specific function called when the Application Manager enters state ::AS7050_APPMGR_STATE_PROCESSING. It
 *        shall provide the sample period of the given channel. If the channel is not sampled according to the
 *        measurement configuration provided to ::process_measurement_config, an error code that is not ::ERR_SUCCESS
 *        shall be returned.
 */
static err_code_t get_sample_period_for_channel(as7050_appmgr_channel_id_t id, uint32_t *p_sample_period_us)
{
    as7050_channel_flags_t channel_flag = (as7050_channel_flags_t)(1 << (id - AS7050_CHANNEL_PPG_1));
    if (!(channel_flag & g_measurement_config.fifo_map)) {
        return ERR_ARGUMENT;
    }

    if (id >= AS7050_CHANNEL_PPG_1 && id <= AS7050_CHANNEL_PPG_8) {
        *p_sample_period_us = g_measurement_config.ppg_sample_period_us;
        return ERR_SUCCESS;
    } else if (AS7050_CHANNEL_TIA == id) {
        *p_sample_period_us = g_measurement_config.ecg_sample_period_us;
        return ERR_SUCCESS;
    } else {
        return ERR_ARGUMENT;
    }
}

/*!
 * \brief Chip-specific function called when the raw app is enabled and the Application Manager enters state
 *        ::AS7050_APPMGR_STATE_PROCESSING.
 */
static err_code_t handle_raw_app_start(void)
{
    return raw_app_start(g_measurement_config.ppg_sample_period_us, g_measurement_config.sample_size);
}

/*!
 * \brief Chip-specific function called in state ::AS7050_APPMGR_STATE_PROCESSING to provide data to the chip-specific
 *        raw data app.
 */
static err_code_t handle_raw_app_set_input(const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                                           as7050_appmgr_chip_status_t chip_status,
                                           const bio_agc_status_t *p_agc_statuses, uint16_t agc_statuses_num,
                                           const vs_acc_data_t *p_acc_samples, uint16_t acc_samples_num,
                                           bio_execution_status_t *p_result)
{
    M_UNUSED_PARAM(chip_status);

    uint8_t ext_event_count = g_pending_ext_event_count;
    g_pending_ext_event_count = 0;

    return raw_app_set_input(p_fifo_data, fifo_data_size, p_agc_statuses, agc_statuses_num, p_acc_samples,
                             acc_samples_num, ext_event_count, p_result);
}

/*!
 * \brief Chip-specific function called when the GSR app is enabled and the Application Manager enters state
 *        ::AS7050_APPMGR_STATE_PROCESSING.
 */
static err_code_t handle_gsr_app_start(void)
{
    if (g_app_table[AS7050_APPMGR_APP_ID_GSR].p_signal_mapping[GSR_APP_SIGNAL_ECG] != AS7050_CHANNEL_TIA) {
        return ERR_CONFIG;
    }

    return gsr_app_start(&g_measurement_config.ecg_sample_period_us, GSR_APP_SIGNAL_NUM,
                         (g_measurement_config.fifo_map & AS7050_CHANNEL_FLAG_GSR) >> AS7050_CHANNEL_FLAG_GSR_OFFSET);
}

/*!
 * \brief Chip-specific function called in state ::AS7050_APPMGR_STATE_PROCESSING to extract data of the given channels
 *        from the provided FIFO data.
 *
 * The samples shall be extracted from each channel contained in the array pointed to by p_channels. This array contains
 * channels_num entries. The samples shall not undergo type conversion and shall be placed in the contigious array
 * pointed to by p_sample_buffer. The p_sample_buffer array shall first contain the samples extracted from the first
 * channel entry in the p_channels array, followed by the samples of the second channel entry, and so on. If the
 * p_channels array contains multiple entries with the same channel, the data shall be extracted for each occurrence
 * independently. A total maximum of sample_buffer_max_num samples may be written to the p_sample_buffer array. The
 * array pointed to by pp_signal shall be updated to point to the first sample of each channel entry and past the last
 * sample of the last channel entry, resulting in a total of channels_num + 1 entries. Please refer to the drawing below
 * for more information:
 *
 * Input:
 *                   p_channels
 *                        │
 *               ┌─────── └─►┌───────────┐
 *               │  Signal 0 │ Channel A │
 *       Count = │           ├───────────┤
 *     channels_ │  Signal 1 │ Channel B │
 *           num │           ├───────────┤
 *               │  Signal 2 │ Channel C │
 *               └────────── └───────────┘
 *
 * Output:
 *                                            p_sample_
 *                   pp_samples                buffer
 *                        │                      │
 *               ┌─────── └─►┌───────────┐    ┌─►└─►┌──────────┐ ─┐ ────────────────────────┐
 *               │  Signal 0 │           ├────┘     │ Sample 0 │  │                         │
 *               │           ├───────────┤          ├──────────┤  │ Samples of Signal 0     │
 *       Count = │  Signal 1 │           ├─────┐    │ Sample 1 │  │ (extracted from         │
 *     channels_ │           ├───────────┤     │    ├──────────┤  │ Channel A)              │
 *       num + 1 │  Signal 2 │           ├───┐ │    │ Sample 2 │  │                         │
 *               │           ├───────────┤   │ └───►├──────────┤ ─┘ ─┐                      │
 *               │           │           ├─┐ │      │ Sample 0 │     │                      │
 *               └────────── └───────────┘ │ │      ├──────────┤     │ Samples of Signal 1  │
 *                                         │ │      │ Sample 1 │     │ (extracted from      │
 *                                         │ │      ├──────────┤     │ Channel B)           │ Count =
 *                                         │ │      │ Sample 2 │     │                      │ sample_buffer
 *                                         │ └─────►├──────────┤ ─┐ ─┘                      │ _max_num
 *                                         │        │ Sample 0 │  │                         │
 *                                         │        ├──────────┤  │ Samples of Signal 2     │
 *                                         │        │ Sample 1 │  │ (extracted from         │
 *                                         │        ├──────────┤  │ Channel C)              │
 *                                         │        │ Sample 2 │  │                         │
 *                                         └───────►├──────────┤ ─┘                         │
 *                                                      ••••                                │
 *                                                  ├──────────┤                            │
 *                                                  │          │                            │
 *                                                  └──────────┘ ───────────────────────────┘
 */
static err_code_t extract_channels(const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                                   as7050_appmgr_channel_id_t *p_channels, uint8_t channels_num,
                                   const as7050_appmgr_sample_t **pp_samples, as7050_appmgr_sample_t *p_sample_buffer,
                                   uint16_t sample_buffer_max_num)
{
    M_CHECK_NULL_POINTER(p_fifo_data);
    M_CHECK_NULL_POINTER(p_channels);
    M_CHECK_NULL_POINTER(pp_samples);
    M_CHECK_NULL_POINTER(p_sample_buffer);

    as7050_appmgr_sample_t *p_sample_buffer_current = p_sample_buffer;
    uint16_t sample_buffer_remaining_num = sample_buffer_max_num;

    for (uint8_t i = 0; i < channels_num; i++) {
        if (0 == sample_buffer_remaining_num) {
            return ERR_SIZE;
        }

        uint16_t sample_num = sample_buffer_remaining_num;
        as7050_channel_flags_t channel_flag = (as7050_channel_flags_t)(1 << (p_channels[i] - AS7050_CHANNEL_PPG_1));
        M_CHECK_SUCCESS(as7050_ifce_extract_samples(g_measurement_config.fifo_map, g_measurement_config.sample_size,
                                                    channel_flag, (uint8_t *)p_fifo_data, fifo_data_size,
                                                    p_sample_buffer_current, &sample_num));

   

        pp_samples[i] = p_sample_buffer_current;
        p_sample_buffer_current += sample_num;
        sample_buffer_remaining_num -= sample_num;
    }

    pp_samples[channels_num] = p_sample_buffer_current;

    return ERR_SUCCESS;
}

/*!
 * \brief Chip-specific function called in state ::AS7050_APPMGR_STATE_PROCESSING to convert a sample from the native
 *        data type of the chip to an unsigned 16-bit integer.
 */
static err_code_t convert_sample_u16(as7050_appmgr_sample_t input, uint16_t *p_output)
{
    M_CHECK_NULL_POINTER(p_output);

    uint8_t bit_shift_num = 0;
    if (g_measurement_config.max_adc_bit_width >= 16) {
        bit_shift_num = g_measurement_config.max_adc_bit_width - 16;
    }

    uint32_t temp = input >> bit_shift_num;

    if (temp > UINT16_MAX + 1) {
        return ERR_SIZE;
    } else if (temp == UINT16_MAX + 1) {
        /* Value is one bit greater in case of saturation */
        temp--;
    }

    *p_output = (uint16_t)temp;
    return ERR_SUCCESS;
}

/*!
 * \brief Chip-specific function called in state ::AS7050_APPMGR_STATE_PROCESSING to convert a sample from the native
 *        data type of the chip to an unsigned 32-bit integer.
 */
static err_code_t convert_sample_u32(as7050_appmgr_sample_t input, uint32_t *p_output)
{
    M_CHECK_NULL_POINTER(p_output);

    *p_output = input;

    return ERR_SUCCESS;
}

/*!
 * \brief Takes the data extracted by ::extract_channels, ensures that the same number of samples is provided for each
 *        signal (optionally, if enabled for an app), converts the data to the required type, and creates the data
 *        structure required by the Bio App Handler interface.
 */
static err_code_t prepare_signal_data(bio_signal_samples_t *p_signal_data,
                                      as7050_appmgr_converted_samples_buffer_t *p_converted_samples_buffer,
                                      const as7050_appmgr_sample_t *const *pp_signal_samples,
                                      const as7050_appmgr_app_definition_t *p_app)
{
    M_CHECK_IS_TRUE_CONFIG(p_app->signals_num >= 1);
    M_CHECK_IS_TRUE_CONFIG((BIO_SIGNAL_SAMPLES_TYPE_U16 == p_app->data_type) ||
                           (BIO_SIGNAL_SAMPLES_TYPE_U32 == p_app->data_type));

    uint16_t min_signal_samples_num = UINT16_MAX;

    /* Calculate minimum sample count of all signals */
    for (uint8_t signal_idx = 0; signal_idx < p_app->signals_num; signal_idx++) {
        if (pp_signal_samples[signal_idx + 1] - pp_signal_samples[signal_idx] > UINT16_MAX) {
            return ERR_ARGUMENT;
        }
        uint16_t signal_sample_num = pp_signal_samples[signal_idx + 1] - pp_signal_samples[signal_idx];
       

        if (p_app->p_excess_samples && p_app->p_excess_samples[signal_idx].has_data) {
            /* This sample count not only includes the newly received signal samples, but also a stored excess sample */
            signal_sample_num++;
        }

        if (signal_sample_num < min_signal_samples_num) {
            min_signal_samples_num = signal_sample_num;
        }
    }

    uint16_t converted_samples_buffer_idx = 0;

    for (uint8_t signal_idx = 0; signal_idx < p_app->signals_num; signal_idx++) {
        if (BIO_SIGNAL_SAMPLES_TYPE_U16 == p_app->data_type) {
            p_signal_data[signal_idx].samples.p_u16 = &p_converted_samples_buffer->u16[converted_samples_buffer_idx];
        } else if (BIO_SIGNAL_SAMPLES_TYPE_U32 == p_app->data_type) {
            p_signal_data[signal_idx].samples.p_u32 = &p_converted_samples_buffer->u32[converted_samples_buffer_idx];
        }

        uint16_t signal_samples_insert_num;
        uint16_t signal_sample_num = pp_signal_samples[signal_idx + 1] - pp_signal_samples[signal_idx];

        if (p_app->p_excess_samples) {
            if (p_app->p_excess_samples[signal_idx].has_data) {
                if (min_signal_samples_num == 0) {
                    /* If the minimum signal sample count is zero, no samples will be provided to the app. Therefore,
                     * the excess sample needs to remain stored and is not inserted in the output. */
                    signal_samples_insert_num = 0;
                } else {
                    /* Convert the stored excess sample and move it to the output */
                    if (converted_samples_buffer_idx >= AS7050_APPMGR_MAX_APP_SAMPLE_NUM) {
                        return ERR_SIZE;
                    }
                    if (BIO_SIGNAL_SAMPLES_TYPE_U16 == p_app->data_type) {
                        M_CHECK_SUCCESS(
                            convert_sample_u16(p_app->p_excess_samples[signal_idx].data,
                                               &p_converted_samples_buffer->u16[converted_samples_buffer_idx++]));
                    } else if (BIO_SIGNAL_SAMPLES_TYPE_U32 == p_app->data_type) {
                        M_CHECK_SUCCESS(
                            convert_sample_u32(p_app->p_excess_samples[signal_idx].data,
                                               &p_converted_samples_buffer->u32[converted_samples_buffer_idx++]));
                    }
                    p_app->p_excess_samples[signal_idx].has_data = FALSE;

                    /* As the output now contains one sample and the number of samples should be min_signal_samples_num,
                     * insert min_signal_samples_num - 1 samples from the newly received data */
                    signal_samples_insert_num = min_signal_samples_num - 1;
                }
            } else {
                signal_samples_insert_num = min_signal_samples_num;
            }
        } else {
            /* Insert all received samples in the output if excess samples functionality is disabled */
            signal_samples_insert_num = signal_sample_num;
        }

        /* Convert the newly received samples and insert them in the output */
        for (uint16_t signal_sample_idx = 0; signal_sample_idx < signal_samples_insert_num; signal_sample_idx++) {
            if (converted_samples_buffer_idx >= AS7050_APPMGR_MAX_APP_SAMPLE_NUM) {
                return ERR_SIZE;
            }
            if (BIO_SIGNAL_SAMPLES_TYPE_U16 == p_app->data_type) {
                M_CHECK_SUCCESS(convert_sample_u16(pp_signal_samples[signal_idx][signal_sample_idx],
                                                   &p_converted_samples_buffer->u16[converted_samples_buffer_idx++]));
            } else if (BIO_SIGNAL_SAMPLES_TYPE_U32 == p_app->data_type) {
                M_CHECK_SUCCESS(convert_sample_u32(pp_signal_samples[signal_idx][signal_sample_idx],
                                                   &p_converted_samples_buffer->u32[converted_samples_buffer_idx++]));
            }
        }

        if (p_app->p_excess_samples) {
            /* Store sample of the newly received samples that was not inserted in the output, if applicable */
            if (signal_samples_insert_num < signal_sample_num) {
                if (signal_sample_num - signal_samples_insert_num > 1) {
                    /* Only one excess sample can be stored per signal */
                    return ERR_OVERFLOW;
                }

                p_app->p_excess_samples[signal_idx].data = pp_signal_samples[signal_idx][signal_samples_insert_num];
                p_app->p_excess_samples[signal_idx].has_data = TRUE;
            }

            /* Update the output sample count to reflect the actual amount of samples */
            p_signal_data[signal_idx].count = min_signal_samples_num;
        } else {
            p_signal_data[signal_idx].count = signal_sample_num;
        }
    }
     
    return ERR_SUCCESS;
}

/******************************************************************************
 *                              GLOBAL FUNCTIONS                              *
 ******************************************************************************/

err_code_t as7050_appmgr_initialize(void)
{
    /* Reset all local variables */
    g_enabled_apps = 0;
    memset(g_signal_mapping_hrm, 0, sizeof(g_signal_mapping_hrm));
    memset(g_signal_mapping_spo2, 0, sizeof(g_signal_mapping_spo2));
    memset(g_agc_mapping_hrm, 0, sizeof(g_agc_mapping_hrm));
    memset(g_agc_mapping_spo2, 0, sizeof(g_agc_mapping_spo2));
    memset(g_excess_samples_spo2, 0, sizeof(g_excess_samples_spo2));
    g_executable_apps = 0;
    g_pending_ext_event_count = 0;
    M_CHECK_SUCCESS(clear_measurement_config());

    /* Initialize all apps */
    for (uint8_t i = 0; i < AS7050_APPMGR_APP_ID_NUM; i++) {
        M_CHECK_IS_TRUE_CONFIG(g_app_table[i].p_initialize);
        M_CHECK_SUCCESS(g_app_table[i].p_initialize());
    }

    g_state = AS7050_APPMGR_STATE_CONFIGURATION;

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_enable_apps(uint32_t enabled_apps)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_CONFIGURATION);

    if (0 == enabled_apps || enabled_apps >= (1 << AS7050_APPMGR_APP_ID_NUM)) {
        /* No apps selected or invalid bits set */
        return ERR_ARGUMENT;
    }

    /* Check whether apps are enabled that cannot be enabled concurrently. */
    uint32_t enabled_apps_unprocessed = enabled_apps;
    uint8_t i = 0;
    while (enabled_apps_unprocessed) {
        if (enabled_apps_unprocessed & 1) {
            uint32_t excluded_apps = g_app_table[i].excluded_apps;

            if (enabled_apps & excluded_apps) {
                return ERR_ARGUMENT;
            }
        }

        i++;
        enabled_apps_unprocessed >>= 1;
    }

    g_enabled_apps = enabled_apps;

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_set_signal_routing(as7050_appmgr_app_id_t app, const as7050_appmgr_channel_id_t *p_channels,
                                            uint8_t channels_num)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_CONFIGURATION);
    M_CHECK_APP_ID(app);

    const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];

    if (channels_num != p_app->signals_num) {
        /* Signal count mismatch */
        return ERR_SIZE;
    }

    if (p_app->signals_num > 0) {
        M_CHECK_NULL_POINTER(p_channels);
        M_CHECK_IS_TRUE_CONFIG(p_app->p_signal_mapping);
    }

    for (uint8_t i = 0; i < channels_num; i++) {
        if (p_channels[i] < AS7050_APPMGR_CHANNEL_ID_FIRST || p_channels[i] > AS7050_APPMGR_CHANNEL_ID_LAST) {
            /* Channel not within valid range */
            return ERR_ARGUMENT;
        }

        /* Copy signal mapping to internal storage */
        p_app->p_signal_mapping[i] = p_channels[i];
    }

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_configure_app(as7050_appmgr_app_id_t app, const void *p_config, uint8_t size)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_CONFIGURATION);
    M_CHECK_APP_ID(app);
    M_CHECK_NULL_POINTER(p_config);
    const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
    M_CHECK_IS_TRUE_CONFIG(p_app->p_configure);

    return p_app->p_configure(p_config, size);
}

err_code_t as7050_appmgr_start_processing(as7050_meas_config_t measurement_config, uint32_t acc_sample_period_us,
                                          const as7050_appmgr_channel_id_t *p_agc_mappings, uint8_t agc_mappings_num)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_CONFIGURATION);
    uint8_t data_available;
    M_CHECK_ARGUMENT_NOT_ZERO(acc_sample_period_us);
    if (agc_mappings_num > 0) {
        M_CHECK_NULL_POINTER(p_agc_mappings);
    }
    M_CHECK_ARGUMENT_LOWER(agc_mappings_num, AS7050_APPMGR_AGC_MAPPING_DISABLED);
    M_CHECK_IS_TRUE_CONFIG(g_enabled_apps);

    g_executable_apps = 0;
    g_pending_ext_event_count = 0;

    M_CHECK_SUCCESS(process_measurement_config(measurement_config));

    if (M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_RAW)) {
        M_CHECK_SUCCESS(handle_raw_app_start());
    }

    if (M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_GSR)) {
        M_CHECK_SUCCESS(handle_gsr_app_start());
    }


    /* Start enabled non-raw apps */
    for (uint8_t app = 0; app < AS7050_APPMGR_APP_ID_NUM; app++) {
        if (AS7050_APPMGR_APP_ID_RAW == app || AS7050_APPMGR_APP_ID_GSR == app || !M_APP_IS_ENABLED(app)) {
            /* Skip apps with non-standard start function and disabled apps */
            continue;
        }

        const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
        if (AS7050_APPMGR_APP_ID_GSR != app) {
            M_CHECK_IS_TRUE_CONFIG(p_app->p_start);
        }

        M_CHECK_IS_TRUE_CONFIG(p_app->signals_num <= AS7050_APPMGR_MAX_APP_SIGNAL_NUM);

        uint32_t signal_sample_periods_us[AS7050_APPMGR_MAX_APP_SIGNAL_NUM];

        for (uint8_t signal = 0; signal < p_app->signals_num; signal++) {
            as7050_appmgr_channel_id_t channel_id = p_app->p_signal_mapping[signal];

            if (p_app->p_agc_mapping) {
                p_app->p_agc_mapping[signal] = AS7050_APPMGR_AGC_MAPPING_DISABLED;
                for (uint8_t agc = 0; agc < agc_mappings_num; agc++) {
                    if (p_agc_mappings[agc] == channel_id) {
                        p_app->p_agc_mapping[signal] = agc;
                        break;
                    }
                }
            }

            /* Get sample period and implicitly check whether signal routing is valid as get_sample_period_for_channel
             * is required to return an error when the channel selection is invalid given the current measurement
             * configuration */
            uint32_t channel_sample_period_us = 0;
            M_CHECK_SUCCESS(get_sample_period_for_channel(channel_id, &channel_sample_period_us));

            signal_sample_periods_us[signal] = channel_sample_period_us;
        }

        /* Reset excess sample storage */
        if (p_app->p_excess_samples && p_app->signals_num >= 1) {
            memset(p_app->p_excess_samples, 0, (p_app->signals_num - 1) * sizeof(as7050_appmgr_excess_sample_t));
        }

        M_CHECK_SUCCESS(p_app->p_start(signal_sample_periods_us, p_app->signals_num, acc_sample_period_us));
    }

    g_state = AS7050_APPMGR_STATE_PROCESSING;

    return ERR_SUCCESS;
}



err_code_t as7050_appmgr_set_input(const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                                   as7050_appmgr_chip_status_t chip_status, const bio_agc_status_t *p_agc_statuses,
                                   uint8_t agc_statuses_num, const vs_acc_data_t *p_acc_samples,
                                   uint16_t acc_samples_num, uint32_t *p_ready_for_execution)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_PROCESSING);

        uint32_t extracted_values[32];
        uint32_t extracted_values_ir[32];
        uint32_t extracted_values_red[32];
        uint16_t num_extracted_values;
        uint16_t num_extracted_values_ir;
        uint16_t num_extracted_values_red;
        spo2_app_agc_states_t spo2_agc_changed;

    if (fifo_data_size > 0) {
        M_CHECK_NULL_POINTER(p_fifo_data);
    }
    if (agc_statuses_num > 0) {
        M_CHECK_NULL_POINTER(p_agc_statuses);
    }
    //if (acc_samples_num > 0) {
    //    M_CHECK_NULL_POINTER(p_acc_samples);
    //}
    M_CHECK_NULL_POINTER(p_ready_for_execution);


    if (M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_RAW)) {
        /* Special handling of raw app as it receives data from Chip Library as-is */
        bio_execution_status_t result = BIO_EXECUTION_STATUS_NOT_EXECUTABLE;
        M_CHECK_SUCCESS(handle_raw_app_set_input(p_fifo_data, fifo_data_size, chip_status, p_agc_statuses,
                                                 agc_statuses_num, p_acc_samples, acc_samples_num, &result));

        /* Store "ready to execute" information */
        if (BIO_EXECUTION_STATUS_EXECUTABLE == result) {
            g_executable_apps |= AS7050_APPMGR_APP_FLAG_RAW;
        }
    }

    for (as7050_appmgr_app_id_t app = AS7050_APPMGR_APP_ID_FIRST_AFTER_RAW; app < AS7050_APPMGR_APP_ID_NUM; app++) {
        if (!M_APP_IS_ENABLED(app)) {
            /* Skip disabled app */
            continue;
        }

        const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
        M_CHECK_IS_TRUE_CONFIG(p_app->p_set_input);
        M_CHECK_IS_TRUE_CONFIG(p_app->signals_num <= AS7050_APPMGR_MAX_APP_SIGNAL_NUM);

        /* Extract samples for all channels used by app */
        const as7050_appmgr_sample_t *p_signal_samples[AS7050_APPMGR_MAX_APP_SIGNAL_NUM + 1];
        as7050_appmgr_sample_t sample_buffer[AS7050_APPMGR_MAX_APP_SAMPLE_NUM];
        M_CHECK_SUCCESS(extract_channels(p_fifo_data, fifo_data_size, p_app->p_signal_mapping, p_app->signals_num,
                                         p_signal_samples, sample_buffer, AS7050_APPMGR_MAX_APP_SAMPLE_NUM));
     
//////////////Added for SPO2 APP////////////////////////
       if(M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_SPO2_A0))
       {
          as7050_ifce_extract_samples(
            g_measurement_config.fifo_map, g_measurement_config.sample_size,
            AS7050_CHANNEL_FLAG_PPG_1, p_fifo_data,
            fifo_data_size, extracted_values_red, &num_extracted_values_red);
          as7050_ifce_extract_samples(
            g_measurement_config.fifo_map, g_measurement_config.sample_size,
            AS7050_CHANNEL_FLAG_PPG_2, p_fifo_data,
            fifo_data_size, extracted_values_ir, &num_extracted_values_ir);
          as7050_ifce_extract_samples(
            g_measurement_config.fifo_map, g_measurement_config.sample_size,
            AS7050_CHANNEL_FLAG_PPG_3, p_fifo_data,
            fifo_data_size, extracted_values, &num_extracted_values);
        }

        /* Create signal data with converted samples */
        bio_signal_samples_t signal_data[AS7050_APPMGR_MAX_APP_SIGNAL_NUM];
        as7050_appmgr_converted_samples_buffer_t converted_samples_buffer;
        M_CHECK_SUCCESS(prepare_signal_data(signal_data, &converted_samples_buffer, p_signal_samples, p_app));
      

        /* Create AGC pointer array */
        const bio_agc_status_t *p_app_agc_statuses[AS7050_APPMGR_MAX_APP_SIGNAL_NUM];
        for (uint8_t signal = 0; signal < p_app->signals_num; signal++) {
            if (p_app->p_agc_mapping) {
                uint8_t agc_idx = p_app->p_agc_mapping[signal];
                if (AS7050_APPMGR_AGC_MAPPING_DISABLED == agc_idx) {
                    p_app_agc_statuses[signal] = NULL;
                } else if (agc_idx < agc_statuses_num) {
                    p_app_agc_statuses[signal] = p_agc_statuses + p_app->p_agc_mapping[signal];
                } 
        //else {
        //            return ERR_SIZE;
        //        }
            } else {
                p_app_agc_statuses[signal] = NULL;
            }
        }
//////////////Added for SPO2 APP////////////////////////
        if(M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_SPO2_A0)){

          if ((AS7050_AGC_STATE_INCREASED == p_agc_statuses->led_change) ||
                (AS7050_AGC_STATE_INCREASED == p_agc_statuses->led_change) ||
                (AS7050_AGC_STATE_INCREASED == p_agc_statuses->pd_offset_change) ||
                (AS7050_AGC_STATE_INCREASED == p_agc_statuses->pd_offset_change)) {
                spo2_agc_changed = SPO2_APP_AGC_STATE_INCREASED;
            } else if ((AS7050_AGC_STATE_DECREASED == p_agc_statuses->led_change) ||
                       (AS7050_AGC_STATE_DECREASED == p_agc_statuses->led_change) ||
                       (AS7050_AGC_STATE_DECREASED == p_agc_statuses->pd_offset_change) ||
                       (AS7050_AGC_STATE_DECREASED == p_agc_statuses->pd_offset_change)) {
                spo2_agc_changed = SPO2_APP_AGC_STATE_DECREASED;

            } else {
                spo2_agc_changed = SPO2_APP_AGC_STATE_UNCHANGED;
            }

          }
        /* Provide data to app */
        bio_execution_status_t result = BIO_EXECUTION_STATUS_NOT_EXECUTABLE;
        M_CHECK_SUCCESS(p_app->p_set_input(p_app->data_type, signal_data, p_app_agc_statuses, p_app->signals_num,
                                           p_acc_samples, acc_samples_num, &result));

//////////////Added for SPO2 APP////////////////////////
      //if(M_APP_IS_ENABLED(AS7050_APPMGR_APP_ID_SPO2_A0)){
      //   M_CHECK_SUCCESS(spo2_app_append_input_data(extracted_values, extracted_values_ir, extracted_values_red,
      //                                          num_extracted_values_ir, spo2_agc_changed,
      //                                          p_agc_statuses->pd_offset_current, &result));
      // }

        //printf("result = %d\n",result);
        /* Store "ready to execute" information */
        if (BIO_EXECUTION_STATUS_EXECUTABLE == result) {
            g_executable_apps |= M_AS7050_APPMGR_APP_FLAG(app);
        }
    }

    /* Provide "ready to execute" information to caller */
    if (p_ready_for_execution) {
        *p_ready_for_execution = g_executable_apps;
    }

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_set_ext_event_occurred(void)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_PROCESSING);

    if (g_pending_ext_event_count >= UINT8_MAX) {
        return ERR_OVERFLOW;
    }

    g_pending_ext_event_count++;
    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_execute(uint32_t *p_data_available)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_PROCESSING);
    M_CHECK_NULL_POINTER(p_data_available);

    *p_data_available = 0;

    for (uint8_t app = 0; app < AS7050_APPMGR_APP_ID_NUM; app++) {
        if (!M_APP_IS_ENABLED(app) || !M_APP_FLAG_SET(g_executable_apps, app)) {
            /* Skip apps that are disabled or not ready for execution */
            continue;
        }

        const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
        M_CHECK_IS_TRUE_CONFIG(p_app->p_execute);

        /* Execute app */
        bio_output_status_t result = BIO_OUTPUT_STATUS_DATA_UNAVAILABLE;
        M_CHECK_SUCCESS(p_app->p_execute(&result));

        /* Store execution result */
        if (BIO_OUTPUT_STATUS_DATA_AVAILABLE == result) {
            *p_data_available |= M_AS7050_APPMGR_APP_FLAG(app);
        }
    }

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_get_output(as7050_appmgr_app_id_t app, void *p_dest, uint16_t *p_size)
{
    M_CHECK_STATE(AS7050_APPMGR_STATE_PROCESSING);
    M_CHECK_APP_ID(app);
    M_CHECK_APP_IS_ENABLED(app);
    M_CHECK_NULL_POINTER(p_dest);
    M_CHECK_NULL_POINTER(p_size);



    const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
    M_CHECK_IS_TRUE_CONFIG(p_app->p_get_output);

    return p_app->p_get_output(p_dest, p_size);

}

err_code_t as7050_appmgr_stop_processing(void)
{
    M_CHECK_STATE_NOT(AS7050_APPMGR_STATE_UNINITIALIZED);

    /* Stop enabled apps */
    for (uint8_t app = 0; app < AS7050_APPMGR_APP_ID_NUM; app++) {
        if (!M_APP_IS_ENABLED(app)) {
            /* Skip disabled app */
            continue;
        }

        const as7050_appmgr_app_definition_t *p_app = &g_app_table[app];
        M_CHECK_IS_TRUE_CONFIG(p_app->p_stop);

        M_CHECK_SUCCESS(p_app->p_stop());
    }

    M_CHECK_SUCCESS(clear_measurement_config());

    g_state = AS7050_APPMGR_STATE_CONFIGURATION;

    return ERR_SUCCESS;
}

err_code_t as7050_appmgr_shutdown(void)
{
    err_code_t return_value = ERR_SUCCESS;

    for (uint8_t i = 0; i < AS7050_APPMGR_APP_ID_NUM; i++) {
        if (g_app_table[i].p_shutdown) {
            err_code_t result = g_app_table[i].p_shutdown();
            if (result != ERR_SUCCESS) {
                return_value = result;
            }
        } else {
            return_value = ERR_CONFIG;
        }
    }

    g_state = AS7050_APPMGR_STATE_UNINITIALIZED;

    return return_value;
}

err_code_t as7050_appmgr_get_version(as7050_appmgr_version_t *p_version)
{
    M_CHECK_NULL_POINTER(p_version);

    p_version->major = AS7050_APPMGR_VER_MAJOR;
    p_version->minor = AS7050_APPMGR_VER_MINOR;
    p_version->patch = AS7050_APPMGR_VER_PATCH;

    return ERR_SUCCESS;
}
