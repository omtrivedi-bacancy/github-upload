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

#include <stddef.h>
#include <string.h>

#include "as7050_typedefs.h"
#include "error_codes.h"
#include "gsr_app.h"
#include "gsr_app_typedefs.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! Number of DACs used for GSR measurement. */
#define DAC_NUM 2

/*! Factor to convert seconds to microseconds. */
#define SEC_IN_US 1000000

/*! Resistance of the internal reference resistor in Ohm. */
#define INTERNAL_REF_RESISTOR 500000

/*! States of the GSR application. */
enum gsr_state {
    GSR_STATE_UNINITIALIZED = 0, /*!< Uninitialized state.*/
    GSR_STATE_CONFIGURATION,     /*!< Configuration state.*/
    GSR_STATE_PROCESSING,        /*!< Processing state.*/
};

/*! Type for ::gsr_state. */
typedef uint8_t gsr_state_t;

/*! Holds state and configuration of the application. */
typedef struct {
    gsr_state_t state;                      /*!< Current state of the application. */
    uint8_t executable;                     /*!< ::TRUE when the app is currently executable, ::FALSE otherwise. */
    uint8_t output_available;               /*!< ::TRUE when app output is currently available, ::FALSE otherwise. */
    uint8_t num_dac_samples;                /*!< Number of acquired DAC samples before the sensor switches to sampling
                                                 the other DAC. The sensor switches between DACs automatically every
                                                 num_dac_samples samples. */
    uint8_t cur_dac_id;                     /*!< Index of the currently sampled DAC. This value is either 0 or 1. */
    uint8_t cur_dac_sample_num;             /*!< Number of samples acquired since the last DAC switch. */
    uint16_t max_average_num;               /*!< Number of samples per DAC to use for averaging so that an output is
                                                 generated approximately every second. */
    uint32_t dac_ref;                       /*!< DAC reference value of the device, as provided via app config. */
    uint32_t resistor_diff;                 /*!< Last calculated difference between the averaged sample values of both
                                                 DACs. */
    uint64_t resistor_average[DAC_NUM];     /*!< Sum of all sample values acquired until now for the current average
                                                 calculation. The sample values are summed separately per DAC. */
    uint16_t resistor_average_num[DAC_NUM]; /*!< Number of samples acquired until now for the current average
                                                 calculation. The samples are counted separately per DAC. */
    gsr_app_output_t output;                /*!< Output structure containing the last calculated resistance value. */
} gsr_config_t;

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

/*! State and configuration of the application. */
static volatile gsr_config_t g_gsr_config = {0};

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t gsr_app_initialize(void)
{
    M_CHECK_SUCCESS(gsr_app_shutdown());

    g_gsr_config.state = GSR_STATE_CONFIGURATION;
    return ERR_SUCCESS;
}

err_code_t gsr_app_configure(const void *p_config, uint8_t size)
{
    if (g_gsr_config.state != GSR_STATE_CONFIGURATION) {
        return ERR_PERMISSION;
    }

    M_CHECK_NULL_POINTER(p_config);
    M_CHECK_SIZE(size, sizeof(gsr_app_configuration_t));

    const gsr_app_configuration_t *p_gsr_config = (const gsr_app_configuration_t *)p_config;

    if (0 == p_gsr_config->dac_ref) {
        return ERR_ARGUMENT;
    }

    g_gsr_config.dac_ref = p_gsr_config->dac_ref;
    return ERR_SUCCESS;
}

err_code_t gsr_app_start(const uint32_t *p_signal_sample_periods_us, uint8_t signal_num, uint8_t num_dac_samples)
{
    uint32_t i;

    if (g_gsr_config.state != GSR_STATE_CONFIGURATION) {
        return ERR_PERMISSION;
    }

    if (signal_num != GSR_APP_SIGNAL_NUM) {
        return ERR_ARGUMENT;
    }

    M_CHECK_NULL_POINTER(p_signal_sample_periods_us);

    if (0 == p_signal_sample_periods_us[GSR_APP_SIGNAL_ECG]) {
        return ERR_ARGUMENT;
    }

    if ((0 == num_dac_samples) || (8u < num_dac_samples)) {
        return ERR_ARGUMENT;
    }

    if (0 == g_gsr_config.dac_ref) {
        return ERR_CONFIG;
    }

    g_gsr_config.num_dac_samples = num_dac_samples;

    /* Calculate number of samples per DAC so that one GSR value can be calculated each second */
    g_gsr_config.max_average_num =
        (((SEC_IN_US / p_signal_sample_periods_us[GSR_APP_SIGNAL_ECG]) / DAC_NUM) / num_dac_samples) * num_dac_samples;
    g_gsr_config.output.resistor = 0;
    g_gsr_config.cur_dac_id = 0;
    g_gsr_config.cur_dac_sample_num = 0;
    g_gsr_config.resistor_diff = 0;
    g_gsr_config.executable = FALSE;
    g_gsr_config.output_available = FALSE;

    for (i = 0; i < DAC_NUM; i++) {
        g_gsr_config.resistor_average_num[i] = 0;
        g_gsr_config.resistor_average[i] = 0;
    }

    g_gsr_config.state = GSR_STATE_PROCESSING;
    return ERR_SUCCESS;
}

err_code_t gsr_app_set_input(bio_signal_samples_type_t signal_samples_type,
                             const bio_signal_samples_t *p_signal_samples,
                             const bio_agc_status_t *const *pp_agc_statuses, uint8_t signal_num,
                             const vs_acc_data_t *p_acc_samples, uint8_t acc_sample_num,
                             bio_execution_status_t *p_result)
{
    M_UNUSED_PARAM(pp_agc_statuses);
    M_UNUSED_PARAM(p_acc_samples);
    M_UNUSED_PARAM(acc_sample_num);

    const uint32_t *p_channel_data;
    uint16_t channel_data_num;
    uint16_t i;
    err_code_t result = ERR_SUCCESS;

    if (g_gsr_config.state != GSR_STATE_PROCESSING) {
        return ERR_PERMISSION;
    }
    if (signal_samples_type != GSR_APP_SIGNAL_DATA_TYPE) {
        return ERR_ARGUMENT;
    }
    if (signal_num != GSR_APP_SIGNAL_NUM) {
        return ERR_ARGUMENT;
    }

    M_CHECK_NULL_POINTER(p_signal_samples);
    M_CHECK_NULL_POINTER(p_result);
    M_CHECK_NULL_POINTER(p_signal_samples[GSR_APP_SIGNAL_ECG].samples.p_u32);

    p_channel_data = p_signal_samples[GSR_APP_SIGNAL_ECG].samples.p_u32;
    channel_data_num = p_signal_samples[GSR_APP_SIGNAL_ECG].count;

    for (i = 0; i < channel_data_num; i++) {
        g_gsr_config.resistor_average[g_gsr_config.cur_dac_id] += p_channel_data[i];
        g_gsr_config.resistor_average_num[g_gsr_config.cur_dac_id]++;

        g_gsr_config.cur_dac_sample_num++;
        if (g_gsr_config.cur_dac_sample_num >= g_gsr_config.num_dac_samples) {
            g_gsr_config.cur_dac_sample_num = 0;
            g_gsr_config.cur_dac_id = !g_gsr_config.cur_dac_id;

            if ((0 == g_gsr_config.cur_dac_id) &&
                (g_gsr_config.max_average_num <= g_gsr_config.resistor_average_num[1])) {

                if (g_gsr_config.executable) {
                    result = ERR_OVERFLOW;
                }

                /* Calculate averages and differences */
                g_gsr_config.resistor_diff = g_gsr_config.resistor_average[0] / g_gsr_config.resistor_average_num[0] -
                                             (g_gsr_config.resistor_average[1] / g_gsr_config.resistor_average_num[1]);
                g_gsr_config.executable = TRUE;

                /* Reset values */
                g_gsr_config.resistor_average[0] = 0;
                g_gsr_config.resistor_average_num[0] = 0;
                g_gsr_config.resistor_average[1] = 0;
                g_gsr_config.resistor_average_num[1] = 0;
            }
        }
    }

    *p_result = (g_gsr_config.executable ? BIO_EXECUTION_STATUS_EXECUTABLE : BIO_EXECUTION_STATUS_NOT_EXECUTABLE);

    return result;
}

err_code_t gsr_app_execute(bio_output_status_t *p_result)
{
    if (g_gsr_config.state != GSR_STATE_PROCESSING) {
        return ERR_PERMISSION;
    }

    M_CHECK_NULL_POINTER(p_result);

    if (g_gsr_config.executable) {
        if (g_gsr_config.dac_ref <= g_gsr_config.resistor_diff) {
            /* Invalid DAC reference value provided */
            g_gsr_config.output.resistor = UINT32_MAX;
        } else {
            g_gsr_config.output.resistor = INTERNAL_REF_RESISTOR * (uint64_t)g_gsr_config.resistor_diff /
                                           (g_gsr_config.dac_ref - g_gsr_config.resistor_diff);
        }
        g_gsr_config.executable = FALSE;
        g_gsr_config.output_available = TRUE;
    }

    *p_result = (g_gsr_config.output_available ? BIO_OUTPUT_STATUS_DATA_AVAILABLE : BIO_OUTPUT_STATUS_DATA_UNAVAILABLE);

    return ERR_SUCCESS;
}

err_code_t gsr_app_get_output(void *p_dest, uint16_t *p_size)
{
    if (g_gsr_config.state != GSR_STATE_PROCESSING) {
        return ERR_PERMISSION;
    }

    M_CHECK_NULL_POINTER(p_dest);
    M_CHECK_NULL_POINTER(p_size);

    if (*p_size < sizeof(gsr_app_output_t)) {
        return ERR_SIZE;
    }

    if (!g_gsr_config.output_available) {
        return ERR_NO_DATA;
    }

    memcpy(p_dest, (void *)&g_gsr_config.output, sizeof(gsr_app_output_t));
    *p_size = sizeof(gsr_app_output_t);
    g_gsr_config.output_available = FALSE;

    return ERR_SUCCESS;
}

err_code_t gsr_app_stop(void)
{
    if (g_gsr_config.state != GSR_STATE_PROCESSING) {
        return ERR_PERMISSION;
    }

    g_gsr_config.state = GSR_STATE_CONFIGURATION;

    return ERR_SUCCESS;
}

err_code_t gsr_app_shutdown(void)
{
    memset((void *)&g_gsr_config, 0, sizeof(g_gsr_config));

    return ERR_SUCCESS;
}
