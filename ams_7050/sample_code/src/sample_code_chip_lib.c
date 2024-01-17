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

/*
This sample code shows how the AS7050 Chip Library can be used standalone for
measurement.
The interface to the Operation System Abstraction Layer (OSAL) is included.
*/

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <signal.h>
#include <stdio.h>
#include <string.h>
//#include <unistd.h>

#include "as7050_chiplib.h"
#include "as7050_std_include.h"

#include "error_codes.h"

/******************************************************************************
 *                                  DEFINITIONS                               *
 ******************************************************************************/

#define AS7050_CHANNEL_CNT 9 // 8x PPG + 1x ECG

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

// This flag keeps the demo application running. It is changed to 0 when a
// SIGINT occurs (Ctrl+C)
static volatile uint8_t g_keep_running = 1;

// Bitmap of sampled channels
static volatile uint16_t g_fifo_map;

// Sample period of PPG signals
static volatile double g_ppg_sample_period_s;

// Sample period of ECG signals
static volatile double g_ecg_sample_period_s;

// The total number of samples received per channel
static volatile uint32_t g_sample_cnt[AS7050_CHANNEL_CNT];

// Names of the channels
static const char *g_p_channel_names[AS7050_CHANNEL_CNT] = {
    "PPG1", "PPG2", "PPG3", "PPG4", "PPG5", "PPG6", "PPG7", "PPG8", "ECG",
};

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

// Define signal handler to end program by pressing Ctrl-C
void signal_handler(int signal)
{
    if (SIGINT == signal || SIGTERM == signal) {
        g_keep_running = 0;
    }
}

// The AS7050 sensor stores acquired samples in a FIFO buffer. When the buffer fill level reaches a configured
// threshold, the sensor generates an interrupt. The AS7050 Chip Library processes this interrupt by reading the
// FIFO buffer. The Chip Library then calls a user-defined callback function which receives the read data. This
// callback function is defined here.
// See as7050_typedefs.h: as7050_callback_t
static void as7050_callback(err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param)
{
    int channel_index;
    as7050_channel_flags_t channel_flag;
    double channel_sample_period;
    uint32_t samples[48];
    uint16_t sample_cnt;
    int sample_index;
    err_code_t result;

    // These are unused arguments, mark it to avoid failures on static code analysis.
    M_UNUSED_PARAM(p_agc_status);
    M_UNUSED_PARAM(p_cb_param);

    // Check for error reported by Chip Library
    if (error != ERR_SUCCESS && g_keep_running) {
        printf("Chip Library error %d occurred during measurement.\n", error);
    }

    // Extract and print samples of each sampled channel
    // One line is printed for each channel, containing the channel name, the time when the first sample contained in
    // the line was acquired, and all samples of this channel that were included in the received FIFO data
    for (channel_index = 0; channel_index < AS7050_CHANNEL_CNT; channel_index++) {
        channel_flag = (1 << channel_index) & g_fifo_map;

        if (AS7050_CHANNEL_FLAG_NONE == channel_flag) {
            // Skip disabled channels
            continue;
        } else if (AS7050_CHANNEL_FLAG_ECG == channel_flag) {
            channel_sample_period = g_ecg_sample_period_s;
        } else {
            channel_sample_period = g_ppg_sample_period_s;
        }

        sample_cnt = sizeof(samples) / sizeof(samples[0]);
        result = as7050_extract_samples(channel_flag, p_data, data_num, samples, &sample_cnt);
        if (result != ERR_SUCCESS) {
            if (g_keep_running) {
                printf("Error %d occurred during %s sample extraction.\n", result, g_p_channel_names[channel_index]);
            }
            continue;
        }

        if (sample_cnt > 0) {
            printf("%4s, %6.2lf s: ", g_p_channel_names[channel_index],
                   g_sample_cnt[channel_index] * channel_sample_period);
            for (sample_index = 0; sample_index < sample_cnt - 1; sample_index++) {
                printf("%6d, ", samples[sample_index]);
            }
            printf("%6d\n", samples[sample_cnt - 1]);
            g_sample_cnt[channel_index] += sample_cnt;
        }
    }
}

/******************************************************************************
 *                               Main                                         *
 ******************************************************************************/

// We assume there is an as7050_sample_code_chip_lib.exe and the AS7050 EVK board is
// connected to port COM7.
// Call it this way:
// > as7050_sample_code_chip_lib.exe --interface COM:COM7

//int main(int argc, char *argv[])
{
    const char *interface_key = "--interface";
    char interface[30];
    int i;

    // ------------------------------------------------------------------------
    // Check the return value for every called function.
    err_code_t result = ERR_SUCCESS;

    // ------------------------------------------------------------------------
    // Register signal handler to end the program with Ctrl-C
    signal(SIGINT, signal_handler);

    // ------------------------------------------------------------------------
    // Process passed arguments
    memset(interface, 0, sizeof(interface));
    for (i = 0; i < argc; i++) {
        if (NULL != strstr(argv[i], interface_key)) {
            if (((i + 1) < argc) && (0 < strlen(argv[i + 1])) && (strlen(argv[i + 1]) < sizeof(interface))) {
                memcpy(interface, argv[i + 1], strlen(argv[i + 1]));
                printf("*** Initialization with interface: %s\n", interface);
                break;
            }
        }
    }

#ifdef _WIN32
    if (0 == interface[0]) {
        printf("Got no interface parameter, please call with '--interface COM:COM7' for example\n");
        return -1;
    }
#endif

    // ------------------------------------------------------------------------
    // Initialization

    // (1) Initialize the Chip Library
    // This function must be called first.

    // First parameter is the pointer to the callback function.
    // Second parameter is the pointer to an application specific parameter,
    // which will be transmitted with every callback. This parameter is optional.
    // Third parameter is the pointer to an interface description. The
    // Chip Library forwards this to ::as7050_osal_initialize. See the
    // documentation of the Chip Library. This parameter is mandatory in
    // conjunction with the AS7050 EVK board.
    result = as7050_initialize(as7050_callback, NULL, interface);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_initialize: %d\n", result);
        goto ERROR;
    }

    // ------------------------------------------------------------------------
    // Configuration of the Chip Library
    // *** example: ECG 200 Hz, PPG Single Channel 100 Hz, AOC
    // *** The configuration below works together with the AS7050 EVK board.

    // (1) Set the configuration register
    // See as7050_typedefs.h: enum as7050_reg_group_ids
    // See the datasheet of the sensor.

    // configuration of AFE group
    const as7050_config_afe_t afe_config = {
        .reg_vals.afe_dac0l = 0x00,
        .reg_vals.afe_dac1l = 0x00,
        .reg_vals.afe_dach = 0x00,
        .reg_vals.afe_cfga = 0x00,
        .reg_vals.afe_cfgb = 0x00,
        .reg_vals.afe_gsr = 0x00,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
        goto ERROR;
    }

    // configuration of AMP group
    const as7050_config_amp_t amp_config = {
        .reg_vals.ecg_amp_cfga = 0x81,
        .reg_vals.ecg_amp_cfgb = 0xAB,
        .reg_vals.ecg_amp_cfgc = 0xB4,
        .reg_vals.ecg_amp_cfge = 0x44,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_AMP, (uint8_t *)amp_config.reg_buffer, sizeof(as7050_config_amp_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_AMP: %d\n", result);
        goto ERROR;
    }

    // configuration of AOC group
    const as7050_config_aoc_t aoc_config = {
        .reg_vals.aoc_ios_ppg1 = 0xFF,
        .reg_vals.aoc_ios_ppg2 = 0xFF,
        .reg_vals.aoc_ios_ppg3 = 0xFF,
        .reg_vals.aoc_ios_ppg4 = 0xFF,
        .reg_vals.aoc_ios_ppg5 = 0xFF,
        .reg_vals.aoc_ios_ppg6 = 0xFF,
        .reg_vals.aoc_ios_ppg7 = 0xFF,
        .reg_vals.aoc_ios_ppg8 = 0xFF,
        .reg_vals.aoc_ppg_thh = 0xB9,
        .reg_vals.aoc_ppg_thl = 0x82,
        .reg_vals.aoc_ios_ecg = 0xFF,
        .reg_vals.aoc_ecg_thh = 0xB9,
        .reg_vals.aoc_ecg_thl = 0x82,
        .reg_vals.aoc_ecg_cfg = 0x00,
        .reg_vals.aoc_ios_ledoff = 0xFF,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_AOC, (uint8_t *)aoc_config.reg_buffer, sizeof(as7050_config_aoc_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_AOC: %d\n", result);
        goto ERROR;
    }

    // configuration of CTRL group
    const as7050_config_ctrl_t ctrl_config = {
        .reg_vals.control = 0x01,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
        goto ERROR;
    }

    // configuration of ECG group
    const as7050_config_ecg_t ecg_config = {
        .reg_vals.ecg_source = 0x50,
        .reg_vals.ecg_mod_cfga = 0x90,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
        goto ERROR;
    }

    // configuration of FIFO group
    const as7050_config_fifo_t fifo_config = {
        .reg_vals.fifo_ctrl = 0x00,
        .reg_vals.fifo_threshold = 0x18,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
        goto ERROR;
    }

    // configuration of GPIO group
    const as7050_config_gpio_t gpio_config = {
        .reg_vals.gpio1_cfg = 0x00,
        .reg_vals.gpio2_cfg = 0x00,
        .reg_vals.gpio1_cfgb = 0x00,
        .reg_vals.gpio2_cfgb = 0x00,
        .reg_vals.gpio_io = 0x00,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
        goto ERROR;
    }

    // configuration of IIR group
    const as7050_config_iir_t iir_config = {
        .reg_vals.iir_cfg = 0x05,
        /* iir_coeff_data_sos will not be initialized here */
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_IIR, (uint8_t *)iir_config.reg_buffer, sizeof(as7050_config_iir_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_IIR: %d\n", result);
        goto ERROR;
    }

    // configuration of LED group
    const as7050_config_led_t led_config = {
        .reg_vals.lowvds_wait = 0x00,
        .reg_vals.led1_ictrl = 0x03,
        .reg_vals.led2_ictrl = 0x03,
        .reg_vals.led3_ictrl = 0x00,
        .reg_vals.led4_ictrl = 0x00,
        .reg_vals.led5_ictrl = 0x00,
        .reg_vals.led6_ictrl = 0x00,
        .reg_vals.led7_ictrl = 0x00,
        .reg_vals.led8_ictrl = 0x00,
        .reg_vals.led_init = 0x0A,
        .reg_vals.led_ppg1 = 0x03,
        .reg_vals.led_ppg2 = 0x00,
        .reg_vals.led_ppg3 = 0x00,
        .reg_vals.led_ppg4 = 0x00,
        .reg_vals.led_ppg5 = 0x00,
        .reg_vals.led_ppg6 = 0x00,
        .reg_vals.led_ppg7 = 0x00,
        .reg_vals.led_ppg8 = 0x00,
        .reg_vals.led_tia = 0x00,
        .reg_vals.led_mode = 0x0F,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
        goto ERROR;
    }

    // configuration of PD group
    const as7050_config_pd_t pd_config = {
        .reg_vals.pdsel_cfg = 0x00,
        .reg_vals.pd_ppg1 = 0x02,
        .reg_vals.pd_ppg2 = 0x00,
        .reg_vals.pd_ppg3 = 0x00,
        .reg_vals.pd_ppg4 = 0x00,
        .reg_vals.pd_ppg5 = 0x00,
        .reg_vals.pd_ppg6 = 0x00,
        .reg_vals.pd_ppg7 = 0x00,
        .reg_vals.pd_ppg8 = 0x00,
        .reg_vals.pd_tia = 0x00,
    };
    result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
        goto ERROR;
    }

    // configuration of PPG group
    const as7050_config_ppg_t ppg_config = {
        .reg_vals.ppg_mod_cfga = 0xC7,
        .reg_vals.ppg_mod_cfgb = 0x00,
        .reg_vals.ppg_mod_cfgc = 0x07,
        .reg_vals.ppg_mod_cfgd = 0x04,
        .reg_vals.ppg_mod_cfge = 0x1F,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
        goto ERROR;
    }

    // configuration of REF group
    const as7050_config_ref_t ref_config = {
        .reg_vals.ref_cfga = 0xFC,
        .reg_vals.ref_cfgb = 0x02,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_REF, (uint8_t *)ref_config.reg_buffer, sizeof(as7050_config_ref_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_REF: %d\n", result);
        goto ERROR;
    }

    // configuration of SEQ group
    const as7050_config_seq_t seq_config = {
        .reg_vals.cgb_cfg = 0x07,
        .reg_vals.seq_sample = 0x54,
        .reg_vals.seq_ppga = 0x00,
        .reg_vals.seq_ppgb = 0x00,
        .reg_vals.seq_mode = 0xA0,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);
        goto ERROR;
    }

    // configuration of SINC group
    const as7050_config_sinc_t sinc_config = {
        .reg_vals.sinc_ppg_cfga = 0x84,
        .reg_vals.sinc_ppg_cfgb = 0x03,
        .reg_vals.sinc_ppg_cfgc = 0x00,
        .reg_vals.sinc_ecg_cfga = 0x84,
        .reg_vals.sinc_ecg_cfgb = 0x03,
        .reg_vals.sinc_ecg_cfgc = 0x00,
        .reg_vals.ovs_cfg = 0x00,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_SINC, (uint8_t *)sinc_config.reg_buffer, sizeof(as7050_config_sinc_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_SINC: %d\n", result);
        goto ERROR;
    }

    // configuration of STANDBY group
    const as7050_config_standby_t standby_config = {
        .reg_vals.standby_cfga = 0x00,
        .reg_vals.standby_cfgb = 0x00,
    };
    result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                  sizeof(as7050_config_standby_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
        goto ERROR;
    }

    // configuration of TIA group
    const as7050_config_tia_t tia_config = {
        .reg_vals.pd_offset_cfg = 0x04,
        .reg_vals.tia_cfga = 0x00,
        .reg_vals.tia_cfgb = 0x00,
        .reg_vals.tia_cfgc = 0x00,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
        goto ERROR;
    }

    // (2) Disable automatic gain control (AGC)
    // See as7050_typedefs.h: as7050_agc_config_t
    const as7050_agc_config_t agc_config = {
        .mode = AS7050_AGC_MODE_DISABLED,
        .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_DISABLED,
        .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_DISABLED,
        .current_max[AS7050_CHANNEL_GROUP_A] = 0,
        .current_min[AS7050_CHANNEL_GROUP_A] = 0,
        .current_max[AS7050_CHANNEL_GROUP_B] = 0,
        .current_min[AS7050_CHANNEL_GROUP_B] = 0,
        .threshold_max = 0,
        .threshold_min = 0,
        .sample_cnt = 1,
        .reset_interval = 1,
    };
    result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_set_agc_config: %d\n", result);
        goto ERROR;
    }

    // ------------------------------------------------------------------------
    // Prepare measurement

    // (1) Get FIFO map and sample periods from Chip Library measurement config
    as7050_meas_config_t meas_config;
    result = as7050_get_measurement_config(&meas_config);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_get_measurement_config: %d\n", result);
        goto ERROR;
    }
    g_fifo_map = meas_config.fifo_map;
    g_ppg_sample_period_s = (double)meas_config.ppg_sample_period_us / 1000 / 1000;
    g_ecg_sample_period_s = (double)meas_config.ecg_sample_period_us / 1000 / 1000;

    // ------------------------------------------------------------------------
    // Start measurement

    // (1) Start the measurement on the Chip Library
    result = as7050_start_measurement();
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_start_measurement: %d\n", result);
        goto ERROR;
    }

    // ------------------------------------------------------------------------
    // Execution
    // *** Example: Data will be acquired until Ctrl-C pressed.

    printf("\nPress 'Ctrl-C' to stop the measurement!\n\n");

    while (g_keep_running) {
        // Do nothing
        // Data is processed in as7050_callback
    }

    // ------------------------------------------------------------------------
    // Deinitialization

    // (1) Stop measurement on the Chip Library
    result = as7050_stop_measurement();
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_stop_measurement: %d\n", result);
    }

ERROR:

    printf("\n*** Deinitialization\n");

    // (2) Close the Chip Library
    as7050_shutdown();

    return result;
}
