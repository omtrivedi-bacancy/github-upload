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
This sample code shows how the AS7050 Chip Library and the
AS7050 Application Manager can be connected and work together.
The interface to the Operation System Abstraction Layer (OSAL) is included.
*/

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <signal.h>
#include <stdio.h>
#include <string.h>
//#include <unistd.h>

#include "as7050_app_manager.h"
#include "as7050_chiplib.h"
#include "as7050_std_include.h"
#include "bio_spo2_a0_typedefs.h"

#include "error_codes.h"

/******************************************************************************
 *                                  DEFINITIONS                               *
 ******************************************************************************/

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

// Define a flag to handle execution of the Vital Signs Application.
// This flag indicates when the Application Manager is ready for execution.
static uint8_t g_ready_for_execution = 0;

// This flag keeps the demo application running. It is changed to 0 when a
// SIGINT occurs (Ctrl+C)
static volatile uint8_t g_keep_running = 1;

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

// Define a callback function for the Chip Library. This function will be
// called by the Chip Library when new measurement data is available.
// See as7050_typedefs.h: as7050_callback_t
static void as7050_callback(err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param)
{
    err_code_t result = ERR_SUCCESS;
    uint32_t ready_for_execution = 0;

    // This is an unused argument, mark it to avoid failures on static code analysis.
    M_UNUSED_PARAM(p_cb_param);

    // Handle the error argument.
    if (ERR_SUCCESS != error) {
        result = error;
    }

    // Transfer the ADC data and the AGC status from the Chip Library to the Application Manager.
    if (ERR_SUCCESS == result) {
        // Chip Library and Application Manager use different structures for AGC status information.
        // This conversion code assumes that dual-channel AGC is used.
        bio_agc_status_t bio_agc_statuses[2];
        bio_agc_statuses[0].led_change = p_agc_status->led_change[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].led_current = p_agc_status->led_current[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].pd_offset_change = p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[0].pd_offset_current = p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_A];
        bio_agc_statuses[1].led_change = p_agc_status->led_change[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].led_current = p_agc_status->led_current[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].pd_offset_change = p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_B];
        bio_agc_statuses[1].pd_offset_current = p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_B];

        result = as7050_appmgr_set_input(p_data, data_num, (as7050_appmgr_chip_status_t){}, bio_agc_statuses, 2, NULL,
                                         0, &ready_for_execution);
    }

    if (ERR_SUCCESS == result) {
        // The Application Manager indicates if it is ready for execution
        // every time new input data is provided. This can be used in a real
        // implementation to efficiently signal an Application Manager
        // process to execute when it is ready for execution.
        // Set the 'ready for execution' flag.
        if (ready_for_execution && !g_ready_for_execution) {
            g_ready_for_execution = 1;
        }
    } else {
        // handle errors from Application Manager
    }
}

/******************************************************************************
 *                               Main                                         *
 ******************************************************************************/

// We assume there is an as7050_sample_code_spo2.exe and the AS7050 EVK board is
// connected to port COM7.
// Call it this way:
// > as7050_sample_code_spo2.exe --interface COM:COM7

int main(int argc, char *argv[])
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

    // (2) Initialize the Application Manager.
    result = as7050_appmgr_initialize();
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_initialize: %d\n", result);
        goto ERROR;
    }

    // ------------------------------------------------------------------------
    // Configuration of the Chip Library
    // *** example: SPO2, with AGC
    // *** The configuration below works together with the AS7050 EVK board.

    // (1) Set the configuration register
    // See as7050_typedefs.h: enum as7050_reg_group_ids
    // See the datasheet of the sensor.

    // configuration of AFE group
    const as7050_config_afe_t afe_config = {
        .reg_vals.afe_dac0l = 0,
        .reg_vals.afe_dac1l = 0,
        .reg_vals.afe_dach = 0,
        .reg_vals.afe_cfga = 0,
        .reg_vals.afe_cfgb = 0,
        .reg_vals.afe_gsr = 0,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
        goto ERROR;
    }

    // configuration of AMP group
    const as7050_config_amp_t amp_config = {
        .reg_vals.ecg_amp_cfga = 0,
        .reg_vals.ecg_amp_cfgb = 0x23,
        .reg_vals.ecg_amp_cfgc = 0x30,
        .reg_vals.ecg_amp_cfge = 0x01,
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
        .reg_vals.aoc_ppg_cfg = 0,
        .reg_vals.aoc_ios_ecg = 0,
        .reg_vals.aoc_ecg_thh = 0xB9,
        .reg_vals.aoc_ecg_thl = 0x82,
        .reg_vals.aoc_ecg_cfg = 0,
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
        .reg_vals.control = 1,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_CTRL, (uint8_t *)ctrl_config.reg_buffer, sizeof(as7050_config_ctrl_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_CTRL: %d\n", result);
        goto ERROR;
    }

    // configuration of ECG group
    const as7050_config_ecg_t ecg_config = {
        .reg_vals.ecg_source = 0,
        .reg_vals.ecg_mod_cfga = 0,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_ECG, (uint8_t *)ecg_config.reg_buffer, sizeof(as7050_config_ecg_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_ECG: %d\n", result);
        goto ERROR;
    }

    // configuration of FIFO group
    const as7050_config_fifo_t fifo_config = {
        .reg_vals.fifo_ctrl = 0,
        .reg_vals.fifo_threshold = 2,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_FIFO, (uint8_t *)fifo_config.reg_buffer, sizeof(as7050_config_fifo_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_FIFO: %d\n", result);
        goto ERROR;
    }

    // configuration of GPIO group
    const as7050_config_gpio_t gpio_config = {
        .reg_vals.gpio1_cfg = 0,
        .reg_vals.gpio2_cfg = 0,
        .reg_vals.gpio1_cfgb = 0,
        .reg_vals.gpio2_cfgb = 0,
        .reg_vals.gpio_io = 0,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_GPIO, (uint8_t *)gpio_config.reg_buffer, sizeof(as7050_config_gpio_t));
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_REG_GROUP_ID_GPIO: %d\n", result);
        goto ERROR;
    }

    // configuration of IIR group
    const as7050_config_iir_t iir_config = {
        .reg_vals.iir_cfg = 0,
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
        .reg_vals.lowvds_wait = 0,
        .reg_vals.led1_ictrl = 0,
        .reg_vals.led2_ictrl = 0,
        .reg_vals.led3_ictrl = 0,
        .reg_vals.led4_ictrl = 0,
        .reg_vals.led5_ictrl = 0,
        .reg_vals.led6_ictrl = 0,
        .reg_vals.led7_ictrl = 0,
        .reg_vals.led8_ictrl = 0,
        .reg_vals.led_init = 10,
        .reg_vals.led_ppg1 = 4,
        .reg_vals.led_ppg2 = 8,
        .reg_vals.led_ppg3 = 0,
        .reg_vals.led_ppg4 = 0,
        .reg_vals.led_ppg5 = 0,
        .reg_vals.led_ppg6 = 0,
        .reg_vals.led_ppg7 = 0,
        .reg_vals.led_ppg8 = 0,
        .reg_vals.led_tia = 0,
        .reg_vals.led_mode = 15,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_LED, (uint8_t *)led_config.reg_buffer, sizeof(as7050_config_led_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_LED: %d\n", result);
        goto ERROR;
    }

    // configuration of PD group
    const as7050_config_pd_t pd_config = {
        .reg_vals.pdsel_cfg = 0,
        .reg_vals.pd_ppg1 = 1,
        .reg_vals.pd_ppg2 = 1,
        .reg_vals.pd_ppg3 = 1,
        .reg_vals.pd_ppg4 = 0,
        .reg_vals.pd_ppg5 = 0,
        .reg_vals.pd_ppg6 = 0,
        .reg_vals.pd_ppg7 = 0,
        .reg_vals.pd_ppg8 = 0,
        .reg_vals.pd_tia = 0,
    };
    result = as7050_set_reg_group(AS7050_REG_GROUP_ID_PD, (uint8_t *)pd_config.reg_buffer, sizeof(as7050_config_pd_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_PD: %d\n", result);
        goto ERROR;
    }

    // configuration of PPG group
    const as7050_config_ppg_t ppg_config = {
        .reg_vals.ppg_mod_cfga = 0xC6,
        .reg_vals.ppg_mod_cfgb = 0,
        .reg_vals.ppg_mod_cfgc = 0x07,
        .reg_vals.ppg_mod_cfgd = 0x04,
        .reg_vals.ppg_mod_cfge = 0x0F,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_PPG, (uint8_t *)ppg_config.reg_buffer, sizeof(as7050_config_ppg_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_PPG: %d\n", result);
        goto ERROR;
    }

    // configuration of REF group
    const as7050_config_ref_t ref_config = {
        .reg_vals.ref_cfga = 0xAC,
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
        .reg_vals.seq_sample = 0x64,
        .reg_vals.seq_ppga = 0x02,
        .reg_vals.seq_ppgb = 0x01,
        .reg_vals.seq_mode = 0x80,
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
        .reg_vals.sinc_ecg_cfga = 0x00,
        .reg_vals.sinc_ecg_cfgb = 0x01,
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
        .reg_vals.standby_cfga = 0x35,
        .reg_vals.standby_cfgb = 0x01,
    };
    result = as7050_set_reg_group(AS7050_REG_GROUP_ID_STANDBY, (uint8_t *)standby_config.reg_buffer,
                                  sizeof(as7050_config_standby_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_STANDBY: %d\n", result);
        goto ERROR;
    }

    // configuration of TIA group
    const as7050_config_tia_t tia_config = {
        .reg_vals.pd_offset_cfg = 4,
        .reg_vals.tia_cfga = 0,
        .reg_vals.tia_cfgb = 0,
        .reg_vals.tia_cfgc = 0,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_TIA, (uint8_t *)tia_config.reg_buffer, sizeof(as7050_config_tia_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_TIA: %d\n", result);
        goto ERROR;
    }

    // (2) Set automatic gain control (AGC)
    // See as7050_typedefs.h: as7050_agc_config_t
    const as7050_agc_config_t agc_config = {
        // Set the AGC mode - it presents the used algorithm
        // *** For this example the mode is set to dual channel PPG.
        // See as7050_typedefs.h: as7050_agc_mode_t
        .mode = AS7050_AGC_MODE_PPG_TWO_CHANNEL,
        // Set channel group configuration
        .channel[AS7050_CHANNEL_GROUP_A] = AS7050_CHANNEL_PPG_1,
        .channel[AS7050_CHANNEL_GROUP_B] = AS7050_CHANNEL_PPG_2,
        // Set minimum and maximum for channel group A.
        // For dual channel PPG mode the channel A LED current is fixed to current_min
        .current_max[AS7050_CHANNEL_GROUP_A] = 25,
        .current_min[AS7050_CHANNEL_GROUP_A] = 25,
        // Set minimum and maximum for channel group B.
        .current_max[AS7050_CHANNEL_GROUP_B] = 37,
        .current_min[AS7050_CHANNEL_GROUP_B] = 8,
        // Set minium and maximum threshold of the ADC signals.
        .threshold_max = 400000,
        .threshold_min = 110000,
        // Set number of samples to average for calculating the signal's mean.
        .sample_cnt = 20,
        // Set interval in milliseconds to reset temporary AGC parameters like
        // min/max peak signals.
        .reset_interval = 4000,
    };
    result = as7050_set_agc_config((as7050_agc_config_t *)&agc_config);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_set_agc_config: %d\n", result);
        goto ERROR;
    }

    // ------------------------------------------------------------------------
    // Configuration of the Application Manager

    // (1) Set the routing of the ADC input data to the Vital Signs applications
    // *** For that example the following ADC Channels are used for SpO2:
    //     red led --> PPG1
    //     ir led --> PPG2
    //     ambient light --> PPG3
    as7050_appmgr_channel_id_t spo2_channels[BIO_SPO2_A0_SIGNAL_NUM] = {
        [BIO_SPO2_A0_SIGNAL_PPG_RED] = AS7050_CHANNEL_PPG_1,
        [BIO_SPO2_A0_SIGNAL_PPG_IR] = AS7050_CHANNEL_PPG_2,
        [BIO_SPO2_A0_SIGNAL_AMBIENT] = AS7050_CHANNEL_PPG_3,
    };
    result = as7050_appmgr_set_signal_routing(AS7050_APPMGR_APP_ID_SPO2_A0, spo2_channels, BIO_SPO2_A0_SIGNAL_NUM);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_set_signal_routing: %d\n", result);
        goto ERROR;
    }

    // (2) Enable the application(s)
    // *** For this example the application 'SPO2' is enabled.
    result = as7050_appmgr_enable_apps(AS7050_APPMGR_APP_FLAG_SPO2_A0);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_enable_apps: %d\n", result);
        goto ERROR;
    }

    // (3) Configure SPO2 application
    // Note: The SpO2 calibration parameters are optical stack dependent.
    bio_spo2_a0_configuration_t spo2_config = {
        .a = 0,
        .b = 2843,
        .c = 11313,
        .dc_comp_red = 1916,
        .dc_comp_ir = 1916,
    };
    result = as7050_appmgr_configure_app(AS7050_APPMGR_APP_ID_SPO2_A0, &spo2_config, sizeof(spo2_config));
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_configure_app: %d\n", result);
        goto ERROR;
    }

    // (4) Get the measurement info from the Chip Library
    as7050_meas_config_t meas_config;
    result = as7050_get_measurement_config(&meas_config);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_get_measurement_config: %d\n", result);
        goto ERROR;
    }

    // (5) Set the Application Manager to processing state
    // The following information is provided to the Application Manager
    // - Chip Library measurement configuration
    // - Accelerometer sample period (Accelerometer is not used in this example, set to 1 since 0 is an invalid value)
    // - Channels for which AGC is enabled
    // After calling this function, the Application Manager is ready to receive measurement data and to process it.
    // While in processing state, the configuration of the Application Manager and its applications can not be changed.
    // To leave the processing state, as7050_appmgr_stop_processing needs to be called.
    result = as7050_appmgr_start_processing(meas_config, 1, agc_config.channel, 2);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_start_processing: %d\n", result);
        goto ERROR;
    }

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

    bio_spo2_a0_output_t spo2_output;
    // Holds the flags of Vital Signs applications with new output data
    // available.
    uint32_t app_data_available;

    // *** Example: The SpO2 will be measured until Ctrl-C pressed.
    printf("\nPress 'Ctrl-C' to stop the measurement!\n\n");

    i = 0;
    while (g_keep_running) {
        // (1) Execute the Application Manager if flag 'ready for execution'
        //     was set by the Chip Library's callback function
        if (g_ready_for_execution) {
            result = as7050_appmgr_execute(&app_data_available);
            if (ERR_SUCCESS != result) {
                printf("Error on as7050_appmgr_execute: %d\n", result);
                goto ERROR;
            }
            g_ready_for_execution = FALSE;
        } else {
            app_data_available = FALSE;
            //usleep(100000);
        }

        // (2) Get the output data from the Vital Signs applications
        if (AS7050_APPMGR_APP_FLAG_SPO2_A0 & app_data_available) {
            // The SPO2 application has new output data available.
            uint16_t output_size = sizeof(bio_spo2_a0_output_t);
            result = as7050_appmgr_get_output(AS7050_APPMGR_APP_ID_SPO2_A0, &spo2_output, &output_size);
            if (ERR_SUCCESS != result) {
                printf("Error on as7050_appmgr_get_output: %d\n", result);
                goto ERROR;
            }

            // Check if the SpO2 result is valid
            if (!spo2_output.status) {
                // (3) Do something with the Vital Signs application output data ...
                printf("Loop %2d - SpO2: %.0f %%,\t heart rate: %.0f bpm,\t quality: %d %%\n", i,
                       (float)spo2_output.spo2 / 100, (float)spo2_output.heart_rate / 10, spo2_output.quality);
            } else {
                // (3) Signal invalid data
                printf("Loop %2d - invalid SPO2 , wait for more data\n", i);
            }

            i++;
        }
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

    // (2) Close the Application Manager
    as7050_appmgr_shutdown();

    // (3) Close the Chip Library
    as7050_shutdown();

    return result;
}
