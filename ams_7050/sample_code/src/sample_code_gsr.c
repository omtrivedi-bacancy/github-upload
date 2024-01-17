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
This sample code shows how the AS7050 Chip Library and the AS7050 Application
Manager can be used to perform Galvanic Skin Resistance (GSR) measurements.
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
#include "gsr_app_typedefs.h"

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

    // These are unused arguments, mark them to avoid failures on static code analysis.
    M_UNUSED_PARAM(p_agc_status);
    M_UNUSED_PARAM(p_cb_param);

    // Handle the error argument.
    if (ERR_SUCCESS != error) {
        result = error;
    }

    // Transfer the ADC data from the Chip Library to the Application Manager.
    if (ERR_SUCCESS == result) {
        result = as7050_appmgr_set_input(p_data, data_num, (as7050_appmgr_chip_status_t){}, NULL, 0, NULL, 0,
                                         &ready_for_execution);
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

// We assume there is an as7050_sample_code_gsr.exe and the AS7050 EVK board is
// connected to port COM7.
// Call it this way:
// > as7050_sample_code_gsr.exe --interface COM:COM7

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
    // *** example: GSR
    // *** The configuration below works together with the AS7050 EVK board.

    // (1) Set the configuration register
    // See as7050_typedefs.h: enum as7050_reg_group_ids
    // See the datasheet of the sensor.

    // configuration of AFE group
    const as7050_config_afe_t afe_config = {
        .reg_vals.afe_dac0l = 0x80,
        .reg_vals.afe_dac1l = 0x40,
        .reg_vals.afe_dach = 0x5A,
        .reg_vals.afe_cfga = 0xB1,
        .reg_vals.afe_cfgb = 0x17,
        .reg_vals.afe_gsr = 0x12,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_AFE, (uint8_t *)afe_config.reg_buffer, sizeof(as7050_config_afe_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_AFE: %d\n", result);
        goto ERROR;
    }

    // configuration of AMP group
    const as7050_config_amp_t amp_config = {
        .reg_vals.ecg_amp_cfga = 0x82,
        .reg_vals.ecg_amp_cfgb = 0xAB,
        .reg_vals.ecg_amp_cfgc = 0xB4,
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
        .reg_vals.aoc_ios_ppg1 = 0x00,
        .reg_vals.aoc_ios_ppg2 = 0x00,
        .reg_vals.aoc_ios_ppg3 = 0x00,
        .reg_vals.aoc_ios_ppg4 = 0x00,
        .reg_vals.aoc_ios_ppg5 = 0x00,
        .reg_vals.aoc_ios_ppg6 = 0x00,
        .reg_vals.aoc_ios_ppg7 = 0x00,
        .reg_vals.aoc_ios_ppg8 = 0x00,
        .reg_vals.aoc_ppg_thh = 0xFF,
        .reg_vals.aoc_ppg_thl = 0x00,
        .reg_vals.aoc_ios_ecg = 0x00,
        .reg_vals.aoc_ecg_thh = 0x00,
        .reg_vals.aoc_ecg_thl = 0xFF,
        .reg_vals.aoc_ecg_cfg = 0x00,
        .reg_vals.aoc_ios_ledoff = 0x00,
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
        .reg_vals.ecg_source = 0x62,
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
        .reg_vals.fifo_ctrl = 0x01,
        .reg_vals.fifo_threshold = 0x05,
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
        .reg_vals.gpio1_cfgb = 0x05,
        .reg_vals.gpio2_cfgb = 0x05,
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
        .reg_vals.lowvds_wait = 0x00,
        .reg_vals.led1_ictrl = 0x00,
        .reg_vals.led2_ictrl = 0x01,
        .reg_vals.led3_ictrl = 0x00,
        .reg_vals.led4_ictrl = 0x00,
        .reg_vals.led5_ictrl = 0x00,
        .reg_vals.led6_ictrl = 0x00,
        .reg_vals.led7_ictrl = 0x00,
        .reg_vals.led8_ictrl = 0x00,
        .reg_vals.led_init = 0x0A,
        .reg_vals.led_ppg1 = 0x02,
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
        .reg_vals.ppg_mod_cfga = 0xC4,
        .reg_vals.ppg_mod_cfgb = 0x00,
        .reg_vals.ppg_mod_cfgc = 0x0F,
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
        .reg_vals.ref_cfga = 0xD5,
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
        .reg_vals.seq_sample = 0x01,
        .reg_vals.seq_ppga = 0x80,
        .reg_vals.seq_ppgb = 0x00,
        .reg_vals.seq_mode = 0x20,
    };
    result =
        as7050_set_reg_group(AS7050_REG_GROUP_ID_SEQ, (uint8_t *)seq_config.reg_buffer, sizeof(as7050_config_seq_t));
    if (ERR_SUCCESS != result) {
        printf("Error on AS7050_REG_GROUP_ID_SEQ: %d\n", result);
        goto ERROR;
    }

    // configuration of SINC group
    const as7050_config_sinc_t sinc_config = {
        .reg_vals.sinc_ppg_cfga = 0x00,
        .reg_vals.sinc_ppg_cfgb = 0x01,
        .reg_vals.sinc_ppg_cfgc = 0x00,
        .reg_vals.sinc_ecg_cfga = 0x0C,
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

    // ------------------------------------------------------------------------
    // Get DAC reference value, which is chip-specific and required for correct
    // calculation of the GSR resistance value

    // (1) Call the DAC reference calculation function repeately while the DAC
    //     DAC reference calculation is running.
    dac_ref_status_t dac_ref_status = DAC_REF_STATUS_RUNNING;
    uint32_t dac_ref;
    while (DAC_REF_STATUS_RUNNING == dac_ref_status) {
        result = as7050_calculate_dac_reference_value(DAC_REF_CONTROL_CALC, 20, &dac_ref_status, &dac_ref);
        if (ERR_SUCCESS != result) {
            printf("Error on as7050_calculate_dac_reference_value: %d\n", result);
            goto ERROR;
        }
        if (DAC_REF_STATUS_RUNNING == dac_ref_status) {
            usleep(100000);
        }
    }

    // (2) Check that the DAC reference calculation stopped because it finished
    if (dac_ref_status != DAC_REF_STATUS_FINISHED) {
        printf("DAC reference calculation stopped with unexpected status\n");
        goto ERROR;
    }

    printf("\nDAC Reference Value: %u\n", dac_ref);

    // ------------------------------------------------------------------------
    // Configuration of the Application Manager

    // (1) Set the routing of the ADC input data to the Vital Signs applications
    // *** For the GSR app, the ECG channel (AS7050_CHANNEL_TIA) always needs to be used.
    as7050_appmgr_channel_id_t gsr_channel = AS7050_CHANNEL_TIA;
    result = as7050_appmgr_set_signal_routing(AS7050_APPMGR_APP_ID_GSR, &gsr_channel, 1);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_set_signal_routing: %d\n", result);
        goto ERROR;
    }

    // (2) Enable the application(s)
    // *** For this example the application 'GSR' is enabled.
    result = as7050_appmgr_enable_apps(AS7050_APPMGR_APP_FLAG_GSR);
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_appmgr_enable_apps: %d\n", result);
        goto ERROR;
    }

    // (3) Configure GSR application using the DAC reference value obtained before
    gsr_app_configuration_t gsr_config = {
        .dac_ref = dac_ref,
    };
    result = as7050_appmgr_configure_app(AS7050_APPMGR_APP_ID_GSR, &gsr_config, sizeof(gsr_config));
    if (ERR_SUCCESS != result) {
        printf("Error on as7050_app_spo2_configure: %d\n", result);
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
    // - Channels for which AGC is enabled (AGC is not used in this example)
    // After calling this function, the Application Manager is ready to receive measurement data and to process it.
    // While in processing state, the configuration of the Application Manager and its applications can not be changed.
    // To leave the processing state, as7050_appmgr_stop_processing needs to be called.
    result = as7050_appmgr_start_processing(meas_config, 1, NULL, 0);
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

    gsr_app_output_t gsr_output;
    // Holds the flags of Vital Signs applications with new output data
    // available.
    uint32_t app_data_available;

    // *** Example: The GSR measurement will be performed until Ctrl-C pressed.
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
            usleep(100000);
        }

        // (2) Get the output data from the Vital Signs applications
        if (AS7050_APPMGR_APP_FLAG_GSR & app_data_available) {
            // The GSR application has new output data available.
            uint16_t output_size = sizeof(gsr_app_output_t);
            result = as7050_appmgr_get_output(AS7050_APPMGR_APP_ID_GSR, &gsr_output, &output_size);
            if (ERR_SUCCESS != result) {
                printf("Error on as7050_appmgr_get_output: %d\n", result);
                goto ERROR;
            }

            // Check if the GSR result is valid
            if (gsr_output.resistor != UINT32_MAX) {
                // (3) Do something with the Vital Signs application output data ...
                printf("Loop %2d - GSR: %d Ohm\n", i, gsr_output.resistor);
            } else {
                // (3) Signal invalid data
                printf("Loop %2d - GSR: Invalid\n", i);
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
