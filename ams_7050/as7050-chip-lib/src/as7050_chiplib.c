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

#include "as7050_chiplib.h"
#include "as7050_extract.h"
#include "as7050_interface.h"
#include "as7050_osal_chiplib.h"
#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "as7050_version.h"
#include "error_codes.h"
#include "common.h"
#include "nrf_drv_gpiote.h"
#include <sys/time.h>

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#define AOC_PPG_CFG__PPG_AOC_ON 0x80
#define AFE_DACH__AFE_DAC_SEL 0x80
#define AFE_CFGA__AFE_GPIO_GST_IN 0x60
#define AFE_CFGA__AFE_MEAS_DAC 0x0C
#define AFE_CFGA__AFE_GPIO_GST_IN_OPEN 0x00
#define AFE_CFGA__AFE_MEAS_DAC_WITH_BUFFER 0x08

#define DAC_REF_MAX_DATA_SIZE 100
#define DAC_REF_MAX_SAMPLES (DAC_REF_MAX_DATA_SIZE / 4)

#define AS7050_CHANNEL_FLAG_ALL_PPG                                                                                    \
    (AS7050_CHANNEL_FLAG_PPG_1 | AS7050_CHANNEL_FLAG_PPG_2 | AS7050_CHANNEL_FLAG_PPG_3 | AS7050_CHANNEL_FLAG_PPG_4 |   \
     AS7050_CHANNEL_FLAG_PPG_5 | AS7050_CHANNEL_FLAG_PPG_6 | AS7050_CHANNEL_FLAG_PPG_7 | AS7050_CHANNEL_FLAG_PPG_8)

enum LIB_STATES {
    LIB_STATE_UNINITIALIZED = 0,
    LIB_STATE_CONFIGURATION = 1,
    LIB_STATE_MEASUREMENT = 2,
    LIB_STATE_DAC_REF = 3,
};

enum DAC_REF_FSM_STATES {
    DAC_REF_FSM_STATES_IDLE = 0,
    DAC_REF_FSM_STATES_START,
    DAC_REF_FSM_STATES_CONF_0,
    DAC_REF_FSM_STATES_WAIT_0,
    DAC_REF_FSM_STATES_POST_0,
    DAC_REF_FSM_STATES_CONF_1,
    DAC_REF_FSM_STATES_WAIT_1,
    DAC_REF_FSM_STATES_POST_1,
    DAC_REF_FSM_STATES_FINALIZE,
    DAC_REF_FSM_STATES_ABORT,
};

struct device_config {
    as7050_callback_t p_callback;
    void *p_cb_params;
    enum LIB_STATES lib_state;
    as7050_meas_config_t meas_config;
    volatile uint8_t is_meas_running;
};

struct dac_ref_meas {
    as7050_config_afe_t afe_config_backup;
    as7050_callback_t p_callback_backup;
    volatile uint8_t meas_data[DAC_REF_MAX_DATA_SIZE];
    volatile uint16_t meas_data_fillsize;
    uint16_t fill_size_threshold;
    volatile err_code_t callback_error;
    uint8_t state;
    uint32_t dac_averages[2];
};

#define REG_BITS_STATUS_IRQ_FIFOTHRESHOLD 0x04

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

// Static variables are initialized with 0. That means for all members of the device
// config structure: static struct device_config g_dev_config = {NULL, NULL,
// LIB_STATE_UNINITIALIZED, {0, 0, 0, 0, 0}};
static struct device_config g_dev_config;
static struct dac_ref_meas g_dac_ref_config;

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/
void as7050_start(void);

err_code_t interrupt_handler(void)
{
    err_code_t result;
    as7050_interrupt_status_t status;
    uint8_t fifo_data[512];
    uint16_t fifo_level;
    uint16_t fifo_data_size = 0;
    uint32_t ms_tick;
    uint8_t is_retry;
    as7050_agc_status_t agc_status;
    /* Stop measurement again if global flag was already set.
       Read IRQ status register and FIFO anyway to reset interrupt pin */
    if (!g_dev_config.is_meas_running) {
        return ERR_PERMISSION;
    }

    result = as7050_ifce_get_interrupt_status(&status);
    
   do
   { 
    is_retry = FALSE;
    if (ERR_SUCCESS == result) {

        if (status.irq_fifooverflow) {
            result = ERR_OVERFLOW;
        } else if (status.irq_fifothreshold)
        {
            result = as7050_ifce_get_fifo_level(&fifo_level);
            if (ERR_SUCCESS == result) {

                /* Prevent division by zero */
                if (0 == g_dev_config.meas_config.fifo_threshold || 0 == g_dev_config.meas_config.sample_size) {
                    result = ERR_CONFIG;
                } else {

                    /* Calculate read length as multiple of FIFO threshold */
                    fifo_level = (fifo_level / g_dev_config.meas_config.fifo_threshold) *
                                 g_dev_config.meas_config.fifo_threshold;

                    /* Calculate the real data size */
                    fifo_data_size = fifo_level * g_dev_config.meas_config.sample_size;

                    /* We support only 512bytes at maximum */
                    if (sizeof(fifo_data) < fifo_data_size) {
                        fifo_data_size = 0;
                        result = ERR_SIZE;
                    } else {
                        result = as7050_ifce_get_fifo_data(fifo_data, fifo_data_size);
                    }
                }
            }

            if (ERR_SUCCESS == result) {
                result = as7050_agc_get_status(&agc_status);
            }

            if (ERR_SUCCESS == result && g_dev_config.p_callback && g_dev_config.is_meas_running) {
                g_dev_config.p_callback(result, fifo_data, fifo_data_size, &agc_status, g_dev_config.p_cb_params);
            }

            if (ERR_SUCCESS == result) {
                result = time_ms_tick(&ms_tick);
            }
            if (ERR_SUCCESS == result) {
                result = as7050_agc_execute(ms_tick, g_dev_config.meas_config.fifo_map,
                                            g_dev_config.meas_config.sample_size, fifo_data, fifo_data_size);
            }
        }

        }
        
    

     /* Check if interrupt is set again */
            if (ERR_SUCCESS == result) {
                result = as7050_ifce_get_interrupt_status(&status);
                if (ERR_SUCCESS == result && status.irq_fifothreshold) {
                    is_retry = TRUE;
                }
            }
            
        

    } while (is_retry);

    if (ERR_SUCCESS != result && g_dev_config.p_callback) {
        g_dev_config.p_callback(result, NULL, 0, NULL, g_dev_config.p_cb_params);
        as7050_stop_measurement();
    }
   
   
}

static err_code_t get_measurement_config_intern(as7050_meas_config_t *p_meas_config)
{
    err_code_t result;

    M_CHECK_NULL_POINTER(p_meas_config);

    result = as7050_ifce_get_ppg_ecg_sample_periods(&p_meas_config->ppg_sample_period_us,
                                                    &p_meas_config->ecg_sample_period_us);
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_fifo_threshold(&p_meas_config->fifo_threshold, &p_meas_config->sample_size);
    }

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_channel_mapping(&p_meas_config->fifo_map);
    }

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_max_adc_count(&p_meas_config->max_adc_count, &p_meas_config->max_adc_bit_width);
    }

    if (ERR_SUCCESS == result) {
        p_meas_config->reserved = 0;
    }

    return result;
}

static void gsr_callback(err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                         void *p_cb_param)
{
    M_UNUSED_PARAM(p_cb_param);
    M_UNUSED_PARAM(p_agc_status);

    uint16_t copy_size = (sizeof(g_dac_ref_config.meas_data) - g_dac_ref_config.meas_data_fillsize) < data_num
                             ? sizeof(g_dac_ref_config.meas_data) - g_dac_ref_config.meas_data_fillsize
                             : data_num;

    if (ERR_SUCCESS != error) {
        g_dac_ref_config.callback_error = error;
    } else {
        memcpy((uint8_t *)g_dac_ref_config.meas_data + g_dac_ref_config.meas_data_fillsize, p_data, copy_size);
        g_dac_ref_config.meas_data_fillsize += copy_size;
    }
    return;
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t CHIPLIB_DECLDIR as7050_initialize(const as7050_callback_t p_callback, const void *p_cb_param,
                                             const char *p_interface_descr)
{
    err_code_t result;
    uint8_t revision;

    M_CHECK_NULL_POINTER(p_callback);

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_reset_chip();
    }

    if (ERR_SUCCESS == result) {
        g_dev_config.p_callback = p_callback;
        g_dev_config.p_cb_params = (void *)p_cb_param;
    }

    /* Revision lower than 2 needs a workaround to set the PD offset, because it
       can't be set during measurement. In this case, the measurement will be
       stopped, reconfiguration of PD offset and restart of the measurement */
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_revision(&revision);
    }
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_set_pd_offset_workaround(2 > revision);
    }

    if (ERR_SUCCESS == result) {
        result = as7050_agc_init(2 > revision);
    }

    if (ERR_SUCCESS != result) {
        as7050_shutdown();
    } else {
        g_dev_config.lib_state = LIB_STATE_CONFIGURATION;
        g_dev_config.is_meas_running = FALSE;
    }

    return result;
}

void as7050_start(void)
{
    as7050_power_on();
}

err_code_t CHIPLIB_DECLDIR as7050_shutdown(void)
{
    err_code_t result, result_osal;

    if (LIB_STATE_UNINITIALIZED != g_dev_config.lib_state) {
        /* Workaround as otherwise as7050_shutdown cannot be called successfully during DAC reference calculation */
        g_dev_config.lib_state = LIB_STATE_MEASUREMENT;

        result = as7050_stop_measurement();

        if (ERR_SUCCESS == result) {
            result = as7050_ifce_reset_chip();
        }
    } else {
        result = ERR_SUCCESS;
    }

    memset(&g_dev_config, 0, sizeof(g_dev_config));
    memset(&g_dac_ref_config, 0, sizeof(g_dac_ref_config));
    return result;
}

err_code_t CHIPLIB_DECLDIR as7050_set_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t size)
{
    if (LIB_STATE_CONFIGURATION != g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_ifce_set_reg_group(id, p_data, size);
}

err_code_t CHIPLIB_DECLDIR as7050_get_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t *p_size)
{
    if (LIB_STATE_CONFIGURATION != g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_ifce_get_reg_group(id, p_data, p_size);
}

err_code_t CHIPLIB_DECLDIR as7050_write_register(uint8_t reg_addr, uint8_t reg_val)
{
    if (LIB_STATE_UNINITIALIZED == g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_ifce_write_register(reg_addr, reg_val);
}

err_code_t CHIPLIB_DECLDIR as7050_read_register(uint8_t reg_addr, uint8_t *p_reg_val)
{
    if (LIB_STATE_UNINITIALIZED == g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_ifce_read_register(reg_addr, p_reg_val);
}

err_code_t CHIPLIB_DECLDIR as7050_set_agc_config(const as7050_agc_config_t *p_agc_config)
{
    if (LIB_STATE_CONFIGURATION != g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_agc_set_config(p_agc_config);
}

err_code_t CHIPLIB_DECLDIR as7050_get_agc_config(as7050_agc_config_t *p_agc_config)
{
    if (LIB_STATE_CONFIGURATION != g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    return as7050_agc_get_config(p_agc_config);
}

err_code_t CHIPLIB_DECLDIR as7050_get_measurement_config(as7050_meas_config_t *p_meas_config)
{
    err_code_t result;
    M_CHECK_NULL_POINTER(p_meas_config);

    switch (g_dev_config.lib_state) {

    case LIB_STATE_CONFIGURATION:

        result = get_measurement_config_intern(p_meas_config);
        break;

    case LIB_STATE_MEASUREMENT:

        memcpy(p_meas_config, &g_dev_config.meas_config, sizeof(as7050_meas_config_t));
        result = ERR_SUCCESS;
        break;

    default:

        result = ERR_PERMISSION;
        break;
    }

    return result;
}

err_code_t CHIPLIB_DECLDIR as7050_extract_samples(as7050_channel_flags_t chan, const uint8_t *p_fifo_data,
                                                  uint16_t fifo_data_num, uint32_t *p_chan_data,
                                                  uint16_t *p_chan_data_num)
{
    err_code_t result;
    as7050_meas_config_t meas_config;

    if (LIB_STATE_UNINITIALIZED == g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    result = as7050_get_measurement_config(&meas_config);

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_extract_samples(meas_config.fifo_map, meas_config.sample_size, chan, p_fifo_data,
                                             fifo_data_num, p_chan_data, p_chan_data_num);
    }

    return result;
}

err_code_t time_ms_tick(uint32_t *p_tick_ms)
{
    struct timeval time;

    M_CHECK_NULL_POINTER(p_tick_ms);

    gettimeofday(&time, 0);
    *p_tick_ms = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    return ERR_SUCCESS;
}

err_code_t CHIPLIB_DECLDIR as7050_start_measurement(void)
{
    err_code_t result;
    uint32_t ms_tick;
    as7050_agc_config_t agc_config;
    uint8_t value;

    if (LIB_STATE_CONFIGURATION != g_dev_config.lib_state) {
        return ERR_PERMISSION;
    }

    /* do not allow parallel AGC and AOC */
    result = as7050_agc_get_config(&agc_config);
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_read_register(AS7050_REGADDR_AOC_PPG_CFG, &value);
    }
    if (ERR_SUCCESS == result) {
        if ((AOC_PPG_CFG__PPG_AOC_ON & value) && (AS7050_AGC_MODE_DISABLED != agc_config.mode)) {
            result = ERR_CONFIG;
        }
    }

    if (ERR_SUCCESS == result) {
        result = get_measurement_config_intern(&g_dev_config.meas_config);
    }

    /* reset AGC at first to set inital values for PD offset and LED current
       before measurement starts */
    if (ERR_SUCCESS == result) {
        result = time_ms_tick(&ms_tick);
    }
    if (ERR_SUCCESS == result) {
        result = as7050_agc_reset(ms_tick);
    }
    printf("\nstart  measurement\n");
    if (ERR_SUCCESS == result) {
        g_dev_config.is_meas_running = TRUE;
        result = as7050_ifce_start_measurement();
    }

    if (ERR_SUCCESS == result) {
        g_dev_config.lib_state = LIB_STATE_MEASUREMENT;
    } else {
        as7050_stop_measurement();
    }

    return result;
}

err_code_t CHIPLIB_DECLDIR as7050_stop_measurement(void)
{
    err_code_t result = ERR_DATA_TRANSFER;

    if ((LIB_STATE_UNINITIALIZED == g_dev_config.lib_state) || (LIB_STATE_DAC_REF == g_dev_config.lib_state)) {
        return ERR_PERMISSION;
    }
    printf("\nstop measurement\n");
    g_dev_config.is_meas_running = FALSE;

    result = as7050_ifce_stop_measurement();

    if (ERR_SUCCESS == result) {
        g_dev_config.lib_state = LIB_STATE_CONFIGURATION;
    }

    return result;
}

err_code_t CHIPLIB_DECLDIR as7050_get_version(as7050_version_t *p_version)
{
    M_CHECK_NULL_POINTER(p_version);

    p_version->major = AS7050_VER_MAJOR;
    p_version->minor = AS7050_VER_MINOR;
    p_version->patch = AS7050_VER_PATCH;

    return ERR_SUCCESS;
}

err_code_t CHIPLIB_DECLDIR as7050_calculate_dac_reference_value(dac_ref_control_t control, uint16_t average_cnt,
                                                                dac_ref_status_t *p_status, uint32_t *p_dac_ref)
{
    err_code_t result = ERR_SUCCESS;
    as7050_meas_config_t meas_config;
    as7050_config_afe_t afe_config;
    uint8_t config_size;
    uint32_t dac_samples[DAC_REF_MAX_SAMPLES];
    uint16_t dac_samples_cnt = sizeof(dac_samples) / sizeof(dac_samples[0]);
    uint8_t i;

    if ((LIB_STATE_UNINITIALIZED == g_dev_config.lib_state) || (LIB_STATE_MEASUREMENT == g_dev_config.lib_state)) {
        return ERR_PERMISSION;
    }

    if (!p_status || !p_dac_ref) {
        return ERR_POINTER;
    }

    *p_status = 0;
    *p_dac_ref = 0;

    if (DAC_REF_CONTROL_CALC == control) {
        if (DAC_REF_FSM_STATES_IDLE == g_dac_ref_config.state) {
            g_dac_ref_config.state = DAC_REF_FSM_STATES_START;
        }
    } else if (DAC_REF_CONTROL_ABORT == control) {
        g_dac_ref_config.state = DAC_REF_FSM_STATES_ABORT;
    } else {
        return ERR_ARGUMENT;
    }

    switch (g_dac_ref_config.state) {
    case DAC_REF_FSM_STATES_IDLE:

        /* do nothing */
        result = ERR_SUCCESS;
        break;

    case DAC_REF_FSM_STATES_START:

        if (0 == average_cnt) {
            result = ERR_ARGUMENT;
        } else {
            result = ERR_SUCCESS;
        }

        /* Check if current configuration is compatible with DAC reference measurement */
        if (ERR_SUCCESS == result) {
            result = as7050_get_measurement_config(&meas_config);
        }
        if (ERR_SUCCESS == result) {
            g_dac_ref_config.fill_size_threshold = meas_config.sample_size * average_cnt;
            if (g_dac_ref_config.fill_size_threshold > sizeof(g_dac_ref_config.meas_data)) {
                result = ERR_ARGUMENT;
            } else if (meas_config.fifo_map & AS7050_CHANNEL_FLAG_ALL_PPG) {
                result = ERR_CONFIG;
            } else if (!((meas_config.fifo_map & AS7050_CHANNEL_FLAG_ECG) &&
                         (meas_config.fifo_map & AS7050_CHANNEL_FLAG_GSR))) {
                result = ERR_CONFIG;
            } else if ((meas_config.fifo_threshold * meas_config.sample_size) > sizeof(g_dac_ref_config.meas_data)) {
                result = ERR_CONFIG;
            }
        }

        /* Replace normal callback function with internal GSR callback */
        /* Backup normal registers and overwrite with new values */
        if (ERR_SUCCESS == result) {
            g_dac_ref_config.p_callback_backup = g_dev_config.p_callback;
            g_dev_config.p_callback = gsr_callback;
            config_size = sizeof(g_dac_ref_config.afe_config_backup.reg_buffer);
            result = as7050_ifce_get_reg_group(AS7050_REG_GROUP_ID_AFE, g_dac_ref_config.afe_config_backup.reg_buffer,
                                               &config_size);
        }

        if (ERR_SUCCESS == result) {
            g_dac_ref_config.callback_error = ERR_SUCCESS;
            g_dev_config.lib_state = LIB_STATE_DAC_REF;
            g_dac_ref_config.state++;
        }

        break;

    case DAC_REF_FSM_STATES_CONF_0:
    case DAC_REF_FSM_STATES_CONF_1:

        memcpy(&afe_config, &g_dac_ref_config.afe_config_backup, sizeof(afe_config));
        afe_config.reg_vals.afe_gsr = 0x00;

        /* Disconnect the external circuit from GPIOs to measure internal resistor only. */
        afe_config.reg_vals.afe_cfga &= ~(AFE_CFGA__AFE_GPIO_GST_IN | AFE_CFGA__AFE_MEAS_DAC);
        afe_config.reg_vals.afe_cfga |= (AFE_CFGA__AFE_GPIO_GST_IN_OPEN | AFE_CFGA__AFE_MEAS_DAC_WITH_BUFFER);

        /* Set configuration for DAC value 0 or 1 */
        if (DAC_REF_FSM_STATES_CONF_0 == g_dac_ref_config.state) {
            afe_config.reg_vals.afe_dach &= ~AFE_DACH__AFE_DAC_SEL;
        } else {
            afe_config.reg_vals.afe_dach |= AFE_DACH__AFE_DAC_SEL;
        }
        result =
            as7050_ifce_set_reg_group(AS7050_REG_GROUP_ID_AFE, afe_config.reg_buffer, sizeof(afe_config.reg_buffer));

        /* start measurement */
        if (ERR_SUCCESS == result) {
            g_dac_ref_config.meas_data_fillsize = 0;
            g_dev_config.lib_state = LIB_STATE_CONFIGURATION;
            result = as7050_start_measurement();
            g_dev_config.lib_state = LIB_STATE_DAC_REF;
        }

        if (ERR_SUCCESS == result) {
            g_dac_ref_config.state++;
        }

        break;

    case DAC_REF_FSM_STATES_WAIT_0:
    case DAC_REF_FSM_STATES_WAIT_1:

        /* wait for finished measurements */
        if (g_dac_ref_config.meas_data_fillsize >= g_dac_ref_config.fill_size_threshold) {
            g_dac_ref_config.state++;
        }
        result = g_dac_ref_config.callback_error;

        break;

    case DAC_REF_FSM_STATES_POST_0:
    case DAC_REF_FSM_STATES_POST_1:

        g_dev_config.lib_state = LIB_STATE_MEASUREMENT;
        result = as7050_stop_measurement();
        if (ERR_SUCCESS == result) {
            result = as7050_extract_samples(AS7050_CHANNEL_FLAG_ECG, (uint8_t *)g_dac_ref_config.meas_data,
                                            g_dac_ref_config.meas_data_fillsize, dac_samples, &dac_samples_cnt);
        }
        if (ERR_SUCCESS == result) {
            // Calculates the average value from the measured data per DAC
            uint8_t dac_id = (g_dac_ref_config.state == DAC_REF_FSM_STATES_POST_0) ? 0 : 1;
            g_dac_ref_config.dac_averages[dac_id] = 0;
            for (i = 0; i < dac_samples_cnt; i++) {
                g_dac_ref_config.dac_averages[dac_id] += dac_samples[i];
            }
            g_dac_ref_config.dac_averages[dac_id] /= dac_samples_cnt;

            g_dac_ref_config.state++;
        }

        g_dev_config.lib_state = LIB_STATE_DAC_REF;

        break;

    case DAC_REF_FSM_STATES_FINALIZE:

        /* Calculate absolute DAC reference value. */
        if (g_dac_ref_config.dac_averages[0] > g_dac_ref_config.dac_averages[1]) {
            *p_dac_ref = g_dac_ref_config.dac_averages[0] - g_dac_ref_config.dac_averages[1];
        } else {
            *p_dac_ref = g_dac_ref_config.dac_averages[1] - g_dac_ref_config.dac_averages[0];
        }

        /* Restore old callback and overwritten data */
        g_dev_config.p_callback = g_dac_ref_config.p_callback_backup;
        g_dac_ref_config.p_callback_backup = NULL;
        result = as7050_ifce_set_reg_group(AS7050_REG_GROUP_ID_AFE, g_dac_ref_config.afe_config_backup.reg_buffer,
                                           sizeof(g_dac_ref_config.afe_config_backup.reg_buffer));

        if (ERR_SUCCESS == result) {
            g_dac_ref_config.state = DAC_REF_FSM_STATES_IDLE;
            g_dev_config.lib_state = LIB_STATE_CONFIGURATION;
            *p_status = DAC_REF_STATUS_FINISHED;
        }

        break;

    default:

        g_dac_ref_config.state = DAC_REF_FSM_STATES_ABORT;
        result = ERR_SUCCESS;
        break;
    }

    if ((DAC_REF_FSM_STATES_ABORT == g_dac_ref_config.state) || (ERR_SUCCESS != result)) {
        if (g_dac_ref_config.p_callback_backup) {
            g_dev_config.p_callback = g_dac_ref_config.p_callback_backup;
            as7050_ifce_set_reg_group(AS7050_REG_GROUP_ID_AFE, g_dac_ref_config.afe_config_backup.reg_buffer,
                                      sizeof(g_dac_ref_config.afe_config_backup.reg_buffer));
            g_dev_config.lib_state = LIB_STATE_MEASUREMENT;
            as7050_stop_measurement();
        }
        g_dac_ref_config.state = DAC_REF_FSM_STATES_IDLE;
        g_dev_config.lib_state = LIB_STATE_CONFIGURATION;
        *p_status = DAC_REF_STATUS_ABORTED;
    } else if (g_dac_ref_config.state != DAC_REF_FSM_STATES_IDLE) {
        *p_status = DAC_REF_STATUS_RUNNING;
    }

    return result;
}
