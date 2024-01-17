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

#include "as7050_agc.h"
#include "as7050_extract.h"
#include "as7050_interface.h"
#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "error_codes.h"
#include "nrf_log.h"
#include "export.h"
#include "hardware.h"
#include "common.h"
#include "as7050_chiplib.h"
#include "nrf_delay.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#ifndef AMS_PRINTF
#define AMS_PRINTF(...)                                                                                                \
    do {                                                                                                               \
    } while (0)
#endif

/*! Maximum PD Offset */
#define MAX_PD_OFFSET 255
/*! Minimum PD Offset */
#define MIN_PD_OFFSET 0

/*! Maximum PD Offset step size for dual channel AGC mode */
#define DUAL_CHAN_MAX_PD_OFFSET_STEP_SIZE 8

/*! Number of signal averaging for minimum and maximum value */
#define MV_AVG_NUM 4

#define LED_CURR_IDX AS7050_CHANNEL_GROUP_A
#define PD_OFFSET_IDX AS7050_CHANNEL_GROUP_B

/*! This value will be set inside the average function to mark the value as saturated.
    The chip returns zero on saturation which runs the agc algorithm in the wrong direction */
#define MAX_ADC_VALUE_FOR_SATURATION 0xFFFFF

/*! AGC working states */
typedef enum agc_work_states {
    AGC_IDLE = 0,   /*!< Offset and amplitude control do nothing */
    AGC_ACTIVE = 1, /*!< Offset and amplitude control has been run at least once, signal was not in range and PD offset
                       not at the limits - signal can still be improved. */
} agc_work_states_t;

/*! Current state of the offset and amplitude control algorithm parameters */
typedef struct {
    uint32_t reset_tick;
    uint16_t sampsCntr;               /*!< Counts the number of samples collected for signal averaging */
    uint8_t sat_detected;             /*!< Marker that saturation was detected. Needed for chip revision lower than 2 */
    uint8_t chan_b_high_sat_detected; /*!< Marker for chan B that saturation was detected. Needed for chip revision
                                        lower than 2 */
    uint8_t saturation_workaround_enabled; /*!< For chip revisions lower than 2 a workaround regarding the saturation
                                              detection is needed. Lower and upper saturated valued will be set to 0. */
    uint8_t led_current_step_size[AS7050_NUM_CHANNEL_GROUP]; /*!< Defines the LED current change steps */
    uint8_t pd_offset_current[AS7050_NUM_CHANNEL_GROUP]; /*!< The new value of LED on PD offset found during AGC, it is
                                                            also the currently configured one in the chip before and
                                                            after the AGC call */
    uint8_t
        led_current[AS7050_NUM_CHANNEL_GROUP];    /*!< The new value of LED current found during AGC, it is also the
                                                     currently configured one in the chip before and after the AGC call */
    uint8_t led_change[AS7050_NUM_CHANNEL_GROUP]; /*!< Indicates what changed in the LED current - decreased, increased,
                                                     did not change... */
    uint8_t led_group[AS7050_NUM_CHANNEL_GROUP];  /*!< Defines which LEDs will be used in the same group together */
    uint8_t pd_offset_change[AS7050_NUM_CHANNEL_GROUP];     /*!< Indicates what changed in the PD offset - decreased,
                                                               increased, did not change... */
    agc_work_states_t state_flag[AS7050_NUM_CHANNEL_GROUP]; /*!< Indicates whether the AGC was able to bring the signal
                                                               within range, it can take the values defined by AGC_IDLE
                                                               and AGC_ACTIVE */
    int16_t ledUnder[AS7050_NUM_CHANNEL_GROUP];
    int16_t ledOver[AS7050_NUM_CHANNEL_GROUP];
    struct {
        uint32_t min; /*!< The minimum of the corresponding signal - calculated over a moving average */
        uint32_t max; /*!< The maximum of the corresponding signal - calculated over a moving average */
        uint32_t cnt; /*!< Count of the collected samples since the last time AGC was executed */
        int32_t accu; /*!< Accumulates values */
        uint32_t mv_avg_vals[MV_AVG_NUM];
        uint8_t mv_avg_cnt;
        uint8_t mv_avg_idx;
        uint8_t zero_detected;
    } channels[AS7050_NUM_CHANNEL_GROUP];

} as7050_agc_internal_t;

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
export_data mixdata;
export_ecg ecgdata;
export_flash flashdata;
/******************************************************************************
 *                                 LOCALS                                   *
 ******************************************************************************/
static as7050_agc_config_t g_agc_config;
static volatile as7050_agc_internal_t g_agc_internal;
uint32_t raw_vals[32];
uint32_t raw_vals1[32];
uint16_t length_array = 0 ;
volatile uint16_t var_length_array = 0 ;
uint32_t ecg_start_address = 0x0000;
uint32_t ppg_start_address = 0x40000;
int total_data_collected = 0;
/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

static inline uint8_t calculate_step_size(uint8_t range)
{
    uint8_t step_size;

    if (range > 0) {
        step_size = 16;
    } else {
        step_size = 0;
    }
    while (step_size > 1) {
        if ((range & 0x80) == 0x80) {
            break;
        }
        range = range << 1;
        step_size = step_size >> 1;
    }

    return step_size;
}

static uint32_t calcMvAvg(uint32_t sample_value, uint32_t *mvavg_buf, uint8_t mvagv_buf_size, uint8_t *mvavg_cnt,
                          uint8_t *buf_idx)
{
    uint32_t avg_val = 0;
    uint32_t i;

    if (*buf_idx >= mvagv_buf_size) {
        *buf_idx = 0;
    }

    mvavg_buf[*buf_idx] = sample_value;
    *buf_idx = *buf_idx + 1;

    if (*mvavg_cnt < mvagv_buf_size) {
        *mvavg_cnt = *mvavg_cnt + 1;
    }

    for (i = 0; i < *mvavg_cnt; i++) {
        avg_val += mvavg_buf[i];
    }
    avg_val = avg_val / *mvavg_cnt;

    return avg_val;
}

static err_code_t set_led_current(as7050_channel_group_t group_id, uint8_t current)
{
    err_code_t result = ERR_SUCCESS;
    uint8_t led_current_group = g_agc_internal.led_group[group_id];
    uint8_t i = 0;

    while (led_current_group && (ERR_SUCCESS == result)) {

        if (led_current_group & 0x01) {
            result = as7050_ifce_set_led_current((as7050_channel_t)((int)AS7050_CHANNEL_PPG_1 + i), current);
        }
        led_current_group >>= 1;
        i++;
    }

    return result;
}

/*
    This function is used by the AGC algorithm to reset the minimum and maximum detected signal values
 */
static void resetMinMaxAvg(uint32_t tick_ms)
{
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].min = UINT32_MAX;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].max = 0;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].min = UINT32_MAX;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].max = 0;
    g_agc_internal.reset_tick = tick_ms;
}

/*
    This function is used by the AGC algorithm to calculate the minimum, maximum and average signal values
 */
static err_code_t calcMinMaxAvg(as7050_channel_group_t group_id, uint16_t number, uint32_t *p_raw_values)
{
    uint32_t avgVal;

    for (int i = 0; i < number; i++) {

        if (0 == p_raw_values[i]) {
            g_agc_internal.channels[group_id].zero_detected = TRUE;
        }
        g_agc_internal.channels[group_id].accu += p_raw_values[i];
        g_agc_internal.channels[group_id].cnt++;

        avgVal = calcMvAvg(p_raw_values[i], (uint32_t *)g_agc_internal.channels[group_id].mv_avg_vals, MV_AVG_NUM,
                           (uint8_t *)&g_agc_internal.channels[group_id].mv_avg_cnt,
                           (uint8_t *)&g_agc_internal.channels[group_id].mv_avg_idx);

        if (g_agc_internal.channels[group_id].min > avgVal) {
            g_agc_internal.channels[group_id].min = avgVal;
        }
        if (g_agc_internal.channels[group_id].max < avgVal) {
            g_agc_internal.channels[group_id].max = avgVal;
        }
    }
    return ERR_SUCCESS;
}

/*!
 * \brief AGC for calculating current and PD offset from one signal source
 *
 * \note This function is used by the AGC algorithm AS7050_AGC_MODE_PPG_TWO_CHANNEL.
 */
static void calculate_led_current_from_single_channel_ulp(int32_t mean)
{
    uint16_t step_size;

    uint16_t orig_led_current = g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B];

    if (mean > g_agc_config.threshold_max) {
        if (orig_led_current <= g_agc_internal.ledUnder[LED_CURR_IDX]) {
            // reset lower range, values unhelpful
            g_agc_internal.ledUnder[LED_CURR_IDX] = -1;
        }

        // setup search LED current values
        g_agc_internal.ledOver[LED_CURR_IDX] = orig_led_current;

        // PD offset is maxed, decrease LED current
        g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

        if (g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] > g_agc_config.current_min[AS7050_CHANNEL_GROUP_B]) {
            // led current above minimum, decrease
            g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_DECREASED;

            if (g_agc_internal.ledUnder[LED_CURR_IDX] == -1) {
                // take pseudo log step, not enough info to search
                step_size = g_agc_internal.led_current_step_size[AS7050_CHANNEL_GROUP_B];
            } else {
                // search to midpoint of last low and high seen
                step_size = (g_agc_internal.ledOver[LED_CURR_IDX] - g_agc_internal.ledUnder[LED_CURR_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if ((g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] - step_size) >=
                g_agc_config.current_min[AS7050_CHANNEL_GROUP_B]) {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] -= step_size;
            } else {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] = g_agc_config.current_min[AS7050_CHANNEL_GROUP_B];
            }

            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#LED;%d;%d;%d;%d\n", 0, mean, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B],
                       -(int)(orig_led_current - g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B]));
            /* -----------------------------------------------------------------------*/
        } else {
            // LED current already at min
            g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_ATMIN;
        }
    } else if (mean < g_agc_config.threshold_min) {
        if (orig_led_current >= g_agc_internal.ledOver[LED_CURR_IDX]) {
            // reset upper range, values unhelpful
            g_agc_internal.ledOver[LED_CURR_IDX] = -1;
        }

        if (g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] < g_agc_config.current_max[AS7050_CHANNEL_GROUP_B]) {
            g_agc_internal.ledUnder[LED_CURR_IDX] = orig_led_current;
            // LED current less than max, increase current
            g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

            g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_INCREASED;

            if (g_agc_internal.ledOver[LED_CURR_IDX] == -1) {
                // take pseudo log step, not enough info to search
                step_size = g_agc_internal.led_current_step_size[AS7050_CHANNEL_GROUP_B];
            } else {
                // take binary search step
                step_size = (g_agc_internal.ledOver[LED_CURR_IDX] - g_agc_internal.ledUnder[LED_CURR_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if ((g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] + step_size) <=
                g_agc_config.current_max[AS7050_CHANNEL_GROUP_B]) {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] += step_size;
            } else {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] = g_agc_config.current_max[AS7050_CHANNEL_GROUP_B];
            }

            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#LED;%d;%d;%d;%d\n", 0, mean, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B],
                       g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] - orig_led_current);
            /* -----------------------------------------------------------------------*/
        } else // LED current is maxed, reduce pdOffx
        {
            g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_ATMAX;
        }
    } else {
        g_agc_internal.state_flag[LED_CURR_IDX] = AGC_IDLE;
    }
}

/*!
 * \brief AGC for calculating PD offset from one signal source
 *
 * \note This function is used by the AGC algorithm AS7050_AGC_MODE_PPG_TWO_CHANNEL.
 */
static void calculate_pdoffset_from_single_channel_ulp(int32_t mean)
{
    uint8_t step_size;

    const uint8_t orig_offset = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];

    if (mean > g_agc_config.threshold_max) {
        if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= g_agc_internal.ledUnder[PD_OFFSET_IDX]) {
            // reset lower range, values unhelpful
            g_agc_internal.ledUnder[PD_OFFSET_IDX] = -1;
        }

        if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= MAX_PD_OFFSET) {
            // setup search LED current values
            g_agc_internal.ledOver[PD_OFFSET_IDX] = -1;

            // PD offset is maxed, decrease LED current
            g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

            g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_IDLE;
            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMAX;
        } else // PD offset is less than max, increase
        {
            g_agc_internal.ledOver[PD_OFFSET_IDX] = orig_offset;
            g_agc_internal.ledOver[LED_CURR_IDX] = -1;

            g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_ACTIVE;

            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_INCREASED;

            // not enough information to make educated step size guess
            if (g_agc_internal.ledUnder[PD_OFFSET_IDX] == -1) {
                // take large step, not enough information to search
                step_size = DUAL_CHAN_MAX_PD_OFFSET_STEP_SIZE;
            } else {
                // take binary search step
                step_size = (g_agc_internal.ledUnder[PD_OFFSET_IDX] - g_agc_internal.ledOver[PD_OFFSET_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if ((g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] + step_size) < 255) {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] += step_size;
            } else // step would put over max
            {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = 255;
            }
            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#AGC;%d;%d;%d;%d\n", 0, mean, g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A],
                       g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] - orig_offset);
            /* -----------------------------------------------------------------------*/
        }

    } else if (mean < g_agc_config.threshold_min) {
        if (orig_offset <= g_agc_internal.ledOver[PD_OFFSET_IDX]) {
            // reset upper range, values unhelpful
            g_agc_internal.ledOver[PD_OFFSET_IDX] = -1;
        }

        g_agc_internal.ledUnder[PD_OFFSET_IDX] = orig_offset;
        if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] > MIN_PD_OFFSET) {
            g_agc_internal.ledUnder[LED_CURR_IDX] = -1;

            // room to decrease PD offset, set flags
            g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_ACTIVE;

            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_DECREASED;

            // not enough information to make educated step size guess
            if (g_agc_internal.ledOver[PD_OFFSET_IDX] == -1) {
                step_size = DUAL_CHAN_MAX_PD_OFFSET_STEP_SIZE;
            } else {
                step_size = (g_agc_internal.ledUnder[PD_OFFSET_IDX] - g_agc_internal.ledOver[PD_OFFSET_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= (MIN_PD_OFFSET + step_size)) {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] -= step_size;
            } else {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = MIN_PD_OFFSET;
            }

            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#AGC;%d;%d;%d;%d\n", 0, mean, g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A],
                       -(int)(orig_offset - g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A]));
            /* -----------------------------------------------------------------------*/
        } else {
            // pdoffX already at min, set flags
            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMIN;
            g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_IDLE;
        }

    } else {
        g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_IDLE;
    }

    if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] !=
        g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B]) {
        g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B] =
            g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];
    }
    if (g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] !=
        g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B]) {
        g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B] =
            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A];
    }
}

/*!
 * \brief AGC for calculating current and PD offset from one signal source
 *
 * \note This function is used by the AGC algorithm AS7050_AGC_MODE_PPG_ONE_CHANNEL.
 */
static void calculate_led_current_and_pdoffset_from_single_channel_ulp(int32_t mean)
{
    uint16_t step_size;

    uint16_t orig_led_current = g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A];
    uint16_t orig_offset = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];

    if (mean > g_agc_config.threshold_max) {
        if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= g_agc_internal.ledUnder[PD_OFFSET_IDX]) {
            // reset lower range, values unhelpful
            g_agc_internal.ledUnder[PD_OFFSET_IDX] = -1;
        }
        if (orig_led_current <= g_agc_internal.ledUnder[LED_CURR_IDX]) {
            // reset lower range, values unhelpful
            g_agc_internal.ledUnder[LED_CURR_IDX] = -1;
        }

        if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= MAX_PD_OFFSET) {
            // setup search LED current values
            g_agc_internal.ledOver[LED_CURR_IDX] = orig_led_current;
            g_agc_internal.ledOver[PD_OFFSET_IDX] = -1;

            // PD offset is maxed, decrease LED current
            g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

            if (g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] > g_agc_config.current_min[AS7050_CHANNEL_GROUP_A]) {
                // led current above minimum, decrease
                g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_DECREASED;

                if (g_agc_internal.ledUnder[LED_CURR_IDX] == -1) {
                    // take pseudo log step, not enough info to search
                    step_size = g_agc_internal.led_current_step_size[AS7050_CHANNEL_GROUP_A];
                } else {
                    // search to midpoint of last low and high seen
                    step_size = (g_agc_internal.ledOver[LED_CURR_IDX] - g_agc_internal.ledUnder[LED_CURR_IDX]) / 2;

                    if (step_size < 2) {
                        step_size = 1;
                    }
                }

                if ((g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] - step_size) >=
                    g_agc_config.current_min[AS7050_CHANNEL_GROUP_A]) {
                    g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] -= step_size;
                } else {
                    g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] =
                        g_agc_config.current_min[AS7050_CHANNEL_GROUP_A];
                }

                /* -------------------------------- DEBUG OUTPUT -------------------------*/
                AMS_PRINTF("#LED;%d;%d;%d;%d\n", 0, mean, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A],
                           -(int)(orig_led_current - g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A]));
                /* -----------------------------------------------------------------------*/
            } else {
                // LED current already at min, pdoff already at max
                g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMIN;
                g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMAX;

                AMS_PRINTF("#AGC LIMIT:%d;%d;%d;%d\n", 0, mean, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A],
                           g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A]);
            }
        } else // PD offset is less than max, increase
        {
            g_agc_internal.ledOver[PD_OFFSET_IDX] = orig_offset;
            g_agc_internal.ledOver[LED_CURR_IDX] = -1;

            g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_ACTIVE;

            g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_INCREASED;

            // not enough information to make educated step size guess
            if (g_agc_internal.ledUnder[PD_OFFSET_IDX] == -1) {
                // take large step, not enough information to search
                step_size = 32;
            } else {
                // take binary search step
                step_size = (g_agc_internal.ledUnder[PD_OFFSET_IDX] - g_agc_internal.ledOver[PD_OFFSET_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if ((g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] + step_size) < MAX_PD_OFFSET) {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] += step_size;
            } else // step would put over max
            {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = MAX_PD_OFFSET;
            }
            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#AGC;%d;%d;%d;%d\n", 0, mean, g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A],
                       g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] - orig_offset);
            /* -----------------------------------------------------------------------*/
        }

    } else if (mean < g_agc_config.threshold_min) {
        if (orig_led_current >= g_agc_internal.ledOver[LED_CURR_IDX]) {
            // reset upper range, values unhelpful
            g_agc_internal.ledOver[LED_CURR_IDX] = -1;
        }
        if (orig_offset <= g_agc_internal.ledOver[PD_OFFSET_IDX]) {
            // reset upper range, values unhelpful
            g_agc_internal.ledOver[PD_OFFSET_IDX] = -1;
        }

        if (g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] < g_agc_config.current_max[AS7050_CHANNEL_GROUP_A]) {
            g_agc_internal.ledUnder[LED_CURR_IDX] = orig_led_current;
            g_agc_internal.ledUnder[PD_OFFSET_IDX] = -1;
            // LED current less than max, increase current
            g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

            g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_INCREASED;

            if (g_agc_internal.ledOver[LED_CURR_IDX] == -1) {
                // take pseudo log step, not enough info to search
                step_size = g_agc_internal.led_current_step_size[AS7050_CHANNEL_GROUP_A];
            } else {
                // take binary search step
                step_size = (g_agc_internal.ledOver[LED_CURR_IDX] - g_agc_internal.ledUnder[LED_CURR_IDX]) / 2;

                if (step_size < 2) {
                    step_size = 1;
                }
            }

            if ((g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] + step_size) <=
                g_agc_config.current_max[AS7050_CHANNEL_GROUP_A]) {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] += step_size;
            } else {
                g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] = g_agc_config.current_max[AS7050_CHANNEL_GROUP_A];
            }

            /* -------------------------------- DEBUG OUTPUT -------------------------*/
            AMS_PRINTF("#LED;%d;%d;%d;%d\n", 0, mean, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A],
                       g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] - orig_led_current);
            /* -----------------------------------------------------------------------*/
        } else // LED current is maxed, reduce pdOffx
        {
            g_agc_internal.ledUnder[PD_OFFSET_IDX] = orig_offset;
            g_agc_internal.ledUnder[LED_CURR_IDX] = -1;
            if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] > MIN_PD_OFFSET) {
                // room to decrease PD offset, set flags
                g_agc_internal.state_flag[LED_CURR_IDX] = AGC_ACTIVE;

                g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_DECREASED;

                // not enough information to make educated step size guess
                if (g_agc_internal.ledOver[PD_OFFSET_IDX] == -1) {
                    step_size = 32;
                } else {
                    step_size = (g_agc_internal.ledUnder[PD_OFFSET_IDX] + g_agc_internal.ledOver[PD_OFFSET_IDX]) / 2;

                    if (step_size < 2) {
                        step_size = 1;
                    }
                }

                if (g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] >= (MIN_PD_OFFSET + step_size)) {
                    g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] -= step_size;
                } else {
                    g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = MIN_PD_OFFSET;
                }

                /* -------------------------------- DEBUG OUTPUT -------------------------*/
                AMS_PRINTF("#AGC:%d;%d;%d;%d\n", 0, mean, g_agc_internal.pd_offset_current,
                           -(int)(orig_offset - g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A]));
                /* -----------------------------------------------------------------------*/
            } else {
                // pdoffX already at min, set flags
                g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMAX;
                g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_ATMIN;
            }
        }
    } else {
        g_agc_internal.state_flag[PD_OFFSET_IDX] = AGC_IDLE;
        g_agc_internal.state_flag[LED_CURR_IDX] = AGC_IDLE;
    }

    AMS_PRINTF("#AGC:ledU%d;LedO%d;pdU%d;pdO%d\n", g_agc_internal.ledUnder[LED_CURR_IDX],
               g_agc_internal.ledOver[LED_CURR_IDX], g_agc_internal.ledUnder[PD_OFFSET_IDX],
               g_agc_internal.ledOver[PD_OFFSET_IDX]);
}

static void reset_pd_offset_and_led_current(void)
{

    /* Decrease PD offset. Only in this case we can be sure that the measured values are not in lower saturation */
    g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_DECREASED;
    g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = MIN_PD_OFFSET;

    if (g_agc_config.mode == AS7050_AGC_MODE_PPG_TWO_CHANNEL) {

        /* Decrease PD offset. */
        g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_DECREASED;
        g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B] = MIN_PD_OFFSET;

        g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_DECREASED;
        g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] = g_agc_config.current_min[AS7050_CHANNEL_GROUP_B];
    } else {

        /* Decrease LED current as well, so that we can start again */
        g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_DECREASED;
        g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] = g_agc_config.current_min[AS7050_CHANNEL_GROUP_A];
    }
}

/*!
 * \brief Handling of different AGC algorithms
 */
static err_code_t calculate_optimized_agc_settings(uint32_t tick_ms)
{
    int32_t mean_a;
    int32_t mean_b = 0;
    as7050_pd_offset_config_t pd_offset_config[AS7050_NUM_CHANNEL_GROUP];
    uint8_t num_pd_offset_config = 0;
    err_code_t result = ERR_SUCCESS;

    if (g_agc_config.mode == AS7050_AGC_MODE_PPG_ONE_CHANNEL) {
        if (0 < g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].cnt) {

            mean_a = g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].accu /
                     g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].cnt;

            /* workaround for chip revisions 0 and 1: saturated values are set to zero */
            if (g_agc_internal.saturation_workaround_enabled) {
                if ((0 == mean_a) && (0 != g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A]) &&
                    (FALSE == g_agc_internal.sat_detected)) {
                    reset_pd_offset_and_led_current();

                    /* Mark here that we are in saturation.
                       Maybe we have to increase the PD offset afterwards to came out of saturation state */
                    g_agc_internal.sat_detected = TRUE;
                } else {
                    if (g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].zero_detected) {
                        mean_a = MAX_ADC_VALUE_FOR_SATURATION;
                    }
                    calculate_led_current_and_pdoffset_from_single_channel_ulp(mean_a);

                    /* Signal is still in saturation and agc is on control limit. Reset configuration again */
                    if ((MAX_ADC_VALUE_FOR_SATURATION == mean_a) &&
                        (AS7050_AGC_STATE_ATMAX == g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A])) {
                        g_agc_internal.sat_detected = FALSE;
                    }
                }
                /* normal behavior: saturated values are the highest digits */
            } else {
                calculate_led_current_and_pdoffset_from_single_channel_ulp(mean_a);
            }
        }
    } else if (g_agc_config.mode == AS7050_AGC_MODE_PPG_TWO_CHANNEL) {

        if (0 < g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].cnt) {
            mean_b = g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].accu /
                     g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].cnt;
        }

        if (0 < g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].cnt) {
            mean_a = g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].accu /
                     g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].cnt;

            /* workaround for chip revisions 0 and 1: saturated values are set to zero */
            if (g_agc_internal.saturation_workaround_enabled) {
                if ((0 == mean_a) && (0 != g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A]) &&
                    (FALSE == g_agc_internal.sat_detected)) {
                    reset_pd_offset_and_led_current();

                    /* Mark here that we are in saturation.
                       Maybe we have to increase the PD offset afterwards to came out of saturation state */
                    g_agc_internal.sat_detected = TRUE;

                    /* Mark as saturated */
                    g_agc_internal.chan_b_high_sat_detected = TRUE;
                } else {
                    if (g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].zero_detected) {
                        mean_a = MAX_ADC_VALUE_FOR_SATURATION;
                    }
                    calculate_pdoffset_from_single_channel_ulp(mean_a);

                    /* Signal is still in saturation and agc is on control limit. Reset configuration again */
                    if ((MAX_ADC_VALUE_FOR_SATURATION == mean_a) &&
                        (AS7050_AGC_STATE_ATMAX == g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A])) {
                        g_agc_internal.sat_detected = FALSE;
                    }

                    if (0 != mean_b) {
                        g_agc_internal.chan_b_high_sat_detected = FALSE;
                    }
                }
                /* normal behavior: saturated values are the highest digits */
            } else {
                calculate_pdoffset_from_single_channel_ulp(mean_a);
            }
        }

        if ((g_agc_internal.state_flag[PD_OFFSET_IDX] == AGC_IDLE) &&
            (0 < g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].cnt)) {

            /* workaround for chip revisions 0 and 1: saturated values are set to zero */
            if (g_agc_internal.saturation_workaround_enabled) {
                if ((0 == mean_b) && (0 != g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B]) &&
                    (TRUE == g_agc_internal.chan_b_high_sat_detected)) {
                    reset_pd_offset_and_led_current();

                    /* Mark here that we are in saturation.
                       Maybe we have to increase the PD offset afterwards to came out of saturation state */
                    g_agc_internal.sat_detected = TRUE;
                } else {
                    if ((g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].zero_detected) &&
                        (TRUE == g_agc_internal.chan_b_high_sat_detected)) {
                        mean_b = MAX_ADC_VALUE_FOR_SATURATION;
                    }
                    calculate_led_current_from_single_channel_ulp(mean_b);
                }
                /* normal behavior: saturated values are the highest digits */
            } else {
                calculate_led_current_from_single_channel_ulp(mean_b);
            }
        }
    }

    /*
        See if the signal checks above caused change in LED current and/or PD offset
    */
    if (g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] == AS7050_AGC_STATE_INCREASED ||
        g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] == AS7050_AGC_STATE_DECREASED) {
        pd_offset_config[num_pd_offset_config].channel_type =
            (as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_A];
        pd_offset_config[num_pd_offset_config].pd_offset = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];
        num_pd_offset_config++;
    }
    if (g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B] == AS7050_AGC_STATE_INCREASED ||
        g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B] == AS7050_AGC_STATE_DECREASED) {
        pd_offset_config[num_pd_offset_config].channel_type =
            (as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_B];
        pd_offset_config[num_pd_offset_config].pd_offset = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B];
        num_pd_offset_config++;
    }

    if ((g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] == AS7050_AGC_STATE_INCREASED) ||
        (g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] == AS7050_AGC_STATE_DECREASED)) {
        result = set_led_current(AS7050_CHANNEL_GROUP_A, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A]);
    }
    if ((ERR_SUCCESS == result) &&
        ((g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] == AS7050_AGC_STATE_INCREASED) ||
         (g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] == AS7050_AGC_STATE_DECREASED))) {
        result = set_led_current(AS7050_CHANNEL_GROUP_B, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B]);
    }

    if ((ERR_SUCCESS == result) && (0 < num_pd_offset_config)) {
        resetMinMaxAvg(tick_ms);
        result = as7050_ifce_set_pd_offset(pd_offset_config, num_pd_offset_config);
    }

    g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].accu = 0;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].cnt = 0;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_A].zero_detected = FALSE;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].accu = 0;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].cnt = 0;
    g_agc_internal.channels[AS7050_CHANNEL_GROUP_B].zero_detected = FALSE;

    return result;
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t as7050_agc_init(uint8_t saturation_workaround_enabled)
{
    memset(&g_agc_config, 0x00, sizeof(as7050_agc_config_t));
    memset((void *)&g_agc_internal, 0x00, sizeof(as7050_agc_internal_t));

    g_agc_internal.saturation_workaround_enabled = (0 != saturation_workaround_enabled);

    return ERR_SUCCESS;
}

err_code_t as7050_agc_reset(uint32_t tick_ms)
{
    err_code_t result = ERR_SUCCESS;
    uint8_t i;
    as7050_pd_offset_config_t pd_offset_config[AS7050_NUM_CHANNEL_GROUP];
    uint8_t num_pd_offset_config = 0;

    g_agc_internal.sampsCntr = 0;
    g_agc_internal.sat_detected = FALSE;
    g_agc_internal.chan_b_high_sat_detected = TRUE;

    for (i = 0; i < AS7050_NUM_CHANNEL_GROUP && ERR_SUCCESS == result; i++) {
        g_agc_internal.channels[i].zero_detected = FALSE;
        g_agc_internal.channels[i].accu = 0;
        g_agc_internal.channels[i].cnt = 0;
        g_agc_internal.channels[i].min = MAX_ADC_VALUE_FOR_SATURATION;
        g_agc_internal.channels[i].max = 0;

        g_agc_internal.channels[i].mv_avg_cnt = 0;
        g_agc_internal.channels[i].mv_avg_idx = 0;

        g_agc_internal.state_flag[i] = AGC_IDLE;

        g_agc_internal.led_current[i] = 0;
        g_agc_internal.led_change[i] = AS7050_AGC_STATE_UNCHANGED;

        g_agc_internal.ledUnder[i] = -1;
        g_agc_internal.ledOver[i] = -1;

        g_agc_internal.led_group[i] = 0;
        g_agc_internal.pd_offset_current[i] = 0;
        g_agc_internal.pd_offset_change[i] = 0;

        g_agc_internal.led_current_step_size[i] =
            calculate_step_size(g_agc_config.current_max[i] - g_agc_config.current_min[i]);
    }

    if (ERR_SUCCESS == result && AS7050_AGC_MODE_DISABLED != g_agc_config.mode) {

        result = as7050_ifce_get_channel_led_config((as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_A],
                                                    (uint8_t *)&g_agc_internal.led_group[AS7050_CHANNEL_GROUP_A]);

        if ((ERR_SUCCESS == result) && (AS7050_AGC_MODE_PPG_TWO_CHANNEL == g_agc_config.mode)) {
            result = as7050_ifce_get_channel_led_config((as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_B],
                                                        (uint8_t *)&g_agc_internal.led_group[AS7050_CHANNEL_GROUP_B]);
        }

        /* CH1: Configuration of LED current */
        if (ERR_SUCCESS == result) {
            g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A] = g_agc_config.current_min[AS7050_CHANNEL_GROUP_A];
            result = set_led_current(AS7050_CHANNEL_GROUP_A, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A]);
        }
        /* CH2: Configuration of LED current */
        if ((ERR_SUCCESS == result) && (AS7050_AGC_MODE_PPG_TWO_CHANNEL == g_agc_config.mode)) {
            g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B] = g_agc_config.current_min[AS7050_CHANNEL_GROUP_B];
            result = set_led_current(AS7050_CHANNEL_GROUP_B, g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B]);
        }

        if (ERR_SUCCESS == result) {

            /* CH1: Configuration of PD offset */
            g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A] = MIN_PD_OFFSET;
            pd_offset_config[num_pd_offset_config].channel_type =
                (as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_A];
            pd_offset_config[num_pd_offset_config].pd_offset = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];
            num_pd_offset_config++;

            /* CH2: Configuration of PD offset */
            if (AS7050_AGC_MODE_PPG_TWO_CHANNEL == g_agc_config.mode) {
                g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B] = MIN_PD_OFFSET;
                pd_offset_config[num_pd_offset_config].channel_type =
                    (as7050_channel_t)g_agc_config.channel[AS7050_CHANNEL_GROUP_B];
                pd_offset_config[num_pd_offset_config].pd_offset =
                    g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B];
                num_pd_offset_config++;
            }

            if (0 < num_pd_offset_config) {
                result = as7050_ifce_set_pd_offset(pd_offset_config, num_pd_offset_config);
            }
        }
    }

    g_agc_internal.reset_tick = tick_ms;

    return result;
}

err_code_t as7050_agc_set_config(const as7050_agc_config_t *p_agc_config)
{
    M_CHECK_NULL_POINTER(p_agc_config);

    /* check general errors */
    if (AS7050_AGC_MODE_DISABLED != p_agc_config->mode) {
        if (0 == p_agc_config->sample_cnt) {
            return ERR_ARGUMENT;
        }
        if (0 == p_agc_config->reset_interval) {
            return ERR_ARGUMENT;
        }
    }

    if (p_agc_config->current_max[AS7050_CHANNEL_GROUP_A] < p_agc_config->current_min[AS7050_CHANNEL_GROUP_A]) {
        return ERR_ARGUMENT;
    }

    if (p_agc_config->current_max[AS7050_CHANNEL_GROUP_B] < p_agc_config->current_min[AS7050_CHANNEL_GROUP_B]) {
        return ERR_ARGUMENT;
    }

    if (p_agc_config->threshold_max < p_agc_config->threshold_min) {
        return ERR_ARGUMENT;
    }

    switch (p_agc_config->mode) {
    case AS7050_AGC_MODE_DISABLED:
        break;
    case AS7050_AGC_MODE_PPG_ONE_CHANNEL:
        if (AS7050_CHANNEL_PPG_1 > p_agc_config->channel[AS7050_CHANNEL_GROUP_A] ||
            AS7050_CHANNEL_TIA < p_agc_config->channel[AS7050_CHANNEL_GROUP_A]) {
            return ERR_ARGUMENT;
        }
        break;

    case AS7050_AGC_MODE_PPG_TWO_CHANNEL:
        if (AS7050_CHANNEL_PPG_1 > p_agc_config->channel[AS7050_CHANNEL_GROUP_A] ||
            AS7050_CHANNEL_TIA < p_agc_config->channel[AS7050_CHANNEL_GROUP_A]) {
            return ERR_ARGUMENT;
        }
        if (AS7050_CHANNEL_PPG_1 > p_agc_config->channel[AS7050_CHANNEL_GROUP_B] ||
            AS7050_CHANNEL_TIA < p_agc_config->channel[AS7050_CHANNEL_GROUP_B]) {
            return ERR_ARGUMENT;
        }
        break;

    default:
        return ERR_ARGUMENT;
        break;
    }

    memcpy(&g_agc_config, p_agc_config, sizeof(as7050_agc_config_t));

    return ERR_SUCCESS;
}

err_code_t as7050_agc_get_config(as7050_agc_config_t *p_agc_config)
{
    M_CHECK_NULL_POINTER(p_agc_config);

    memcpy(p_agc_config, &g_agc_config, sizeof(as7050_agc_config_t));

    return ERR_SUCCESS;
}

err_code_t as7050_agc_get_status(as7050_agc_status_t *p_agc_status)
{
    M_CHECK_NULL_POINTER(p_agc_status);

    p_agc_status->led_change[AS7050_CHANNEL_GROUP_A] = g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A];
    p_agc_status->led_change[AS7050_CHANNEL_GROUP_B] = g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B];
    p_agc_status->led_current[AS7050_CHANNEL_GROUP_A] = g_agc_internal.led_current[AS7050_CHANNEL_GROUP_A];
    p_agc_status->led_current[AS7050_CHANNEL_GROUP_B] = g_agc_internal.led_current[AS7050_CHANNEL_GROUP_B];

    p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_A] = g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A];
    p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_A] = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_A];

    p_agc_status->pd_offset_change[AS7050_CHANNEL_GROUP_B] = g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B];
    p_agc_status->pd_offset_current[AS7050_CHANNEL_GROUP_B] = g_agc_internal.pd_offset_current[AS7050_CHANNEL_GROUP_B];

    return ERR_SUCCESS;
}

err_code_t as7050_agc_execute(uint32_t tick_ms, uint16_t fifo_adc_map, uint8_t sample_size, const uint8_t *p_fifo_data,
                              uint16_t fifo_data_size)
{
    err_code_t result = ERR_SUCCESS;
    uint16_t num_raw_vals;
    int i;

    if (AS7050_AGC_MODE_DISABLED == g_agc_config.mode) 
    {
        if (ERR_SUCCESS == result) {
            num_raw_vals = sizeof(raw_vals1) / sizeof(raw_vals1[0]);
            result = as7050_ifce_extract_samples(fifo_adc_map, sample_size,AS7050_CHANNEL_FLAG_ECG, p_fifo_data,
            fifo_data_size, raw_vals1, &num_raw_vals);
        }
        if (0 == flash_write_enable) {
          memcpy(&ecgdata.ecg_vals[length_array], raw_vals1, num_raw_vals * 4);
        }
        length_array += num_raw_vals;
        var_length_array = length_array;
        if (0 == flash_write_enable) 
        {
          if (var_length_array >= 44) {
            data_collected = 1;
            length_array = 0;
          }
        }

        return ERR_SUCCESS;
    }

    g_agc_internal.led_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_UNCHANGED;
    g_agc_internal.led_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_UNCHANGED;
    g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_A] = AS7050_AGC_STATE_UNCHANGED;
    g_agc_internal.pd_offset_change[AS7050_CHANNEL_GROUP_B] = AS7050_AGC_STATE_UNCHANGED;

    /* ULP AGC */
    if (AS7050_AGC_MODE_PPG_ONE_CHANNEL == g_agc_config.mode) {
        num_raw_vals = sizeof(raw_vals) / sizeof(raw_vals[0]);
        result = as7050_ifce_extract_samples(
            fifo_adc_map, sample_size,
            (as7050_channel_flags_t)(1 << (g_agc_config.channel[AS7050_CHANNEL_GROUP_A] - 1)), p_fifo_data,
            fifo_data_size, raw_vals, &num_raw_vals);

        if (ERR_SUCCESS == result) {
          result = calcMinMaxAvg(AS7050_CHANNEL_GROUP_A, num_raw_vals, raw_vals);
        }

        if (ERR_SUCCESS == result) {
          num_raw_vals = sizeof(raw_vals1) / sizeof(raw_vals1[0]);
          result = as7050_ifce_extract_samples(fifo_adc_map, sample_size, AS7050_CHANNEL_FLAG_ECG, p_fifo_data,
              fifo_data_size, raw_vals1, &num_raw_vals);
        }
        if (0 == flash_write_enable) {
          memcpy(&mixdata.ppg_vals[length_array], raw_vals, num_raw_vals * 4);
          memcpy(&mixdata.ecg_vals[length_array], raw_vals1, num_raw_vals * 4);
        }
        if (1 == flash_write_enable) {
          memcpy(&flashdata.ppg_vals[length_array], raw_vals, num_raw_vals * 4);
          memcpy(&flashdata.ecg_vals[length_array], raw_vals1, num_raw_vals * 4);
        }
        length_array += num_raw_vals;
        var_length_array = length_array;
        if (0 == flash_write_enable) {
          if (var_length_array >= EXPORT_SIZE) {
            data_collected = 1;
            length_array = 0;
          }
        }

        if (1 == flash_write_enable) {
          if (var_length_array >= 4000) {
            as7050_stop_measurement();
            flash_data_exist = 1;
            for (int i = 0; i <= var_length_array; i++) {
              if ((i % 64) == 0) {
                if (ecg_start_address < 262143) //0x3FFFF
                {
                  qspi_page_write(&flashdata.ecg_vals[i], ecg_start_address);
                  ecg_start_address = ecg_start_address + QSPI_PAGE_OFFSET;
                  ecg_end_address = ecg_start_address;
                }
                if (ppg_start_address < 524287) //0x7FFFF
                {
                  qspi_page_write(&flashdata.ppg_vals[i], ppg_start_address);
                  ppg_start_address = ppg_start_address + QSPI_PAGE_OFFSET;
                  ppg_end_address = ppg_start_address;
                } else {
                  flash_write_enable = 0;
                  NRF_LOG_INFO("Flash Memory Full");
                }
                nrf_delay_ms(50);
              }
            }
            printf("\nData written to Flash\n");
            total_data_collected += var_length_array;
            printf("\ntotal_data_collected = %d\n", total_data_collected);
            length_array = 0;
          }
        }
    }
    /* SPO2 AGC */
    else if (AS7050_AGC_MODE_PPG_TWO_CHANNEL == g_agc_config.mode) {

        if (ERR_SUCCESS == result) {
            num_raw_vals = sizeof(raw_vals1) / sizeof(raw_vals1[0]);
            result = as7050_ifce_extract_samples(fifo_adc_map, sample_size,AS7050_CHANNEL_FLAG_ECG, p_fifo_data,
            fifo_data_size, raw_vals1, &num_raw_vals);
        }

        num_raw_vals = sizeof(raw_vals) / sizeof(raw_vals[0]);
        result = as7050_ifce_extract_samples(
            fifo_adc_map, sample_size,
            (as7050_channel_flags_t)(1 << (g_agc_config.channel[AS7050_CHANNEL_GROUP_A] - 1)), p_fifo_data,
            fifo_data_size, raw_vals, &num_raw_vals);

 
        if (ERR_SUCCESS == result) {
            result = calcMinMaxAvg(AS7050_CHANNEL_GROUP_A, num_raw_vals, raw_vals);
        }


        if (ERR_SUCCESS == result) {
            num_raw_vals = sizeof(raw_vals) / sizeof(raw_vals[0]);
            result = as7050_ifce_extract_samples(
                fifo_adc_map, sample_size,
                (as7050_channel_flags_t)(1 << (g_agc_config.channel[AS7050_CHANNEL_GROUP_B] - 1)), p_fifo_data,
                fifo_data_size, raw_vals, &num_raw_vals);
        }
        if (ERR_SUCCESS == result) {
            result = calcMinMaxAvg(AS7050_CHANNEL_GROUP_B, num_raw_vals, raw_vals);
        }
    }

    if (ERR_SUCCESS == result) {
        /* Update sample counter */
        g_agc_internal.sampsCntr += num_raw_vals;

        /* Check if min and max values need to be reset */
        if (tick_ms - g_agc_internal.reset_tick >= (uint32_t)g_agc_config.reset_interval) {
            resetMinMaxAvg(tick_ms);
        }

        /* got enough data, check if adjustment is necessary */
        if (g_agc_internal.sampsCntr >= g_agc_config.sample_cnt) {
            result = calculate_optimized_agc_settings(tick_ms);
            g_agc_internal.sampsCntr = 0;
        }
    }

    if (0 == flash_write_enable)
    {
      memcpy(&mixdata.ppg_vals[length_array], raw_vals, num_raw_vals * 4);
      memcpy(&mixdata.ecg_vals[length_array], raw_vals1, num_raw_vals * 4);
    }
    if (1 == flash_write_enable)
    {
      memcpy(&flashdata.ppg_vals[length_array], raw_vals, num_raw_vals * 4);
      memcpy(&flashdata.ecg_vals[length_array], raw_vals1, num_raw_vals * 4);
    }
    length_array += num_raw_vals;
    var_length_array = length_array;
    if (0 == flash_write_enable) 
    {
      if (var_length_array >= EXPORT_SIZE) 
      {
        data_collected = 1;
        length_array = 0;
      }
    }

    if (1 == flash_write_enable)
    {
      if (var_length_array >= 4000)
      {
        as7050_stop_measurement();
        flash_data_exist = 1;
        for (int i = 0; i <= var_length_array; i++)
        {
          if ((i % 64) == 0)
          {
            if (ecg_start_address < 262143) //0x3FFFF
            {
              qspi_page_write(&flashdata.ecg_vals[i], ecg_start_address);
              ecg_start_address = ecg_start_address + QSPI_PAGE_OFFSET;
              ecg_end_address = ecg_start_address;
            }
            if (ppg_start_address < 524287)  //0x7FFFF
            {
              qspi_page_write(&flashdata.ppg_vals[i], ppg_start_address);
              ppg_start_address = ppg_start_address + QSPI_PAGE_OFFSET;
              ppg_end_address = ppg_start_address;
            }
            else
            {
               flash_write_enable = 0;
               NRF_LOG_INFO("Flash Memory Full");
            }
            nrf_delay_ms(50);
          }
        }
        printf("\nData written to Flash\n");
        total_data_collected += var_length_array;
        printf("\ntotal_data_collected = %d\n",total_data_collected);
        length_array = 0;
      }
    }
    return result;
}