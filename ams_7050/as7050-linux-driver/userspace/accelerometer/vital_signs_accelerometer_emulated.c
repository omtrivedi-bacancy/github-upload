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

#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include "error_codes.h"
#include "vital_signs_accelerometer.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#define MULT_MS 1000
#define ACC_I2C_ADDR 0x19

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

static const char *gp_osal_config;

static uint8_t g_is_initialized = FALSE;
static uint8_t g_is_running = FALSE;
static uint32_t g_last_ms_tick = 0;
static uint32_t g_start_ms_tick = 0;
static uint32_t g_sim_update_time_ms = 0;

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/
static err_code_t vs_acc_osal_get_tick(const char *gp_osal_config, uint32_t *p_tick_ms)
{
    M_UNUSED_PARAM(gp_osal_config);
    struct timeval time;

    M_CHECK_NULL_POINTER(p_tick_ms);

    gettimeofday(&time, 0);
    *p_tick_ms = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    return ERR_SUCCESS;
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t vs_acc_initialize(const char *p_config, const uint32_t sample_period_us)
{
    err_code_t err = ERR_SUCCESS;

    if (ERR_SUCCESS == err) {
        err = vs_acc_shutdown();
    }

    if (ERR_SUCCESS == err) {
        g_is_initialized = TRUE;
        g_sim_update_time_ms = sample_period_us / MULT_MS;
        gp_osal_config = p_config;
        err = ERR_SUCCESS;
    }

    return err;
}

err_code_t vs_acc_shutdown(void)
{
    err_code_t err = ERR_SUCCESS;

    g_is_initialized = FALSE;
    g_is_running = FALSE;
    gp_osal_config = NULL;

    return err;
}

err_code_t vs_acc_start(void)
{
    err_code_t err = ERR_SUCCESS;
    if (FALSE == g_is_initialized) {
        err = ERR_PERMISSION;
    }

    if (ERR_SUCCESS == err) {
        err = vs_acc_osal_get_tick(gp_osal_config, &g_last_ms_tick);
        g_start_ms_tick = g_last_ms_tick;
    }

    g_is_running = (ERR_SUCCESS == err);

    return err;
}

err_code_t vs_acc_stop(void)
{
    err_code_t err = ERR_SUCCESS;
    if (FALSE == g_is_initialized) {
        err = ERR_PERMISSION;
    }

    if (ERR_SUCCESS == err) {
        g_is_running = (ERR_SUCCESS == err) ? FALSE : TRUE;
    }
    return err;
}

err_code_t vs_acc_get_data(vs_acc_data_t *p_acc_data, uint8_t *p_num_acc_data)
{
    err_code_t err = ERR_SUCCESS;
    uint32_t tick_ms;
    uint32_t delta;
    uint8_t num_acc_data;

    M_CHECK_NULL_POINTER(p_acc_data);
    M_CHECK_NULL_POINTER(p_num_acc_data);
    M_CHECK_ARGUMENT_LOWER(0, *p_num_acc_data);

    if (FALSE == g_is_initialized) {
        return ERR_PERMISSION;
    }
    if (FALSE == g_is_running) {
        return ERR_PERMISSION;
    }

    if (g_sim_update_time_ms) {
        err = vs_acc_osal_get_tick(gp_osal_config, &tick_ms);
        if (ERR_SUCCESS == err) {
            /* overflow handling. Here we will miss some data but that is not critical. */
            if (tick_ms < g_start_ms_tick) {
                g_start_ms_tick = 0;
                g_last_ms_tick = 0;
            }
            delta = (tick_ms - g_start_ms_tick) - (g_last_ms_tick - g_start_ms_tick);
            num_acc_data = delta / g_sim_update_time_ms;

            if (*p_num_acc_data > num_acc_data) {
                *p_num_acc_data = num_acc_data;
            }
            if (0 < num_acc_data) {
                g_last_ms_tick =
                    g_start_ms_tick + (((tick_ms - g_start_ms_tick) / g_sim_update_time_ms) * g_sim_update_time_ms);
            }
            memset(p_acc_data, 0, sizeof(p_acc_data[0]) * (*p_num_acc_data));
        } else {
            *p_num_acc_data = 0;
        }
    }
    return err;
}
