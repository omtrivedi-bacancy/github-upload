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

#ifndef __AS7050_AGC_H__
#define __AS7050_AGC_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/
extern uint8_t data_collected;
extern int total_data_collected;

err_code_t as7050_agc_init(uint8_t saturation_workaround_enabled);

err_code_t as7050_agc_set_config(const as7050_agc_config_t *p_agc_config);
err_code_t as7050_agc_get_config(as7050_agc_config_t *p_agc_config);

err_code_t as7050_agc_get_status(as7050_agc_status_t *p_agc_status);

err_code_t as7050_agc_reset(uint32_t tick_ms);

/*!
 * Runs the configured AGC algorithm
 * When the collected number of channel samples reaches the configured count or
 * calculates the parameter values used by the different AGC algorithms, if the samples count is not yet reached.
 */
err_code_t as7050_agc_execute(uint32_t tick_ms, uint16_t fifo_adc_map, uint8_t sample_size, const uint8_t *p_fifo_data,
                              uint16_t fifo_data_size);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __AS7050_AGC_H__ */
