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

#ifndef __AS7050_INTERFACE_H__
#define __AS7050_INTERFACE_H__

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

/*! Interrupt status register */
typedef struct {
    uint8_t irq_sequencer : 1; /*!< Interrupt for sequencer */
    uint8_t : 1;
    uint8_t irq_fifothreshold : 1; /*!< FIFO threshold reached */
    uint8_t irq_fifooverflow : 1;  /*!< FIFO overflow */
    uint8_t irq_led_lowvds : 1;    /*!< LED current does not reach the expected value */
    uint8_t irq_asat : 1;          /*!< Analog saturation */
    uint8_t irq_iir_overflow : 1;  /*!< IIR Filter overflow Interrupt*/
    uint8_t : 1;
} as7050_interrupt_status_t;

/*! Structure for setting PD offset */
typedef struct {
    as7050_channel_t channel_type; /*!< channel of sub sample */
    uint8_t pd_offset;             /*!< PD offset value */
} as7050_pd_offset_config_t;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

err_code_t as7050_ifce_write_register(uint8_t reg_addr, uint8_t reg_val);
err_code_t as7050_ifce_read_register(uint8_t reg_addr, uint8_t *p_reg_val);

err_code_t as7050_ifce_reset_chip(void);
err_code_t as7050_ifce_get_revision(uint8_t *p_revision);
err_code_t as7050_ifce_clear_fifo(void);
err_code_t as7050_ifce_get_fifo_level(uint16_t *p_fifo_level);
err_code_t as7050_ifce_get_fifo_threshold(uint16_t *p_fifo_threshold, uint8_t *p_sample_size);

err_code_t as7050_ifce_get_interrupt_status(as7050_interrupt_status_t *p_status);

//err_code_t as7050_ifce_set_reg_group(enum as7050_reg_group_ids id, const uint8_t *p_data, uint8_t size);
err_code_t as7050_ifce_set_reg_group(const enum as7050_reg_group_ids id, uint8_t *p_data, const uint8_t size);
err_code_t as7050_ifce_get_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t *p_size);

err_code_t as7050_ifce_get_ppg_ecg_sample_periods(uint32_t *p_ppg_sample_period_us, uint32_t *p_ecg_sample_period_us);

err_code_t as7050_ifce_start_measurement(void);
err_code_t as7050_ifce_stop_measurement(void);

err_code_t as7050_ifce_get_fifo_data(uint8_t *p_data, uint16_t data_num);

err_code_t as7050_ifce_get_channel_mapping(uint16_t *p_channel_map);

err_code_t as7050_ifce_get_saturation_status(uint8_t *p_ppg_sat, uint8_t *p_ecg_sat);

err_code_t as7050_ifce_get_max_adc_count(uint32_t *p_max_adc_count, uint8_t *p_max_adc_bits);

/* used by AGC */
err_code_t as7050_ifce_set_led_current(as7050_channel_t led_channel, uint8_t current);
err_code_t as7050_ifce_set_pd_offset_workaround(uint8_t enable);
err_code_t as7050_ifce_set_pd_offset(as7050_pd_offset_config_t *p_pd_offset_config, uint8_t num);
err_code_t as7050_ifce_get_channel_led_config(as7050_channel_t channel_type, uint8_t *p_led_config);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __AS7050_INTERFACE_H__ */
