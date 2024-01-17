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

#include "as7050_interface.h"
#include "as7050_osal_chiplib.h"
#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

#define REG_SUBID_SUBID_OFS 3
#define LED_CFG_REG_BITS_ECG 0x40

#define REG_BITS_SEQ_START_SEQ_START 0x01
#define REG_BITS_SEQ_CFG_SD_SUBS_ALWAYS 0x01
#define ADC_FIFO_DATA_SHIFT 2

/* new AS7050 register bits */
#define REG_BITS_FIFO_CTRL__FIFO_CLEAR 0x01
#define REG_BITS_CONTROL__SW_RESET 0x02
#define REG_BITS_CGB_CFG__PLL_ON 0x04
#define REG_BITS_CGB_CFG__HF_OSC_ON 0x02
#define REG_BITS_CGB_CFG__LF_OSC_ON 0x01
#define REG_BITS_SEQ_START__START_SEQ 0x01
#define REG_BITS_IRQ_ENABLE__IRQ_EN_ASAT 0x20
#define REG_BITS_IRQ_ENABLE__IRQ_EN_FIFOOVERFLOW 0x08
#define REG_BITS_IRQ_ENABLE__IRQ_EN_FIFOTHRESHOLD 0x04
#define REG_BITS_FIFO_LEVEL1__FIFO_OVERFLOW 0x04
#define REG_BITS_FIFO_CTRL__FIFO_THRESHOLD_8 0x04
#define REG_BITS_FIFO_CTRL__DATA_MODE 0x02

#define REG_BITS_SEQ_MODE__PPG_EN 0x80
#define REG_BITS_SEQ_MODE__ECG_EN 0x20
#define REG_BITS_SEQ_PPGA__PPG_SUB_SAMPLE 0x07

#define REG_BITS_SINC_PPG_CFGA__PPG_SINC_DEC 0x1C
#define REG_BITS_SINC_PPG_CFGB__PPG_SEL_ORDER 0x02
#define REG_BITS_SINC_PPG_CFGB__PPG_FILTER_MODE 0x01

#define REG_BITS_AGE_GSR__GSR_DAC_ON 0x10
#define REG_BITS_AGE_GSR__GSR_DAC_DIV 0x07

#define NUM_PPG_FREQUENCIES 11
#define NUM_ECG_FREQUENCIES 12

#define REG_BITS_STATUS_MOD__PPG_ASAT 0x0F
#define REG_BITS_STATUS_MOD__ECG_ASAT 0xF0

#define REG_BITS_REVISION__REVISION 0x0F

#define REG_BITS_OVS_CFG__ECG_OVS_SAMPLE 0x07
#define REG_BITS_OVS_CFG__ECG_OVS_SAMPLE_SHIFT 0
#define REG_BITS_OVS_CFG__PPG_OVS_SAMPLE 0x70
#define REG_BITS_OVS_CFG__PPG_OVS_SAMPLE_SHIFT 4

struct reg_group_entry {
    uint8_t reg_addr;
    uint8_t reg_count;
};

struct reg_group {
    uint8_t size;
    struct reg_group_entry entries[3];
};

struct d_max_groups {
    uint8_t sinc_ppg_cfga_reg;
    uint8_t sinc_ppg_cfgb_reg;
    uint32_t d_max;
};

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

static uint8_t g_is_pd_offset_workaround = FALSE;

const struct reg_group g_reg_groups[AS7050_REG_GROUP_ID_NUM] = {
    {sizeof(as7050_config_ctrl_t), {{AS7050_REGADDR_CONTROL, 1}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_gpio_t),
     {{AS7050_REGADDR_GPIO1_CFG, 2}, {AS7050_REGADDR_GPIO1_CFGB, 2}, {AS7050_REGADDR_GPIO_IO, 1}}},
    {sizeof(as7050_config_standby_t), {{AS7050_REGADDR_STANDBY_CFGA, 2}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_iir_t), {{AS7050_REGADDR_IIR_CFG, 1}, {AS7050_REGADDR_IIR_COEFF_DATA, 120}, {0, 0}}},
    {sizeof(as7050_config_ref_t), {{AS7050_REGADDR_REF_CFGA, 2}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_aoc_t), {{AS7050_REGADDR_AOC_IOS_PPG1, 16}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_ppg_t), {{AS7050_REGADDR_PPG_MOD_CFGA, 5}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_ecg_t), {{AS7050_REGADDR_ECG_SOURCE, 2}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_amp_t), {{AS7050_REGADDR_ECG_AMP_CFGA, 3}, {AS7050_REGADDR_ECG_AMP_CFGE, 1}, {0, 0}}},
    {sizeof(as7050_config_tia_t),
     {{AS7050_REGADDR_PD_OFFSET_CFG, 1}, {AS7050_REGADDR_TIA_CFGA, 2}, {AS7050_REGADDR_TIA_CFGC, 1}}},
    {sizeof(as7050_config_afe_t), {{AS7050_REGADDR_AFE_DAC0L, 5}, {AS7050_REGADDR_AFE_GSR, 1}, {0, 0}}},
    {sizeof(as7050_config_sinc_t), {{AS7050_REGADDR_SINC_PPG_CFGA, 6}, {AS7050_REGADDR_OVS_CFG, 1}, {0, 0}}},
    {sizeof(as7050_config_led_t), {{AS7050_REGADDR_LOWVDS_WAIT, 9}, {AS7050_REGADDR_LED_INIT, 11}, {0, 0}}},
    {sizeof(as7050_config_pd_t), {{AS7050_REGADDR_PDSEL_CFG, 1}, {AS7050_REGADDR_PD_PPG1, 9}, {0, 0}}},
    {sizeof(as7050_config_fifo_t), {{AS7050_REGADDR_FIFO_CTRL, 2}, {0, 0}, {0, 0}}},
    {sizeof(as7050_config_seq_t),
     {{AS7050_REGADDR_CGB_CFG, 1}, {AS7050_REGADDR_SEQ_SAMPLE, 3}, {AS7050_REGADDR_SEQ_MODE, 1}}},
};

/*
sincPPG         1       2       3       4       5       6       7       8       9       10      11      12
SINC_PPG_CFGA   0x00    0x04    0x08    0x0C    0x10    0x00    0x04    0x08    0x0C    0x10    0x00    0x04
SINC_PPG_CFGB   0x00    0x00    0x00    0x00    0x00    0x02    0x02    0x02    0x02    0x02    0x01    0x01
Dmax            4845    58905   407192  377587  363366  20349   217948 351203   313869  296370  65536   524288

sincPPG         13      14      15      16      17      18      19      20
SINC_PPG_CFGA   0x08    0x0C    0x10    0x00    0x04    0x08    0x0C    0x10
SINC_PPG_CFGB   0x01    0x01    0x01    0x03    0x03    0x03    0x03    0x03
Dmax            524288  524288  524288  524288  524288  524288  524288  524288

*/
static const struct d_max_groups g_d_max_groups[20] = {
    {0x00, 0x00, 4845},   {0x04, 0x00, 58905},  {0x08, 0x00, 407192}, {0x0C, 0x00, 377587}, {0x10, 0x00, 363366},
    {0x00, 0x02, 20349},  {0x04, 0x02, 217948}, {0x08, 0x02, 351203}, {0x0C, 0x02, 313869}, {0x10, 0x02, 296370},
    {0x00, 0x01, 65536},  {0x04, 0x01, 524288}, {0x08, 0x01, 524288}, {0x0C, 0x01, 524288}, {0x10, 0x01, 524288},
    {0x00, 0x03, 524288}, {0x04, 0x03, 524288}, {0x08, 0x03, 524288}, {0x0C, 0x03, 524288}, {0x10, 0x03, 524288}};

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t as7050_ifce_write_register(uint8_t reg_addr, uint8_t reg_val)
{
    return as7050_osal_write_registers(reg_addr, sizeof(reg_val), &reg_val);
}

err_code_t as7050_ifce_read_register(uint8_t reg_addr, uint8_t *p_reg_val)
{
    M_CHECK_NULL_POINTER(p_reg_val);
    return as7050_osal_read_registers(reg_addr, sizeof(*p_reg_val), p_reg_val);
}

err_code_t as7050_ifce_reset_chip(void)
{
    err_code_t result;
    uint8_t value = 0;

    result = as7050_ifce_write_register(AS7050_REGADDR_CONTROL, REG_BITS_CONTROL__SW_RESET);

    if (ERR_SUCCESS == result) {
        /* Read register to get reset status bit.
           Reading of this register clears this bit */
        result = as7050_ifce_read_register(AS7050_REGADDR_CONTROL, &value);
    }

    return result;
}

err_code_t as7050_ifce_get_revision(uint8_t *p_revision)
{
    err_code_t result;

    M_CHECK_NULL_POINTER(p_revision);

    result = as7050_ifce_read_register(AS7050_REGADDR_REVISION, p_revision);
    if (ERR_SUCCESS == result) {
        *p_revision &= REG_BITS_REVISION__REVISION;
    }

    return result;
}

err_code_t as7050_ifce_clear_fifo(void)
{
    err_code_t result;
    uint8_t reg_val;

    result = as7050_ifce_read_register(AS7050_REGADDR_FIFO_CTRL, &reg_val);

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_write_register(AS7050_REGADDR_FIFO_CTRL, reg_val | REG_BITS_FIFO_CTRL__FIFO_CLEAR);
    }

    return result;
}

err_code_t as7050_ifce_get_fifo_level(uint16_t *p_fifo_level)
{
    err_code_t result;

    M_CHECK_NULL_POINTER(p_fifo_level);

    result = as7050_osal_read_registers(AS7050_REGADDR_FIFO_LEVEL0, sizeof(*p_fifo_level), (uint8_t *)p_fifo_level);
    if ((ERR_SUCCESS == result) && ((REG_BITS_FIFO_LEVEL1__FIFO_OVERFLOW << 8) & *p_fifo_level)) {
        result = ERR_OVERFLOW;
    }
    return result;
}

err_code_t as7050_ifce_get_fifo_threshold(uint16_t *p_fifo_threshold, uint8_t *p_sample_size)
{
    err_code_t result;

    M_CHECK_NULL_POINTER(p_fifo_threshold);

    result =
        as7050_osal_read_registers(AS7050_REGADDR_FIFO_CTRL, sizeof(*p_fifo_threshold), (uint8_t *)p_fifo_threshold);
    if (ERR_SUCCESS == result) {
        if (NULL != p_sample_size) {
            *p_sample_size = 3 + ((*p_fifo_threshold & REG_BITS_FIFO_CTRL__DATA_MODE) == REG_BITS_FIFO_CTRL__DATA_MODE);
        }
        if (REG_BITS_FIFO_CTRL__FIFO_THRESHOLD_8 & *p_fifo_threshold) {
            *p_fifo_threshold = 0x100 | ((*p_fifo_threshold >> 8) & 0xFF);
        } else {
            *p_fifo_threshold = (*p_fifo_threshold >> 8) & 0xFF;
        }
        /* increment because interrupt fires only if FIFO_LEVEL > FIFO_THRESHOLD and not FIFO_LEVEL >= FIFO_THRESHOLD */
        /* prevent division by zero */
        (*p_fifo_threshold)++;
    }
    return result;
}

err_code_t as7050_ifce_get_interrupt_status(as7050_interrupt_status_t *p_status)
{
    M_CHECK_NULL_POINTER(p_status);

    return as7050_ifce_read_register(AS7050_REGADDR_STATUS, (uint8_t *)p_status);
}

err_code_t as7050_ifce_set_reg_group(enum as7050_reg_group_ids id, const uint8_t *p_data, uint8_t size)
{
    err_code_t result = ERR_SUCCESS;
    uint8_t i, total_size = 0;

    M_CHECK_NULL_POINTER(p_data);
    M_CHECK_ARGUMENT_LOWER(id, AS7050_REG_GROUP_ID_NUM);
    M_CHECK_SIZE(size, g_reg_groups[id].size);

    if (AS7050_REG_GROUP_ID_IIR == id) {
        result = as7050_ifce_write_register(g_reg_groups[id].entries[0].reg_addr, p_data[0]);
        if (ERR_SUCCESS == result) {
            total_size++;
            /* set sub address to zero. Autoincrement is supported */
            result = as7050_ifce_write_register(AS7050_REGADDR_IIR_COEFF_ADDR, 0);
        }
        if (ERR_SUCCESS == result) {
            result = as7050_osal_write_registers(g_reg_groups[id].entries[1].reg_addr,
                                                 g_reg_groups[id].entries[1].reg_count, (uint8_t *)p_data + 1);
            total_size += g_reg_groups[id].entries[1].reg_count;
        }
    } else {
        for (i = 0; ERR_SUCCESS == result && i < sizeof(g_reg_groups[id].entries) / sizeof(g_reg_groups[id].entries[0]);
             i++) {
            if (0 == g_reg_groups[id].entries[i].reg_count) {
                break;
            }
            if (total_size + g_reg_groups[id].entries[i].reg_count <= g_reg_groups[id].size) {
                result =
                    as7050_osal_write_registers(g_reg_groups[id].entries[i].reg_addr,
                                                g_reg_groups[id].entries[i].reg_count, (uint8_t *)p_data + total_size);
                total_size += g_reg_groups[id].entries[i].reg_count;
            }
        }
    }

    if (ERR_SUCCESS == result && total_size != g_reg_groups[id].size) {
        result = ERR_SIZE;
    }

    return result;
}

err_code_t as7050_ifce_get_reg_group(enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t *p_size)
{
    err_code_t result = ERR_SUCCESS;
    uint8_t i, total_size = 0;

    M_CHECK_ARGUMENT_LOWER(id, AS7050_REG_GROUP_ID_NUM);
    M_CHECK_NULL_POINTER(p_size);

    /* return size of given group id */
    if ((NULL == p_data) && (0 == *p_size)) {
        *p_size = g_reg_groups[id].size;
        return ERR_SUCCESS;
    }

    M_CHECK_NULL_POINTER(p_data);

    if (g_reg_groups[id].size > *p_size) {
        return ERR_SIZE;
    }

    if (AS7050_REG_GROUP_ID_IIR == id) {
        result = as7050_ifce_read_register(g_reg_groups[id].entries[0].reg_addr, p_data);
        if (ERR_SUCCESS == result) {
            total_size++;
            /* set sub address to zero. Autoincrement is supported */
            result = as7050_ifce_write_register(AS7050_REGADDR_IIR_COEFF_ADDR, 0);
        }
        if (ERR_SUCCESS == result) {
            result = as7050_osal_read_registers(g_reg_groups[id].entries[1].reg_addr,
                                                g_reg_groups[id].entries[1].reg_count, p_data + 1);
            total_size += g_reg_groups[id].entries[1].reg_count;
        }
    } else {
        for (i = 0; ERR_SUCCESS == result && i < sizeof(g_reg_groups[id].entries) / sizeof(g_reg_groups[id].entries[0]);
             i++) {
            if (0 == g_reg_groups[id].entries[i].reg_count) {
                break;
            }
            if (total_size + g_reg_groups[id].entries[i].reg_count <= g_reg_groups[id].size) {
                result = as7050_osal_read_registers(g_reg_groups[id].entries[i].reg_addr,
                                                    g_reg_groups[id].entries[i].reg_count, p_data + total_size);
                total_size += g_reg_groups[id].entries[i].reg_count;
            }
        }
    }

    if (ERR_SUCCESS == result && total_size != g_reg_groups[id].size) {
        result = ERR_SIZE;
    }

    *p_size = g_reg_groups[id].size;

    return result;
}

err_code_t as7050_ifce_get_ppg_ecg_sample_periods(uint32_t *p_ppg_sample_period_us, uint32_t *p_ecg_sample_period_us)
{
    err_code_t result;
    uint8_t reg_val;

    const uint16_t ppg_freq[NUM_PPG_FREQUENCIES] = {1, 5, 20, 25, 50, 100, 200, 250, 400, 500, 1000};
    const uint16_t ecg_freq[NUM_ECG_FREQUENCIES] = {20, 25, 50, 100, 200, 250, 400, 500, 1000, 2000, 4000, 8000};

    M_CHECK_NULL_POINTER(p_ppg_sample_period_us);
    M_CHECK_NULL_POINTER(p_ecg_sample_period_us);

    result = as7050_ifce_read_register(AS7050_REGADDR_SEQ_SAMPLE, &reg_val);
    if (ERR_SUCCESS == result) {
        if (NUM_PPG_FREQUENCIES <= (reg_val >> 4)) {
            *p_ppg_sample_period_us = 0;
        } else {
            *p_ppg_sample_period_us = 1000000 / (uint32_t)ppg_freq[reg_val >> 4];
        }
        if (NUM_ECG_FREQUENCIES <= (reg_val & 0x0F)) {
            *p_ecg_sample_period_us = 0;
        } else {
            *p_ecg_sample_period_us = 1000000 / (uint32_t)ecg_freq[reg_val & 0x0F];
        }

        result = as7050_ifce_read_register(AS7050_REGADDR_OVS_CFG, &reg_val);
    }

    if (ERR_SUCCESS == result) {

        /* ECG multiplier */
        *p_ecg_sample_period_us <<=
            (uint32_t)((reg_val & REG_BITS_OVS_CFG__ECG_OVS_SAMPLE) >> REG_BITS_OVS_CFG__ECG_OVS_SAMPLE_SHIFT);

        /* PPG multiplier */
        *p_ppg_sample_period_us <<=
            (uint32_t)((reg_val & REG_BITS_OVS_CFG__PPG_OVS_SAMPLE) >> REG_BITS_OVS_CFG__PPG_OVS_SAMPLE_SHIFT);
    }

    return result;
}

err_code_t as7050_ifce_start_measurement(void)
{
    err_code_t result;
    uint8_t reg_val;
    as7050_interrupt_status_t status;

    /* check if PPG or ECG are enabled */
    result = as7050_ifce_read_register(AS7050_REGADDR_SEQ_MODE, &reg_val);
    if (ERR_SUCCESS == result) {
        if (0 == ((REG_BITS_SEQ_MODE__PPG_EN | REG_BITS_SEQ_MODE__ECG_EN) & reg_val)) {
            result = ERR_CONFIG;
        }
    }

    /* Reset saturation status via reading STATUS_MOD register */
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_saturation_status(NULL, NULL);
    }

    /* read old interrupt status register to reset interrupt pin */
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_get_interrupt_status(&status);
    }

    /* enable interrupts */
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_write_register(AS7050_REGADDR_IRQ_ENABLE, REG_BITS_IRQ_ENABLE__IRQ_EN_FIFOOVERFLOW |
                                                                           REG_BITS_IRQ_ENABLE__IRQ_EN_FIFOTHRESHOLD);
    }

    /* Clear FIFO level register and possible active interrupt pin */
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_clear_fifo();
    }

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_write_register(AS7050_REGADDR_SEQ_START, REG_BITS_SEQ_START__START_SEQ);
    }

    return result;
}

err_code_t as7050_ifce_stop_measurement(void)
{
    return as7050_ifce_write_register(AS7050_REGADDR_SEQ_START, 0);
}

err_code_t as7050_ifce_get_fifo_data(uint8_t *p_data, uint16_t data_num)
{
    return as7050_osal_read_registers(AS7050_REGADDR_FIFOL, data_num, p_data);
}

err_code_t as7050_ifce_get_channel_mapping(uint16_t *p_channel_map)
{
    err_code_t result;
    uint8_t seq_mode;
    uint8_t sub_sample;
    uint8_t afe_gsr;
    uint8_t i;

    M_CHECK_NULL_POINTER(p_channel_map);

    result = as7050_ifce_read_register(AS7050_REGADDR_SEQ_MODE, &seq_mode);
    if (ERR_SUCCESS == result) {
        result = as7050_ifce_read_register(AS7050_REGADDR_SEQ_PPGA, &sub_sample);
    }
    if (ERR_SUCCESS == result) {
        *p_channel_map = 0;
        if (REG_BITS_SEQ_MODE__ECG_EN & seq_mode) {
            *p_channel_map |= AS7050_CHANNEL_FLAG_ECG;
        }
        if (REG_BITS_SEQ_MODE__PPG_EN & seq_mode) {
            sub_sample &= REG_BITS_SEQ_PPGA__PPG_SUB_SAMPLE;
            for (i = 0; i <= sub_sample; i++) {
                *p_channel_map |= AS7050_CHANNEL_FLAG_PPG_1 << i;
            }
        }

        /* get gsr configuration */
        result = as7050_ifce_read_register(AS7050_REGADDR_AFE_GSR, &afe_gsr);
    }
    if ((ERR_SUCCESS == result) && (REG_BITS_AGE_GSR__GSR_DAC_ON & afe_gsr)) {
        *p_channel_map |= ((REG_BITS_AGE_GSR__GSR_DAC_DIV & afe_gsr) + 1) << AS7050_CHANNEL_FLAG_GSR_OFFSET;
    }
    return result;
}

err_code_t as7050_ifce_get_saturation_status(uint8_t *p_ppg_sat, uint8_t *p_ecg_sat)
{
    err_code_t result;
    uint8_t reg_val;

    result = as7050_ifce_read_register(AS7050_REGADDR_STATUS_MOD, &reg_val);
    if (ERR_SUCCESS == result) {
        if (NULL != p_ppg_sat) {
            *p_ppg_sat = !!(REG_BITS_STATUS_MOD__PPG_ASAT & reg_val);
        }
        if (NULL != p_ecg_sat) {
            *p_ecg_sat = !!(REG_BITS_STATUS_MOD__ECG_ASAT & reg_val);
        }
    }
    return result;
}

err_code_t as7050_ifce_get_max_adc_count(uint32_t *p_max_adc_count, uint8_t *p_max_adc_bits)
{
    err_code_t result;
    uint8_t sinc_ppg_regs[2];
    uint8_t i;
    uint32_t d_max;

    M_CHECK_NULL_POINTER(p_max_adc_count);
    M_CHECK_NULL_POINTER(p_max_adc_bits);

    result = as7050_osal_read_registers(AS7050_REGADDR_SINC_PPG_CFGA, sizeof(sinc_ppg_regs), sinc_ppg_regs);

    if (ERR_SUCCESS == result) {
        for (i = 0; i < (sizeof(g_d_max_groups) / sizeof(g_d_max_groups[0])); i++) {
            if (((sinc_ppg_regs[0] & REG_BITS_SINC_PPG_CFGA__PPG_SINC_DEC) == g_d_max_groups[i].sinc_ppg_cfga_reg) &&
                ((sinc_ppg_regs[1] &
                  (REG_BITS_SINC_PPG_CFGB__PPG_SEL_ORDER | REG_BITS_SINC_PPG_CFGB__PPG_FILTER_MODE)) ==
                 g_d_max_groups[i].sinc_ppg_cfgb_reg)) {
                *p_max_adc_count = g_d_max_groups[i].d_max;

                *p_max_adc_bits = 0;
                d_max = g_d_max_groups[i].d_max - 1;
                while (0 < d_max) {
                    (*p_max_adc_bits)++;
                    d_max >>= 1;
                }
                break;
            }
        }
        if ((sizeof(g_d_max_groups) / sizeof(g_d_max_groups[0])) <= i) {
            result = ERR_CONFIG;
        }
    }

    return result;
}

err_code_t as7050_ifce_set_led_current(as7050_channel_t led_channel, uint8_t current)
{
    if (AS7050_CHANNEL_PPG_1 > led_channel || AS7050_CHANNEL_PPG_8 < led_channel) {
        return ERR_ARGUMENT;
    }

    /* The highest bit of the current register sets the range: 0 - 150mA, 1 - 300mA.
       We want to set a linear current curve. Therefore, we modify values above 127 digits. */
    if (127 < current) {
        current = 0x80 | (current / 2);
    }

    return as7050_ifce_write_register(AS7050_REGADDR_LED1_ICTRL + led_channel - AS7050_CHANNEL_PPG_1, current);
}

err_code_t as7050_ifce_set_pd_offset_workaround(uint8_t enable)
{
    g_is_pd_offset_workaround = !!enable;
    return ERR_SUCCESS;
}

err_code_t as7050_ifce_set_pd_offset(as7050_pd_offset_config_t *p_pd_offset_config, uint8_t num)
{
    uint8_t reg;
    err_code_t result = ERR_SUCCESS;
    uint8_t measurement_start = 0;
    uint8_t i;
    uint8_t reg_val;

    M_CHECK_NULL_POINTER(p_pd_offset_config);

    /* check arguments */
    for (i = 0; i < num; i++) {
        if (AS7050_CHANNEL_DISABLED == p_pd_offset_config[i].channel_type ||
            AS7050_CHANNEL_TIA < p_pd_offset_config[i].channel_type) {
            return ERR_ARGUMENT;
        }
    }

    /* check if measurement is already started */
    if (g_is_pd_offset_workaround) {
        result = as7050_ifce_read_register(AS7050_REGADDR_SEQ_START, &measurement_start);
        /* stop measurement, because PD offset can't be set during measurement */
        if (ERR_SUCCESS == result && (REG_BITS_SEQ_START__START_SEQ & measurement_start)) {
            result = as7050_ifce_write_register(AS7050_REGADDR_SEQ_START, 0);
        }
    }

    for (i = 0; (i < num) && (ERR_SUCCESS == result); i++) {

        if (AS7050_CHANNEL_TIA == p_pd_offset_config[i].channel_type) {
            reg = AS7050_REGADDR_AOC_IOS_ECG;
        } else {
            reg = AS7050_REGADDR_AOC_IOS_PPG1 + p_pd_offset_config[i].channel_type - 1;
        }

        if (g_is_pd_offset_workaround) {
            /* use register value directly on chip revision 0 and 1 */
            reg_val = p_pd_offset_config[i].pd_offset;
        } else {
            /* chip revision >= 2: PD offset is inverted now */
            reg_val = UINT8_MAX - p_pd_offset_config[i].pd_offset;
        }

        /* set PD offset */
        if (ERR_SUCCESS == result) {
            result = as7050_ifce_write_register(reg, reg_val);
        }
    }

    /* restart measurement */
    if (ERR_SUCCESS == result && (REG_BITS_SEQ_START__START_SEQ & measurement_start)) {
        result = as7050_ifce_write_register(AS7050_REGADDR_SEQ_START, measurement_start);
    }

    return result;
}

err_code_t as7050_ifce_get_channel_led_config(as7050_channel_t channel_type, uint8_t *p_led_config)
{
    if (AS7050_CHANNEL_DISABLED == channel_type || AS7050_CHANNEL_TIA < channel_type) {
        return ERR_ARGUMENT;
    }
    return as7050_ifce_read_register(AS7050_REGADDR_LED_PPG1 + channel_type - AS7050_CHANNEL_PPG_1, p_led_config);
}
