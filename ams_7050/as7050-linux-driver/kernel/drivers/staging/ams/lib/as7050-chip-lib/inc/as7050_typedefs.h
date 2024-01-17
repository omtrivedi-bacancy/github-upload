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

#ifndef __AS7050_TYPEDEFS_H__
#define __AS7050_TYPEDEFS_H__

/*!
 * \file      as7050_typedefs.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup definition_group
 *
 * \brief Description of the used data types.
 *
 * These are the type definitions used by AS7050 chip library.
 *
 *  @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_std_include.h"
#include "error_codes.h"

/* USER_CODE BEGIN INCLUDES */

/* USER_CODE END INCLUDES */

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/* USER_CODE BEGIN DEFINITIONS */

#define AS7050_MAX_GROUP_SIZE 128 /*!< maximum space for payload of register groups */

/*! Definition of the register group IDs */
typedef enum as7050_reg_group_ids {
    AS7050_REG_GROUP_ID_CTRL = 0,    /*!< Group-ID of all registers of group 'Control'. See ::as7050_config_ctrl_t */
    AS7050_REG_GROUP_ID_GPIO = 1,    /*!< Group-ID of all registers of group 'GPIO'. See ::as7050_config_gpio_t */
    AS7050_REG_GROUP_ID_STANDBY = 2, /*!< Group-ID of all registers of group 'Standby'. See ::as7050_config_standby_t */
    AS7050_REG_GROUP_ID_IIR = 3,     /*!< Group-ID of all registers of group 'IIR filter'. See ::as7050_config_iir_t */
    AS7050_REG_GROUP_ID_REF = 4,     /*!< Group-ID of all registers of group 'Reference'. See ::as7050_config_ref_t */
    AS7050_REG_GROUP_ID_AOC =
        5, /*!< Group-ID of all registers of group 'Automatic Offset Control'. See ::as7050_config_aoc_t */
    AS7050_REG_GROUP_ID_PPG = 6, /*!< Group-ID of all registers of group 'PPG'. See ::as7050_config_ppg_t */
    AS7050_REG_GROUP_ID_ECG = 7, /*!< Group-ID of all registers of group 'ECG'. See ::as7050_config_ecg_t */
    AS7050_REG_GROUP_ID_AMP = 8, /*!< Group-ID of all registers of group 'amplifier'. See ::as7050_config_amp_t */
    AS7050_REG_GROUP_ID_TIA =
        9, /*!< Group-ID of all registers of group 'Transimpedance amplifier'. See ::as7050_config_tia_t */
    AS7050_REG_GROUP_ID_AFE =
        10, /*!< Group-ID of all registers of group 'Analog front end'. See ::as7050_config_afe_t */
    AS7050_REG_GROUP_ID_SINC = 11, /*!< Group-ID of all registers of group 'Sinc filter'. See ::as7050_config_sinc_t */
    AS7050_REG_GROUP_ID_LED = 12,  /*!< Group-ID of all registers of group 'LED'. See ::as7050_config_led_t */
    AS7050_REG_GROUP_ID_PD = 13,   /*!< Group-ID of all registers of group 'Photodiodes'. See ::as7050_config_pd_t */
    AS7050_REG_GROUP_ID_FIFO = 14, /*!< Group-ID of all registers of group 'FIFO'. See ::as7050_config_fifo_t */
    AS7050_REG_GROUP_ID_SEQ = 15,  /*!< Group-ID of all registers of group 'Sequencer'. See ::as7050_config_seq_t */

    AS7050_REG_GROUP_ID_NUM = 16, /*!< Maximum number of supported register groups */
} as7050_reg_group_ids_t;

/*! Channel flags */
typedef enum {
    AS7050_CHANNEL_FLAG_NONE = 0x0000,   /*!< No channel is selected */
    AS7050_CHANNEL_FLAG_PPG_1 = 0x0001,  /*!< Flag for PPG 1 channel */
    AS7050_CHANNEL_FLAG_PPG_2 = 0x0002,  /*!< Flag for PPG 2 channel */
    AS7050_CHANNEL_FLAG_PPG_3 = 0x0004,  /*!< Flag for PPG 3 channel */
    AS7050_CHANNEL_FLAG_PPG_4 = 0x0008,  /*!< Flag for PPG 4 channel */
    AS7050_CHANNEL_FLAG_PPG_5 = 0x0010,  /*!< Flag for PPG 5 channel */
    AS7050_CHANNEL_FLAG_PPG_6 = 0x0020,  /*!< Flag for PPG 6 channel */
    AS7050_CHANNEL_FLAG_PPG_7 = 0x0040,  /*!< Flag for PPG 7 channel */
    AS7050_CHANNEL_FLAG_PPG_8 = 0x0080,  /*!< Flag for PPG 8 channel */
    AS7050_CHANNEL_FLAG_ECG = 0x0100,    /*!< Flag for ECG channel */
    AS7050_CHANNEL_FLAG_STATUS = 0x0200, /*!< Flag for status channel */

    AS7050_CHANNEL_FLAG_GSR = 0xF000, /*!< When performing GSR measurement, DAC 0 and DAC 1 are measured on the ECG
                                           channel. The chip alternates between DAC 0 and DAC 1 automatically. The bits
                                           masked by this bitmask contain the number of samples that are taken per DAC
                                           before the chip alternates to the other DAC. */
} as7050_channel_flags_t;

/*! Bit position of GSR data inside ::as7050_channel_flags_t */
#define AS7050_CHANNEL_FLAG_GSR_OFFSET 12

/*! First two bits of the measurement data, which describes the type of the sample */
enum FIFO_DATA_MARKERS {
    FIFO_DATA_MARKER_PPG_2_8 = 0x00, /*!< PPG sub samples 2 - 8 */
    FIFO_DATA_MARKER_ECG = 0x01,     /*!< ECG sample */
    FIFO_DATA_MARKER_PPG_1 = 0x02,   /*!< First PPG sample */
    FIFO_DATA_MARKER_STATUS = 0x03,  /*!< Status marker for AOC information */
};

/*! Control states for DAC reference calculation */
enum dac_ref_control {
    DAC_REF_CONTROL_CALC = 0, /*!< Start or continue the DAC reference calculation */
    DAC_REF_CONTROL_ABORT,    /*!< Abort the DAC reference calculation */
};

/*! Type for ::dac_ref_control. */
typedef uint8_t dac_ref_control_t;

/*! Status for DAC reference calculation */
enum dac_ref_status {
    DAC_REF_STATUS_RUNNING = 0, /*!< DAC reference calculation is running */
    DAC_REF_STATUS_ABORTED,     /*!< DAC reference calculation was aborted */
    DAC_REF_STATUS_FINISHED,    /*!< DAC reference calculation is finished */
};

/*! Type for ::dac_ref_status. */
typedef uint8_t dac_ref_status_t;

/*! Register definition of AS7050 */
enum as7050_reg_addresses {
    AS7050_REGADDR_GPIO1_CFG = 0x10,      /*!< Register GPIO1_CFG */
    AS7050_REGADDR_GPIO2_CFG = 0x11,      /*!< Register GPIO2_CFG */
    AS7050_REGADDR_INT_CFG = 0x12,        /*!< Register INT_CFG */
    AS7050_REGADDR_IO_CFGA = 0x13,        /*!< Register IO_CFGA */
    AS7050_REGADDR_IO_CFGB = 0x14,        /*!< Register IO_CFGB */
    AS7050_REGADDR_GPIO1_CFGB = 0x15,     /*!< Register GPIO1_CFGB */
    AS7050_REGADDR_GPIO2_CFGB = 0x16,     /*!< Register GPIO2_CFGB */
    AS7050_REGADDR_INT_CFGB = 0x17,       /*!< Register INT_CFGB */
    AS7050_REGADDR_PD_OFFSET_CFG = 0x18,  /*!< Register PD_OFFSET_CFG */
    AS7050_REGADDR_PPG_MOD_CFGA = 0x19,   /*!< Register PPG_MOD_CFGA */
    AS7050_REGADDR_PPG_MOD_CFGB = 0x1a,   /*!< Register PPG_MOD_CFGB */
    AS7050_REGADDR_PPG_MOD_CFGC = 0x1b,   /*!< Register PPG_MOD_CFGC */
    AS7050_REGADDR_PPG_MOD_CFGD = 0x1c,   /*!< Register PPG_MOD_CFGD */
    AS7050_REGADDR_PPG_MOD_CFGE = 0x1d,   /*!< Register PPG_MOD_CFGE */
    AS7050_REGADDR_TIA_CFGA = 0x1e,       /*!< Register TIA_CFGA */
    AS7050_REGADDR_TIA_CFGB = 0x1f,       /*!< Register TIA_CFGB */
    AS7050_REGADDR_ECG_AMP_CFGA = 0x20,   /*!< Register ECG_AMP_CFGA */
    AS7050_REGADDR_ECG_AMP_CFGB = 0x21,   /*!< Register ECG_AMP_CFGB */
    AS7050_REGADDR_ECG_AMP_CFGC = 0x22,   /*!< Register ECG_AMP_CFGC */
    AS7050_REGADDR_PDSEL_CFG = 0x23,      /*!< Register PDSEL_CFG */
    AS7050_REGADDR_ECG_SOURCE = 0x24,     /*!< Register ECG_SOURCE */
    AS7050_REGADDR_ECG_MOD_CFGA = 0x25,   /*!< Register ECG_MOD_CFGA */
    AS7050_REGADDR_TIA_CFGC = 0x26,       /*!< Register TIA_CFGC */
    AS7050_REGADDR_LOWVDS_WAIT = 0x27,    /*!< Register LOWVDS_WAIT */
    AS7050_REGADDR_LED1_ICTRL = 0x28,     /*!< Register LED1_ICTRL */
    AS7050_REGADDR_LED2_ICTRL = 0x29,     /*!< Register LED2_ICTRL */
    AS7050_REGADDR_LED3_ICTRL = 0x2a,     /*!< Register LED3_ICTRL */
    AS7050_REGADDR_LED4_ICTRL = 0x2b,     /*!< Register LED4_ICTRL */
    AS7050_REGADDR_LED5_ICTRL = 0x2c,     /*!< Register LED5_ICTRL */
    AS7050_REGADDR_LED6_ICTRL = 0x2d,     /*!< Register LED6_ICTRL */
    AS7050_REGADDR_LED7_ICTRL = 0x2e,     /*!< Register LED7_ICTRL */
    AS7050_REGADDR_LED8_ICTRL = 0x2f,     /*!< Register LED8_ICTRL */
    AS7050_REGADDR_REF_CFGA = 0x30,       /*!< Register REF_CFGA */
    AS7050_REGADDR_REF_CFGB = 0x31,       /*!< Register REF_CFGB */
    AS7050_REGADDR_AFE_DAC0L = 0x32,      /*!< Register AFE_DAC0L */
    AS7050_REGADDR_AFE_DAC1L = 0x33,      /*!< Register AFE_DAC1L */
    AS7050_REGADDR_AFE_DACH = 0x34,       /*!< Register AFE_DACH */
    AS7050_REGADDR_AFE_CFGA = 0x35,       /*!< Register AFE_CFGA */
    AS7050_REGADDR_AFE_CFGB = 0x36,       /*!< Register AFE_CFGB */
    AS7050_REGADDR_CONTROL = 0x37,        /*!< Register CONTROL */
    AS7050_REGADDR_CGB_CFG = 0x38,        /*!< Register CGB_CFG */
    AS7050_REGADDR_ECG_AMP_CFGE = 0x39,   /*!< Register ECG_AMP_CFGE */
    AS7050_REGADDR_AFE_GSR = 0x3a,        /*!< Register AFE_GSR */
    AS7050_REGADDR_SEQ_SAMPLE = 0x40,     /*!< Register SEQ_SAMPLE */
    AS7050_REGADDR_SEQ_PPGA = 0x41,       /*!< Register SEQ_PPGA */
    AS7050_REGADDR_SEQ_PPGB = 0x42,       /*!< Register SEQ_PPGB */
    AS7050_REGADDR_PD_PPG1 = 0x43,        /*!< Register PD_PPG1 */
    AS7050_REGADDR_PD_PPG2 = 0x44,        /*!< Register PD_PPG2 */
    AS7050_REGADDR_PD_PPG3 = 0x45,        /*!< Register PD_PPG3 */
    AS7050_REGADDR_PD_PPG4 = 0x46,        /*!< Register PD_PPG4 */
    AS7050_REGADDR_PD_PPG5 = 0x47,        /*!< Register PD_PPG5 */
    AS7050_REGADDR_PD_PPG6 = 0x48,        /*!< Register PD_PPG6 */
    AS7050_REGADDR_PD_PPG7 = 0x49,        /*!< Register PD_PPG7 */
    AS7050_REGADDR_PD_PPG8 = 0x4a,        /*!< Register PD_PPG8 */
    AS7050_REGADDR_PD_TIA = 0x4b,         /*!< Register PD_TIA */
    AS7050_REGADDR_LED_INIT = 0x4c,       /*!< Register LED_INIT */
    AS7050_REGADDR_LED_PPG1 = 0x4d,       /*!< Register LED_PPG1 */
    AS7050_REGADDR_LED_PPG2 = 0x4e,       /*!< Register LED_PPG2 */
    AS7050_REGADDR_LED_PPG3 = 0x4f,       /*!< Register LED_PPG3 */
    AS7050_REGADDR_LED_PPG4 = 0x50,       /*!< Register LED_PPG4 */
    AS7050_REGADDR_LED_PPG5 = 0x51,       /*!< Register LED_PPG5 */
    AS7050_REGADDR_LED_PPG6 = 0x52,       /*!< Register LED_PPG6 */
    AS7050_REGADDR_LED_PPG7 = 0x53,       /*!< Register LED_PPG7 */
    AS7050_REGADDR_LED_PPG8 = 0x54,       /*!< Register LED_PPG8 */
    AS7050_REGADDR_LED_TIA = 0x55,        /*!< Register LED_TIA */
    AS7050_REGADDR_LED_MODE = 0x56,       /*!< Register LED_MODE */
    AS7050_REGADDR_SEQ_COUNT = 0x57,      /*!< Register SEQ_COUNT */
    AS7050_REGADDR_SEQ_MODE = 0x59,       /*!< Register SEQ_MODE */
    AS7050_REGADDR_SEQ_START = 0x5a,      /*!< Register SEQ_START */
    AS7050_REGADDR_SINC_PPG_CFGA = 0x5b,  /*!< Register SINC_PPG_CFGA */
    AS7050_REGADDR_SINC_PPG_CFGB = 0x5c,  /*!< Register SINC_PPG_CFGB */
    AS7050_REGADDR_SINC_PPG_CFGC = 0x5d,  /*!< Register SINC_PPG_CFGC */
    AS7050_REGADDR_SINC_ECG_CFGA = 0x5e,  /*!< Register SINC_ECG_CFGA */
    AS7050_REGADDR_SINC_ECG_CFGB = 0x5f,  /*!< Register SINC_ECG_CFGB */
    AS7050_REGADDR_SINC_ECG_CFGC = 0x60,  /*!< Register SINC_ECG_CFGC */
    AS7050_REGADDR_IIR_CFG = 0x61,        /*!< Register IIR_CFG */
    AS7050_REGADDR_IIR_COEFF_ADDR = 0x62, /*!< Register IIR_COEFF_ADDR */
    AS7050_REGADDR_IIR_COEFF_DATA = 0x63, /*!< Register IIR_COEFF_DATA */
    AS7050_REGADDR_OVS_CFG = 0x64,        /*!< Register OVS_CFG */
    AS7050_REGADDR_AOC_IOS_PPG1 = 0x65,   /*!< Register AOC_IOS_PPG1 */
    AS7050_REGADDR_AOC_IOS_PPG2 = 0x66,   /*!< Register AOC_IOS_PPG2 */
    AS7050_REGADDR_AOC_IOS_PPG3 = 0x67,   /*!< Register AOC_IOS_PPG3 */
    AS7050_REGADDR_AOC_IOS_PPG4 = 0x68,   /*!< Register AOC_IOS_PPG4 */
    AS7050_REGADDR_AOC_IOS_PPG5 = 0x69,   /*!< Register AOC_IOS_PPG5 */
    AS7050_REGADDR_AOC_IOS_PPG6 = 0x6a,   /*!< Register AOC_IOS_PPG6 */
    AS7050_REGADDR_AOC_IOS_PPG7 = 0x6b,   /*!< Register AOC_IOS_PPG7 */
    AS7050_REGADDR_AOC_IOS_PPG8 = 0x6c,   /*!< Register AOC_IOS_PPG8 */
    AS7050_REGADDR_AOC_PPG_THH = 0x6d,    /*!< Register AOC_PPG_THH */
    AS7050_REGADDR_AOC_PPG_THL = 0x6e,    /*!< Register AOC_PPG_THL */
    AS7050_REGADDR_AOC_PPG_CFG = 0x6f,    /*!< Register AOC_PPG_CFG */
    AS7050_REGADDR_AOC_IOS_ECG = 0x70,    /*!< Register AOC_IOS_ECG */
    AS7050_REGADDR_AOC_ECG_THH = 0x71,    /*!< Register AOC_ECG_THH */
    AS7050_REGADDR_AOC_ECG_THL = 0x72,    /*!< Register AOC_ECG_THL */
    AS7050_REGADDR_AOC_ECG_CFG = 0x73,    /*!< Register AOC_ECG_CFG */
    AS7050_REGADDR_AOC_IOS_LEDOFF = 0x74, /*!< Register AOC_IOS_LEDOFF */
    AS7050_REGADDR_FIFO_CTRL = 0x75,      /*!< Register FIFO_CTRL */
    AS7050_REGADDR_FIFO_THRESHOLD = 0x76, /*!< Register FIFO_THRESHOLD */
    AS7050_REGADDR_FIFO_LEVEL0 = 0x77,    /*!< Register FIFO_LEVEL0 */
    AS7050_REGADDR_FIFO_LEVEL1 = 0x78,    /*!< Register FIFO_LEVEL1 */
    AS7050_REGADDR_STATUS = 0x80,         /*!< Register STATUS */
    AS7050_REGADDR_STATUS_CGBA = 0x81,    /*!< Register STATUS_CGBA */
    AS7050_REGADDR_STATUS_CGBB = 0x82,    /*!< Register STATUS_CGBB */
    AS7050_REGADDR_STATUS_MOD = 0x83,     /*!< Register STATUS_MOD */
    AS7050_REGADDR_STATUS_LED = 0x84,     /*!< Register STATUS_LED */
    AS7050_REGADDR_IRQ_ENABLE = 0x88,     /*!< Register IRQ_ENABLE */
    AS7050_REGADDR_GPIO_IO = 0x8f,        /*!< Register GPIO_IO */
    AS7050_REGADDR_REVISION = 0x90,       /*!< Register REVISION */
    AS7050_REGADDR_STANDBY_CFGA = 0xb0,   /*!< Register STANDBY_CFGA */
    AS7050_REGADDR_STANDBY_CFGB = 0xb1,   /*!< Register STANDBY_CFGB */
    AS7050_REGADDR_PPG_BYTEL = 0xf4,      /*!< Register PPG_BYTEL */
    AS7050_REGADDR_PPG_BYTEM = 0xf5,      /*!< Register PPG_BYTEM */
    AS7050_REGADDR_PPG_BYTEH = 0xf6,      /*!< Register PPG_BYTEH */
    AS7050_REGADDR_ECG_BYTEL = 0xf8,      /*!< Register ECG_BYTEL */
    AS7050_REGADDR_ECG_BYTEM = 0xf9,      /*!< Register ECG_BYTEM */
    AS7050_REGADDR_ECG_BYTEH = 0xfa,      /*!< Register ECG_BYTEH */
    AS7050_REGADDR_FIFOL = 0xfc,          /*!< Register FIFOL  */
    AS7050_REGADDR_FIFOM = 0xfd,          /*!< Register FIFOM */
    AS7050_REGADDR_FIFOH = 0xfe,          /*!< Register FIFOH */
    AS7050_REGADDR_BYTE0 = 0xff,          /*!< Register BYTE0  */
};

/*! AGC channel selection */
typedef enum {
    AS7050_CHANNEL_DISABLED = 0,
    AS7050_CHANNEL_PPG_1 = 1,
    AS7050_CHANNEL_PPG_2 = 2,
    AS7050_CHANNEL_PPG_3 = 3,
    AS7050_CHANNEL_PPG_4 = 4,
    AS7050_CHANNEL_PPG_5 = 5,
    AS7050_CHANNEL_PPG_6 = 6,
    AS7050_CHANNEL_PPG_7 = 7,
    AS7050_CHANNEL_PPG_8 = 8,
    AS7050_CHANNEL_TIA = 9
} as7050_channel_t;

/*! The possible values of the iLed_change field of the STATE_AGC structure. */
typedef enum {
    AS7050_AGC_STATE_UNCHANGED = 0, /*!< AGC state was not changed. */
    AS7050_AGC_STATE_INCREASED = 1, /*!< AGC state was increased. */
    AS7050_AGC_STATE_DECREASED = 2, /*!< AGC state was decreased. */
    AS7050_AGC_STATE_ATMIN = 3,     /*!< AGC needed a decrease, but was not possible as is is at the minumum already. */
    AS7050_AGC_STATE_ATMAX = 4,   /*!< AGC needed an increase, but was not possible as is is at the maximum already. */
    AS7050_AGC_STATE_UNKNOWN = 5, /*!< No check on the AGC state was done, as the algorithm has not run yet. */
} as7050_agc_change_states_t;

/*! AGC algorithm modes */
typedef enum {
    AS7050_AGC_MODE_DISABLED = 0,        /*!< AGC-Algorithm is disables */
    AS7050_AGC_MODE_PPG_ONE_CHANNEL = 1, /*!< AGC-Algorithm is configured for HRM measurement */
    AS7050_AGC_MODE_PPG_TWO_CHANNEL = 2, /*!< AGC-Algorithm is configured for SPO2 measurement */

    AS7050_AGC_MODE_MAX_NUM = 4, /*!< only the first three modes are suported! */
} as7050_agc_mode_t;

/*! AGC channel groups to support more than one LED configuration */
typedef enum {
    AS7050_CHANNEL_GROUP_A = 0,
    AS7050_CHANNEL_GROUP_B = 1,

    AS7050_NUM_CHANNEL_GROUP = 2,
} as7050_channel_group_t;

/*! This is the external configuration structure for the AGC algorithm */
typedef struct {
    uint8_t mode;                              /*!< Definition which algorithms will be used. See ::as7050_agc_mode_t */
    uint8_t res1;                              /*!< unused parameter, only for alignment purposes */
    uint8_t channel[AS7050_NUM_CHANNEL_GROUP]; /*!< ::as7050_channel_t */
    uint8_t current_min[AS7050_NUM_CHANNEL_GROUP]; /*!< The minimum allowed LED group current */
    uint8_t current_max[AS7050_NUM_CHANNEL_GROUP]; /*!< The maximum allowed LED group current */
    int32_t threshold_min;                         /*!< Minimum threshold of the controlled signal in ADC counts */
    int32_t threshold_max;                         /*!< Maximum threshold of the controlled signal in ADC counts */
    uint16_t sample_cnt;                           /*!< Number of samples to average for calculating the signal mean */
    uint16_t reset_interval;                       /*!< Interval in milliseconds to reset temporary AGC parameters */
} as7050_agc_config_t;

/*! This is the external status structure for the AGC algorithm */
typedef struct {
    uint8_t led_change[AS7050_NUM_CHANNEL_GROUP];        /*!< Gives an information if the AGC changed the LED current.
                                                                      See ::as7050_agc_change_states_t */
    uint8_t led_current[AS7050_NUM_CHANNEL_GROUP];       /*!< Actual configured current of the LED group */
    uint8_t pd_offset_change[AS7050_NUM_CHANNEL_GROUP];  /*!< Gives an information if the AGC changed the PD offset
                                                              during LED-ON. See ::as7050_agc_change_states_t */
    uint8_t pd_offset_current[AS7050_NUM_CHANNEL_GROUP]; /*!< PD offset value */
} as7050_agc_status_t;

/*! Version information of the library */
typedef struct as7050_version {
    uint8_t major; /*!< Major version position */
    uint8_t minor; /*!< Minor version position */
    uint8_t patch; /*!< Patch version position */
} as7050_version_t;

#pragma pack(push, 1)

/*! Register group for configuration of Automatic Offset Control (AOC) */
typedef union {
    /*! Register content for AOC configuration */
    struct aoc_regs {
        uint8_t aoc_ios_ppg1;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG1 */
        uint8_t aoc_ios_ppg2;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG2 */
        uint8_t aoc_ios_ppg3;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG3 */
        uint8_t aoc_ios_ppg4;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG4 */
        uint8_t aoc_ios_ppg5;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG5 */
        uint8_t aoc_ios_ppg6;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG6 */
        uint8_t aoc_ios_ppg7;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG7 */
        uint8_t aoc_ios_ppg8;   /*!< Content of register ::AS7050_REGADDR_AOC_IOS_PPG8 */
        uint8_t aoc_ppg_thh;    /*!< Content of register ::AS7050_REGADDR_AOC_PPG_THH */
        uint8_t aoc_ppg_thl;    /*!< Content of register ::AS7050_REGADDR_AOC_PPG_THL */
        uint8_t aoc_ppg_cfg;    /*!< Content of register ::AS7050_REGADDR_AOC_PPG_CFG */
        uint8_t aoc_ios_ecg;    /*!< Content of register ::AS7050_REGADDR_AOC_IOS_ECG */
        uint8_t aoc_ecg_thh;    /*!< Content of register ::AS7050_REGADDR_AOC_ECG_THH */
        uint8_t aoc_ecg_thl;    /*!< Content of register ::AS7050_REGADDR_AOC_ECG_THL */
        uint8_t aoc_ecg_cfg;    /*!< Content of register ::AS7050_REGADDR_AOC_ECG_CFG */
        uint8_t aoc_ios_ledoff; /*!< Content of register ::AS7050_REGADDR_AOC_IOS_LEDOFF */
    } reg_vals;                 /*!< Register content for AOC configuration */
    /*! Register content for AOC configuration */
    uint8_t reg_buffer[sizeof(struct aoc_regs)];
} as7050_config_aoc_t;

/*! Register group for LED configuration */
typedef union {
    /*! Register content for LED configuration */
    struct led_regs {
        uint8_t lowvds_wait; /*!< Content of register ::AS7050_REGADDR_LOWVDS_WAIT */
        uint8_t led1_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED1_ICTRL */
        uint8_t led2_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED2_ICTRL */
        uint8_t led3_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED3_ICTRL */
        uint8_t led4_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED4_ICTRL */
        uint8_t led5_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED5_ICTRL */
        uint8_t led6_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED6_ICTRL */
        uint8_t led7_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED7_ICTRL */
        uint8_t led8_ictrl;  /*!< Content of register ::AS7050_REGADDR_LED8_ICTRL */
        uint8_t led_init;    /*!< Content of register ::AS7050_REGADDR_LED_INIT */
        uint8_t led_ppg1;    /*!< Content of register ::AS7050_REGADDR_LED_PPG1 */
        uint8_t led_ppg2;    /*!< Content of register ::AS7050_REGADDR_LED_PPG2 */
        uint8_t led_ppg3;    /*!< Content of register ::AS7050_REGADDR_LED_PPG3 */
        uint8_t led_ppg4;    /*!< Content of register ::AS7050_REGADDR_LED_PPG4 */
        uint8_t led_ppg5;    /*!< Content of register ::AS7050_REGADDR_LED_PPG5 */
        uint8_t led_ppg6;    /*!< Content of register ::AS7050_REGADDR_LED_PPG6 */
        uint8_t led_ppg7;    /*!< Content of register ::AS7050_REGADDR_LED_PPG7 */
        uint8_t led_ppg8;    /*!< Content of register ::AS7050_REGADDR_LED_PPG8 */
        uint8_t led_tia;     /*!< Content of register ::AS7050_REGADDR_LED_TIA */
        uint8_t led_mode;    /*!< Content of register ::AS7050_REGADDR_LED_MODE */
    } reg_vals;              /*!< Register content for LED configuration */
    /*! Register content for LED configuration */
    uint8_t reg_buffer[sizeof(struct led_regs)];
} as7050_config_led_t;

/*! Register group for configuration of the photodiodes */
typedef union {
    /*! Register content for photodiodes configuration */
    struct pd_regs {
        uint8_t pdsel_cfg; /*!< Content of register ::AS7050_REGADDR_PDSEL_CFG */
        uint8_t pd_ppg1;   /*!< Content of register ::AS7050_REGADDR_PD_PPG1 */
        uint8_t pd_ppg2;   /*!< Content of register ::AS7050_REGADDR_PD_PPG2 */
        uint8_t pd_ppg3;   /*!< Content of register ::AS7050_REGADDR_PD_PPG3 */
        uint8_t pd_ppg4;   /*!< Content of register ::AS7050_REGADDR_PD_PPG4 */
        uint8_t pd_ppg5;   /*!< Content of register ::AS7050_REGADDR_PD_PPG5 */
        uint8_t pd_ppg6;   /*!< Content of register ::AS7050_REGADDR_PD_PPG6 */
        uint8_t pd_ppg7;   /*!< Content of register ::AS7050_REGADDR_PD_PPG7 */
        uint8_t pd_ppg8;   /*!< Content of register ::AS7050_REGADDR_PD_PPG8 */
        uint8_t pd_tia;    /*!< Content of register ::AS7050_REGADDR_PD_TIA */
    } reg_vals;            /*!< Register content for photodiodes configuration */
    /*! Register content for photodiodes configuration */
    uint8_t reg_buffer[sizeof(struct pd_regs)];
} as7050_config_pd_t;

/*! Register group for configuration of the PPG channels */
typedef union {
    /*! Register content for PPG configuration */
    struct ppg_regs {
        uint8_t ppg_mod_cfga; /*!< Content of register ::AS7050_REGADDR_PPG_MOD_CFGA */
        uint8_t ppg_mod_cfgb; /*!< Content of register ::AS7050_REGADDR_PPG_MOD_CFGB */
        uint8_t ppg_mod_cfgc; /*!< Content of register ::AS7050_REGADDR_PPG_MOD_CFGC */
        uint8_t ppg_mod_cfgd; /*!< Content of register ::AS7050_REGADDR_PPG_MOD_CFGD */
        uint8_t ppg_mod_cfge; /*!< Content of register ::AS7050_REGADDR_PPG_MOD_CFGE */
    } reg_vals;               /*!< Register content for PPG configuration */
    /*! Register content for PPG configuration */
    uint8_t reg_buffer[sizeof(struct ppg_regs)];
} as7050_config_ppg_t;

/*! Register group for configuration of the SINC filter */
typedef union {
    /*! Register content for SINC filter configuration */
    struct sinc_regs {
        uint8_t sinc_ppg_cfga; /*!< Content of register ::AS7050_REGADDR_SINC_PPG_CFGA */
        uint8_t sinc_ppg_cfgb; /*!< Content of register ::AS7050_REGADDR_SINC_PPG_CFGB */
        uint8_t sinc_ppg_cfgc; /*!< Content of register ::AS7050_REGADDR_SINC_PPG_CFGC */
        uint8_t sinc_ecg_cfga; /*!< Content of register ::AS7050_REGADDR_SINC_ECG_CFGA */
        uint8_t sinc_ecg_cfgb; /*!< Content of register ::AS7050_REGADDR_SINC_ECG_CFGB */
        uint8_t sinc_ecg_cfgc; /*!< Content of register ::AS7050_REGADDR_SINC_ECG_CFGC */
        uint8_t ovs_cfg;       /*!< Content of register ::AS7050_REGADDR_OVS_CFG */
    } reg_vals;                /*!< Register content for SINC filter configuration */
    /*! Register content for SINC filter configuration */
    uint8_t reg_buffer[sizeof(struct sinc_regs)];
} as7050_config_sinc_t;

/*! Register group for configuration of the sequencer */
typedef union {
    /*! Register content for sequencer configuration */
    struct seq_regs {
        uint8_t cgb_cfg;    /*!< Content of register ::AS7050_REGADDR_CGB_CFG */
        uint8_t seq_sample; /*!< Content of register ::AS7050_REGADDR_SEQ_SAMPLE */
        uint8_t seq_ppga;   /*!< Content of register ::AS7050_REGADDR_SEQ_PPGA */
        uint8_t seq_ppgb;   /*!< Content of register ::AS7050_REGADDR_SEQ_PPGB */
        uint8_t seq_mode;   /*!< Content of register ::AS7050_REGADDR_SEQ_MODE */
    } reg_vals;             /*!< Register content for sequencer configuration */
    /*! Register content for sequencer configuration */
    uint8_t reg_buffer[sizeof(struct seq_regs)];
} as7050_config_seq_t;

/*! Register group for configuration of the reference registers */
typedef union {
    /*! Register content for REF-register configuration */
    struct ref_regs {
        uint8_t ref_cfga; /*!< Content of register ::AS7050_REGADDR_REF_CFGA */
        uint8_t ref_cfgb; /*!< Content of register ::AS7050_REGADDR_REF_CFGB */
    } reg_vals;           /*!< Register content for REF-register configuration */
    /*! Register content for REF-register configuration */
    uint8_t reg_buffer[sizeof(struct ref_regs)];
} as7050_config_ref_t;

/*! Register group for configuration of the GPIOs */
typedef union {
    /*! Register content for GPIO configuration */
    struct gpio_regs {
        uint8_t gpio1_cfg;  /*!< Content of register ::AS7050_REGADDR_GPIO1_CFG */
        uint8_t gpio2_cfg;  /*!< Content of register ::AS7050_REGADDR_GPIO2_CFG */
        uint8_t gpio1_cfgb; /*!< Content of register ::AS7050_REGADDR_GPIO1_CFGB */
        uint8_t gpio2_cfgb; /*!< Content of register ::AS7050_REGADDR_GPIO2_CFGB */
        uint8_t gpio_io;    /*!< Content of register ::AS7050_REGADDR_GPIO_IO */
    } reg_vals;             /*!< Register content for GPIO configuration */
    /*! Register content for GPIO configuration */
    uint8_t reg_buffer[sizeof(struct gpio_regs)];
} as7050_config_gpio_t;

/*! Register group for configuration of the CONTROL register */
typedef union {
    /*! Register content for CONTROL register */
    struct ctrl_regs {
        uint8_t control; /*!< Content of register ::AS7050_REGADDR_CONTROL */
    } reg_vals;          /*!< Register content for CONTROL register */
    /*! Register content for CONTROL register */
    uint8_t reg_buffer[sizeof(struct ctrl_regs)];
} as7050_config_ctrl_t;

/*! Register group for configuration for STANDBY mode */
typedef union {
    /*! Register content for STANDBY configuration */
    struct standby_regs {
        uint8_t standby_cfga; /*!< Content of register ::AS7050_REGADDR_STANDBY_CFGA */
        uint8_t standby_cfgb; /*!< Content of register ::AS7050_REGADDR_STANDBY_CFGB */
    } reg_vals;               /*!< Register content for STANDBY configuration */
    /*! Register content for STANDBY configuration */
    uint8_t reg_buffer[sizeof(struct standby_regs)];
} as7050_config_standby_t;

/*! Register group for configuration of the FIFO handling */
typedef union {
    /*! Register content for FIFO configuration */
    struct fifo_regs {
        uint8_t fifo_ctrl;      /*!< Content of register ::AS7050_REGADDR_FIFO_CTRL */
        uint8_t fifo_threshold; /*!< Content of register ::AS7050_REGADDR_FIFO_THRESHOLD */
    } reg_vals;                 /*!< Register content for FIFO configuration */
    /*! Register content for FIFO configuration */
    uint8_t reg_buffer[sizeof(struct fifo_regs)];
} as7050_config_fifo_t;

/*! Register group for configuration of the ECG channel */
typedef union {
    /*! Register content for ECG configuration */
    struct ecg_regs {
        uint8_t ecg_source;   /*!< Content of register ::AS7050_REGADDR_ECG_SOURCE */
        uint8_t ecg_mod_cfga; /*!< Content of register ::AS7050_REGADDR_ECG_MOD_CFGA */
    } reg_vals;               /*!< Register content for ECG configuration */
    /*! Register content for ECG configuration */
    uint8_t reg_buffer[sizeof(struct ecg_regs)];
} as7050_config_ecg_t;

/*! Register group for configuration of the Analog Front End (AFE) */
typedef union {
    /*! Register content for AFE configuration */
    struct afe_regs {
        uint8_t afe_dac0l; /*!< Content of register ::AS7050_REGADDR_AFE_DAC0L */
        uint8_t afe_dac1l; /*!< Content of register ::AS7050_REGADDR_AFE_DAC1L */
        uint8_t afe_dach;  /*!< Content of register ::AS7050_REGADDR_AFE_DACH */
        uint8_t afe_cfga;  /*!< Content of register ::AS7050_REGADDR_AFE_CFGA */
        uint8_t afe_cfgb;  /*!< Content of register ::AS7050_REGADDR_AFE_CFGB */
        uint8_t afe_gsr;   /*!< Content of register ::AS7050_REGADDR_AFE_GSR */
    } reg_vals;            /*!< Register content for AFE configuration */
    /*! Register content for AFE configuration */
    uint8_t reg_buffer[sizeof(struct afe_regs)];
} as7050_config_afe_t;

/*! Register group for configuration of the amplifer */
typedef union {
    /*! Register content for amplifer configuration */
    struct amp_regs {
        uint8_t ecg_amp_cfga; /*!< Content of register ::AS7050_REGADDR_ECG_AMP_CFGA */
        uint8_t ecg_amp_cfgb; /*!< Content of register ::AS7050_REGADDR_ECG_AMP_CFGB */
        uint8_t ecg_amp_cfgc; /*!< Content of register ::AS7050_REGADDR_ECG_AMP_CFGC */
        uint8_t ecg_amp_cfge; /*!< Content of register ::AS7050_REGADDR_ECG_AMP_CFGE */
    } reg_vals;               /*!< Register content for amplifer configuration */
    /*! Register content for amplifer configuration */
    uint8_t reg_buffer[sizeof(struct amp_regs)];
} as7050_config_amp_t;

/*! Register group for configuration of the Transimpedance Amplifer (TIA) */
typedef union {
    /*! Register content for TIA configuration */
    struct tia_regs {
        uint8_t pd_offset_cfg; /*!< Content of register ::AS7050_REGADDR_PD_OFFSET_CFG */
        uint8_t tia_cfga;      /*!< Content of register ::AS7050_REGADDR_TIA_CFGA */
        uint8_t tia_cfgb;      /*!< Content of register ::AS7050_REGADDR_TIA_CFGB */
        uint8_t tia_cfgc;      /*!< Content of register ::AS7050_REGADDR_TIA_CFGC */
    } reg_vals;                /*!< Register content for TIA configuration */
    /*! Register content for TIA configuration */
    uint8_t reg_buffer[sizeof(struct tia_regs)];
} as7050_config_tia_t;

/*! Register group for configuration of the Infinite Impulse Response filter (IIR-filter) */
typedef union {
    /*! Register content for IIR configuration */
    struct iir_regs {
        uint8_t iir_cfg;                   /*!< Content of register ::AS7050_REGADDR_IIR_CFG */
        int16_t iir_coeff_data_sos[12][5]; /*!< RAM area where IIR-coefficients will be saved */
    } reg_vals;                            /*!< Register content for IIR configuration */
    /*! Register content for IIR configuration */
    uint8_t reg_buffer[sizeof(struct iir_regs)];
} as7050_config_iir_t;

#pragma pack(pop)

/*! Measurement settings, which can be readout after register configuration */
typedef struct {
    uint32_t ppg_sample_period_us; /*!< Sample period of PPG signals in microseconds */
    uint32_t ecg_sample_period_us; /*!< Sample period of ECG signals in microseconds */
    uint32_t max_adc_count;        /*!< Maximum possible ADC count for the current configuration */
    uint16_t fifo_map;       /*!< Definition which channels are mapped inside FIFO. (More than one flag can be set) See
                                ::as7050_channel_flags_t */
    uint16_t fifo_threshold; /*!< FIFO threshold, when data shall be read */
    uint8_t sample_size;     /*!< 3 or 4 bytes for every sample */
    uint8_t max_adc_bit_width; /*!< Maximum bit width of the ADC count for the current configuration */
    uint16_t reserved;         /*!< only for alignment, not used. Always set to 0 */
} as7050_meas_config_t;

/*!
 * \brief Callback function, which transfers the measurement results to the application.
 *
 * This callback type will be registered via the function ::as7050_initialize.
 * During the measurement, this function transfers the cyclic results.
 *
 * \param[in] error         Default ::ERR_SUCCESS, otherwise an error is occurred during measurement and the
 *                          measurement. stops. See ::error_codes
 * \param[in] p_data        Pointer to the measurement data, the content depends
 *                          on configuration.
 * \param[in] data_num      Number of 16bit data
 * \param[in] p_agc_status  Pointer to AGC status structure
 * \param[in] p_cb_param    Application parameter which was defined during call of ::as7050_initialize.
 */
typedef void (*as7050_callback_t)(err_code_t error, uint8_t *p_data, uint16_t data_num,
                                  as7050_agc_status_t *p_agc_status, void *p_cb_param);

/* USER_CODE END DEFINITIONS */

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*! @} */

#endif /* __AS7050_TYPEDEFS_H__ */
