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

#ifndef __BSP_IDS_H__
#define __BSP_IDS_H__

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/* Alias defines for pins used for multiple functions*/
#define PIO_ID_STAT_LED PIO_ID_LED_G
#define PIO_OT_STAT_LED PIO_MODE_OUTPUT_TYPE_OD
#define PIO_ON_STAT_LED PIO_STATE_RESET
#define PIO_OFF_STAT_LED PIO_STATE_SET

enum pio_ids {
    PIO_ID_BTN_MODE = 0,  /*!< P0.11 (MD, MODE button)*/
    PIO_ID_LED_R = 1,     /*!< P0.13 (Red LED) */
    PIO_ID_LED_G = 2,     /*!< P0.14 (Green LED) */
    PIO_ID_LED_B = 3,     /*!< P0.15 (Blue LED) */
    PIO_ID_SDA = 4,       /*!< P0.26 (SDA) */
    PIO_ID_SCL = 5,       /*!< P0.27 (SCL) */
    PIO_ID_D2 = 6,        /*!< P1.01 (D2, SDA1, SPI1_SCK) */
    PIO_ID_D3 = 7,        /*!< P1.02 (D3, SCL1, SPI1_MOSI) */
    PIO_ID_D4 = 8,        /*!< P1.08 (D4, UART2_TX, SPI1_MISO)*/
    PIO_ID_D5 = 9,        /*!< P1.10 (D5, UART2_RX) */
    PIO_ID_D6 = 10,       /*!< P1.11 (D6) */
    PIO_ID_D7 = 11,       /*!< P1.12 (D7, User Led) */
    PIO_ID_D8 = 12,       /*!< P1.03 (D8) */
    PIO_ID_A0 = 13,       /*!< P0.03 (A0, ADC0) */
    PIO_ID_A1 = 14,       /*!< P0.04 (A1, ADC1) */
    PIO_ID_A2 = 15,       /*!< P0.28 (A2, ADC2) */
    PIO_ID_A3 = 16,       /*!< P0.29 (A3, ADC3) */
    PIO_ID_A4 = 17,       /*!< P0.30 (A4, ADC4) */
    PIO_ID_A5 = 18,       /*!< P0.31 (A5, SPI_SS, ADC5) */
    PIO_ID_SCK = 19,      /*!< P1.15 (SCK, SPI_SCK) */
    PIO_ID_MO = 20,       /*!< P1.13 (MO, SPI_MOSI) */
    PIO_ID_MI = 21,       /*!< P1.14 (MI, SPI_MISO */
    PIO_ID_RX = 22,       /*!< P0.08 (RX, UART1_RX) */
    PIO_ID_TX = 23,       /*!< P0.06 (TX, UART1_TX */
    PIO_ID_RST = 24,      /*!< P0.18 (RST, RESET) */
    PIO_ID_CHG = 25,      /*!< P1.09 (CHG, Battery Charge indicator) */
    PIO_ID_VUSB_DET = 26, /*!< P0.12 (VUSB_DET, USB Detect) */

    PIO_ID_NUM = 27
};
typedef enum pio_ids pio_ids_t;

enum i2c_ids {
    I2C_ID_MAIN = 0,

    I2C_ID_NUM = 1
};
typedef enum i2c_ids i2c_ids_t;

enum spi_ids {
    SPI_ID_MAIN = 0,
    SPI_ID_SECONDARY = 1,

    SPI_ID_NUM = 2
};

typedef enum spi_ids spi_ids_t;

enum tim_ids {
    TIM_ID_MAIN = 0,
    TIM_ID_TEST = 1,
    TIM_ID_POWER_MANAGER = 2,

    TIM_ID_NUM = 3
};
typedef enum tim_ids tim_ids_t;

enum pwm_ids {
    PWM_ID_0 = 0,

    PWM_ID_NUM = 1
};
typedef enum pwm_ids pwm_ids_t;

enum adc_ids {
    ADC_ID_BATTERY = 0,

    ADC_ID_NUM = 1
};
typedef enum adc_ids adc_ids_t;

#endif /*__BSP_IDS_H__ */
