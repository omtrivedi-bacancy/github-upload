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

/*!
 * \file cmd_base_definition.h
 * \authors ARIT
 * \copyright ams
 * \addtogroup cmd_base_group
 *
 * \brief Handles the command IDs and the corresponding functions
 * * Managing of the base command IDs
 * * Definition of the message structures
 * * Definition of the message functions
 *
 * @{
 */

#ifndef __CMD_BASE_DEF_H_
#define __CMD_BASE_DEF_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#ifndef HOST_INTERFACE
#include "server_protocol.h"
#endif

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! Identifiers for the base commands which shall be supported by all core devices */
enum E_CMD_BASE_ID {

    /*!
     * This command returns the name of the application.
     *
     * \n\n \b Input [Payload-Size: 0]:
     *
     * \n \b Output [Payload-Size: 0 - 40 byte]:
     * * <em>application_name:</em> Name of the application
     */
    CMD_BASE_ID_APPL_NAME = 0,

    /*!
     * This command returns the firmware version.
     *
     * \n\n \b Input [Payload-Size: 0]:
     *
     * \n \b Output [Payload-Size: 0 - 20 byte]:
     * * <em>firmware_version:</em> Version string like 1.0.0
     */
    CMD_BASE_ID_VERSION = 1,

    /*!
     * This command triggers the firmware to restart
     *
     * \attention On success, no acknowledge will be send to the host
     *
     * \n\n \b Input [Payload-Size: 0]
     *
     * \n \b Output [Payload-Size: 0]
     *
     */
    CMD_BASE_ID_RESET = 2,

    /*!
     * This command initialize the I2C interface
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 8]:
     * * <em>enable:</em> Size uint8. 1 - interface will be enabled
     * * <em>reserved:</em> Size uint8[3]. Reserved for future usage
     * * <em>frequency:</em> Size uint32. I2C bus frequency in Hz (eg. 100kHz, 400kHz)
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_I2C_CONFIG = 3,

    /*!
     * Transfer of an I2C datagram.
     *
     * \attention Deprecated since version 3.0.0, use CMD_BASE_ID_I2C_XFER_16BIT instead.
     *
     * \note Target-id will be used to select the channel.
     *
     * \note Structure ::i2c_xfer_t can be used for header initialization
     *
     * \n\n \b Input [Payload-Size: 2 - 257]:
     * * <em>address:</em> Size uint8. I2C slave device address
     * * <em>recv_size:</em> Size uint8. Number of data which shall be read after sent data
     * * <em>send_data:</em> Optional: Size uint8[0..255]. Content of send data
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_I2C_XFER = 4,

    /*!
     * This command initialize the SPI interface
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 8]:
     * * <em>enable:</em> Size uint8. 1 - interface will be enabled
     * * <em>mode:</em> Size uint8. See enumeration ::spi_modes
     * * <em>first_bit:</em> Size uint8. See enumeration ::spi_first_bit
     * * <em>reserved:</em> Size uint8. Reserved for future usage
     * * <em>frequency:</em> Size uint32. SPI bus frequency in Hz (eg. 1MHz)
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_SPI_CONFIG = 5,

    /*!
     * Transfer of a SPI datagram
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 1 - 65535]:
     * * <em>send_data:</em> Size uint8[1..65535]. Content of send data
     *
     * \n \b Output [Payload-Size: 0 byte]
     * * <em>recv_data:</em> Size of send_data. Content of recv data
     */
    CMD_BASE_ID_SPI_XFER = 6,

    /*!
     * This command initialize the PIO (Pin Input Output) interface
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 3]:
     * * <em>enable:</em> Size uint8. 1 - pin will be enabled
     * * <em>mode:</em> Size uint8. See enumeration ::pio_modes
     * * <em>pull:</em> Size uint8. See enumeration ::pio_pulls
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_PIO_CONFIG = 7,

    /*!
     * Set/Get the state of a pin
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 0 - 1]:
     * * <em>state:</em> Optional: Size uint8. New pin state. See ::pio_states_t
     *
     * \n \b Output [Payload-Size: 1 byte]:
     * * <em>state:</em> Size uint8. Current pin state. See ::pio_states_t
     */
    CMD_BASE_ID_PIO_XFER = 8,

    /*!
     * Asynchronous message ID for GPIO state
     *
     * \note Target-id will be used to select the channel.
     *
     * \attention This command ID is not supported directly.
     * It will be used to send cyclic pin change-states (if registered).
     *
     * \n\n \b Input [Payload-Size: 0]
     *
     * \n \b Output [Payload-Size: 1 byte]
     * * <em>state:</em> Size uint8. Current pin state. See ::pio_states_t
     */
    CMD_BASE_ID_PIO_STATE = 9,

    /*!
     * This command triggers the firmware to start the bootloader
     *
     * \attention On success, no acknowledge will be send to the host
     *
     * \n\n \b Input [Payload-Size: 0]
     *
     * \n \b Output [Payload-Size: 0]
     *
     */
    CMD_BASE_ID_SYS_START_BL = 10,

    /*!
     * This command initialize a PWM channel
     *
     * \note Target-id will be used to select the channel.
     *
     * \n\n \b Input [Payload-Size: 8]:
     * * <em>enable:</em> Size uint8. 1 - interface will be enabled
     * * <em>reserved:</em> Size uint8. Reserved for future usage
     * * <em>duty_cycle:</em> Size uint16. duty cycle [0 - 1000]
     * * <em>frequency:</em> Size uint32. PWM freqency in hertz
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_PWM_CONFIG = 11,

    /*!
     * This command requests a stream of test messages
     *
     * \n\n \b Input [Payload-Size: 8]:
     * * <em>count:</em> Size uint16. Number of test messages to send
     * * <em>size:</em> Size uint16. Size of test message payload
     * * <em>delay_us:</em> Size uint32. Minimum time between the transmission of two test messages in microseconds
     *
     * \n \b Output [Payload-Size: 0 byte]
     */
    CMD_BASE_ID_TEST_REQ = 12,

    /*!
     * This command is used to stream test data requested with \ref CMD_BASE_ID_TEST_REQ
     *
     * \n\n \b Output [Payload-Size: variable]
     * * <em>counter:</em> Size uint16. Number of remaining test messages, not including the current message. Field
     *   is only present if requested payload size is larger than the size of this field.
     * * <em>pattern:</em> All payload bytes not used by other fields contain their index in the payload (modulo 255),
     *   i.e. payload bytes at indices 2 and 258 both have the value 2.
     */
    CMD_BASE_ID_TEST_RSP = 13,

    /*!
     * Transfer of an I2C datagram with extended sizes of more than 8bit
     *
     * \note Target-id will be used to select the channel.
     *
     * \note Structure ::i2c_xfer_16bit_t can be used for header initialization
     *
     * \n\n \b Input [Payload-Size: 4 - ...]:
     * * <em>address:</em> Size uint8. I2C slave device address
     * * <em>reserved:</em> Size uint8. not used, set to zero (padding byte)
     * * <em>recv_size:</em> Size uint16. Number of bytes which shall be read after sent data (Can be zero)
     * * <em>send_data:</em> Optional: Bytes which shall be sent before data will be read
     *
     * \n \b Output [Payload-Size: 0 - ...]
     * * <em>recv_data:</em> Optional: Received bytes
     */
    CMD_BASE_ID_I2C_XFER_16BIT = 14
};

/*! Configuration structure for I2C and SPI periphery */
typedef struct i2c_spi_config {
    uint8_t enable;     /*!< Enable/Disable the periphery */
    uint8_t mode;       /*!< SPI only, see enumeration ::spi_modes */
    uint8_t first_bit;  /*!< SPI only, see enumeration ::spi_first_bit */
    uint8_t reserved;   /*!< not used, set to zero (padding byte) */
    uint32_t frequency; /*!< Bus frequency of the periphery */
} i2c_spi_config_t;

/*! I2C transfer data */
typedef struct i2c_xfer {
    uint8_t dev_addr;     /*!< I2C slave device address */
    uint8_t recv_size;    /*!< Number of bytes which shall be read from the bus */
    uint8_t send_data[1]; /*!< First byte of the transmit-data */
} i2c_xfer_t;

/*! I2C transfer data */
typedef struct i2c_xfer_16bit {
    uint8_t dev_addr;    /*!< I2C slave device address */
    uint8_t reserved;    /*!< not used, set to zero (padding byte) */
    uint16_t recv_size;  /*!< Number of bytes which shall be read from the bus */
    uint8_t send_data[]; /*!< Begin of the transmit data */
} i2c_xfer_16bit_t;

/*! Configuration structure for the initialization of the GPIO module */
typedef struct pio_config {
    uint8_t enable; /*!< Enable/Disable the pin */
    uint8_t mode;   /*!< see enumeration ::pio_modes */
    uint8_t pull;   /*!< see enumeration ::pio_pulls */
} pio_config_t;

/*! Configuration structure for the initialization of the PWM pin */
typedef struct pwm_config {
    uint8_t enable;      /*!< Enable/Disable the PWM pin */
    uint8_t reserved;    /*!< not used, set to zero (padding byte) */
    uint16_t duty_cycle; /*!< duty cycle [0 - 1000] */
    uint32_t frequency;  /*!< PWM freqency in hertz */
} pwm_config_t;

/*! Structure for test message stream request */
typedef struct test_req {
    uint16_t count;    /*!< Number of test messages to send */
    uint16_t size;     /*!< Size of test message payload */
    uint32_t delay_us; /*!< Minimum time between the transmission of two test messages in microseconds */
} test_req_t;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

#ifndef HOST_INTERFACE

/*!
 * \brief Returns the supported command table and entry number
 *
 * This table defines the callback functions, which will be used by the protcol handler

 * \param[in] p_num     memory, where the number of table elements can be saved

 * \returns Address which points to the command table
 */
const prt_table_entry_t *cmd_base_get_table(uint32_t *p_num);

/*!
 * \brief Handle ongoing command related tasks
 *
 * This function should be called on every main loop iteration.
 */
void cmd_main_loop(void);

#endif

/*! @} */

#endif /* __CMD_BASE_DEF_H_ */
