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

#ifndef __ERROR_CODES_H__
#define __ERROR_CODES_H__

/*!
 * \file      error_codes.h
 * \authors   ARIT
 * \copyright ams
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*!
 * \addtogroup error_group Error Codes
 *
 * Generic error codes used by ams libraries.
 *
 * @{ */

/*! Values represent the error codes. */
enum error_codes {

    ERR_SUCCESS = 0, /*!< Normal return code if everything was successful executed. */

    ERR_PERMISSION = 1,          /*!< Operation not permitted */
    ERR_MESSAGE = 2,             /*!< Message is invalid.
                                      For example:
                                        - Message type is not supported
                                        - incorrect crc
                                        - ...
                                  */
    ERR_MESSAGE_SIZE = 3,        /*!< Message has the wrong size. */
    ERR_POINTER = 4,             /*!< Pointer is invalid. Can be a NULL Pointer
                                      or point to a wrong memory area. */
    ERR_ACCESS = 5,              /*!< Access denied */
    ERR_ARGUMENT = 6,            /*!< Invalid argument */
    ERR_SIZE = 7,                /*!< Argument size is too long or too short. */
    ERR_NOT_SUPPORTED = 8,       /*!< Function is not supported/implemented. */
    ERR_TIMEOUT = 9,             /*!< Got timeout while waiting for answer. */
    ERR_CHECKSUM = 10,           /*!< Checksum comparision failed. */
    ERR_OVERFLOW = 11,           /*!< Data overflow detected. */
    ERR_EVENT = 12,              /*!< Error to get or set an event.
                                      For example:
                                        - event queue is full or empty
                                        - receive an unexpected event
                                        - ...
                                  */
    ERR_INTERRUPT = 13,          /*!< Error to get or set an interrupt.
                                      For example a interrupt resource is not available.
                                  */
    ERR_TIMER_ACCESS = 14,       /*!< Error while accessing timer periphery. */
    ERR_LED_ACCESS = 15,         /*!< Error while accessing LED periphery. */
    ERR_TEMP_SENSOR_ACCESS = 16, /*!< Error while accessing temperature sensor. */
    ERR_DATA_TRANSFER = 17,      /*!< Communication error */
    ERR_FIFO = 18,               /*!< Faulty FIFO handling */
    ERR_OVER_TEMP = 19,          /*!< Overtemperature detected. */
    ERR_IDENTIFICATION = 20,     /*!< Sensor identification failed. */
    ERR_COM_INTERFACE = 21,      /*!< Generic communication interface error.
                                      For example:
                                        - communication interface is not available
                                        - error during open or close an communication interface
                                        - ...
                                  */
    ERR_SYNCHRONISATION = 22,    /*!< Synchronisation error, e.g. on protocol */
    ERR_PROTOCOL = 23,           /*!< Generic protocol error */
    ERR_MEMORY = 24,             /*!< Memory allocation error */
    ERR_THREAD = 25,             /*!< Thread can not created. */
    ERR_SPI = 26,                /*!< Error while accessing SPI periphery */
    ERR_DAC_ACCESS = 27,         /*!< Error while accessing DAC periphery. */
    ERR_I2C = 28,                /*!< Error while accessing I2C periphery. */
    ERR_NO_DATA = 29,            /*!< No data available. */
    ERR_SYSTEM_CONFIG = 30,      /*!< Error during system configuration.
                                      When a system resource is not available or generates an error for example.
                                  */
    ERR_USB_ACCESS = 31,         /*!< USB error */
    ERR_ADC_ACCESS = 32,         /*!< Error while accessing ADC periphery. */
    ERR_SENSOR_CONFIG = 33,      /*!< Error during sensor configuration. */
    ERR_SATURATION = 34,         /*!< Saturation detected */
    ERR_MUTEX = 35,              /*!< Error while mutex handling */
    ERR_ACCELEROMETER = 36,      /*!< Error while reading accelerometer data */
    ERR_CONFIG = 37,             /*!< Software component is not fully or correctly configured */
    ERR_BLE = 38                 /*!< Error while executing BLE stack function */
};

/*! This definition will be used for function return values. */
typedef enum error_codes err_code_t;

/*! @} */

/*! Returns an error code if the arg_value is greater than the max_value. */
#define M_CHECK_ARGUMENT_LOWER_EQUAL(arg_value, max_value)                                                             \
    do {                                                                                                               \
        if (arg_value > max_value) {                                                                                   \
            return ERR_ARGUMENT;                                                                                       \
        }                                                                                                              \
    } while (0)

/*! Returns an error code if the first value is greater or equal than the second one. */
#define M_CHECK_ARGUMENT_LOWER(arg_value, max_value)                                                                   \
    do {                                                                                                               \
        if (arg_value >= max_value) {                                                                                  \
            return ERR_ARGUMENT;                                                                                       \
        }                                                                                                              \
    } while (0)

/*! Returns an error code if the value is zero. */
#define M_CHECK_ARGUMENT_NOT_ZERO(value)                                                                               \
    do {                                                                                                               \
        if (0 == value) {                                                                                              \
            return ERR_ARGUMENT;                                                                                       \
        }                                                                                                              \
    } while (0)

/** Returns an error code if the value is not a multiple of multiplier. */
#define M_CHECK_ARGUMENT_MULTIPLE_OF(multiplier, value)                                                                \
    do {                                                                                                               \
        if ((multiplier % value) != 0) {                                                                               \
            return ERR_ARGUMENT;                                                                                       \
        }                                                                                                              \
    } while (0)

/*! Returns an error code if the given address is zero. */
#define M_CHECK_NULL_POINTER(pointer)                                                                                  \
    do {                                                                                                               \
        if (NULL == pointer) {                                                                                         \
            return ERR_POINTER;                                                                                        \
        }                                                                                                              \
    } while (0)

/** Returns an error code if the values are not equal. */
#define M_CHECK_SIZE(expected, given)                                                                                  \
    do {                                                                                                               \
        if (expected != given) {                                                                                       \
            return ERR_SIZE;                                                                                           \
        }                                                                                                              \
    } while (0)

/*! Checks whether the provided result is ERR_SUCCESS and returns the result as-is otherwise. */
#define M_CHECK_SUCCESS(result)                                                                                        \
    do {                                                                                                               \
        err_code_t _result = (result);                                                                                 \
        if (_result != ERR_SUCCESS) {                                                                                  \
            return _result;                                                                                            \
        }                                                                                                              \
    } while (0)

/*! Mark unused arguments to get no fails on static code analysis. */
#ifndef M_UNUSED_PARAM
#define M_UNUSED_PARAM(x) (void)(x)
#endif

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

#endif /* __ERROR_CODES_H__ */
