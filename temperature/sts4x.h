#ifndef STS4X_I2C_H
#define STS4X_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdint.h"
//#include "sensirion_config.h"

// i2c adresses
#define ADDR_STS4X 0x46
#define ADDR_STS4X_ALT 0x44
#define ADDR_STS4X_ALT2 0x45

#define NO_ERROR 0

typedef struct sts4x_tag {
    uint8_t i2c_address;
} sts4x_t;

/**
 * init_driver() - initialize the driver with the i2c address to be used to
 *                 communicate with the STS4x sensor
 *
 * @param i2c_address i2c address of the STS4x sensor
 */
void init_driver(uint8_t i2c_address);

/**
 * sts4x_measure_high_precision_ticks() - SHT4x command for a single shot
 * measurement with high repeatability.
 *
 * @param temperature_ticks Temperature ticks. Convert to degrees celsius by
 * (175 * value / 65535) - 45
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_high_precision_ticks(uint16_t* temperature_ticks);

/**
 * sts4x_measure_high_precision() - SHT4x command for a single shot
 * measurement with high repeatability.
 *
 * @param temperature Temperature in milli degrees centigrade.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_high_precision(int32_t* temperature);

/**
 * sts4x_measure_medium_precision_ticks() - SHT4x command for a single shot
 * measurement with medium repeatability.
 *
 * @param temperature_ticks Temperature ticks. Convert to degrees celsius by
 * (175 * value / 65535) - 45
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_medium_precision_ticks(uint16_t* temperature_ticks);

/**
 * sts4x_measure_medium_precision() - SHT4x command for a single shot
 * measurement with medium repeatability.
 *
 * @param temperature Temperature in milli degrees centigrade.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_medium_precision(int32_t* temperature);

/**
 * sts4x_measure_lowest_precision_ticks() - SHT4x command for a single shot
 * measurement with lowest repeatability.
 *
 * @param temperature_ticks Temperature ticks. Convert to degrees celsius by
 * (175 * value / 65535) - 45
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_lowest_precision_ticks(uint16_t* temperature_ticks);

/**
 * sts4x_measure_lowest_precision() - SHT4x command for a single shot
 * measurement with lowest repeatability.
 *
 * @param temperature Temperature in milli degrees centigrade.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_measure_lowest_precision(int32_t* temperature);

/**
 * sts4x_serial_number() - Read out the serial number
 *
 * @note Each sensor has a unique serial number that is assigned by Sensirion
 * during production. It is stored in the one-time-programmable memory and
 * cannot be manipulated after production.
 *
 * @param serial_number Unique serial number
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_serial_number(uint32_t* serial_number);

/**
 * sts4x_soft_reset() - Perform a soft reset.
 *
 * @note A reset of the sensor can be achieved in three ways: By perform a soft
 * reset using this function, by using an I2C general call, at which all devices
 * on the I2C bus will be reset, or by a power down (incl. pulling SCL and SDA
 * low). See the datasheet for more detailed information.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sts4x_soft_reset(void);

void sensirion_i2c_write_data(uint8_t address, const uint8_t data,
                                 uint16_t data_length);

uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t* bytes);

#ifdef __cplusplus
}
#endif

#endif /* STS4X_I2C_H */