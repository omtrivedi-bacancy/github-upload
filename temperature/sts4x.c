#include "sts4x.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

 /* Number of possible TWI addresses. */
 #define TWI_ADDRESSES      127

 #define SENSIRION_WORD_SIZE 2
 #define CRC8_LEN            1
 #define CRC8_INIT 0xFF
 
#define CRC8_POLYNOMIAL 0x31

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static sts4x_t _driver = {.i2c_address = ADDR_STS4X};

#define STS4X_I2C_ADDRESS _driver.i2c_address

static int32_t convert_ticks_to_celsius(uint16_t ticks) {
    return ((21875 * (int32_t)ticks) >> 13) - 45000;
}


uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t* bytes) {
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

uint32_t sensirion_common_bytes_to_uint32_t(const uint8_t* bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

uint8_t sensirion_i2c_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

int8_t sensirion_i2c_check_crc(const uint8_t* data, uint16_t count,
                               uint8_t checksum) {
    ret_code_t err_code;
    if (sensirion_i2c_generate_crc(data, count) != checksum)
        return err_code;
    //return NO_ERROR;
}

void temp_reg_write(unsigned char reg_addr, uint8_t* data)
  {
    int i;
    ret_code_t err_code;
    uint8_t bytes_to_send[2] = {reg_addr, *data};

    //printf("Writing value 0x%2.2X at Register address 0x%2.2X...\n", data, reg_addr);

    //for(i = 0; i < I2C_MAX_RETRIES; i++)
    {
        err_code = nrf_drv_twi_tx(&m_twi, ADDR_STS4X, data, sizeof(bytes_to_send), false);
        if(!err_code)
        {
            //break;
        }
        nrf_delay_ms(1);
    }

    if(err_code)
    {
        printf("Failed to write value 0x%2.2X at Register address 0x%2.2X ! \n", data, reg_addr);
    }
    //printf("ERROR CODE FOR TX %d \n", err_code);

}


/**
 * @brief: I2C read Function
 *
 * @param[in] uint8_t reg_address,uint8_t *data
 *
 * @param[out] bool
 */
bool temp_read(uint8_t reg_address,uint8_t *data,uint16_t size)
{
     
    int i;
    ret_code_t err_code;
    bool result = false;

    for(i = 0; i < 3; i++)
    {
        err_code = nrf_drv_twi_tx(&m_twi, ADDR_STS4X, &reg_address,1, false);
        nrf_delay_ms(10);
    }

    if(!err_code)
    {
        for(i = 0; i < 3; i++)
        {
            err_code = nrf_drv_twi_rx(&m_twi, ADDR_STS4X, data,size);
            if(!err_code)
            {
                result = 0;
                break;
            }
            nrf_delay_ms(2);
        }

        if(err_code)
        {
            printf("Failed to read value 0x%2.2X at Register address 0x%2.2X !", data, reg_address);
        }
    }
    else
    {
        printf("Failed to write Register address 0x%2.2X !", reg_address);
    }
    //printf("Result of rx  %d \n", result);

    return result;

}


void sensirion_i2c_write_data(uint8_t address, const uint8_t data,
                                 uint16_t data_length) 
{
    temp_reg_write(address, data);
}

int16_t sensirion_i2c_read_data_inplace(uint8_t address, uint8_t* buffer,
                                        uint16_t expected_data_length) {
    int16_t error;
    uint16_t i, j;
    uint16_t size = (expected_data_length / SENSIRION_WORD_SIZE) *
                    (SENSIRION_WORD_SIZE + CRC8_LEN);

    if (expected_data_length % SENSIRION_WORD_SIZE != 0) {
        return error;
    }

    //error = sensirion_i2c_hal_read(address, buffer, size);
    //if (error) {
    //    return error;
    //}

    error = temp_read(buffer[0], buffer,size);
    if (error) {
        return error;
    }

    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

        error = sensirion_i2c_check_crc(&buffer[i], SENSIRION_WORD_SIZE,
                                        buffer[i + SENSIRION_WORD_SIZE]);
        if (error) {
            return error;
        }
        buffer[j++] = buffer[i];
        buffer[j++] = buffer[i + 1];
    }

    //return NO_ERROR;
}

int16_t sts4x_measure_high_precision_ticks(uint16_t* temperature_ticks) {
    int16_t error;
    uint8_t buffer[3];
    
    error = temp_read(0xFD,buffer,3);
    if (error) {
        return error;
    }
    *temperature_ticks = sensirion_common_bytes_to_uint16_t(buffer);
    return NO_ERROR;
}

int16_t sts4x_measure_high_precision(int32_t* temperature) {
    int16_t error;
    uint16_t temperature_ticks;

    error = sts4x_measure_high_precision_ticks(&temperature_ticks);
    if (error) {
        return error;
    }
    *temperature = convert_ticks_to_celsius(temperature_ticks);
    return NO_ERROR;
}

int16_t sts4x_measure_medium_precision_ticks(uint16_t* temperature_ticks) {
    int16_t error;
    uint8_t buffer[3];

    error = temp_read(0xF6,buffer,3);
    if (error) {
        return error;
    }
    *temperature_ticks = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sts4x_measure_medium_precision(int32_t* temperature) {
    int16_t error;
    uint16_t temperature_ticks;

    error = sts4x_measure_medium_precision_ticks(&temperature_ticks);
    if (error) {
        return error;
    }
    *temperature = convert_ticks_to_celsius(temperature_ticks);
    return NO_ERROR;
}

int16_t sts4x_measure_lowest_precision_ticks(uint16_t* temperature_ticks) {
    int16_t error;
    uint8_t buffer[3];

    error = temp_read(0xE0,buffer,3);
    if (error) {
        return error;
    }
    *temperature_ticks = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sts4x_measure_lowest_precision(int32_t* temperature) {
    int16_t error;
    uint16_t temperature_ticks;

    error = sts4x_measure_lowest_precision_ticks(&temperature_ticks);
    if (error) {
        return error;
    }
    *temperature = convert_ticks_to_celsius(temperature_ticks);
    return NO_ERROR;
}

int16_t sts4x_serial_number(uint32_t* serial_number) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;

    error = temp_read(0x89,buffer,3);
    if (error) {
        return error;
    }
    *serial_number = sensirion_common_bytes_to_uint32_t(&buffer[0]);
    return NO_ERROR;
}

int16_t sts4x_soft_reset(void) {
    int16_t error;
    uint8_t buffer[2];
    buffer[0] = (uint8_t)0x94;

    temp_reg_write(0x94,&buffer[0]);
    return NO_ERROR;
}