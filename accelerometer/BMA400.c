/*
 *      Copyright (c) 2021 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Om Trivedi and Brijesh Vaghasiya
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
/*!
 * \file     BMA400.c
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "error_codes.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "BMA400.h"

/*
 * @brief Accel self test diff xyz data structure
 */
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
#define I2C_MAX_RETRIES     5

struct bma400_selftest_delta_limit
{
    /* Accel X  data */
    int32_t x;

    /* Accel Y  data */
    int32_t y;

    /* Accel Z  data */
    int32_t z;
};

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
/************************************************************************************/
/*********************** Static function declarations *******************************/
/************************************************************************************/

/*
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bma400_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t null_ptr_check(const struct bma400_dev *dev);
void bma400_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes);
int8_t bma400_register_write(uint8_t register_address, uint8_t value);
/*
 * @brief This internal API is used to set sensor configurations
 *
 * @param[in] data      : Data to be mapped with interrupt
 * @param[in] conf      : Sensor configurations to be set
 * @param[in] dev       : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_sensor_conf(uint8_t *data, const struct bma400_sensor_conf *conf);

/*
 * @brief This internal API is used to get sensor configurations
 *
 * @param[in] data      : Data to be mapped with interrupt
 * @param[in] conf      : Sensor configurations to be set
 * @param[in] dev       : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_sensor_conf(const uint8_t *data, struct bma400_sensor_conf *conf, struct bma400_dev *dev);

/*
 * @brief This internal API is used to set the accel configurations in sensor
 *
 * @param[in] accel_conf : Structure instance with accel configurations
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_accel_conf(const struct bma400_acc_conf *accel_conf);

/*
 * @brief This API reads accel data along with sensor time
 *
 * @param[in] data_sel   : Variable to select the data to be read
 * @param[in,out] accel  : Structure instance to store the accel data
 * @param[in] dev        : Structure instance of bma400_dev
 *
 * Assignable values for data_sel:
 *   - BMA400_DATA_ONLY
 *   - BMA400_DATA_SENSOR_TIME
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_accel_data(uint8_t data_sel, struct bma400_sensor_data *accel, struct bma400_dev *dev);

/*
 * @brief This API enables the auto-wakeup feature
 * of the sensor using a timeout value
 *
 * @param[in] wakeup_conf : Structure instance of wakeup configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_autowakeup_timeout(const struct bma400_auto_wakeup_conf *wakeup_conf, struct bma400_dev *dev);

/*
 * @brief This API enables the auto-wakeup feature of the sensor
 *
 * @param[in] conf  : Configuration value to enable/disable
 *                    auto-wakeup interrupt
 * @param[in] dev   : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_auto_wakeup(uint8_t conf, struct bma400_dev *dev);

/*
 * @brief This API sets the parameters for auto-wakeup feature
 * of the sensor
 *
 * @param[in] wakeup_conf : Structure instance of wakeup configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_autowakeup_interrupt(const struct bma400_wakeup_conf *wakeup_conf, struct bma400_dev *dev);

/*
 * @brief This API sets the sensor to enter low power mode
 * automatically  based on the configurations
 *
 * @param[in] auto_lp_conf : Structure instance of auto-low power settings
 * @param[in] dev            : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_auto_low_power(const struct bma400_auto_lp_conf *auto_lp_conf, struct bma400_dev *dev);

/*
 * @brief This API sets the tap setting parameters
 *
 * @param[in] tap_set : Structure instance of tap configurations
 * @param[in] dev     : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
int8_t set_tap_conf(const struct bma400_tap_conf *tap_set);

/*
 * @brief This API sets the parameters for activity change detection
 *
 * @param[in] act_ch_set : Structure instance of activity change
 *                         configurations
 * @param[in] dev        : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_activity_change_conf(const struct bma400_act_ch_conf *act_ch_set);

/*
 * @brief This API sets the parameters for generic interrupt1 configuration
 *
 * @param[in] gen_int_set : Structure instance of generic interrupt
 *                          configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_gen1_int(const struct bma400_gen_int_conf *gen_int_set);

/*
 * @brief This API sets the parameters for generic interrupt2 configuration
 *
 * @param[in] gen_int_set : Structure instance of generic interrupt
 *                          configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_gen2_int(const struct bma400_gen_int_conf *gen_int_set);

/*
 * @brief This API sets the parameters for orientation interrupt
 *
 * @param[in] orient_conf : Structure instance of orient interrupt
 *                          configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_orient_int(const struct bma400_orient_int_conf *orient_conf);

/*
 * @brief This internal API is used to get the accel configurations in sensor
 *
 * @param[in,out] accel_conf  : Structure instance of basic
 *                              accelerometer configuration
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_accel_conf(struct bma400_acc_conf *accel_conf, struct bma400_dev *dev);

/*
 * @brief This API gets the set sensor settings for auto-wakeup timeout feature
 *
 * @param[in,out] wakeup_conf : Structure instance of wake-up configurations
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_autowakeup_timeout(struct bma400_auto_wakeup_conf *wakeup_conf, struct bma400_dev *dev);

/*
 * @brief This API gets the set sensor settings for
 * auto-wakeup interrupt feature
 *
 * @param[in,out] wakeup_conf : Structure instance of wake-up configurations
 * @param[in] dev               : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_autowakeup_interrupt(struct bma400_wakeup_conf *wakeup_conf, struct bma400_dev *dev);

/*
 * @brief This API gets the sensor to get the auto-low
 * power mode configuration settings
 *
 * @param[in,out] auto_lp_conf : Structure instance of low power
 *                                 configurations
 * @param[in] dev                : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_auto_low_power(struct bma400_auto_lp_conf *auto_lp_conf, struct bma400_dev *dev);

/*
 * @brief This API sets the tap setting parameters
 *
 * @param[in,out] tap_set : Structure instance of tap configurations
 * @param[in] dev         : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_tap_conf(struct bma400_tap_conf *tap_set, struct bma400_dev *dev);

/*
 * @brief This API gets the parameters for activity change detection
 *
 * @param[in,out] act_ch_set : Structure instance of activity
 *                             change configurations
 * @param[in] dev            : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_activity_change_conf(struct bma400_act_ch_conf *act_ch_set);

/*
 * @brief This API gets the generic interrupt1 configuration
 *
 * @param[in,out] gen_int_set : Structure instance of generic
 *                               interrupt configurations
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_gen1_int(struct bma400_gen_int_conf *gen_int_set);

/*
 * @brief This API gets the generic interrupt2 configuration
 *
 * @param[in,out] gen_int_set : Structure instance of generic
 *                              interrupt configurations
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_gen2_int(struct bma400_gen_int_conf *gen_int_set);

/*
 * @brief This API gets the parameters for orientation interrupt
 *
 * @param[in,out] orient_conf : Structure instance of orient
 *                              interrupt configurations
 * @param[in] dev             : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_orient_int(struct bma400_orient_int_conf *orient_conf);

/*
 * @brief This API sets the selected interrupt to be mapped to
 * the hardware interrupt pin of the sensor
 *
 * @param[in,out] data_array  : Data array of interrupt pin configurations
 * @param[in] int_enable      : Interrupt selected for pin mapping
 * @param[in] int_map         : Interrupt channel to be mapped
 *
 * @return Nothing
 */
static void map_int_pin(uint8_t *data_array, uint8_t int_enable, enum bma400_int_chan int_map);

/*
 * @brief This API checks whether the interrupt is mapped to the INT pin1
 * or INT pin2 of the sensor
 *
 * @param[in] int_1_map     : Variable to denote whether the interrupt is
 *                            mapped to INT1 pin or not
 * @param[in] int_2_map     : Variable to denote whether the interrupt is
 *                            mapped to INT2 pin or not
 * @param[in,out] int_map   : Interrupt channel which is mapped
 *                            INT1/INT2/NONE/BOTH
 *
 * @return Nothing
 */
static void check_mapped_interrupts(uint8_t int_1_map, uint8_t int_2_map, enum bma400_int_chan *int_map);

/*
 * @brief This API gets the selected interrupt and its mapping to
 * the hardware interrupt pin of the sensor
 *
 * @param[in,out] data_array  : Data array of interrupt pin configurations
 * @param[in] int_enable      : Interrupt selected for pin mapping
 * @param[out] int_map        : Interrupt channel which is mapped
 *
 * @return Nothing
 */
static void get_int_pin_map(const uint8_t *data_array, uint8_t int_enable, enum bma400_int_chan *int_map);

/*
 * @brief This API is used to set the interrupt pin configurations
 *
 * @param[in] int_conf     : Interrupt pin configuration
 * @param[in] dev          : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_int_pin_conf(struct bma400_int_pin_conf int_conf, struct bma400_dev *dev);

/*
 * @brief This API is used to set the interrupt pin configurations
 *
 * @param[in,out] int_conf     : Interrupt pin configuration
 * @param[in] dev              : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_int_pin_conf(struct bma400_int_pin_conf *int_conf, struct bma400_dev *dev);

/*
 * @brief This API is used to set the FIFO configurations
 *
 * @param[in,out] fifo_conf       : Structure instance containing the FIFO
 *                                  configuration set in the sensor
 * @param[in] dev                 : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t set_fifo_conf(const struct bma400_fifo_conf *fifo_conf, struct bma400_dev *dev);

/*
 * @brief This API is used to get the FIFO configurations
 *
 * @param[in,out] fifo_conf       : Structure instance containing the FIFO
 *                                  configuration set in the sensor
 * @param[in] dev                 : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_fifo_conf(struct bma400_fifo_conf *fifo_conf, struct bma400_dev *dev);

/*
 * @brief This API is used to get the number of bytes filled in FIFO
 *
 * @param[in,out] fifo_byte_cnt   : Number of bytes in the FIFO buffer
 *                                  actually filled by the sensor
 * @param[in] dev                 : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t get_fifo_length(uint16_t *fifo_byte_cnt, struct bma400_dev *dev);

/*
 * @brief This API is used to read the FIFO of BMA400
 *
 * @param[in,out] fifo : Pointer to the fifo structure.
 *
 * @param[in] dev      : Structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t read_fifo(struct bma400_fifo_data *fifo, struct bma400_dev *dev);

/*
 * @brief This API is used to unpack the accelerometer frames from the FIFO
 *
 * @param[in,out] fifo            : Pointer to the fifo structure.
 * @param[in,out] accel_data      : Structure instance to store the accel data
 * @param[in,out] frame_count     : Number of frames requested by user as input
 *                                  Number of frames actually parsed as output
 * @param[in] dev                 : Structure instance of bma400_dev
 *
 * @return Nothing
 */
static void unpack_accel_frame(struct bma400_fifo_data *fifo,
                               struct bma400_sensor_data *accel_data,
                               uint16_t *frame_count,
                               const struct bma400_dev *dev);

/*
 * @brief This API is used to check for a frame availability in FIFO
 *
 * @param[in,out] fifo            : Pointer to the fifo structure.
 * @param[in,out] frame_available : Variable to denote availability of a frame
 * @param[in] accel_width         : Variable to denote 12/8 bit accel data
 * @param[in] data_en             : Data enabled in FIFO
 * @param[in,out] data_index      : Index of the currently parsed FIFO data
 *
 * @return Nothing
 */
static void check_frame_available(const struct bma400_fifo_data *fifo,
                                  uint8_t *frame_available,
                                  uint8_t accel_width,
                                  uint8_t data_en,
                                  uint16_t *data_index);

/*
 * @brief This API is used to unpack the accelerometer xyz data from the FIFO
 * and store it in the user defined buffer
 *
 * @param[in,out] fifo         : Pointer to the fifo structure.
 * @param[in,out] accel_data   : Structure instance to store the accel data
 * @param[in,out] data_index   : Index of the currently parsed FIFO data
 * @param[in] accel_width      : Variable to denote 12/8 bit accel data
 * @param[in] frame_header     : Variable to get the data enabled
 *
 * @return Nothing
 */
static void unpack_accel(const struct bma400_fifo_data *fifo,
                         struct bma400_sensor_data *accel_data,
                         uint16_t *data_index,
                         uint8_t accel_width,
                         uint8_t frame_header);

/*
 * @brief This API is used to parse and store the sensor time from the
 * FIFO data in the structure instance dev
 *
 * @param[in,out] fifo         : Pointer to the fifo structure.
 * @param[in,out] data_index   : Index of the FIFO data which has sensor time
 *
 * @return Nothing
 */
static void unpack_sensortime_frame(struct bma400_fifo_data *fifo, uint16_t *data_index);

/*
 * @brief This API validates the self test results
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 *
 *@param[in] dev   : structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval +ve value -> Warning
 * @retval -ve value -> Error
 */
static int8_t validate_accel_self_test(const struct bma400_sensor_data *accel_pos,
                                       const struct bma400_sensor_data *accel_neg);

/*
 * @brief This API performs self test with positive excitation
 *
 * @param[in] accel_pos : Structure pointer to store accel data
 *                        for positive excitation
 * @param[in] dev   : structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval -ve value -> Error
 */
static int8_t positive_excited_accel(struct bma400_sensor_data *accel_pos, struct bma400_dev *dev);

/*
 * @brief This API performs self test with negative excitation
 *
 * @param[in] accel_neg : Structure pointer to store accel data
 *                        for negative excitation
 * @param[in] dev   : structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval -ve value -> Error
 */
static int8_t negative_excited_accel(struct bma400_sensor_data *accel_neg, struct bma400_dev *dev);

/*
 * @brief This API performs the pre-requisites needed to perform the self test
 *
 * @param[in] dev   : structure instance of bma400_dev
 *
 * @return Result of API execution status
 * @retval zero -> Success
 * @retval -ve value -> Error
 */
static int8_t enable_self_test(struct bma400_dev *dev);

/************************************************************************************/
/*********************** User function definitions **********************************/
/************************************************************************************/

int8_t bma400_init(void) {
  int8_t rslt;
  uint8_t chip_id = 0;

  /* Chip ID of the sensor is read */
  bma400_register_read(0x00, &chip_id, 1);
  /* Check for chip id validity */
  if (chip_id == BMA400_CHIP_ID) {
    /* Store the chip ID in dev structure */
    printf("\nAccel I2C successful %x", chip_id);
    rslt = BMA400_OK;
  } else {
    rslt = BMA400_E_DEV_NOT_FOUND;
    printf("BMA400 I2C fail");
  }

  return rslt;
}

void bma400_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    
    ret_code_t err_code;
    int i;
 
    for(i = 0; i < I2C_MAX_RETRIES; i++)
    {
        err_code = nrf_drv_twi_tx(&m_twi, BMA400_I2C_ADDRESS_SDO_LOW, &register_address, 1, false);
        if(!err_code)
        {
            break;
        }
        nrf_delay_ms(1);
    }

    if(!err_code)
    {
        for(i = 0; i < I2C_MAX_RETRIES; i++)
        {
            err_code = nrf_drv_twi_rx(&m_twi, BMA400_I2C_ADDRESS_SDO_LOW, destination, number_of_bytes);
            if(!err_code)
            {
               
                break;
            }
        }

        if(err_code)
        {
            NRF_LOG_INFO("Failed to read value 0x%2.2X at Register address 0x%2.2X !", destination, register_address);
        }
    }
    else
    {
        NRF_LOG_INFO("Failed to write Register address 0x%2.2X !", register_address);
    }
}


int8_t bma400_register_write(uint8_t register_address, uint8_t value)
{
    int i;
    ret_code_t err_code;
    uint8_t bytes_to_send[] = {register_address, value};

    err_code = nrf_drv_twi_tx(&m_twi, BMA400_I2C_ADDRESS_SDO_LOW, bytes_to_send, sizeof(bytes_to_send), false);
    nrf_delay_ms(1);

    if(err_code)
    {
        NRF_LOG_INFO("Failed to write value 0x%2.2X at Register address 0x%2.2X ! \n", value, register_address);
    }
    return err_code;

}


int8_t bma400_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
{
    int8_t rslt = 0;
    uint16_t index;
    uint8_t dummy_byte;

    {
      uint32_t temp_len = len + dummy_byte;
      uint8_t temp_buff[temp_len];

      /* Read the data from the reg_addr */
      bma400_register_read(reg_addr, temp_buff, temp_len);
      {
        for (index = 0; index < len; index++) {
          /* Parse the data read and store in "reg_data"
                   * buffer so that the dummy byte is removed
                   * and user will get only valid data
                   */
          reg_data[index] = temp_buff[index + dummy_byte];
        }
      }
    }

    return rslt;
}

int8_t bma400_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    int8_t rslt;
    uint8_t count;


    {
        /* Write the data to the reg_addr */

        /* SPI write requires to set The MSB of reg_addr as 0
         * but in default the MSB is always 0
         */
        if (len == 1)
        {
          rslt = bma400_register_write(reg_addr, *reg_data);
          //printf("reg_data = %d\n",*reg_data);
            if (rslt != BMA400_INTF_RET_SUCCESS)
            {
                /* Failure case */
                rslt = BMA400_E_COM_FAIL;
            }
        }

        /* Burst write is not allowed thus we split burst case write
         * into single byte writes Thus user can write multiple bytes
         * with ease
         */
        if (len > 1)
        {
            for (count = 0; (count < len) && (rslt == BMA400_OK); count++)
            {
               rslt = bma400_register_write(reg_addr, *(&reg_data[count]));
                reg_addr++;
                nrf_delay_ms(1);
                if (rslt != BMA400_INTF_RET_SUCCESS)
                {
                    /* Failure case */
                    rslt = BMA400_E_COM_FAIL;
                }
            }
        }
    }


    return rslt;
}

int8_t bma400_soft_reset(void)
{
    int8_t rslt;
    uint8_t data = BMA400_SOFT_RESET_CMD;

    {
      /* Reset the device */
      rslt = bma400_set_regs(BMA400_REG_COMMAND, &data, 1);
      nrf_delay_ms(70);
    }

    return rslt;
}

int8_t bma400_set_power_mode(uint8_t power_mode)
{
    int8_t rslt;
    uint8_t reg_data = 0;


    rslt = bma400_get_regs(BMA400_REG_ACCEL_CONFIG_0, &reg_data, 1);


    if (rslt == BMA400_OK)
    {
        reg_data = BMA400_SET_BITS_POS_0(reg_data, BMA400_POWER_MODE, power_mode);
        nrf_delay_ms(40);

        /* Set the power mode of sensor */
        rslt = bma400_set_regs(BMA400_REG_ACCEL_CONFIG_0, &reg_data, 1);
        nrf_delay_ms(40);
    }

    return rslt;
}

int8_t bma400_get_power_mode(uint8_t *power_mode)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bma400_get_regs(BMA400_REG_STATUS, &reg_data, 1);
    *power_mode = BMA400_GET_BITS(reg_data, BMA400_POWER_MODE_STATUS);
    printf("power mode %s",*power_mode);
    return rslt;
}

int8_t bma400_get_interrupt_status(uint16_t *int_status, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[3];

    {
        /* Read the interrupt status registers */
        rslt = bma400_get_regs(BMA400_REG_INT_STAT0, reg_data, 3);
        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_INT_STATUS, reg_data[2]);

        /* Concatenate the interrupt status to the output */
        *int_status = ((uint16_t)reg_data[1] << 8) | reg_data[0];
    }


    return rslt;
}

int8_t bma400_set_step_counter_param(const uint8_t *sccr_conf)
{
    int8_t rslt;

    /* Set the step counter parameters in the sensor */
    rslt = bma400_set_regs(0x59, &sccr_conf[0], 1);
    rslt = bma400_set_regs(0x59 + 1, &sccr_conf[1], 1);
    rslt = bma400_set_regs(0x59 + 2, &sccr_conf[2], 1);
    rslt = bma400_set_regs(0x59 + 3, &sccr_conf[3], 1);
    rslt = bma400_set_regs(0x59 + 4, &sccr_conf[4], 1);
    rslt = bma400_set_regs(0x59 + 5, &sccr_conf[5], 1);
    rslt = bma400_set_regs(0x59 + 6, &sccr_conf[6], 1);
    rslt = bma400_set_regs(0x59 + 7, &sccr_conf[7], 1);
    rslt = bma400_set_regs(0x59 + 8, &sccr_conf[8], 1);
    rslt = bma400_set_regs(0x59 + 9, &sccr_conf[9], 1);
    rslt = bma400_set_regs(0x59 + 10, &sccr_conf[10], 1);
    rslt = bma400_set_regs(0x59 + 11, &sccr_conf[11], 1);
    rslt = bma400_set_regs(0x59 + 12, &sccr_conf[12], 1);
    rslt = bma400_set_regs(0x59 + 13, &sccr_conf[13], 1);
    rslt = bma400_set_regs(0x59 + 14, &sccr_conf[14], 1);
    rslt = bma400_set_regs(0x59 + 15, &sccr_conf[15], 1);
    rslt = bma400_set_regs(0x59 + 16, &sccr_conf[16], 1);
    rslt = bma400_set_regs(0x59 + 17, &sccr_conf[17], 1);
    rslt = bma400_set_regs(0x59 + 18, &sccr_conf[18], 1);
    rslt = bma400_set_regs(0x59 + 19, &sccr_conf[19], 1);
    rslt = bma400_set_regs(0x59 + 20, &sccr_conf[20], 1);
    rslt = bma400_set_regs(0x59 + 21, &sccr_conf[21], 1);
    rslt = bma400_set_regs(0x59 + 22, &sccr_conf[22], 1);
    rslt = bma400_set_regs(0x59 + 23, &sccr_conf[23], 1);

    return rslt;
}

int8_t bma400_get_steps_counted(uint32_t *step_count, uint8_t *activity_data, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[4];
    uint8_t count = 10;
    uint32_t step_count_0 = 0;
    uint32_t step_count_1 = 0;
    uint32_t step_count_2 = 0;

    rslt = bma400_get_regs(BMA400_REG_STEP_CNT_0, data_array, 4);

    step_count_0 = (uint32_t)data_array[0];
    step_count_1 = (uint32_t)data_array[1] << 8;
    step_count_2 = (uint32_t)data_array[2] << 16;
    *step_count = step_count_0 | step_count_1 | step_count_2;

    *activity_data = data_array[3];
    switch (*activity_data) {
    case BMA400_STILL_ACT:
      //printf("Step counter interrupt received\n");
      //printf("Steps counted : %ld\n", step_count);
      //printf("Still Activity detected\n");
      //count--;
      break;
    case BMA400_WALK_ACT:
      //printf("Step counter interrupt received\n");
      //printf("Steps counted : %ld\n", step_count);
      //printf("Walking Activity detected\n");
      //count--;
      break;
    case BMA400_RUN_ACT:
      //printf("Step counter interrupt received\n");
      //printf("Steps counted : %ld\n", step_count);
      //printf("Running Activity detected\n");
      //count--;

      break;
    }


    return rslt;
}

int8_t bma400_get_interrupts_enabled(struct bma400_int_enable *int_select, uint8_t n_sett, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t idx = 0;
    uint8_t reg_data[2];
    uint8_t wkup_int;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMA400_OK) && (int_select != NULL))
    {
        rslt = bma400_get_regs(BMA400_REG_INT_CONF_0, reg_data, 2);
        if (rslt == BMA400_OK)
        {
            for (idx = 0; idx < n_sett; idx++)
            {
                /* Read the enable/disable of interrupts
                 * based on user selection
                 */
                switch (int_select[idx].type)
                {
                    case BMA400_DRDY_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_DRDY);
                        break;
                    case BMA400_FIFO_WM_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_FIFO_WM);
                        break;
                    case BMA400_FIFO_FULL_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_FIFO_FULL);
                        break;
                    case BMA400_GEN2_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_GEN2);
                        break;
                    case BMA400_GEN1_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_GEN1);
                        break;
                    case BMA400_ORIENT_CHANGE_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[0], BMA400_EN_ORIENT_CH);
                        break;
                    case BMA400_LATCH_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[1], BMA400_EN_LATCH);
                        break;
                    case BMA400_ACTIVITY_CHANGE_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[1], BMA400_EN_ACTCH);
                        break;
                    case BMA400_DOUBLE_TAP_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[1], BMA400_EN_D_TAP);
                        break;
                    case BMA400_SINGLE_TAP_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS(reg_data[1], BMA400_EN_S_TAP);
                        break;
                    case BMA400_STEP_COUNTER_INT_EN:
                        int_select[idx].conf = BMA400_GET_BITS_POS_0(reg_data[1], BMA400_EN_STEP_INT);
                        break;
                    case BMA400_AUTO_WAKEUP_EN:
                        rslt = bma400_get_regs(BMA400_REG_AUTOWAKEUP_1, &wkup_int, 1);
                        if (rslt == BMA400_OK)
                        {
                            /* Auto-Wakeup int status */
                            int_select[idx].conf = BMA400_GET_BITS(wkup_int, BMA400_WAKEUP_INTERRUPT);
                        }

                        break;
                    default:
                        rslt = BMA400_E_INVALID_CONFIG;
                        break;
                }
            }
        }
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }

    return rslt;
}

int8_t bma400_enable_interrupt(const struct bma400_int_enable *int_select, uint8_t n_sett, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t conf, idx = 0;
    uint8_t reg_data[2];
    uint8_t reg_data1[2];

   
    {
        rslt = bma400_get_regs(BMA400_REG_INT_CONF_0, reg_data, 2);
        if (rslt == BMA400_OK)
        {
            for (idx = 0; idx < n_sett; idx++)
            {
                conf = int_select[idx].conf;

                /* Enable the interrupt based on user selection */
                switch (int_select[idx].type)
                {
                    case BMA400_DRDY_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_DRDY, conf);
                        break;
                    case BMA400_FIFO_WM_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_FIFO_WM, conf);
                        break;
                    case BMA400_FIFO_FULL_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_FIFO_FULL, conf);
                        break;
                    case BMA400_GEN2_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_GEN2, conf);
                        break;
                    case BMA400_GEN1_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_GEN1, conf);
                        break;
                    case BMA400_ORIENT_CHANGE_INT_EN:
                        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_EN_ORIENT_CH, conf);
                        break;
                    case BMA400_LATCH_INT_EN:
                        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_EN_LATCH, conf);
                        break;
                    case BMA400_ACTIVITY_CHANGE_INT_EN:
                        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_EN_ACTCH, conf);
                        break;
                    case BMA400_DOUBLE_TAP_INT_EN:
                        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_EN_D_TAP, conf);
                        break;
                    case BMA400_SINGLE_TAP_INT_EN:
                        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_EN_S_TAP, conf);
                        break;
                    case BMA400_STEP_COUNTER_INT_EN:
                        reg_data[1] = BMA400_SET_BITS_POS_0(reg_data[1], BMA400_EN_STEP_INT, conf);
                        break;
                    case BMA400_AUTO_WAKEUP_EN:
                        rslt = set_auto_wakeup(conf, dev);
                        break;
                    default:
                        rslt = BMA400_E_INVALID_CONFIG;
                        break;
                }
            }

            if (rslt == BMA400_OK)
            {
                /* Set the configurations in the sensor */
                rslt = bma400_set_regs(BMA400_REG_INT_CONF_0, &reg_data[0], 1);
                rslt = bma400_set_regs(BMA400_REG_INT_CONF_0+1, &reg_data[1], 1);
            }
             rslt = bma400_get_regs(0x24, reg_data1, 2);
        }
    }
  

    return rslt;
}

int8_t bma400_get_accel_data(uint8_t data_sel, struct bma400_sensor_data *accel, struct bma400_dev *dev)
{
    int8_t rslt;

    {
        /* Read and store the accel data */
        rslt = get_accel_data(data_sel, accel, dev);
    }


    return rslt;
}

int8_t bma400_set_sensor_conf(const struct bma400_sensor_conf *conf, uint16_t n_sett)
{
    int8_t rslt;
    uint16_t idx = 0;
    uint8_t data_array[3] = { 0 };
    uint8_t reg_data[3] = { 0 };


    {
        /* Read the interrupt pin mapping configurations */
        rslt = bma400_get_regs(BMA400_REG_INT_MAP, data_array, 3);
        nrf_delay_ms(2);
        if (rslt == BMA400_OK)
        {
            for (idx = 0; (idx < n_sett) && (rslt == BMA400_OK); idx++)
            {
                rslt = set_sensor_conf(data_array, conf + idx);
            }

            if (rslt == BMA400_OK)
            {
                /* Set the interrupt pin mapping configurations */
                rslt = bma400_set_regs(BMA400_REG_INT_MAP+0, &data_array[0], 1);
                rslt = bma400_set_regs(BMA400_REG_INT_MAP+1, &data_array[1], 1);
                rslt = bma400_set_regs(BMA400_REG_INT_MAP+2, &data_array[2], 1);

                rslt = bma400_get_regs(BMA400_REG_INT_MAP, reg_data, 3);
            }
        }
    }

    return rslt;
}

int8_t bma400_get_sensor_conf(struct bma400_sensor_conf *conf, uint16_t n_sett, struct bma400_dev *dev)
{
    int8_t rslt;
    uint16_t idx;
    uint8_t data_array[3] = { 0 };


    {
        /* Read the interrupt pin mapping configurations */
        rslt = bma400_get_regs(BMA400_REG_INT_MAP, data_array, 3);

        for (idx = 0; (idx < n_sett) && (rslt == BMA400_OK); idx++)
        {
            rslt = get_sensor_conf(data_array, conf + idx, dev);
        }
    }


    return rslt;
}

int8_t bma400_set_device_conf(const struct bma400_device_conf *conf, uint8_t n_sett, struct bma400_dev *dev)
{
    int8_t rslt;
    uint16_t idx;
    uint8_t data_array[3] = { 0 };

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    if ((rslt == BMA400_OK) && (conf != NULL))
    {

        /* Read the interrupt pin mapping configurations */
        rslt = bma400_get_regs(BMA400_REG_INT_MAP, data_array, 3);

        for (idx = 0; (idx < n_sett) && (rslt == BMA400_OK); idx++)
        {
            switch (conf[idx].type)
            {
                case BMA400_AUTOWAKEUP_TIMEOUT:
                    rslt = set_autowakeup_timeout(&conf[idx].param.auto_wakeup, dev);
                    break;
                case BMA400_AUTOWAKEUP_INT:
                    rslt = set_autowakeup_interrupt(&conf[idx].param.wakeup, dev);
                    if (rslt == BMA400_OK)
                    {
                        /* Interrupt pin mapping */
                        map_int_pin(data_array, BMA400_WAKEUP_INT_MAP, conf[idx].param.wakeup.int_chan);
                    }

                    break;
                case BMA400_AUTO_LOW_POWER:
                    rslt = set_auto_low_power(&conf[idx].param.auto_lp, dev);
                    break;
                case BMA400_INT_PIN_CONF:
                    rslt = set_int_pin_conf(conf[idx].param.int_conf, dev);
                    break;
                case BMA400_INT_OVERRUN_CONF:

                    /* Interrupt pin mapping */
                    map_int_pin(data_array, BMA400_INT_OVERRUN_MAP, conf[idx].param.overrun_int.int_chan);
                    break;
                case BMA400_FIFO_CONF:
                    rslt = set_fifo_conf(&conf[idx].param.fifo_conf, dev);
                    if (rslt == BMA400_OK)
                    {
                        /* Interrupt pin mapping */
                        map_int_pin(data_array, BMA400_FIFO_WM_INT_MAP, conf[idx].param.fifo_conf.fifo_wm_channel);
                        map_int_pin(data_array, BMA400_FIFO_FULL_INT_MAP, conf[idx].param.fifo_conf.fifo_full_channel);
                    }

                    break;
                default:
                    rslt = BMA400_E_INVALID_CONFIG;
            }
        }

        if (rslt == BMA400_OK)
        {
            /* Set the interrupt pin mapping configurations */
            rslt = bma400_set_regs(BMA400_REG_INT_MAP, data_array, 3);
        }
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }

    return rslt;
}

int8_t bma400_get_device_conf(struct bma400_device_conf *conf, uint8_t n_sett, struct bma400_dev *dev)
{
    int8_t rslt;
    uint16_t idx = 0;
    uint8_t data_array[3] = { 0 };

    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMA400_OK) && (conf != NULL))
    {
        /* Read the interrupt pin mapping configurations */
        rslt = bma400_get_regs(BMA400_REG_INT_MAP, data_array, 3);

        for (idx = 0; (idx < n_sett) && (rslt == BMA400_OK); idx++)
        {
            switch (conf[idx].type)
            {
                case BMA400_AUTOWAKEUP_TIMEOUT:
                    rslt = get_autowakeup_timeout(&conf[idx].param.auto_wakeup, dev);
                    break;
                case BMA400_AUTOWAKEUP_INT:
                    rslt = get_autowakeup_interrupt(&conf[idx].param.wakeup, dev);
                    if (rslt == BMA400_OK)
                    {
                        /* Get the INT pin mapping */
                        get_int_pin_map(data_array, BMA400_WAKEUP_INT_MAP, &conf[idx].param.wakeup.int_chan);
                    }

                    break;
                case BMA400_AUTO_LOW_POWER:
                    rslt = get_auto_low_power(&conf[idx].param.auto_lp, dev);
                    break;
                case BMA400_INT_PIN_CONF:
                    rslt = get_int_pin_conf(&conf[idx].param.int_conf, dev);
                    break;
                case BMA400_INT_OVERRUN_CONF:
                    get_int_pin_map(data_array, BMA400_INT_OVERRUN_MAP, &conf[idx].param.overrun_int.int_chan);
                    break;
                case BMA400_FIFO_CONF:
                    rslt = get_fifo_conf(&conf[idx].param.fifo_conf, dev);
                    if (rslt == BMA400_OK)
                    {
                        get_int_pin_map(data_array,
                                        BMA400_FIFO_FULL_INT_MAP,
                                        &conf[idx].param.fifo_conf.fifo_full_channel);
                        get_int_pin_map(data_array, BMA400_FIFO_WM_INT_MAP, &conf[idx].param.fifo_conf.fifo_wm_channel);
                    }

                    break;
            }
        }
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }

    return rslt;
}



int8_t bma400_get_fifo_data(struct bma400_fifo_data *fifo, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint16_t fifo_byte_cnt = 0;
    uint16_t user_fifo_len = 0;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMA400_OK) && (fifo != NULL))
    {
        /* Resetting the FIFO data byte index */
        fifo->accel_byte_start_idx = 0;

        /* Reading the FIFO length */
        rslt = get_fifo_length(&fifo_byte_cnt, dev);
        if (rslt == BMA400_OK)
        {
            /* Get the FIFO configurations
             * from the sensor */
            rslt = bma400_get_regs(BMA400_REG_FIFO_CONFIG_0, &data, 1);
            if (rslt == BMA400_OK)
            {
                /* Get the data from FIFO_CONFIG0 register */
                fifo->fifo_8_bit_en = BMA400_GET_BITS(data, BMA400_FIFO_8_BIT_EN);
                fifo->fifo_data_enable = BMA400_GET_BITS(data, BMA400_FIFO_AXES_EN);
                fifo->fifo_time_enable = BMA400_GET_BITS(data, BMA400_FIFO_TIME_EN);
                fifo->fifo_sensor_time = 0;
                user_fifo_len = fifo->length;
                if (fifo->length > fifo_byte_cnt)
                {
                    /* Handling case where user requests
                     * more data than available in FIFO
                     */
                    fifo->length = fifo_byte_cnt;
                }

                /* Reading extra bytes as per the macro
                 * "BMA400_FIFO_BYTES_OVERREAD"
                 * when FIFO time is enabled
                 */
                if ((fifo->fifo_time_enable == BMA400_ENABLE) &&
                    (fifo_byte_cnt + BMA400_FIFO_BYTES_OVERREAD <= user_fifo_len))
                {
                    /* Handling sensor time availability*/
                    fifo->length = fifo->length + BMA400_FIFO_BYTES_OVERREAD;
                }

                /* Read the FIFO data */
                rslt = read_fifo(fifo, dev);
            }
        }
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }

    return rslt;
}

int8_t bma400_extract_accel(struct bma400_fifo_data *fifo,
                            struct bma400_sensor_data *accel_data,
                            uint16_t *frame_count,
                            const struct bma400_dev *dev)
{
    int8_t rslt;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == BMA400_OK) && (fifo != NULL) && (accel_data != NULL) && (frame_count != NULL))
    {
        /* Parse the FIFO data */
        unpack_accel_frame(fifo, accel_data, frame_count, dev);
    }
    else
    {
        rslt = BMA400_E_NULL_PTR;
    }

    return rslt;
}

int8_t bma400_set_fifo_flush(struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data = BMA400_FIFO_FLUSH_CMD;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMA400_OK)
    {
        /* FIFO flush command is set */
        rslt = bma400_set_regs(BMA400_REG_COMMAND, &data, 1);
    }

    return rslt;
}

int8_t EnableDetectionOfFreeFall(void)
{
  int8_t rslt;
  uint8_t mode_normal = 0x02;
  uint8_t data = 0x49;
  uint8_t Manual_update = 0xF0;
  uint8_t inact_det= 0x01;
  uint8_t inact_thres= 0x3F;
  uint8_t inter_dura =0x0C;///free fall threshold
  uint8_t comm_temp = 0x00;
  uint8_t gen_int = 0x04;
  uint8_t push_pull = 0x22;
  uint8_t clr_reg = 0x80;

   //COMMON REGISTERS:
  rslt = bma400_set_regs(0x19, &mode_normal, 1);//Write 0x02 to register 0x19 (BMA400_ACC_CONFIG_0); // bring BMA400 to normal mode form sleep mode ->  SLEEP = 0x00, // Stop conversion LOW_POWER = 0x01, NORMAL = 0x02,
  nrf_delay_us(10);
  rslt = bma400_set_regs(0x1A, &data ,1);//Write 0x49 to register 0x1A (BMA400_ACC_CONFIG_1); // set BMA400 to 200Hz ODR, +/-4g full scale range and 0 over sampling rate (OSR) meaning one single measurement without averaging. The current consumption is about 3.5uA
  nrf_delay_us(10);
  rslt = bma400_set_regs(0x1B, &comm_temp,1);//Write 0x00 to register 0x1B (BMA400_ACC_CONFIG_2); // select acc_filt1 200Hz as source for data registers
  nrf_delay_us(10);

  //INTERRUPT 1 PARAMETERS FOR FREE FALL DETECTION:
  rslt = bma400_set_regs(0x3F, &Manual_update, 1);//Write 0xF0 to register 0x3F (BMA400_GEN_1_INT_CONFIG_0);   // enable X/Y/Z axis for interrupt evaluation. Gen1 interrupt engine data source is acc_filt2 which is fixed 100Hz ODR or 10ms time interval. Manual update and hysteresis 0mg
  nrf_delay_us(10);
  rslt = bma400_set_regs(0x40, &inact_det,1);//Write 0x01 to register 0x40 (BMA400_GEN_1_INT_CONFIG_1);   // select inactivity detection which means the interrupt will be generated within the positive and negative threshold zone. Select AND logic meaning that when all enabled axes enter the threshold zone simultaneously an interrupt will be generated
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x41, &inact_thres,1);//Write 0x3F to register 0x41 (BMA400_GEN_1_INT_CONFIG_2);   //  set inactivity threshold to 0x3F = 63LSBs = 63LSBs * 8mg/LSB = 504mg (can be fine-tuned). So the threshold zone is +/-504mg
  nrf_delay_us(10);
  rslt= bma400_set_regs(0x42, &comm_temp,1);  //Write 0x00 to register 0x42 (BMA400_GEN_1_INT_CONFIG_3); // set MSB of Gen1 interrupt duration to 0x00
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x43, &inter_dura,1);//Write 0x0C to register 0x43 (BMA400_GEN_1_INT_CONFIG_3_1); // set LSB of Gen1 interrupt duration to 0x0C = 12LSBs = 12 * 10ms =120ms (can be fine-tuned). This corresponds to about 7cm height freefall. H =0.5 * g * t^2 = 0.5 * 9.81m/s^2 * (0.12s)^2
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x44, &comm_temp,1);//Write 0x00 to register 0x44 (BMA400_GEN_1_INT_CONFIG_4);
  nrf_delay_us(10);
  rslt=bma400_set_regs(0x45, &comm_temp,1);//Write 0x00 to register 0x45 (BMA400_GEN_1_INT_CONFIG_5);   // set X axis reference to 0mg
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x46, &comm_temp,1); //Write 0x00 to register 0x46 (BMA400_GEN_1_INT_CONFIG_6);
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x47, &comm_temp,1);//Write 0x00 to register 0x47 (BMA400_GEN_1_INT_CONFIG_7);   // set Y axis reference to 0mg
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x48, &comm_temp,1);//Write 0x00 to register 0x48 (BMA400_GEN_1_INT_CONFIG_8);
  nrf_delay_us(10);
  rslt =bma400_set_regs(0x49, &comm_temp,1);//Write 0x00 to register 0x49 (BMA400_GEN_1_INT_CONFIG_9);   // set Z axis reference to 0mg > Because the manual update is selected for Gen1, every 10ms BMA400 Gen1 will compare the X/Y/Z measurement data against the X/Y/Z references respectively to check if the differences are all within the threshold or not. If yes, then the duration timer will start counting. If not,then the duration timer will be reset to 0.
  nrf_delay_us(10);
  //

  //CONFIGURE INTERRUPT REGISTERS
  rslt = bma400_set_regs(0x1F, &gen_int,1); //Write 0x04 to register 0x1F (BMA400_INT_CONFIG_0);  // enable generic interrupt 1 (Gen1)
  nrf_delay_us(10);
 rslt = bma400_set_regs(0x21, &gen_int,1);//Write 0x04 to register 0x21 (BMA400_INT_1_MAP);     // route Gen1 interrupt signal to INT1 pin
  nrf_delay_us(10);
 rslt = bma400_set_regs(0x24, &push_pull,1);//Write 0x22 to register 0x24 (BMA400_INT_1_2_CTRL);  // set INT1 pin and INT2 pin both to push-pull and active-high
  nrf_delay_us(10);

  //FORCE STOP
  //force to accept the interrupt on regist 0x20 (BMA400_INT_CONFIG_1) until clear regist 0x0E (BMA400_INT_STAT0)
 //rslt = bma400_set_regs(0x20, &clr_reg,1);
   nrf_delay_us(10);
   
return rslt;
    
}

/************************************************************************************/
/*********************** Static function definitions **********************************/
/************************************************************************************/

static int8_t null_ptr_check(const struct bma400_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->intf_ptr == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = BMA400_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMA400_OK;
    }

    return rslt;
}

int8_t set_sensor_conf(uint8_t *data, const struct bma400_sensor_conf *conf)
{
    int8_t rslt = BMA400_E_INVALID_CONFIG;
    uint8_t int_enable = 0;
    enum bma400_int_chan int_map = BMA400_UNMAP_INT_PIN;

    if (BMA400_ACCEL == conf->type)
    {
        /* Setting Accel configurations */
        rslt = set_accel_conf(&conf->param.accel);
        int_enable = BMA400_DATA_READY_INT_MAP;
        int_map = conf->param.accel.int_chan;
    }

    if (BMA400_TAP_INT == conf->type)
    {
        /* Setting tap configurations */
        rslt = set_tap_conf(&conf->param.tap);
        int_enable = BMA400_TAP_INT_MAP;
        int_map = conf->param.tap.int_chan;
    }

    if (BMA400_ACTIVITY_CHANGE_INT == conf->type)
    {
        /* Setting activity change configurations */
        rslt = set_activity_change_conf(&conf->param.act_ch);
        int_enable = BMA400_ACT_CH_INT_MAP;
        int_map = conf->param.act_ch.int_chan;
    }

    if (BMA400_GEN1_INT == conf->type)
    {
        /* Setting generic int 1 configurations */
        rslt = set_gen1_int(&conf->param.gen_int);
        int_enable = BMA400_GEN1_INT_MAP;
        int_map = conf->param.gen_int.int_chan;
    }

    if (BMA400_GEN2_INT == conf->type)
    {
        /* Setting generic int 2 configurations */
        rslt = set_gen2_int(&conf->param.gen_int);
        int_enable = BMA400_GEN2_INT_MAP;
        int_map = conf->param.gen_int.int_chan;
    }

    if (BMA400_ORIENT_CHANGE_INT == conf->type)
    {
        /* Setting orient int configurations */
        rslt = set_orient_int(&conf->param.orient);
        int_enable = BMA400_ORIENT_CH_INT_MAP;
        int_map = conf->param.orient.int_chan;
    }

    if (BMA400_STEP_COUNTER_INT == conf->type)
    {
        rslt = BMA400_OK;
        int_enable = BMA400_STEP_INT_MAP;
        int_map = conf->param.step_cnt.int_chan;
    }

    if (rslt == BMA400_OK)
    {
        /* Int pin mapping settings */
        map_int_pin(data, int_enable, int_map);
    }
      //rslt = 0;
    return rslt;
}

static int8_t get_sensor_conf(const uint8_t *data, struct bma400_sensor_conf *conf, struct bma400_dev *dev)
{
    int8_t rslt = BMA400_E_INVALID_CONFIG;
    uint8_t int_enable = 0;
    enum bma400_int_chan int_map = BMA400_UNMAP_INT_PIN;

    if (BMA400_ACCEL == conf->type)
    {
        /* Get Accel configurations */
        rslt = get_accel_conf(&conf->param.accel, dev);
        int_enable = BMA400_DATA_READY_INT_MAP;
        int_map = conf->param.accel.int_chan;
    }

    if (BMA400_TAP_INT == conf->type)
    {
        /* Get tap configurations */
        rslt = get_tap_conf(&conf->param.tap, dev);
        int_enable = BMA400_TAP_INT_MAP;
        int_map = conf->param.tap.int_chan;
    }

    if (BMA400_ACTIVITY_CHANGE_INT == conf->type)
    {
        /* Get activity change configurations */
        rslt = get_activity_change_conf(&conf->param.act_ch);
        int_enable = BMA400_ACT_CH_INT_MAP;
        int_map = conf->param.act_ch.int_chan;
    }

    if (BMA400_GEN1_INT == conf->type)
    {
        /* Get generic int 1 configurations */
        rslt = get_gen1_int(&conf->param.gen_int);
        int_enable = BMA400_GEN1_INT_MAP;
        int_map = conf->param.gen_int.int_chan;
    }

    if (BMA400_GEN2_INT == conf->type)
    {
        /* Get generic int 2 configurations */
        rslt = get_gen2_int(&conf->param.gen_int);
        int_enable = BMA400_GEN2_INT_MAP;
        int_map = conf->param.gen_int.int_chan;
    }

    if (BMA400_ORIENT_CHANGE_INT == conf->type)
    {
        /* Get orient int configurations */
        rslt = get_orient_int(&conf->param.orient);
        int_enable = BMA400_ORIENT_CH_INT_MAP;
        int_map = conf->param.orient.int_chan;
    }

    if (BMA400_STEP_COUNTER_INT == conf->type)
    {
        rslt = BMA400_OK;
        int_enable = BMA400_STEP_INT_MAP;
        int_map = conf->param.step_cnt.int_chan;
    }

    if (rslt == BMA400_OK)
    {
        /* Int pin mapping settings */
        get_int_pin_map(data, int_enable, &int_map);
    }

    return rslt;
}

static int8_t set_accel_conf(const struct bma400_acc_conf *accel_conf)
{
    int8_t rslt;
    uint8_t data_array[3] = { 0, 0, 0xE0 };

    /* Update the accel configurations from the user structure
     * accel_conf
     */
    //rslt = bma400_get_regs(BMA400_REG_ACCEL_CONFIG_0, data_array, 3);
    bma400_get_regs(BMA400_REG_ACCEL_CONFIG_0, data_array, 3);
    //if (rslt == BMA400_OK)
    {
        data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_FILT_1_BW, accel_conf->filt1_bw);
        data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_OSR_LP, accel_conf->osr_lp);
        data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_ACCEL_RANGE, accel_conf->range);
        data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_OSR, accel_conf->osr);
        data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_ACCEL_ODR, accel_conf->odr);
        data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_DATA_FILTER, accel_conf->data_src);

        /* Set the accel configurations in the sensor */
        rslt = bma400_set_regs(BMA400_REG_ACCEL_CONFIG_0, data_array, 1);
        rslt = bma400_set_regs(BMA400_REG_ACCEL_CONFIG_0+1, data_array, 1);
        rslt = bma400_set_regs(BMA400_REG_ACCEL_CONFIG_0+2, data_array, 1);
    }

    return rslt;
}

static int8_t get_accel_conf(struct bma400_acc_conf *accel_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[3];

    rslt = bma400_get_regs(BMA400_REG_ACCEL_CONFIG_0, data_array, 3);
    if (rslt == BMA400_OK)
    {
        accel_conf->filt1_bw = BMA400_GET_BITS(data_array[0], BMA400_FILT_1_BW);
        accel_conf->osr_lp = BMA400_GET_BITS(data_array[0], BMA400_OSR_LP);
        accel_conf->range = BMA400_GET_BITS(data_array[1], BMA400_ACCEL_RANGE);
        accel_conf->osr = BMA400_GET_BITS(data_array[1], BMA400_OSR);
        accel_conf->odr = BMA400_GET_BITS_POS_0(data_array[1], BMA400_ACCEL_ODR);
        accel_conf->data_src = BMA400_GET_BITS(data_array[2], BMA400_DATA_FILTER);
    }

    return rslt;
}

static int8_t get_accel_data(uint8_t data_sel, struct bma400_sensor_data *accel, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[9] = { 0 };
    uint16_t lsb;
    uint8_t msb;
    uint8_t time_0;
    uint16_t time_1;
    uint32_t time_2;

    if (data_sel == BMA400_DATA_ONLY)
    {
        /* Read the sensor data registers only */
        rslt = bma400_get_regs(BMA400_REG_ACCEL_DATA, data_array, 6);
    }
    else if (data_sel == BMA400_DATA_SENSOR_TIME)
    {
        /* Read the sensor data along with sensor time */
        rslt = bma400_get_regs(BMA400_REG_ACCEL_DATA, data_array, 9);
    }
    else
    {
        /* Invalid use of "data_sel" */
        rslt = BMA400_E_INVALID_CONFIG;
    }

    if (rslt == BMA400_OK)
    {
        lsb = data_array[0];
        msb = data_array[1];

        /* accel X axis data */
        accel->x = (int16_t)(((uint16_t)msb * 256) + lsb);
        if (accel->x > 2047)
        {
            /* Computing accel data negative value */
            accel->x = accel->x - 4096;
        }

        lsb = data_array[2];
        msb = data_array[3];

        /* accel Y axis data */
        accel->y = (int16_t)(((uint16_t)msb * 256) | lsb);
        if (accel->y > 2047)
        {
            /* Computing accel data negative value */
            accel->y = accel->y - 4096;
        }

        lsb = data_array[4];
        msb = data_array[5];

        /* accel Z axis data */
        accel->z = (int16_t)(((uint16_t)msb * 256) | lsb);
        if (accel->z > 2047)
        {
            /* Computing accel data negative value */
            accel->z = accel->z - 4096;
        }

        if (data_sel == BMA400_DATA_ONLY)
        {
            /* Update sensortime as 0 */
            accel->sensortime = 0;
        }

        if (data_sel == BMA400_DATA_SENSOR_TIME)
        {
            /* Sensor-time data*/
            time_0 = data_array[6];
            time_1 = ((uint16_t)data_array[7] << 8);
            time_2 = ((uint32_t)data_array[8] << 16);
            accel->sensortime = (uint32_t)(time_2 + time_1 + time_0);
        }
    }

    return rslt;
}


static int8_t set_autowakeup_timeout(const struct bma400_auto_wakeup_conf *wakeup_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[2];
    uint8_t lsb;
    uint8_t msb;

    rslt = bma400_get_regs(BMA400_REG_AUTOWAKEUP_1, &data_array[1], 1);
    if (rslt == BMA400_OK)
    {
        data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_WAKEUP_TIMEOUT, wakeup_conf->wakeup_timeout);

        /* LSB of timeout threshold */
        lsb = BMA400_GET_BITS_POS_0(wakeup_conf->timeout_thres, BMA400_WAKEUP_THRES_LSB);

        /* MSB of timeout threshold */
        msb = BMA400_GET_BITS(wakeup_conf->timeout_thres, BMA400_WAKEUP_THRES_MSB);

        /* Set the value in the data_array */
        data_array[0] = msb;
        data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_WAKEUP_TIMEOUT_THRES, lsb);
        rslt = bma400_set_regs(BMA400_REG_AUTOWAKEUP_0, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_AUTOWAKEUP_0+1, &data_array[1], 1);
    }

    return rslt;
}

static int8_t get_autowakeup_timeout(struct bma400_auto_wakeup_conf *wakeup_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[2];
    uint8_t lsb;
    uint8_t msb;

    rslt = bma400_get_regs(BMA400_REG_AUTOWAKEUP_0, data_array, 2);
    if (rslt == BMA400_OK)
    {
        wakeup_conf->wakeup_timeout = BMA400_GET_BITS(data_array[1], BMA400_WAKEUP_TIMEOUT);
        msb = data_array[0];
        lsb = BMA400_GET_BITS(data_array[1], BMA400_WAKEUP_TIMEOUT_THRES);

        /* Store the timeout value in the wakeup structure */
        wakeup_conf->timeout_thres = msb << 4 | lsb;
    }

    return rslt;
}

static int8_t set_auto_wakeup(uint8_t conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bma400_get_regs(BMA400_REG_AUTOWAKEUP_1, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        reg_data = BMA400_SET_BITS(reg_data, BMA400_WAKEUP_INTERRUPT, conf);

        /* Enabling the Auto wakeup interrupt */
        rslt = bma400_set_regs(BMA400_REG_AUTOWAKEUP_1, &reg_data, 1);
    }

    return rslt;
}

static int8_t set_autowakeup_interrupt(const struct bma400_wakeup_conf *wakeup_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[5] = { 0 };

    /* Set the wakeup reference update */
    data_array[0] = BMA400_SET_BITS_POS_0(data_array[0], BMA400_WKUP_REF_UPDATE, wakeup_conf->wakeup_ref_update);

    /* Set the number of samples for interrupt condition evaluation */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_SAMPLE_COUNT, wakeup_conf->sample_count);

    /* Enable low power wake-up interrupt for X,Y,Z axes*/
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_WAKEUP_EN_AXES, wakeup_conf->wakeup_axes_en);

    /* Set interrupt threshold configuration  */
    data_array[1] = wakeup_conf->int_wkup_threshold;

    /* Set the reference acceleration x-axis for the wake-up interrupt */
    data_array[2] = wakeup_conf->int_wkup_ref_x;

    /* Set the reference acceleration y-axis for the wake-up interrupt */
    data_array[3] = wakeup_conf->int_wkup_ref_y;

    /* Set the reference acceleration z-axis for the wake-up interrupt */
    data_array[4] = wakeup_conf->int_wkup_ref_z;

    /* Set the wakeup interrupt configurations in the sensor */
    rslt = bma400_set_regs(BMA400_REG_WAKEUP_INT_CONF_0, &data_array[0], 1);
    rslt = bma400_set_regs(BMA400_REG_WAKEUP_INT_CONF_0+1, &data_array[1], 1);
    rslt = bma400_set_regs(BMA400_REG_WAKEUP_INT_CONF_0+2, &data_array[2], 1);
    rslt = bma400_set_regs(BMA400_REG_WAKEUP_INT_CONF_0+3, &data_array[3], 1);
    rslt = bma400_set_regs(BMA400_REG_WAKEUP_INT_CONF_0+4, &data_array[4], 1);

    return rslt;
}

static int8_t get_autowakeup_interrupt(struct bma400_wakeup_conf *wakeup_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[5];

    rslt = bma400_get_regs(BMA400_REG_WAKEUP_INT_CONF_0, data_array, 5);
    if (rslt == BMA400_OK)
    {
        /* get the wakeup reference update */
        wakeup_conf->wakeup_ref_update = BMA400_GET_BITS_POS_0(data_array[0], BMA400_WKUP_REF_UPDATE);

        /* Get the number of samples for interrupt condition evaluation */
        wakeup_conf->sample_count = BMA400_GET_BITS(data_array[0], BMA400_SAMPLE_COUNT);

        /* Get the axes enabled */
        wakeup_conf->wakeup_axes_en = BMA400_GET_BITS(data_array[0], BMA400_WAKEUP_EN_AXES);

        /* Get interrupt threshold configuration  */
        wakeup_conf->int_wkup_threshold = data_array[1];

        /* Get the reference acceleration x-axis for the wake-up interrupt */
        wakeup_conf->int_wkup_ref_x = data_array[2];

        /* Get the reference acceleration y-axis for the wake-up interrupt */
        wakeup_conf->int_wkup_ref_y = data_array[3];

        /* Get the reference acceleration z-axis for the wake-up interrupt */
        wakeup_conf->int_wkup_ref_z = data_array[4];
    }

    return rslt;
}

static int8_t set_auto_low_power(const struct bma400_auto_lp_conf *auto_lp_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;
    uint8_t timeout_msb;
    uint8_t timeout_lsb;

    rslt = bma400_get_regs(BMA400_REG_AUTO_LOW_POW_1, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        reg_data = BMA400_SET_BITS_POS_0(reg_data, BMA400_AUTO_LOW_POW, auto_lp_conf->auto_low_power_trigger);

        /* If auto Low power timeout threshold is enabled */
        if (auto_lp_conf->auto_low_power_trigger & 0x0C)
        {
            rslt = bma400_get_regs(BMA400_REG_AUTO_LOW_POW_0, &timeout_msb, 1);
            if (rslt == BMA400_OK)
            {
                /* Compute the timeout threshold MSB value */
                timeout_msb = BMA400_GET_BITS(auto_lp_conf->auto_lp_timeout_threshold, BMA400_AUTO_LP_THRES);

                /* Compute the timeout threshold LSB value */
                timeout_lsb = BMA400_GET_BITS_POS_0(auto_lp_conf->auto_lp_timeout_threshold, BMA400_AUTO_LP_THRES_LSB);
                reg_data = BMA400_SET_BITS(reg_data, BMA400_AUTO_LP_TIMEOUT_LSB, timeout_lsb);

                /* Set the timeout threshold MSB value */
                rslt = bma400_set_regs(BMA400_REG_AUTO_LOW_POW_0, &timeout_msb, 1);
            }
        }

        if (rslt == BMA400_OK)
        {
            /* Set the Auto low power configurations */
            rslt = bma400_set_regs(BMA400_REG_AUTO_LOW_POW_1, &reg_data, 1);
        }
    }

    return rslt;
}

static int8_t get_auto_low_power(struct bma400_auto_lp_conf *auto_lp_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[2];
    uint8_t timeout_msb;
    uint8_t timeout_lsb;

    rslt = bma400_get_regs(BMA400_REG_AUTO_LOW_POW_0, data_array, 2);
    if (rslt == BMA400_OK)
    {
        /* Get the auto low power trigger */
        auto_lp_conf->auto_low_power_trigger = BMA400_GET_BITS_POS_0(data_array[1], BMA400_AUTO_LOW_POW);
        timeout_msb = data_array[0];
        timeout_lsb = BMA400_GET_BITS(data_array[1], BMA400_AUTO_LP_TIMEOUT_LSB);

        /* Get the auto low power timeout threshold */
        auto_lp_conf->auto_lp_timeout_threshold = timeout_msb << 4 | timeout_lsb;
    }

    return rslt;
}

int8_t set_tap_conf(const struct bma400_tap_conf *tap_set)
{
    int8_t rslt;
    uint8_t reg_data[2] = { 0, 0 };

    rslt = bma400_get_regs(BMA400_REG_TAP_CONFIG, reg_data, 2);
    if (rslt == BMA400_OK)
    {
        /* Set the axis to sense for tap */
        reg_data[0] = BMA400_SET_BITS(reg_data[0], BMA400_TAP_AXES_EN, tap_set->axes_sel);

        /* Set the threshold for tap sensing */
        reg_data[0] = BMA400_SET_BITS_POS_0(reg_data[0], BMA400_TAP_SENSITIVITY, tap_set->sensitivity);

        /* Set the Quiet_dt setting */
        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_TAP_QUIET_DT, tap_set->quiet_dt);

        /* Set the Quiet setting */
        reg_data[1] = BMA400_SET_BITS(reg_data[1], BMA400_TAP_QUIET, tap_set->quiet);

        /* Set the tics_th setting */
        reg_data[1] = BMA400_SET_BITS_POS_0(reg_data[1], BMA400_TAP_TICS_TH, tap_set->tics_th);

        /* Set the TAP configuration in the sensor*/
        printf("reg_data in tap %d   %d\n",reg_data[0],reg_data[1]);
        rslt = bma400_set_regs(BMA400_REG_TAP_CONFIG, &reg_data[0], 1);
        rslt = bma400_set_regs(BMA400_REG_TAP_CONFIG+1, &reg_data[1], 1);
    }
    
    return rslt;
}

static int8_t get_tap_conf(struct bma400_tap_conf *tap_set, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data[2];

    rslt = bma400_get_regs(BMA400_REG_TAP_CONFIG, reg_data, 2);
    if (rslt == BMA400_OK)
    {
        /* Get the axis enabled for tap sensing */
        tap_set->axes_sel = BMA400_GET_BITS(reg_data[0], BMA400_TAP_AXES_EN);

        /* Get the threshold for tap sensing */
        tap_set->sensitivity = BMA400_GET_BITS_POS_0(reg_data[0], BMA400_TAP_SENSITIVITY);

        /* Get the Quiet_dt setting */
        tap_set->quiet_dt = BMA400_GET_BITS(reg_data[1], BMA400_TAP_QUIET_DT);

        /* Get the Quiet setting */
        tap_set->quiet = BMA400_GET_BITS(reg_data[1], BMA400_TAP_QUIET);

        /* Get the tics_th setting */
        tap_set->tics_th = BMA400_GET_BITS_POS_0(reg_data[1], BMA400_TAP_TICS_TH);
    }

    return rslt;
}

static int8_t set_activity_change_conf(const struct bma400_act_ch_conf *act_ch_set)
{
    int8_t rslt;
    uint8_t data_array[2] = { 0 };

    /* Set the activity change threshold */
    data_array[0] = act_ch_set->act_ch_thres;

    /* Set the axis to sense for activity change */
    data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_ACT_CH_AXES_EN, act_ch_set->axes_sel);

    /* Set the data source for activity change */
    data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_ACT_CH_DATA_SRC, act_ch_set->data_source);

    /* Set the Number of sample points(NPTS)
     * for sensing activity change
     */
    data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_ACT_CH_NPTS, act_ch_set->act_ch_ntps);

    /* Set the Activity change configuration in the sensor*/
    rslt = bma400_set_regs(BMA400_REG_ACT_CH_CONFIG_0, &data_array[0], 1);
    rslt = bma400_set_regs(BMA400_REG_ACT_CH_CONFIG_0+1, &data_array[1], 1);

    return rslt;
}

static int8_t get_activity_change_conf(struct bma400_act_ch_conf *act_ch_set)
{
    int8_t rslt;
    uint8_t data_array[2];

    rslt = bma400_get_regs(BMA400_REG_ACT_CH_CONFIG_0, data_array, 2);
    if (rslt == BMA400_OK)
    {
        /* Get the activity change threshold */
        act_ch_set->act_ch_thres = data_array[0];

        /* Get the axis enabled for activity change detection */
        act_ch_set->axes_sel = BMA400_GET_BITS(data_array[1], BMA400_ACT_CH_AXES_EN);

        /* Get the data source for activity change */
        act_ch_set->data_source = BMA400_GET_BITS(data_array[1], BMA400_ACT_CH_DATA_SRC);

        /* Get the Number of sample points(NPTS)
         * for sensing activity change
         */
        act_ch_set->act_ch_ntps = BMA400_GET_BITS_POS_0(data_array[1], BMA400_ACT_CH_NPTS);
    }

    return rslt;
}

static int8_t set_gen1_int(const struct bma400_gen_int_conf *gen_int_set)
{
    int8_t rslt;
    uint8_t data_array[11] = { 0 };

    /* Set the axes to sense for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_AXES_EN, gen_int_set->axes_sel);

    /* Set the data source for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_DATA_SRC, gen_int_set->data_src);

    /* Set the reference update mode */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_REFU, gen_int_set->ref_update);

    /* Set the hysteresis for interrupt calculation */
    data_array[0] = BMA400_SET_BITS_POS_0(data_array[0], BMA400_INT_HYST, gen_int_set->hysteresis);

    /* Set the criterion to generate interrupt on either
     * ACTIVITY OR INACTIVITY
     */
    data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_GEN_INT_CRITERION, gen_int_set->criterion_sel);

    /* Set the interrupt axes logic (AND/OR) for the
     * enabled axes to generate interrupt
     */
    data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_GEN_INT_COMB, gen_int_set->evaluate_axes);

    /* Set the interrupt threshold */
    data_array[2] = gen_int_set->gen_int_thres;

    /* Set the MSB of gen int dur */
    data_array[3] = BMA400_GET_MSB(gen_int_set->gen_int_dur);

    /* Set the LSB of gen int dur */
    data_array[4] = BMA400_GET_LSB(gen_int_set->gen_int_dur);

    /* Handling case of manual reference update */
    if (gen_int_set->ref_update == BMA400_UPDATE_MANUAL)
    {
        /* Set the LSB of reference x threshold */
        data_array[5] = BMA400_GET_LSB(gen_int_set->int_thres_ref_x);

        /* Set the MSB of reference x threshold */
        data_array[6] = BMA400_GET_MSB(gen_int_set->int_thres_ref_x);

        /* Set the LSB of reference y threshold */
        data_array[7] = BMA400_GET_LSB(gen_int_set->int_thres_ref_y);

        /* Set the MSB of reference y threshold */
        data_array[8] = BMA400_GET_MSB(gen_int_set->int_thres_ref_y);

        /* Set the LSB of reference z threshold */
        data_array[9] = BMA400_GET_LSB(gen_int_set->int_thres_ref_z);

        /* Set the MSB of reference z threshold */
        data_array[10] = BMA400_GET_MSB(gen_int_set->int_thres_ref_z);

        /* Set the GEN1 INT configuration in the sensor */
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+3, &data_array[3], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+4, &data_array[4], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+5, &data_array[5], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+6, &data_array[6], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+7, &data_array[7], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+8, &data_array[8], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+9, &data_array[9], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+10, &data_array[10], 1);
    }
    else
    {
        /* Set the GEN1 INT configuration in the sensor */
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+3, &data_array[3], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN1_INT_CONFIG+4, &data_array[4], 1);
    }

    return rslt;
}

static int8_t get_gen1_int(struct bma400_gen_int_conf *gen_int_set)
{
    int8_t rslt;
    uint8_t data_array[11];

    rslt = bma400_get_regs(BMA400_REG_GEN1_INT_CONFIG, data_array, 11);
    if (rslt == BMA400_OK)
    {
        /* Get the axes to sense for interrupt */
        gen_int_set->axes_sel = BMA400_GET_BITS(data_array[0], BMA400_INT_AXES_EN);

        /* Get the data source for interrupt */
        gen_int_set->data_src = BMA400_GET_BITS(data_array[0], BMA400_INT_DATA_SRC);

        /* Get the reference update mode */
        gen_int_set->ref_update = BMA400_GET_BITS(data_array[0], BMA400_INT_REFU);

        /* Get the hysteresis for interrupt calculation */
        gen_int_set->hysteresis = BMA400_GET_BITS_POS_0(data_array[0], BMA400_INT_HYST);

        /* Get the interrupt axes logic (AND/OR) to generate interrupt */
        gen_int_set->evaluate_axes = BMA400_GET_BITS_POS_0(data_array[1], BMA400_GEN_INT_COMB);

        /* Get the criterion to generate interrupt ACTIVITY/INACTIVITY */
        gen_int_set->criterion_sel = BMA400_GET_BITS(data_array[1], BMA400_GEN_INT_CRITERION);

        /* Get the interrupt threshold */
        gen_int_set->gen_int_thres = data_array[2];

        /* Get the interrupt duration */
        gen_int_set->gen_int_dur = ((uint16_t)data_array[3] << 8) | data_array[4];

        /* Get the interrupt threshold */
        data_array[6] = data_array[6] & 0x0F;
        gen_int_set->int_thres_ref_x = ((uint16_t)data_array[6] << 8) | data_array[5];
        data_array[8] = data_array[8] & 0x0F;
        gen_int_set->int_thres_ref_y = ((uint16_t)data_array[8] << 8) | data_array[7];
        data_array[10] = data_array[10] & 0x0F;
        gen_int_set->int_thres_ref_z = ((uint16_t)data_array[10] << 8) | data_array[9];
    }

    return rslt;
}

static int8_t set_gen2_int(const struct bma400_gen_int_conf *gen_int_set)
{
    int8_t rslt;
    uint8_t data_array[11] = { 0 };

    /* Set the axes to sense for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_AXES_EN, gen_int_set->axes_sel);

    /* Set the data source for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_DATA_SRC, gen_int_set->data_src);

    /* Set the reference update mode */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_REFU, gen_int_set->ref_update);

    /* Set the hysteresis for interrupt calculation */
    data_array[0] = BMA400_SET_BITS_POS_0(data_array[0], BMA400_INT_HYST, gen_int_set->hysteresis);

    /* Set the criterion to generate interrupt on either
     * ACTIVITY OR INACTIVITY
     */
    data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_GEN_INT_CRITERION, gen_int_set->criterion_sel);

    /* Set the interrupt axes logic (AND/OR) for the
     * enabled axes to generate interrupt
     */
    data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_GEN_INT_COMB, gen_int_set->evaluate_axes);

    /* Set the interrupt threshold */
    data_array[2] = gen_int_set->gen_int_thres;

    /* Set the MSB of gen int dur */
    data_array[3] = BMA400_GET_MSB(gen_int_set->gen_int_dur);

    /* Set the LSB of gen int dur */
    data_array[4] = BMA400_GET_LSB(gen_int_set->gen_int_dur);

    /* Handling case of manual reference update */
    if (gen_int_set->ref_update == BMA400_UPDATE_MANUAL)
    {
        /* Set the LSB of reference x threshold */
        data_array[5] = BMA400_GET_LSB(gen_int_set->int_thres_ref_x);

        /* Set the MSB of reference x threshold */
        data_array[6] = BMA400_GET_MSB(gen_int_set->int_thres_ref_x);

        /* Set the LSB of reference y threshold */
        data_array[7] = BMA400_GET_LSB(gen_int_set->int_thres_ref_y);

        /* Set the MSB of reference y threshold */
        data_array[8] = BMA400_GET_MSB(gen_int_set->int_thres_ref_y);

        /* Set the LSB of reference z threshold */
        data_array[9] = BMA400_GET_LSB(gen_int_set->int_thres_ref_z);

        /* Set the MSB of reference z threshold */
        data_array[10] = BMA400_GET_MSB(gen_int_set->int_thres_ref_z);

        /* Set the GEN2 INT configuration in the sensor */
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+3, &data_array[3], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+4, &data_array[4], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+5, &data_array[5], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+6, &data_array[6], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+7, &data_array[7], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+8, &data_array[8], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+9, &data_array[9], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+10, &data_array[10], 1);
    }
    else
    {
        /* Set the GEN2 INT configuration in the sensor */
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+3, &data_array[3], 1);
        rslt = bma400_set_regs(BMA400_REG_GEN2_INT_CONFIG+4, &data_array[4], 1);
    }

    return rslt;
}

static int8_t get_gen2_int(struct bma400_gen_int_conf *gen_int_set)
{
    int8_t rslt;
    uint8_t data_array[11];

    rslt = bma400_get_regs(BMA400_REG_GEN2_INT_CONFIG, data_array, 11);
    if (rslt == BMA400_OK)
    {
        /* Get the axes to sense for interrupt */
        gen_int_set->axes_sel = BMA400_GET_BITS(data_array[0], BMA400_INT_AXES_EN);

        /* Get the data source for interrupt */
        gen_int_set->data_src = BMA400_GET_BITS(data_array[0], BMA400_INT_DATA_SRC);

        /* Get the reference update mode */
        gen_int_set->ref_update = BMA400_GET_BITS(data_array[0], BMA400_INT_REFU);

        /* Get the hysteresis for interrupt calculation */
        gen_int_set->hysteresis = BMA400_GET_BITS_POS_0(data_array[0], BMA400_INT_HYST);

        /* Get the interrupt axes logic (AND/OR) to generate interrupt */
        gen_int_set->evaluate_axes = BMA400_GET_BITS_POS_0(data_array[1], BMA400_GEN_INT_COMB);

        /* Get the criterion to generate interrupt ACTIVITY/INACTIVITY */
        gen_int_set->criterion_sel = BMA400_GET_BITS(data_array[1], BMA400_GEN_INT_CRITERION);

        /* Get the interrupt threshold */
        gen_int_set->gen_int_thres = data_array[2];

        /* Get the interrupt duration */
        gen_int_set->gen_int_dur = ((uint16_t)data_array[3] << 8) | data_array[4];

        /* Get the interrupt threshold */
        data_array[6] = data_array[6] & 0x0F;
        gen_int_set->int_thres_ref_x = ((uint16_t)data_array[6] << 8) | data_array[5];
        data_array[8] = data_array[8] & 0x0F;
        gen_int_set->int_thres_ref_y = ((uint16_t)data_array[8] << 8) | data_array[7];
        data_array[10] = data_array[10] & 0x0F;
        gen_int_set->int_thres_ref_z = ((uint16_t)data_array[10] << 8) | data_array[9];
    }

    return rslt;
}

static int8_t set_orient_int(const struct bma400_orient_int_conf *orient_conf)
{
    int8_t rslt;
    uint8_t data_array[10] = { 0 };

    /* Set the axes to sense for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_AXES_EN, orient_conf->axes_sel);

    /* Set the data source for interrupt */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_DATA_SRC, orient_conf->data_src);

    /* Set the reference update mode */
    data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_INT_REFU, orient_conf->ref_update);

    /* Set the threshold for interrupt calculation */
    data_array[1] = orient_conf->orient_thres;

    /* Set the stability threshold */
    data_array[2] = orient_conf->stability_thres;

    /* Set the interrupt duration */
    data_array[3] = orient_conf->orient_int_dur;

    /* Handling case of manual reference update */
    if (orient_conf->ref_update == BMA400_UPDATE_MANUAL)
    {
        /* Set the LSB of reference x threshold */
        data_array[4] = BMA400_GET_LSB(orient_conf->orient_ref_x);

        /* Set the MSB of reference x threshold */
        data_array[5] = BMA400_GET_MSB(orient_conf->orient_ref_x);

        /* Set the MSB of reference x threshold */
        data_array[6] = BMA400_GET_LSB(orient_conf->orient_ref_y);

        /* Set the LSB of reference y threshold */
        data_array[7] = BMA400_GET_MSB(orient_conf->orient_ref_y);

        /* Set the MSB of reference y threshold */
        data_array[8] = BMA400_GET_LSB(orient_conf->orient_ref_z);

        /* Set the LSB of reference z threshold */
        data_array[9] = BMA400_GET_MSB(orient_conf->orient_ref_z);

        /* Set the orient configurations in the sensor */
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+3, &data_array[3], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+4, &data_array[4], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+5, &data_array[5], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+6, &data_array[6], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+7, &data_array[7], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+8, &data_array[8], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+9, &data_array[9], 1);
    }
    else
    {
        /* Set the orient configurations in the sensor excluding
         * reference values of x,y,z
         */
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG, &data_array[0], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+1, &data_array[1], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+2, &data_array[2], 1);
        rslt = bma400_set_regs(BMA400_REG_ORIENTCH_INT_CONFIG+3, &data_array[3], 1);
    }

    return rslt;
}

static int8_t get_orient_int(struct bma400_orient_int_conf *orient_conf)
{
    int8_t rslt;
    uint8_t data_array[10];

    rslt = bma400_get_regs(BMA400_REG_ORIENTCH_INT_CONFIG, data_array, 10);
    if (rslt == BMA400_OK)
    {
        /* Get the axes to sense for interrupt */
        orient_conf->axes_sel = BMA400_GET_BITS(data_array[0], BMA400_INT_AXES_EN);

        /* Get the data source for interrupt */
        orient_conf->data_src = BMA400_GET_BITS(data_array[0], BMA400_INT_DATA_SRC);

        /* Get the reference update mode */
        orient_conf->ref_update = BMA400_GET_BITS(data_array[0], BMA400_INT_REFU);

        /* Get the threshold for interrupt calculation */
        orient_conf->orient_thres = data_array[1];

        /* Get the stability threshold */
        orient_conf->stability_thres = data_array[2];

        /* Get the interrupt duration */
        orient_conf->orient_int_dur = data_array[3];

        /* Get the interrupt reference values */
        data_array[5] = data_array[5] & 0x0F;
        orient_conf->orient_ref_x = ((uint16_t)data_array[5] << 8) | data_array[4];
        data_array[5] = data_array[7] & 0x0F;
        orient_conf->orient_ref_y = ((uint16_t)data_array[7] << 8) | data_array[6];
        data_array[5] = data_array[9] & 0x0F;
        orient_conf->orient_ref_z = ((uint16_t)data_array[9] << 8) | data_array[8];
    }

    return rslt;
}

static void map_int_pin(uint8_t *data_array, uint8_t int_enable, enum bma400_int_chan int_map)
{
    switch (int_enable)
    {
        case BMA400_DATA_READY_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1*/
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_DRDY, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2*/
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_DRDY, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_DRDY);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_DRDY);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_DRDY, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_DRDY, BMA400_ENABLE);
            }

            break;
        case BMA400_FIFO_WM_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1*/
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_FIFO_WM, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2*/
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_FIFO_WM, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_FIFO_WM);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_FIFO_WM);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_FIFO_WM, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_FIFO_WM, BMA400_ENABLE);
            }

            break;
        case BMA400_FIFO_FULL_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_FIFO_FULL, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_FIFO_FULL, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_FIFO_FULL);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_FIFO_FULL);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_FIFO_FULL, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_FIFO_FULL, BMA400_ENABLE);
            }

            break;
        case BMA400_INT_OVERRUN_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_INT_OVERRUN, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_INT_OVERRUN, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_INT_OVERRUN);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_INT_OVERRUN);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_INT_OVERRUN, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_INT_OVERRUN, BMA400_ENABLE);
            }

            break;
        case BMA400_GEN2_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_GEN2, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_GEN2, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_GEN2);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_GEN2);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_GEN2, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_GEN2, BMA400_ENABLE);
            }

            break;
        case BMA400_GEN1_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_GEN1, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_GEN1, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_GEN1);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_GEN1);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_GEN1, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_GEN1, BMA400_ENABLE);
            }

            break;
        case BMA400_ORIENT_CH_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_ORIENT_CH, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_ORIENT_CH, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_ORIENT_CH);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_ORIENT_CH);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS(data_array[0], BMA400_EN_ORIENT_CH, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS(data_array[1], BMA400_EN_ORIENT_CH, BMA400_ENABLE);
            }

            break;
        case BMA400_WAKEUP_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[0] = BMA400_SET_BITS_POS_0(data_array[0], BMA400_EN_WAKEUP_INT, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_EN_WAKEUP_INT, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[0] = BMA400_SET_BIT_VAL_0(data_array[0], BMA400_EN_WAKEUP_INT);
                data_array[1] = BMA400_SET_BIT_VAL_0(data_array[1], BMA400_EN_WAKEUP_INT);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[0] = BMA400_SET_BITS_POS_0(data_array[0], BMA400_EN_WAKEUP_INT, BMA400_ENABLE);
                data_array[1] = BMA400_SET_BITS_POS_0(data_array[1], BMA400_EN_WAKEUP_INT, BMA400_ENABLE);
            }

            break;
        case BMA400_ACT_CH_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_ACTCH_MAP_INT1, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_ACTCH_MAP_INT2, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_ACTCH_MAP_INT1);
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_ACTCH_MAP_INT2);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_ACTCH_MAP_INT1, BMA400_ENABLE);
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_ACTCH_MAP_INT2, BMA400_ENABLE);
            }

            break;
        case BMA400_TAP_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_TAP_MAP_INT1, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_TAP_MAP_INT2, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_TAP_MAP_INT1);
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_TAP_MAP_INT2);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_TAP_MAP_INT1, BMA400_ENABLE);
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_TAP_MAP_INT2, BMA400_ENABLE);
            }

            break;
        case BMA400_STEP_INT_MAP:
            if (int_map == BMA400_INT_CHANNEL_1)
            {
                /* Mapping interrupt to INT pin 1 */
                data_array[2] = BMA400_SET_BITS_POS_0(data_array[2], BMA400_EN_STEP_INT, BMA400_ENABLE);
            }

            if (int_map == BMA400_INT_CHANNEL_2)
            {
                /* Mapping interrupt to INT pin 2 */
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_STEP_MAP_INT2, BMA400_ENABLE);
            }

            if (int_map == BMA400_UNMAP_INT_PIN)
            {
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_EN_STEP_INT);
                data_array[2] = BMA400_SET_BIT_VAL_0(data_array[2], BMA400_STEP_MAP_INT2);
            }

            if (int_map == BMA400_MAP_BOTH_INT_PINS)
            {
                data_array[2] = BMA400_SET_BITS_POS_0(data_array[2], BMA400_EN_STEP_INT, BMA400_ENABLE);
                data_array[2] = BMA400_SET_BITS(data_array[2], BMA400_STEP_MAP_INT2, BMA400_ENABLE);
            }

            break;
        default:
            break;
    }
}

static void check_mapped_interrupts(uint8_t int_1_map, uint8_t int_2_map, enum bma400_int_chan *int_map)
{
    if ((int_1_map == BMA400_ENABLE) && (int_2_map == BMA400_DISABLE))
    {
        /* INT 1 mapped INT 2 not mapped */
        *int_map = BMA400_INT_CHANNEL_1;
        //printf("/* INT 1 mapped INT 2 not mapped */\n");
    }

    if ((int_1_map == BMA400_DISABLE) && (int_2_map == BMA400_ENABLE))
    {
        /* INT 1 not mapped INT 2 mapped */
        *int_map = BMA400_INT_CHANNEL_2;
        //printf("/* INT 1 not mapped INT 2 mapped */\n");
    }

    if ((int_1_map == BMA400_ENABLE) && (int_2_map == BMA400_ENABLE))
    {
        /* INT 1 ,INT 2 both mapped */
        *int_map = BMA400_MAP_BOTH_INT_PINS;
        //printf("/* INT 1 mapped INT 2 both mapped */\n");
    }

    if ((int_1_map == BMA400_DISABLE) && (int_2_map == BMA400_DISABLE))
    {
        /* INT 1 ,INT 2 not mapped */
        *int_map = BMA400_UNMAP_INT_PIN;
        //printf("/* INT 1 mapped INT 2 both not mapped */\n");
    }
}

static void get_int_pin_map(const uint8_t *data_array, uint8_t int_enable, enum bma400_int_chan *int_map)
{
    uint8_t int_1_map;
    uint8_t int_2_map;

    switch (int_enable)
    {
        case BMA400_DATA_READY_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_DRDY);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_DRDY);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_FIFO_WM_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_FIFO_WM);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_FIFO_WM);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_FIFO_FULL_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_FIFO_FULL);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_FIFO_FULL);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_GEN2_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_GEN2);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_GEN2);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_GEN1_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_GEN1);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_GEN1);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_ORIENT_CH_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_ORIENT_CH);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_ORIENT_CH);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_WAKEUP_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS_POS_0(data_array[0], BMA400_EN_WAKEUP_INT);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS_POS_0(data_array[1], BMA400_EN_WAKEUP_INT);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_ACT_CH_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[2], BMA400_ACTCH_MAP_INT1);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[2], BMA400_ACTCH_MAP_INT2);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_TAP_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[2], BMA400_TAP_MAP_INT1);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[2], BMA400_TAP_MAP_INT2);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);

            //printf("\nTap interrupt enabled\n");
            break;
        case BMA400_STEP_INT_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS_POS_0(data_array[2], BMA400_EN_STEP_INT);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[2], BMA400_STEP_MAP_INT2);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        case BMA400_INT_OVERRUN_MAP:

            /* Interrupt 1 pin mapping status */
            int_1_map = BMA400_GET_BITS(data_array[0], BMA400_EN_INT_OVERRUN);

            /* Interrupt 2 pin mapping status */
            int_2_map = BMA400_GET_BITS(data_array[1], BMA400_EN_INT_OVERRUN);

            /* Check the mapped interrupt pins */
            check_mapped_interrupts(int_1_map, int_2_map, int_map);
            break;
        default:
            break;
    }
}

static int8_t set_int_pin_conf(struct bma400_int_pin_conf int_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bma400_get_regs(BMA400_REG_INT_12_IO_CTRL, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        if (int_conf.int_chan == BMA400_INT_CHANNEL_1)
        {
            /* Setting interrupt pin configurations */
            reg_data = BMA400_SET_BITS(reg_data, BMA400_INT_PIN1_CONF, int_conf.pin_conf);
        }

        if (int_conf.int_chan == BMA400_INT_CHANNEL_2)
        {
            /* Setting interrupt pin configurations */
            reg_data = BMA400_SET_BITS(reg_data, BMA400_INT_PIN2_CONF, int_conf.pin_conf);
        }

        /* Set the configurations in the sensor */
        rslt = bma400_set_regs(BMA400_REG_INT_12_IO_CTRL, &reg_data, 1);
    }

    return rslt;
}

static int8_t get_int_pin_conf(struct bma400_int_pin_conf *int_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;

    rslt = bma400_get_regs(BMA400_REG_INT_12_IO_CTRL, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        if (int_conf->int_chan == BMA400_INT_CHANNEL_1)
        {
            /* reading Interrupt pin configurations */
            int_conf->pin_conf = BMA400_GET_BITS(reg_data, BMA400_INT_PIN1_CONF);
        }

        if (int_conf->int_chan == BMA400_INT_CHANNEL_2)
        {
            /* Setting interrupt pin configurations */
            int_conf->pin_conf = BMA400_GET_BITS(reg_data, BMA400_INT_PIN2_CONF);
        }
    }

    return rslt;
}

static int8_t get_fifo_conf(struct bma400_fifo_conf *fifo_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[3];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMA400_OK)
    {
        /* Get the FIFO configurations and water-mark
         * values from the sensor
         */
        rslt = bma400_get_regs(BMA400_REG_FIFO_CONFIG_0, data_array, 3);
        if (rslt == BMA400_OK)
        {
            /* Get the data of FIFO_CONFIG0 register */
            fifo_conf->conf_regs = data_array[0];

            /* Get the MSB of FIFO water-mark  */
            data_array[2] = BMA400_GET_BITS_POS_0(data_array[2], BMA400_FIFO_BYTES_CNT);

            /* FIFO water-mark value is stored */
            fifo_conf->fifo_watermark = ((uint16_t)data_array[2] << 8) | ((uint16_t)data_array[1]);
        }
    }

    return rslt;
}

static int8_t set_fifo_conf(const struct bma400_fifo_conf *fifo_conf, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[3];
    uint8_t sens_data[3];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == BMA400_OK)
    {
        /* Get the FIFO configurations and water-mark
         * values from the sensor
         */
        rslt = bma400_get_regs(BMA400_REG_FIFO_CONFIG_0, sens_data, 3);
        if (rslt == BMA400_OK)
        {
            /* FIFO configurations */
            data_array[0] = fifo_conf->conf_regs;
            if (fifo_conf->conf_status == BMA400_DISABLE)
            {
                /* Disable the selected interrupt status */
                data_array[0] = sens_data[0] & (~data_array[0]);
            }

            /* FIFO water-mark values */
            data_array[1] = BMA400_GET_LSB(fifo_conf->fifo_watermark);
            data_array[2] = BMA400_GET_MSB(fifo_conf->fifo_watermark);
            data_array[2] = BMA400_GET_BITS_POS_0(data_array[2], BMA400_FIFO_BYTES_CNT);
            if ((data_array[1] == sens_data[1]) && (data_array[2] == sens_data[2]))
            {
                /* Set the FIFO configurations in the
                 * sensor excluding the watermark value
                 */
                rslt = bma400_set_regs(BMA400_REG_FIFO_CONFIG_0, data_array, 1);
            }
            else
            {
                /* Set the FIFO configurations in the sensor*/
                rslt = bma400_set_regs(BMA400_REG_FIFO_CONFIG_0, data_array, 3);
            }
        }
    }

    return rslt;
}

static int8_t get_fifo_length(uint16_t *fifo_byte_cnt, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[2] = { 0 };

    rslt = bma400_get_regs(BMA400_REG_FIFO_LENGTH, data_array, 2);
    if (rslt == BMA400_OK)
    {
        data_array[1] = BMA400_GET_BITS_POS_0(data_array[1], BMA400_FIFO_BYTES_CNT);

        /* Available data in FIFO is stored in fifo_byte_cnt*/
        *fifo_byte_cnt = ((uint16_t)data_array[1] << 8) | ((uint16_t)data_array[0]);
    }

    return rslt;
}

static int8_t read_fifo(struct bma400_fifo_data *fifo, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data;
    uint8_t fifo_addr = BMA400_REG_FIFO_DATA;

    if (dev->intf == BMA400_SPI_INTF)
    {
        /* SPI mask is added */
        fifo_addr = fifo_addr | BMA400_SPI_RD_MASK;
    }

    /* This update will take care of dummy byte necessity based on interface selection */
    fifo->length += dev->dummy_byte;

    /* Read the FIFO enable bit */
    rslt = bma400_get_regs(BMA400_REG_FIFO_READ_EN, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        /* FIFO read disable bit */
        if (reg_data == 0)
        {
            /* Read FIFO Buffer since FIFO read is enabled */
            dev->intf_rslt = dev->read(fifo_addr, fifo->data, (uint32_t)fifo->length, dev->intf_ptr);
            if (dev->intf_rslt != BMA400_INTF_RET_SUCCESS)
            {
                rslt = BMA400_E_COM_FAIL;
            }
        }
        else
        {
            /* Enable FIFO reading */
            reg_data = 0;
            rslt = bma400_set_regs(BMA400_REG_FIFO_READ_EN, &reg_data, 1);
            if (rslt == BMA400_OK)
            {
                /* Delay to enable the FIFO */
                dev->delay_us(1000, dev->intf_ptr);

                /* Read FIFO Buffer since FIFO read is enabled*/
                dev->intf_rslt = dev->read(fifo_addr, fifo->data, (uint32_t)fifo->length, dev->intf_ptr);

                if (dev->intf_rslt == BMA400_OK)
                {
                    /* Disable FIFO reading */
                    reg_data = 1;
                    rslt = bma400_set_regs(BMA400_REG_FIFO_READ_EN, &reg_data, 1);
                }
            }
        }
    }

    return rslt;
}

static void unpack_accel_frame(struct bma400_fifo_data *fifo,
                               struct bma400_sensor_data *accel_data,
                               uint16_t *frame_count,
                               const struct bma400_dev *dev)
{
    /* Frame header information is stored */
    uint8_t frame_header = 0;

    /* Accel data width is stored */
    uint8_t accel_width;

    /* Data index of the parsed byte from FIFO */
    uint16_t data_index;

    /* Number of accel frames parsed */
    uint16_t accel_index = 0;

    /* Variable to check frame availability */
    uint8_t frame_available = BMA400_ENABLE;

    /* Check if this is the first iteration of data unpacking
     * if yes, then consider dummy byte on SPI
     */
    if (fifo->accel_byte_start_idx == 0)
    {
        /* Dummy byte included */
        fifo->accel_byte_start_idx = dev->dummy_byte;
    }

    for (data_index = fifo->accel_byte_start_idx; data_index < fifo->length;)
    {
        /*Header byte is stored in the variable frame_header*/
        frame_header = fifo->data[data_index];

        /* Store the Accel 8 bit or 12 bit mode */
        accel_width = BMA400_GET_BITS(frame_header, BMA400_FIFO_8_BIT_EN);

        /* Exclude the 8/12 bit mode data from frame header */
        frame_header = frame_header & BMA400_AWIDTH_MASK;

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            case BMA400_FIFO_XYZ_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_XYZ_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel xyz data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_X_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_X_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel x data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_Y_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_Y_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel y data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_Z_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_Z_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel z data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_XY_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_XY_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel xy data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_YZ_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_YZ_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel yz data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_XZ_ENABLE:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_YZ_ENABLE, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Extract and store accel xz data */
                    unpack_accel(fifo, &accel_data[accel_index], &data_index, accel_width, frame_header);
                    accel_index++;
                }

                break;
            case BMA400_FIFO_SENSOR_TIME:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_SENSOR_TIME, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Unpack and store the sensor time data */
                    unpack_sensortime_frame(fifo, &data_index);
                }

                break;
            case BMA400_FIFO_EMPTY_FRAME:

                /* Update the data index as complete */
                data_index = fifo->length;
                break;
            case BMA400_FIFO_CONTROL_FRAME:
                check_frame_available(fifo, &frame_available, accel_width, BMA400_FIFO_CONTROL_FRAME, &data_index);
                if (frame_available != BMA400_DISABLE)
                {
                    /* Store the configuration change data from FIFO */
                    fifo->conf_change = fifo->data[data_index++];
                }

                break;
            default:

                /* Update the data index as complete */
                data_index = fifo->length;
                break;
        }
        if (*frame_count == accel_index)
        {
            /* Frames read completely*/
            break;
        }
    }

    /* Update the data index */
    fifo->accel_byte_start_idx = data_index;

    /* Update number of accel frame index */
    *frame_count = accel_index;
}

static void check_frame_available(const struct bma400_fifo_data *fifo,
                                  uint8_t *frame_available,
                                  uint8_t accel_width,
                                  uint8_t data_en,
                                  uint16_t *data_index)
{
    switch (data_en)
    {
        case BMA400_FIFO_XYZ_ENABLE:

            /* Handling case of 12 bit/ 8 bit data available in FIFO */
            if (accel_width == BMA400_12_BIT_FIFO_DATA)
            {
                if ((*data_index + 6) > fifo->length)
                {
                    /* Partial frame available */
                    *data_index = fifo->length;
                    *frame_available = BMA400_DISABLE;
                }
            }
            else if ((*data_index + 3) > fifo->length)
            {
                /* Partial frame available */
                *data_index = fifo->length;
                *frame_available = BMA400_DISABLE;
            }

            break;
        case BMA400_FIFO_X_ENABLE:
        case BMA400_FIFO_Y_ENABLE:
        case BMA400_FIFO_Z_ENABLE:

            /* Handling case of 12 bit/ 8 bit data available in FIFO */
            if (accel_width == BMA400_12_BIT_FIFO_DATA)
            {
                if ((*data_index + 2) > fifo->length)
                {
                    /* Partial frame available */
                    *data_index = fifo->length;
                    *frame_available = BMA400_DISABLE;
                }
            }
            else if ((*data_index + 1) > fifo->length)
            {
                /* Partial frame available */
                *data_index = fifo->length;
                *frame_available = BMA400_DISABLE;
            }

            break;
        case BMA400_FIFO_XY_ENABLE:
        case BMA400_FIFO_YZ_ENABLE:
        case BMA400_FIFO_XZ_ENABLE:

            /* Handling case of 12 bit/ 8 bit data available in FIFO */
            if (accel_width == BMA400_12_BIT_FIFO_DATA)
            {
                if ((*data_index + 4) > fifo->length)
                {
                    /* Partial frame available */
                    *data_index = fifo->length;
                    *frame_available = BMA400_DISABLE;
                }
            }
            else if ((*data_index + 2) > fifo->length)
            {
                /* Partial frame available */
                *data_index = fifo->length;
                *frame_available = BMA400_DISABLE;
            }

            break;
        case BMA400_FIFO_SENSOR_TIME:
            if ((*data_index + 3) > fifo->length)
            {
                /* Partial frame available */
                *data_index = fifo->length;
                *frame_available = BMA400_DISABLE;
            }

            break;
        case BMA400_FIFO_CONTROL_FRAME:
            if ((*data_index + 1) > fifo->length)
            {
                /* Partial frame available */
                *data_index = fifo->length;
                *frame_available = BMA400_DISABLE;
            }

            break;
        default:
            break;
    }
}

static int8_t enable_self_test(struct bma400_dev *dev)
{
    int8_t rslt;

    /* Accelerometer setting structure */
    struct bma400_sensor_conf accel_setting;

    /* Select the type of configuration to be modified */
    accel_setting.type = BMA400_ACCEL;

    /* Get the accel configurations which are set in the sensor */
    rslt = bma400_get_sensor_conf(&accel_setting, 1, dev);
    if (rslt == BMA400_OK)
    {
        /* Modify to the desired configurations */
        accel_setting.param.accel.odr = BMA400_ODR_100HZ;

        accel_setting.param.accel.range = BMA400_RANGE_4G;
        accel_setting.param.accel.osr = BMA400_ACCEL_OSR_SETTING_3;
        accel_setting.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

        /* Set the desired configurations in the sensor */
        rslt = bma400_set_sensor_conf(&accel_setting, 1);
        if (rslt == BMA400_OK)
        {
            /* self test enabling delay */
            //dev->delay_us(BMA400_DELAY_US_SELF_TEST, dev->intf_ptr);
            nrf_delay_ms(70);
        }

        if (rslt == BMA400_OK)
        {
            rslt = bma400_set_power_mode(BMA400_MODE_NORMAL);
        }
    }

    return rslt;
}

int8_t bma400_perform_self_test(struct bma400_dev *dev)
{
    int8_t rslt;
    int8_t self_test_rslt = 0;
    struct bma400_sensor_data accel_pos, accel_neg;
    {
        /* pre-requisites for self test*/
        rslt = enable_self_test(dev);
        nrf_delay_ms(1000);
        if (rslt == BMA400_OK)
        {
            rslt = positive_excited_accel(&accel_pos, dev);
            nrf_delay_ms(1000);
            if (rslt == BMA400_OK)
            {
                rslt = negative_excited_accel(&accel_neg, dev);
                nrf_delay_ms(1000);
                if (rslt == BMA400_OK)
                {
                    /* Validate the self test result */
                    rslt = validate_accel_self_test(&accel_pos, &accel_neg);
                }
            }
        }
    }

    ///* Check to ensure bus error does not occur */
    //if (rslt <= BMA400_OK)
    //{
    //    /* Store the status of self test result */
    //    self_test_rslt = rslt;

    //    /* Perform soft reset */
    //    rslt = bma400_soft_reset(dev);
    //}

    ///* Check to ensure bus operations are success */
    //if (rslt == BMA400_OK)
    //{
    //    /* Restore self_test_rslt as return value */
    //    rslt = self_test_rslt;
    //}

    return rslt;
}

static void unpack_accel(const struct bma400_fifo_data *fifo,
                         struct bma400_sensor_data *accel_data,
                         uint16_t *data_index,
                         uint8_t accel_width,
                         uint8_t frame_header)
{
    uint8_t data_lsb;
    uint8_t data_msb;

    /* Header information of enabled axes */
    frame_header = frame_header & BMA400_FIFO_DATA_EN_MASK;
    if (accel_width == BMA400_12_BIT_FIFO_DATA)
    {
        if (frame_header & BMA400_FIFO_X_ENABLE)
        {
            /* Accel x data */
            data_lsb = fifo->data[(*data_index)++];
            data_msb = fifo->data[(*data_index)++];
            accel_data->x = (int16_t)(((uint16_t)(data_msb << 4)) | data_lsb);
            if (accel_data->x > 2047)
            {
                /* Computing accel x data negative value */
                accel_data->x = accel_data->x - 4096;
            }
        }
        else
        {
            /* Accel x not available */
            accel_data->x = 0;
        }

        if (frame_header & BMA400_FIFO_Y_ENABLE)
        {
            /* Accel y data */
            data_lsb = fifo->data[(*data_index)++];
            data_msb = fifo->data[(*data_index)++];
            accel_data->y = (int16_t)(((uint16_t)(data_msb << 4)) | data_lsb);
            if (accel_data->y > 2047)
            {
                /* Computing accel y data negative value */
                accel_data->y = accel_data->y - 4096;
            }
        }
        else
        {
            /* Accel y not available */
            accel_data->y = 0;
        }

        if (frame_header & BMA400_FIFO_Z_ENABLE)
        {
            /* Accel z data */
            data_lsb = fifo->data[(*data_index)++];
            data_msb = fifo->data[(*data_index)++];
            accel_data->z = (int16_t)(((uint16_t)(data_msb << 4)) | data_lsb);
            if (accel_data->z > 2047)
            {
                /* Computing accel z data negative value */
                accel_data->z = accel_data->z - 4096;
            }
        }
        else
        {
            /* Accel z not available */
            accel_data->z = 0;
        }
    }
    else
    {
        if (frame_header & BMA400_FIFO_X_ENABLE)
        {
            /* Accel x data */
            data_msb = fifo->data[(*data_index)++];
            accel_data->x = (int16_t)((uint16_t)(data_msb << 4));
            if (accel_data->x > 2047)
            {
                /* Computing accel x data negative value */
                accel_data->x = accel_data->x - 4096;
            }
        }
        else
        {
            /* Accel x not available */
            accel_data->x = 0;
        }

        if (frame_header & BMA400_FIFO_Y_ENABLE)
        {
            /* Accel y data */
            data_msb = fifo->data[(*data_index)++];
            accel_data->y = (int16_t)((uint16_t)(data_msb << 4));
            if (accel_data->y > 2047)
            {
                /* Computing accel y data negative value */
                accel_data->y = accel_data->y - 4096;
            }
        }
        else
        {
            /* Accel y not available */
            accel_data->y = 0;
        }

        if (frame_header & BMA400_FIFO_Z_ENABLE)
        {
            /* Accel z data */
            data_msb = fifo->data[(*data_index)++];
            accel_data->z = (int16_t)((uint16_t)(data_msb << 4));
            if (accel_data->z > 2047)
            {
                /* Computing accel z data negative value */
                accel_data->z = accel_data->z - 4096;
            }
        }
        else
        {
            /* Accel z not available */
            accel_data->z = 0;
        }
    }
}

static void unpack_sensortime_frame(struct bma400_fifo_data *fifo, uint16_t *data_index)
{
    uint32_t time_msb;
    uint16_t time_lsb;
    uint8_t time_xlsb;

    time_msb = fifo->data[(*data_index) + 2] << 16;
    time_lsb = fifo->data[(*data_index) + 1] << 8;
    time_xlsb = fifo->data[(*data_index)];

    /* Sensor time */
    fifo->fifo_sensor_time = (uint32_t)(time_msb | time_lsb | time_xlsb);
    *data_index = (*data_index) + 3;
}

static int8_t validate_accel_self_test(const struct bma400_sensor_data *accel_pos,
                                       const struct bma400_sensor_data *accel_neg)
{

    int8_t rslt;

    /* Structure for difference of accel values */
    struct bma400_selftest_delta_limit accel_data_diff = { 0, 0, 0 };

    /* accel x difference value */
    accel_data_diff.x = (accel_pos->x - accel_neg->x);

    /* accel y difference value */
    accel_data_diff.y = (accel_pos->y - accel_neg->y);

    /* accel z difference value */
    accel_data_diff.z = (accel_pos->z - accel_neg->z);

    /* Validate the results of self test */
    if (((accel_data_diff.x) > BMA400_ST_ACC_X_AXIS_SIGNAL_DIFF) &&
        ((accel_data_diff.y) > BMA400_ST_ACC_Y_AXIS_SIGNAL_DIFF) &&
        ((accel_data_diff.z) > BMA400_ST_ACC_Z_AXIS_SIGNAL_DIFF))
    {
        /* Self test pass condition */
        rslt = BMA400_OK;
    }
    else
    {
        /* Self test failed */
        rslt = BMA400_W_SELF_TEST_FAIL;
    }

    return rslt;
}

static int8_t positive_excited_accel(struct bma400_sensor_data *accel_pos, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMA400_SELF_TEST_ENABLE_POSITIVE;

    /* Enable positive excitation for all 3 axes */
    rslt = bma400_set_regs(BMA400_REG_SELF_TEST, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        /* Read accel data after 50ms delay */
        nrf_delay_ms(50);

        rslt = bma400_get_accel_data(BMA400_DATA_ONLY, accel_pos, dev);
    }

    return rslt;
}

static int8_t negative_excited_accel(struct bma400_sensor_data *accel_neg, struct bma400_dev *dev)
{
    int8_t rslt;
    uint8_t reg_data = BMA400_SELF_TEST_ENABLE_NEGATIVE;

    /* Enable negative excitation for all 3 axes */
    rslt = bma400_set_regs(BMA400_REG_SELF_TEST, &reg_data, 1);
    if (rslt == BMA400_OK)
    {
        /* Read accel data after 50ms delay */
        nrf_delay_ms(50);
        rslt = bma400_get_accel_data(BMA400_DATA_ONLY, accel_neg, dev);
        if (rslt == BMA400_OK)
        {
            /* Disable self test */
            reg_data = BMA400_SELF_TEST_DISABLE;
            rslt = bma400_set_regs(BMA400_REG_SELF_TEST, &reg_data, 1);
        }
    }

    return rslt;
}

/**
 * @brief:   Function to put accelerometer in low power mode
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void bma400_low_power(void)
{
    struct bma400_dev bma;
    int8_t rslt;
    rslt = bma400_set_power_mode(BMA400_MODE_LOW_POWER);
    printf("BMA400 LOW POWER MODE %d", rslt);
}

/**
 * @brief:   Function to put accelerometer in sleep modes
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void bma400_sleep(void)
{
    struct bma400_dev bma;
    int8_t rslt;
    rslt = bma400_set_power_mode(BMA400_MODE_SLEEP);
    printf("BMA400 SLEEP MODE %d", rslt);
}

/**
 * @brief:   Function to start accelerometer with step and fall detect config
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void bma400_start(void)
{
    int8_t rslt;
    struct bma400_sensor_conf accel_settin[3] = { { 0 } };
    struct bma400_int_enable int_en[3];
    struct bma400_dev bma;
    //uint8_t step_param[24]={1,45,123,212,68,1,59,122,219,123,63,108,205,39,25,150,160,195,14,12,60,240,0,247};//Wrist Calibration
    uint8_t step_param[24]={1,50,120,230,135,0,132,108,156,117,100,126,170,12,12,74,160,0,0,12,60,240,1,0};//Chest Calibration
    rslt = bma400_soft_reset();
    bma400_init();
    nrf_delay_ms(100);

    rslt = bma400_get_sensor_conf(accel_settin, 2, &bma);

    accel_settin[0].type = BMA400_STEP_COUNTER_INT;
    accel_settin[0].param.step_cnt.int_chan = BMA400_INT_CHANNEL_2;

    accel_settin[1].type = BMA400_ACCEL;
    accel_settin[1].param.accel.odr = BMA400_ODR_200HZ;
    accel_settin[1].param.accel.range = BMA400_RANGE_2G;
    accel_settin[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    //accel_settin[2].type = BMA400_GEN1_INT;
    //accel_settin[2].param.gen_int.int_chan = BMA400_INT_CHANNEL_1;
    //accel_settin[2].param.gen_int.gen_int_dur = 0x01 ;
    //accel_settin[2].param.gen_int.gen_int_thres = 0x3F;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(accel_settin, 2);

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL);

    int_en[0].type = BMA400_STEP_COUNTER_INT_EN;
    int_en[0].conf = BMA400_ENABLE;

    //int_en[1].type = BMA400_LATCH_INT_EN;
    //int_en[1].conf = BMA400_ENABLE;

    int_en[2].type = BMA400_GEN1_INT_EN;
    int_en[2].conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(int_en, 2, &bma);

    rslt = bma400_set_step_counter_param(step_param);

    rslt = EnableDetectionOfFreeFall();

}

/**
 * @brief:   Function to reset step counts of accelerometer
 *
 * @param[in] void
 *
 * @param[out] int8_t    
 */
int8_t reset_step_count(void)
{
    int8_t rslt;
    uint8_t data = BMA400_STEP_RESET_CMD;

    /* Reset step count of device */
    rslt = bma400_set_regs(BMA400_REG_COMMAND, &data, 1);
    nrf_delay_ms(20);

    return rslt;
}