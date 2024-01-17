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
 * \file     common.h
 */
#ifndef COMMON_H__
#define COMMON_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include "error_codes.h"
#include "nrf_gpio.h"
#include "hardware.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#define FW_VERSION          "3.0.1"
                            /* ENABLE DEFINES */ 
#define AS7050_INIT         1
#define BATTERY_MONITOR     1
#define ACCEL_INIT          1
#define BLE_ENABLE          1
                            /* I2C DEFINES */ 
#define TWI_INSTANCE_ID     0
#define AS7050_SLAVE_ADD    0x55
#define I2C_MAX_RETRIES     5
#define I2C_SCL_PIN         14
#define I2C_SDA_PIN         15

                            /* SENSOR ENABLE GPIOS */
#define AS7050_ENABLE_PIN   NRF_GPIO_PIN_MAP(1,3)   /* Bio-sensor LDO Enable GPIO */
#define TEMP_EN             24   
#define ACCEL_EN            22   
#define FLASH_EN            NRF_GPIO_PIN_MAP(1,11)  
#define BAT_MON_ENABLE      NRF_GPIO_PIN_MAP(1,6)   /* Battery Monitoring Enable GPIO */

                            /* INTERRUPT GPIOS */
#define AS7050_INT_PIN      NRF_GPIO_PIN_MAP(1,1)   /* Bio-sensor Interrupt GPIO */
#define ACCEL_INT1          2                       /* Accelerometer Interrupt GPIO */
#define ACCEL_INT2          3                       /* Accelerometer Interrupt GPIO */


#define BAT_MON_ADC         28
#define BAT_STAT            30

                            /* FLASH GPIOS */
#define NRFX_QSPI_PIN_IO1   NRF_GPIO_PIN_MAP(1,8)
#define NRFX_QSPI_PIN_IO3   12
#define NRFX_QSPI_PIN_IO2   7
#define NRFX_QSPI_PIN_SCK   27
#define NRFX_QSPI_PIN_CSN   11
#define NRFX_QSPI_PIN_IO0   6
#define CS_FLASH            11


                            /* BLE_DATA_SIZE MACROS */
#define EXPORT_SIZE         22
                            /* DEFAULT VALUES */
#define MAX_SPO2_OUTPUT     9999
#define DEFAULT_MIN_INT     60000
#define DEFAULT_SEC_INT     20000

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
extern uint8_t sensor_mode;
extern uint8_t flash_write_enable;
extern uint8_t flash_data_exist;
extern uint32_t ecg_end_address;
extern uint32_t ecg_start_address;
extern uint32_t ppg_start_address;
extern uint8_t flash_data_collected;
extern uint32_t ppg_end_address;
extern int total_data_collected;
extern uint8_t as7050_power;
/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/
/**
 * @brief: Interrupt Handler for AS7050 sensor
 *
 * @param[in] void
 *
 * @param[out] err_code_t     return NRF_SUCCESS or NRF_FAIL
 */
err_code_t interrupt_handler(void);

/**
 * @brief:   Interrupt handler for Fall detection
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void fall_interrupt_handler(void);

/**
 * @brief:   Interrupt handler for Step detection
 *
 * @param[in] void
 *
 * @param[out] void    
 */
void step_interrupt_handler(void);


#endif  //COMMON_H__