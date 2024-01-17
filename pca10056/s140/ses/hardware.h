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
 * \file     harware.h
 */
#ifndef HARDWARE_H__
#define HARDWARE_H__
/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#define QSPI_PAGE_OFFSET           0x100
#define QSPI_UINT32_T_PAGE_SIZE    64
#define QSPI_PAGE_FRAME_SIZE       259
#define ACCEL_SUCCESS              "ACCEL_SUCCESS"
#define AS_SUCCESS                 "AS_SUCCESS"
#define F_SUCCESS                  "F_SUCCESS"
#define TEMP_SUCCESS               "TEMP_SUCCESS"
#define ACCEL_ERROR                "ACCEL_ERROR"
#define AS_ERROR                   "AS_ERROR"
#define F_ERROR                    "F_ERROR"
#define TEMP_ERROR                 "TEMP_ERROR"
/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
extern uint32_t ecg_vals[22]; 
extern uint32_t ppg_vals[22]; 
/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

 /**
 * @brief: I2C Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void twi_init (void);

/**
 * @brief: I2C Write Function
 *
 * @param[in] unsigned char reg_addr, unsigned char data
 *
 * @param[out] void
 */
extern void write_reg(unsigned char reg_addr, unsigned char data);


/**
 * @brief: I2C read Function
 *
 * @param[in] uint8_t reg_address,uint8_t *data
 *
 * @param[out] bool
 */
extern bool read_reg_i2c(uint8_t reg_address,uint8_t *data);

/**
 * @brief: Hardware Peripherals Initialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void hardware_init();


/**
 * @brief: QSPI Communication check function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern uint8_t qspi_check(void);


/**
 * @brief: QSPI Erase function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void qspi_erase (void);

/**
 * @brief: QSPI Uninitialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void qspi_deinit(void);

/**
 * @brief: QSPI Initialization function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void qspi_init(void);

/**
 * @brief: I2C Uninitialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void twi_deinit(void);

/**
 * @brief: I2C Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */


/**
 * @brief: GPIO Uninitialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void gpio_deinit(void);

/**
 * @brief: GPIO Initialization Function
 *
 * @param[in] void
 *
 * @param[out] void
 */
extern void gpio_init(void);


/**
 * @brief: QSPI Page Write function
 *
 * @param[in] uint32_t* m_write_buffer,uint32_t const dst_address
 *
 * @param[out] void
 */
extern void qspi_page_write(uint32_t* m_write_buffer,uint32_t const dst_address);

/**
 * @brief: QSPI Page Read function
 *
 * @param[in]uint32_t const dst_address
 *
 * @param[out] void
 */
extern void qspi_page_read(uint32_t const dst_address,uint8_t data_flag);


/**
 * @brief: QSPI Status Register Read function
 *
 * @param[in]uint8_t SelectStatusRegister_1_2_3
 *
 * @param[out] uint8_t
 */
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusRegister_1_2_3);

/**
 * @brief: Function to turn off temp
 *
 * @param[in] void
 *
 * @param[out] void
 */
void temp_power_off(void);

/**
 * @brief: Function to turn on temp
 *
 * @param[in] void
 *
 * @param[out] void
 */
void temp_power_on(void);

/**
 * @brief: Function to turn on flash
 *
 * @param[in] void
 *
 * @param[out] void
 */
void flash_power_on(void);

/**
 * @brief: Function to turn off flash
 *
 * @param[in] void
 *
 * @param[out] void
 */
void flash_power_off(void);


/**
 * @brief: Function to turn off accelerometer
 *
 * @param[in] void
 *
 * @param[out] void
 */
void accel_power_off(void);

/**
 * @brief: Function to turn on accelerometer
 *
 * @param[in] void
 *
 * @param[out] void
 */
void accel_power_on(void);

/**
 * @brief: Function to turn off AS7050
 *
 * @param[in] void
 *
 * @param[out] void
 */
void as7050_power_off(void);

/**
 * @brief: Function to turn on AS7050
 *
 * @param[in] void
 *
 * @param[out] void
 */
void as7050_power_on(void);

/**
 * @brief: Function to disable battery monitoring
 *
 * @param[in] void
 *
 * @param[out] void
 */
void battery_mon_disable(void);


/**
 * @brief: Function to enable battery monitoring
 *
 * @param[in] void
 *
 * @param[out] void
 */
void battery_mon_en(void);

/**
 * @brief: Function to configure flash settings
 *
 * @param[in] void
 *
 * @param[out] void
 */
void configure_flash_settings(void);

/**
 * @brief: Function to write limit value through QSPI
 *
 * @param[in] uint32_t * m_write_buffer,uint32_t dst_address,uint16_t length
 *
 * @param[out] void
 */
void qspi_write(uint32_t * m_write_buffer,uint32_t dst_address,uint16_t length);

#endif //HARDWARE_H__