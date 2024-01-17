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
 * \file     accelerometer.c
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "accelerometer.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
static volatile bool m_xfer_done = false;
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/******************************************************************************
 *                                  LOCALS                                 *
 ******************************************************************************/

    
/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/


/**
 * @brief:  A Function to verify the product id
  (its a basic test to check if we are communicating with the right slave, every type of I2C Device has 
  a special WHO_AM_I register which holds a specific value, we can read it from the MPU6050 or any device
  to confirm we are communicating with the right device)
 *
 * @param[in] void
 *
 * @param[out] bool     return NRF_SUCCESS or NRF_FAIL
 */


/**
 * @brief:  Function to initialize the mpu6050
 *
 * @param[in] void
 *
 * @param[out] bool     return NRF_SUCCESS or NRF_FAIL
 */


/**
 * @brief:   A function to write a Single Byte to MPU6050's internal Register
 *
 * @param[in] uint8_t register_address, uint8_t value
 *
 * @param[out] bool     return NRF_SUCCESS or NRF_FAIL
 */



/**
 * @brief:   A Function to read data from the MPU6050
 *
 * @param[in] uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes
 *
 * @param[out] bool     return NRF_SUCCESS or NRF_FAIL
 */
 


/**
 * @brief:  Read the Gyro values from the MPU6050's internal Registers
 *
 * @param[in] int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z 
 *
 * @param[out] void  
 */


 

/**
 * @brief:  A Function to read accelerometer's values from the internal registers of MPU6050
 *
 * @param[in] int16_t *pACC_X , int16_t *pACC_Y , int16_t *pACC_Z 
 *
 * @param[out] void  
 */




/*
  END OF FILE
*/ 