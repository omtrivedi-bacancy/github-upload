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
 * \file     accelerometer.h
 * \brief   Define
 */
#ifndef ACCELEROMETER_H__
#define ACCELEROMETER_H__
/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 *                                REGISTER MAP                                 *
 ******************************************************************************/


/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/


/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/


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
 * @brief:   Function to Calculate Step count
 *
 * @param[in] void
 *
 * @param[out] void    
 */


#endif //ACCELEROMETER_H__