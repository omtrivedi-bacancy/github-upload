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
 * \file     as7050.h
 */
#ifndef AS7050_H__
#define AS7050_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include "as7050_app_manager_typedefs.h"
#include "as7050_app_manager.h"
#include "as7050_chiplib.h"
#include "as7050_agc.h"
#include "export.h"


/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

extern uint8_t g_ready_for_execution;
extern export_data mixdata;
extern export_ecg ecgdata;
extern export_flash flashdata;
extern read_flash flashreaddata;
extern uint16_t ir_max;
extern uint16_t ir_min;
extern uint16_t red_max;
extern uint16_t red_min;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/**
 * @brief: Function to Initialize AS7050 
 *
 * @param[in] void
 *
 * @param[out] void 
 */
extern void as7050_init(void);

/**
 * @brief:  Define a callback function for the Chip Library. This function will be
            called by the Chip Library when new measurement data is available.
 *
 * @param[in] err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param
 *
 * @param[out] void 
 */
extern void as7050_callback(err_code_t error, uint8_t *p_data, uint16_t data_num, as7050_agc_status_t *p_agc_status,
                            void *p_cb_param);

/**
 * @brief:  Define a function to turn off internal analog LDO.
 *
 * @param[in] void
 *
 * @param[out] void 
 */
extern void internal_ldo_off(void);

/**
 * @brief:  Function to check AS7050 Biosensor
 *
 * @param[in] void
 *
 * @param[out] void 
 */
uint8_t as7050_check(void);

#endif //AS7050_H__