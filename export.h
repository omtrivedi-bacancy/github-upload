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
 * \file     export.h
 */

#ifndef EXPORT_H__
#define EXPORT_H__
/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include "common.h"

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/
                       /* mixdata.ecgnppg_msg_id = 10
                          stepdata.step_msg_id = 11
                          tempdata.temp_msg_id = 12
                          spo2data.spo2_msg_id = 13
                          ecgdata.ecg_msg_id = 14
                          ppgdata.ppg_msg_id = 15
                          batdata.bat_msg_id = 16
                          falldata.fall_msg_id = 17
                          schedulerdata.sch_msg_id = 18 */
/**
 * Structure of BLE Data Packets
 */
typedef struct{
uint8_t ecgnppg_msg_id;
uint32_t ecg_vals[EXPORT_SIZE]; 
uint32_t ppg_vals[EXPORT_SIZE];
}PACKED export_data;

typedef struct {
uint8_t step_msg_id;
uint16_t step_vals;
uint8_t activity;
}PACKED export_step;

typedef struct {
uint8_t sch_msg_id;
uint8_t sch_val;
}PACKED export_scheduler;

typedef struct {
uint8_t fall_msg_id;
uint8_t fall_vals;
}PACKED export_fall;

typedef struct {
uint8_t bat_msg_id;
uint16_t bat_val;
}PACKED export_bat;

typedef struct {
uint8_t temp_msg_id;
uint16_t temp_vals;
}PACKED export_temp;

typedef struct {
uint8_t spo2_msg_id;
uint16_t spo2val;
uint16_t spo2_quality;
}PACKED export_spo2;

typedef struct {
uint8_t ecg_msg_id;
uint32_t ecg_vals[44]; 
}PACKED export_ecg;

typedef struct {
uint8_t ppg_msg_id;
uint32_t ppg_vals[EXPORT_SIZE]; 
}PACKED export_ppg;

/**
 * Structure of Flash Memory Data Packets
 */
typedef struct {
uint32_t ecg_vals[4000]; 
uint32_t ppg_vals[4000];
uint32_t spo2_vals;
}PACKED export_flash;     
        
typedef struct {
uint32_t ecg_vals[4000]; 
uint32_t ppg_vals[4000];
uint32_t spo2_vals;
}PACKED read_flash;  
#endif //EXPORT_H__