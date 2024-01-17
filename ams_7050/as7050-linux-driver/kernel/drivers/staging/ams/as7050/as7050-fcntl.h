/* 
 * AS7050 sensor driver
 * Copyright (c) 2019 ams AG. All rights reserved.
 *
 * Author: Sebastian Schwabe <sebastian.schwabe@ams.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __AS7050_FCNTL_H__
#define __AS7050_FCNTL_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <linux/ioctl.h>
#include <linux/types.h>

#include "as7050_typedefs.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/**
 * struct as7050_reg_group - Type stores information of a register group
 * @id: -  Identification number of an item, see ::as7050_reg_group_ids.
 * @data: - Pointer, where the data of the register group can be saved.
 * @size: - Size in byte of the register group data
 */
struct as7050_reg_group {
	uint32_t id; //define as uint32_t instead of being an enum to avoid variable-size enums
	uint8_t size;
	uint8_t *data;
};

/**
 * struct as7050_reg_pair - Type storing information about a register
 * @addr: - The address of the register 
 * @val: - The value of the register
 */
struct as7050_reg_pair {
	uint8_t addr;
	uint8_t val;
};

/**
 * struct as7050_version_info - The version information of the driver
 * @major: - the major part of the version
 * @minor: - the minor part of the version
 * @patch: - the patch level of the version
 */
struct as7050_version_info {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
};

/**
 * struct as7050_ref_val - Type storing information about a register
 * @control: -     control byte
 * @average_cnt: - Minimum number of measured samples per DAC
 * @status: -      result of calculation status
 * @dac_ref: -     result of DAC reference value
 */
struct as7050_dac_ref_val {
	dac_ref_control_t control;
	uint16_t average_cnt;
	dac_ref_status_t status;
	uint32_t dac_ref;
};

/**
 * enum as7050_ioctl_cmds - Lists all available sys calls of the driver
 * @INITIALIZE: - call chiplib function as7050_initialize
 * @SHUTDOWN: - call chiplib function as7050_shutdown
 * @SET_REG_GROUP: - call chiplib function as7050_set_reg_group
 * @GET_REG_GROUP: - call chiplib function as7050_get_reg_group
 * @SET_AGC_CONFIG: - call chiplib function as7050_set_agc_config
 * @GET_AGC_CONFIG: - call chiplib function as7050_get_agc_config
 * @WRITE_REGISTER: - call chiplib function as7050_write_register
 * @READ_REGISTER: - call chiplib function as7050_read_register
 * @GET_MEASUREMENT_CONFIG: - call chiplib function as7050_get_measurement_config
 * @START_MEASUREMENT: - call chiplib function as7050_start_measurement
 * @STOP_MEASUREMENT: - call chiplib function as7050_stop_measurement
 * @GET_VERSION: - read out actual version information of the driver
 * @GET_FIFO_SIZE: - return actual size of the kernel fifo in bytes
 * @GET_DAC_REF_VAL: - return chiplib function as7050_calculate_dac_reference_value
 */
enum as7050_ioctl_cmds {
	INITIALIZE,
	SHUTDOWN,
	SET_REG_GROUP,
	GET_REG_GROUP,
	SET_AGC_CONFIG,
	GET_AGC_CONFIG,
	WRITE_REGISTER,
	READ_REGISTER,
	GET_MEASUREMENT_CONFIG,
	START_MEASUREMENT,
	STOP_MEASUREMENT,
	GET_VERSION,
	GET_FIFO_SIZE,
	GET_DAC_REF_VAL
};

/**
 * AS7050_IOCTL_MAGIC - The magic number for all sys calls of the device
 */
#define AS7050_IOCTL_MAGIC '9'

/**
 * AS7050_INITIALIZE - Declaration of system call for calling chiplib initialize
 *
 * Parameters: none
 */
#define AS7050_INITIALIZE _IO(AS7050_IOCTL_MAGIC, INITIALIZE)

/**
 * AS7050_SHUTDOWN - Declaration of system call for calling chiplib shutdown
 *
 * Parameters: none
 */
#define AS7050_SHUTDOWN _IO(AS7050_IOCTL_MAGIC, SHUTDOWN)

/**
 * AS7050_SET_REG_GROUP - Declaration of system call for calling chiplib set_reg_group
 * 
 * Parameters: Pointer to struct as7050_reg_group holding information to set a register group
 */
#define AS7050_SET_REG_GROUP                                                   \
	_IOW(AS7050_IOCTL_MAGIC, SET_REG_GROUP, struct as7050_reg_group)

/**
 * AS7050_GET_REG_GROUP - Declaration of system call for calling chiplib get_reg_group
 * 
 * Parameters: Pointer to struct as7050_reg_group which can store information of a register group
 */
#define AS7050_GET_REG_GROUP                                                   \
	_IOR(AS7050_IOCTL_MAGIC, GET_REG_GROUP, struct as7050_reg_group)

/**
 * AS7050_SET_AGC_CONFIG - Declaration of system call for calling chiplib set_agc_config
 * 
 * Parameters: Pointer to struct as7050_agc_config holding information to set agc configuration
 */
#define AS7050_SET_AGC_CONFIG                                                  \
	_IOW(AS7050_IOCTL_MAGIC, SET_AGC_CONFIG, as7050_agc_config_t)

/**
 * AS7050_GET_AGC_CONFIG - Declaration of system call for calling chiplib get_agc_config
 * 
 * Parameters: Pointer to struct as7050_agc_config which can store information of agc configuration
 */
#define AS7050_GET_AGC_CONFIG                                                  \
	_IOR(AS7050_IOCTL_MAGIC, GET_AGC_CONFIG, as7050_agc_config_t)

/**
 * AS7050_WRITE_REGISTER - Declaration of system call for calling chiplib write_register
 * 
 * Parameters: Pointer to struct as7050_reg_pair holding information to write register
 */
#define AS7050_WRITE_REGISTER                                                  \
	_IOW(AS7050_IOCTL_MAGIC, WRITE_REGISTER, struct as7050_reg_pair)

/**
 * AS7050_READ_REGISTER - Declaration of system call for calling chiplib read_register
 * 
 * Parameters: Pointer to struct as7050_reg_pair which can store information of register data
 */
#define AS7050_READ_REGISTER                                                   \
	_IOR(AS7050_IOCTL_MAGIC, READ_REGISTER, struct as7050_reg_pair)

/**
 * AS7050_GET_MEASUREMENT_CONFIG - Declaration of system call for calling chiplib get_measurement_config
 * 
 * Parameters: Pointer to as7050_meas_config_t which can store measurement information
 */
#define AS7050_GET_MEASUREMENT_CONFIG                                          \
	_IOR(AS7050_IOCTL_MAGIC, GET_MEASUREMENT_CONFIG, as7050_meas_config_t)

/**
 * AS7050_START_MEASUREMENT - Declaration of system call for calling chiplib start_measurement
 *
 * Parameters: none
 */
#define AS7050_START_MEASUREMENT _IO(AS7050_IOCTL_MAGIC, START_MEASUREMENT)

/**
 * AS7050_STOP_MEASUREMENT - Declaration of system call for calling chiplib stop_measurement
 *
 * Parameters: none
 */
#define AS7050_STOP_MEASUREMENT _IO(AS7050_IOCTL_MAGIC, STOP_MEASUREMENT)

/**
 * AS7050_GET_VERSION - Declaration of system call to read the version of the driver
 *
 * Parameters: Pointer to an object of type struct as7050_version_info which can hold the version info
 */
#define AS7050_GET_VERSION                                                     \
	_IOR(AS7050_IOCTL_MAGIC, GET_VERSION, struct as7050_version_info)

/**
 * AS7050_GET_FIFO_SIZE - Declaration of system call to read the configured fifo size of the driver
 *
 * Parameters: Pointer to an object of type uint32_t which can hold the fifo size
 */
#define AS7050_GET_FIFO_SIZE _IOR(AS7050_IOCTL_MAGIC, GET_FIFO_SIZE, uint32_t)

/**
 * AS7050_GET_DAC_REF_VAL - Declaration of system call for calling chiplib read_register
 *
 * Parameters: Pointer to struct as7050_dac_ref_val which can store information of DAC status
 */
#define AS7050_GET_DAC_REF_VAL                                                 \
	_IOR(AS7050_IOCTL_MAGIC, GET_DAC_REF_VAL, struct as7050_dac_ref_val)

#endif //__AS7050_FCNTL_H__
