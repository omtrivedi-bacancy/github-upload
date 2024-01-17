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

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <linux/slab.h>
#include <linux/uaccess.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "vital-sensor-misc.h"
#include "as7050-fcntl.h"
#include "as7050_chiplib.h"
#include "as7050_typedefs.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#ifdef CONFIG_COMPAT
// translate pointers between 32bit user space and 64bit kernel space
// pointer is a 32bit value in user space but 64bit value in kernel space
struct compat_as7050_reg_group {
	enum as7050_reg_group_ids id;
	uint8_t size;
	compat_caddr_t data; /* uint8_t *data */
};
#endif

typedef err_code_t (*specific_ioctl)(struct vital_sensor_misc *, unsigned long,
				     int *);

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/
static err_code_t ioctl_as7050_initialize(struct vital_sensor_misc *sensor,
					  unsigned long arg, int *perr)
{
	M_UNUSED_PARAM(arg);
	M_UNUSED_PARAM(perr);

	return as7050_initialize(
		(as7050_callback_t)sensor->chiplib.data_callback_fnc, sensor,
		(const char *)sensor);
}

static err_code_t ioctl_as7050_shutdown(struct vital_sensor_misc *sensor,
					unsigned long arg, int *perr)
{
	M_UNUSED_PARAM(sensor);
	M_UNUSED_PARAM(arg);
	M_UNUSED_PARAM(perr);

	return as7050_shutdown();
}

static err_code_t ioctl_as7050_set_reg_group(struct vital_sensor_misc *sensor,
					     unsigned long arg, int *perr)
{
	struct as7050_reg_group reg_group;
	uint8_t *reg_group_data = NULL;

	int err = 0;
	err_code_t ret = ERR_MEMORY;

	// copy reg group object from user space
	err = copy_from_user((void *)&reg_group, (const void __user *)arg,
			     sizeof(reg_group));

	if (0 == err && NULL != reg_group.data) {
		// copy reg group data from user space
		reg_group_data = kzalloc(reg_group.size, GFP_KERNEL);
		err = copy_from_user((void *)reg_group_data,
				     (const void __user *)reg_group.data,
				     reg_group.size);
	}

	if (0 == err) {
		// call chiplib function to set the reg group information
		ret = as7050_set_reg_group(reg_group.id, reg_group_data,
					   reg_group.size);
	}

	if (NULL != reg_group_data) {
		kfree(reg_group_data);
	}

	*perr = err;
	return ret;
}

static err_code_t ioctl_as7050_get_reg_group(struct vital_sensor_misc *sensor,
					     unsigned long arg, int *perr)
{
	struct as7050_reg_group reg_group;
	struct as7050_reg_group *puser_reg_group =
		(struct as7050_reg_group *)arg;
	uint8_t *reg_group_data = NULL;

	int err = 0;
	err_code_t ret = ERR_MEMORY;

	// copy reg group object from user space
	err = copy_from_user((void *)&reg_group,
			     (const void __user *)puser_reg_group,
			     sizeof(reg_group));

	if (0 == err) {
		// allocate memory if user space expects data
		if (NULL != reg_group.data) {
			reg_group_data = kzalloc(reg_group.size, GFP_KERNEL);
		}

		// call chiplib function to get the reg group information and data
		ret = as7050_get_reg_group(reg_group.id, reg_group_data,
					   &reg_group.size);
	}

	if (ERR_SUCCESS == ret && NULL != reg_group.data) {
		// copy reg group data back to user space
		err = copy_to_user((void __user *)reg_group.data,
				   (const void *)reg_group_data,
				   reg_group.size);
		if (0 != err) {
			ret = ERR_MEMORY;
		}
	}

	if (ERR_SUCCESS == ret) {
		put_user(reg_group.size, &puser_reg_group->size);
	}

	if (NULL != reg_group_data) {
		kfree(reg_group_data);
	}

	*perr = err;
	return ret;
}

#ifdef CONFIG_COMPAT
static err_code_t
compat_ioctl_as7050_set_reg_group(struct vital_sensor_misc *sensor,
				  unsigned long arg, int *perr)
{
	struct compat_as7050_reg_group reg_group;
	uint8_t *reg_group_data = NULL;

	int err = 0;
	err_code_t ret = ERR_MEMORY;

	// copy reg group object from user space
	err = copy_from_user((void *)&reg_group, (const void __user *)arg,
			     sizeof(reg_group));

	if (0 == err && NULL != compat_ptr(reg_group.data)) {
		// copy reg group data from user space
		reg_group_data = kzalloc(reg_group.size, GFP_KERNEL);
		err = copy_from_user(
			(void *)reg_group_data,
			(const void __user *)compat_ptr(reg_group.data),
			reg_group.size);
	}

	if (0 == err) {
		// call chiplib function to set the reg group information
		ret = as7050_set_reg_group(reg_group.id, reg_group_data,
					   reg_group.size);
	}

	if (NULL != reg_group_data) {
		kfree(reg_group_data);
	}

	*perr = err;
	return ret;
}

static err_code_t
compat_ioctl_as7050_get_reg_group(struct vital_sensor_misc *sensor,
				  unsigned long arg, int *perr)
{
	struct compat_as7050_reg_group reg_group;
	struct compat_as7050_reg_group *puser_reg_group =
		(struct compat_as7050_reg_group *)arg;
	uint8_t *reg_group_data = NULL;

	int err = 0;
	err_code_t ret = ERR_MEMORY;

	// copy reg group object from user space
	err = copy_from_user((void *)&reg_group,
			     (const void __user *)puser_reg_group,
			     sizeof(reg_group));

	if (0 == err) {
		// allocate memory if user space expects data
		if (NULL != compat_ptr(reg_group.data)) {
			reg_group_data = kzalloc(reg_group.size, GFP_KERNEL);
		}

		// call chiplib function to get the reg group information and data
		ret = as7050_get_reg_group(reg_group.id, reg_group_data,
					   &reg_group.size);
	}

	if (ERR_SUCCESS == ret && NULL != compat_ptr(reg_group.data)) {
		// copy reg group data back to user space
		err = copy_to_user((void __user *)compat_ptr(reg_group.data),
				   (const void *)reg_group_data,
				   reg_group.size);
		if (0 != err) {
			ret = ERR_MEMORY;
		}
	}

	if (ERR_SUCCESS == ret) {
		put_user(reg_group.size, &puser_reg_group->size);
	}

	if (NULL != reg_group_data) {
		kfree(reg_group_data);
	}

	*perr = err;
	return ret;
}
#endif

static err_code_t ioctl_as7050_set_agc_config(struct vital_sensor_misc *sensor,
					      unsigned long arg, int *perr)
{
	as7050_agc_config_t agc_config;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	err = copy_from_user(&agc_config, (void *)arg,
			     sizeof(as7050_agc_config_t));
	if (0 == err) {
		ret = as7050_set_agc_config(&agc_config);
	} else {
		ret = ERR_MEMORY;
	}

	*perr = err;
	return ret;
}

static err_code_t ioctl_as7050_get_agc_config(struct vital_sensor_misc *sensor,
					      unsigned long arg, int *perr)
{
	as7050_agc_config_t agc_config;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	ret = as7050_get_agc_config(&agc_config);
	if (ERR_SUCCESS == ret) {
		err = copy_to_user((void *)arg, &agc_config,
				   sizeof(as7050_agc_config_t));
		if (0 != err) {
			ret = ERR_MEMORY;
		}
	}

	*perr = err;
	return ret;
}

static err_code_t ioctl_as7050_write_register(struct vital_sensor_misc *sensor,
					      unsigned long arg, int *perr)
{
	struct as7050_reg_pair reg_pair;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	err = copy_from_user(&reg_pair, (void *)arg,
			     sizeof(struct as7050_reg_pair));
	if (0 == err) {
		ret = as7050_write_register(reg_pair.addr, reg_pair.val);
	} else {
		ret = ERR_MEMORY;
	}

	*perr = err;
	return ret;
}

static err_code_t ioctl_as7050_read_register(struct vital_sensor_misc *sensor,
					     unsigned long arg, int *perr)
{
	struct as7050_reg_pair reg_pair;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	err = copy_from_user(&reg_pair, (void *)arg,
			     sizeof(struct as7050_reg_pair));
	if (0 == err) {
		ret = as7050_read_register(reg_pair.addr, &reg_pair.val);
		if (ERR_SUCCESS == ret) {
			err = copy_to_user((void *)arg, &reg_pair,
					   sizeof(struct as7050_reg_pair));
			if (0 != err) {
				ret = ERR_MEMORY;
			}
		}
	} else {
		ret = ERR_MEMORY;
	}

	*perr = err;
	return ret;
}

static err_code_t
ioctl_as7050_get_measurement_config(struct vital_sensor_misc *sensor,
				    unsigned long arg, int *perr)
{
	as7050_meas_config_t meas_config;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	ret = as7050_get_measurement_config(&meas_config);
	if (ERR_SUCCESS == ret) {
		err = copy_to_user((void *)arg, &meas_config,
				   sizeof(as7050_meas_config_t));
		if (0 != err) {
			ret = ERR_MEMORY;
		}
	}

	*perr = err;
	return ret;
}
static err_code_t
ioctl_as7050_start_measurement(struct vital_sensor_misc *sensor,
			       unsigned long arg, int *perr)
{
	M_UNUSED_PARAM(sensor);
	M_UNUSED_PARAM(arg);
	M_UNUSED_PARAM(perr);
	return as7050_start_measurement();
}

static err_code_t
ioctl_as7050_stop_measurement(struct vital_sensor_misc *sensor,
			      unsigned long arg, int *perr)
{
	M_UNUSED_PARAM(sensor);
	M_UNUSED_PARAM(arg);
	M_UNUSED_PARAM(perr);
	return as7050_stop_measurement();
}

static err_code_t ioctl_as7050_get_version(struct vital_sensor_misc *sensor,
					   unsigned long arg, int *perr)
{
	as7050_version_t vers;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;
	struct as7050_version_info *puser = (struct as7050_version_info *)arg;

	ret = as7050_get_version(&vers);
	if (ERR_SUCCESS == ret) {
		err = copy_to_user(puser, &vers, sizeof(as7050_version_t));
		if (0 != err) {
			ret = ERR_MEMORY;
		}
	}

	*perr = err;
	return ret;
}

static err_code_t ioctl_as7050_get_fifo_size(struct vital_sensor_misc *sensor,
					     unsigned long arg, int *perr)
{
	M_UNUSED_PARAM(perr);
	put_user(sensor->chiplib.fifo_size, (uint32_t *)arg);
	return ERR_SUCCESS;
}

static err_code_t
ioctl_as7050_calculate_dac_ref_value(struct vital_sensor_misc *sensor,
				     unsigned long arg, int *perr)
{
	struct as7050_dac_ref_val dac_val;
	int err = 0;
	err_code_t ret = ERR_SUCCESS;

	err = copy_from_user(&dac_val, (const void __user *)arg,
			     sizeof(struct as7050_dac_ref_val));
	if (0 == err) {
		ret = as7050_calculate_dac_reference_value(dac_val.control,
							   dac_val.average_cnt,
							   &dac_val.status,
							   &dac_val.dac_ref);
		if (ERR_SUCCESS == ret) {
			err = copy_to_user((void *)arg, &dac_val,
					   sizeof(struct as7050_dac_ref_val));
			if (0 != err) {
				ret = ERR_MEMORY;
			}
		}
	} else {
		ret = ERR_MEMORY;
	}

	*perr = err;
	return ret;
}

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
static specific_ioctl as7050_ioctl[] = {
	[INITIALIZE] = ioctl_as7050_initialize,
	[SHUTDOWN] = ioctl_as7050_shutdown,
	[SET_REG_GROUP] = ioctl_as7050_set_reg_group,
	[GET_REG_GROUP] = ioctl_as7050_get_reg_group,
	[SET_AGC_CONFIG] = ioctl_as7050_set_agc_config,
	[GET_AGC_CONFIG] = ioctl_as7050_get_agc_config,
	[WRITE_REGISTER] = ioctl_as7050_write_register,
	[READ_REGISTER] = ioctl_as7050_read_register,
	[GET_MEASUREMENT_CONFIG] = ioctl_as7050_get_measurement_config,
	[START_MEASUREMENT] = ioctl_as7050_start_measurement,
	[STOP_MEASUREMENT] = ioctl_as7050_stop_measurement,
	[GET_VERSION] = ioctl_as7050_get_version,
	[GET_FIFO_SIZE] = ioctl_as7050_get_fifo_size,
	[GET_DAC_REF_VAL] = ioctl_as7050_calculate_dac_ref_value
};

#ifdef CONFIG_COMPAT
static specific_ioctl as7050_compat_ioctl[] = {
	[INITIALIZE] = ioctl_as7050_initialize,
	[SHUTDOWN] = ioctl_as7050_shutdown,
	[SET_REG_GROUP] = compat_ioctl_as7050_set_reg_group,
	[GET_REG_GROUP] = compat_ioctl_as7050_get_reg_group,
	[SET_AGC_CONFIG] = ioctl_as7050_set_agc_config,
	[GET_AGC_CONFIG] = ioctl_as7050_get_agc_config,
	[WRITE_REGISTER] = ioctl_as7050_write_register,
	[READ_REGISTER] = ioctl_as7050_read_register,
	[GET_MEASUREMENT_CONFIG] = ioctl_as7050_get_measurement_config,
	[START_MEASUREMENT] = ioctl_as7050_start_measurement,
	[STOP_MEASUREMENT] = ioctl_as7050_stop_measurement,
	[GET_VERSION] = ioctl_as7050_get_version,
	[GET_FIFO_SIZE] = ioctl_as7050_get_fifo_size,
	[GET_DAC_REF_VAL] = ioctl_as7050_calculate_dac_ref_value
};
#endif

long vital_sensor_misc_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	int err = 0;
	err_code_t ret = ERR_SUCCESS;
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (_IOC_NR(cmd) > ARRAY_SIZE(as7050_ioctl) - 1) {
		ret = ERR_SYSTEM_CONFIG;
	} else {
		ret = as7050_ioctl[_IOC_NR(cmd)](sensor, arg, &err);
	}

	if (0 != err) {
		VITAL_LOG_ERR(
			sensor->dev,
			"IOCTL 0x%08x (%d): Internal handling failed with %s!",
			_IOC_NR(cmd), _IOC_NR(cmd), vital_get_error(err));
	}
	if (ERR_SUCCESS != ret && 0 == err) {
		VITAL_LOG_WARN(
			sensor->dev,
			"IOCTL 0x%08x (%d): Calling chiplib failed with %d!",
			_IOC_NR(cmd), _IOC_NR(cmd), ret);
	}
	return (long)ret;
}

#ifdef CONFIG_COMPAT
long vital_sensor_misc_compat_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg)
{
	int err = 0;
	err_code_t ret = ERR_SUCCESS;
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (_IOC_NR(cmd) > ARRAY_SIZE(as7050_compat_ioctl) - 1) {
		ret = ERR_SYSTEM_CONFIG;
	} else {
		ret = as7050_compat_ioctl[_IOC_NR(cmd)](sensor, arg, &err);
	}

	if (0 != err) {
		VITAL_LOG_ERR(
			sensor->dev,
			"IOCTL 0x%08x (%d): Internal handling failed with %s!",
			_IOC_NR(cmd), _IOC_NR(cmd), vital_get_error(err));
	}
	if (ERR_SUCCESS != ret && 0 == err) {
		VITAL_LOG_DBG(
			sensor->dev,
			"IOCTL 0x%08x (%d): Calling chiplib failed with %d!",
			_IOC_NR(cmd), _IOC_NR(cmd), ret);
	}
	return ret;
}
#endif