/* 
 * Common vital signs sensor driver
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

#ifndef __VITAL_SENSOR_MISC_H__
#define __VITAL_SENSOR_MISC_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/kfifo.h>

#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/**
 * VITAL_LOG_ERR() - Print an error message to kernel log.
 * @dev: Pointer to a device structure
 * @fmt: Message format
 * @...: Additional parameters for message format
 */
#define VITAL_LOG_ERR(dev, fmt, ...)                                           \
	vital_log_err(dev, __func__, fmt, ##__VA_ARGS__)

/**
 * VITAL_LOG_DBG() - Print a debug message to kernel log.
 * @dev: Pointer to a device structure
 * @fmt: Message format
 * @...: Additional parameters for message format
 */
#define VITAL_LOG_DBG(dev, fmt, ...)                                           \
	vital_log_dbg(dev, __func__, fmt, ##__VA_ARGS__)

/**
 * VITAL_LOG_WARN() - Print warning message to kernel log.
 * @dev: Pointer to a device structure
 * @fmt: Message format
 * @...: Additional parameters for message format
 */
#define VITAL_LOG_WARN(dev, fmt, ...)                                          \
	vital_log_warn(dev, __func__, fmt, ##__VA_ARGS__)

/**
 * VITAL_LOG_INFO() - Print an info message to kernel log.
 * @dev: Pointer to a device structure
 * @fmt: Message format
 * @...: Additional parameters for message format
 */
#define VITAL_LOG_INFO(dev, fmt, ...)                                          \
	vital_log_info(dev, __func__, fmt, ##__VA_ARGS__)

/**
 * struct version_info - The version information of the driver
 * @major: - the major part of the version
 * @minor: - the minor part of the version
 * @patch: - the patch level of the version
 */
struct version_info {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
};

/**
 * struct chiplib_wrapper - Holds additional information for wrapping the chiplibrary
 * @data_set_wq: - wait queue to synchronize polling of the character device
 * @data: - kernel fifo holding actual data received from the chiplibrary
 * @lock_kfifo: - spinlock to synchronize access to the kernel fifo
 * @fifo_size: - size of he kernel fifo in bytes
 * @data_callback_fnc: - callback function called by chiplibrary upon new data
 * @pisr: - routine to call upon interrupt of the sensor
 * @loaded: - flag indicates if chiplibrary was loaded
 */
struct chiplib_wrapper {
	wait_queue_head_t data_set_wq;
	struct kfifo data;
	spinlock_t lock_kfifo;
	uint32_t fifo_size;
	void *data_callback_fnc;
	void *pisr;
	uint8_t loaded;
};

/**
 * struct vital_sensor_misc - Type holds all information for the implemented
 * character device driver
 * @dev: - The corresponding device structure in the system
 * @hw_iface: - the underlying hardware interface (typically a pointer to struct
 * i2c_client)
 * @irqnr: - system interrupt number identifying the interrupt of the sensor
 * @of_node: - Open Firmware descriptor of the device
 * @chiplib: - The chiplib interface functions of the device
 * @version_info: - Version info of the driver
 * @irq_gpio: - GPIO number in the system which can be used as interrupt source
 * @irq_requested: - flag indicates if the driver requested the interrupt
 *
 * The driver is implemented as simple character device driver and supports up
 * to 1 sensor device. For communication with functions of the chiplibrary,
 * the IOCTL calls or the sysfs attributes can be used. With the help of these
 * functions, a measurement can be started or stopped by the user. 
 * A measurement is interrupt controlled, once the chiplibrary is initialized by the driver, 
 * the data is read upon interrupt from the sensor. Read data is stored in a separate
 * kernel fifo and can be read by the user via the read sys call of the driver.
 * To fully support the integration of the chiplib, an OSAL object is
 * implemented which handles the low level access to the sensor device.
 */
struct vital_sensor_misc {
	struct miscdevice miscdev;
	struct device *dev;
	void *hw_iface;
	int irqnr;
	struct device_node *of_node;
	struct chiplib_wrapper chiplib;
	struct version_info ver;
	int irq_gpio;
	uint8_t irq_requested;
};

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/**
 * vital_sensor_misc_probe() - Initialize the sensor device as character device
 * in the system
 * @sensor: Pointer to the device driver structure
 *
 * Return: System error code or 0 on success.
 */
int vital_sensor_misc_probe(struct vital_sensor_misc *sensor);

/**
 * vital_sensor_misc_remove() - Removes the sensor device from the system
 * @sensor: An allocated pointer to the device driver structure
 *
 * Return: System error code or 0 on success.
 */
int vital_sensor_misc_remove(struct vital_sensor_misc *sensor);

/**
 * vital_sensor_misc_setup_attributes() - Sets up the device attributes
 * @dev: Pointer to a device structure
 * @sensor: An allocated pointer to the device driver structure
 *
 * This function is defined in a different file for clear arrangement.
 *
 * Return: System error code or 0 on success.
 */
int vital_sensor_misc_setup_attributes(struct device *dev,
				       struct vital_sensor_misc *sensor);

/**
 * vital_sensor_misc_ioctl() - Implementation of system calls
 * @filp: Pointer identifying the device node
 * @cmd: The sys call to execute
 * @arg: The argument of the sys call
 *
 * This function is defined in a different file for clear arrangement.
 * 
 * Return: Internal error of the chiplib or ERR_SUCCESS on success.
 */
long vital_sensor_misc_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg);

#ifdef CONFIG_COMPAT
long vital_sensor_misc_compat_ioctl(struct file *filp, unsigned int cmd,
				    unsigned long arg);
#endif

/**
 * vital_sensor_misc_process_of_data() - Read open firmware information from device tree
 * @sensor: An allocated pointer to the device driver structure
 *
 * This function is defined in a different file for clear arrangement.
 * 
 * Return: System error code or 0 on success.
 */
int vital_sensor_misc_process_of_data(struct vital_sensor_misc *sensor);

/**
 * vital_sensor_load_chiplib() - Initializes the chiplib.
 * @sensor: An allocated pointer to the device driver structure
 * 
 * Return: Internal error of the chiplib or ERR_SUCCESS on success.
 */
err_code_t vital_sensor_misc_load_chiplib(struct vital_sensor_misc *sensor);

/**
 * vital_sensor_unload_chiplib() - Deinitializes the chiplib.
 * @sensor: An allocated pointer to the device driver structure
 * 
 * Return: Internal error of the chiplib or ERR_SUCCESS on success.
 */
err_code_t vital_sensor_misc_unload_chiplib(struct vital_sensor_misc *sensor);

/**
 * vital_log_err() - Print an error message to kernel log
 * @dev: Pointer to a device structure
 * @func: Name of the function which prints the message
 * @fmt: Message format
 * @...: Additional parameters for message format
 *
 */
void vital_log_err(const void *dev, const char *func, const char *fmt, ...);

/**
 * vital_log_dbg() - Print a debug message to kernel log
 * @dev: Pointer to a device structure
 * @func: Name of the function which prints the message
 * @fmt: Message format
 * @...: Additional parameters for message format
 *
 */
void vital_log_dbg(const void *dev, const char *func, const char *fmt, ...);

/**
 * vital_log_warn() - Print a warning message to kernel log
 * @dev: Pointer to a device structure
 * @func: Name of the function which prints the message
 * @fmt: Message format
 * @...: Additional parameters for message format
 *
 */
void vital_log_warn(const void *dev, const char *func, const char *fmt, ...);

/**
 * vital_log_info() - Print an info message to kernel log
 * @dev: Pointer to a device structure
 * @func: Name of the function which prints the message
 * @fmt: Message format
 * @...: Additional parameters for message format
 *
 */
void vital_log_info(const void *dev, const char *func, const char *fmt, ...);

/**
 * vital_get_error() - Return string to an errno value
 * @error: Error to get the error string from
 *
 */
char *vital_get_error(int error);

#endif //__VITAL_SENSOR_MISC_H__