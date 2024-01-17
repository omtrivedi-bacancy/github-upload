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

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "vital-sensor-misc.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#define M_GET_DRV_DATA()                                                       \
	struct vital_sensor_misc *sensor = NULL;                               \
	do {                                                                   \
		sensor = dev_get_drvdata(dev);                                 \
		if (NULL == sensor) {                                          \
			return -EFAULT;                                        \
		}                                                              \
	} while (0)

#define ATTR_SHOW_FNC(__name)                                                  \
	static ssize_t __name##_show(struct device *dev,                       \
				     struct device_attribute *attr, char *buf)

#define ATTR_STORE_FNC(__name)                                                 \
	static ssize_t __name##_store(struct device *dev,                      \
				      struct device_attribute *attr,           \
				      const char *buf, size_t count)

#define ATTR_RO(__name) ATTR_SHOW_FNC(__name);
#define ATTR_WO(__name) ATTR_STORE_FNC(__name);
#define ATTR_RW(__name) ATTR_RO(__name) ATTR_WO(__name)

ATTR_RO(fifo_size);
ATTR_RW(irq);

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
static DEVICE_ATTR_RO(fifo_size);
static DEVICE_ATTR_RW(irq);

static struct attribute *vital_sensor_attributes[] = {
	&dev_attr_fifo_size.attr,
	&dev_attr_irq.attr,
	NULL,
};

struct attribute_group vital_sensor_attr_grp = {
	.attrs = vital_sensor_attributes,
};

struct device *dev;
/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/
ATTR_SHOW_FNC(fifo_size)
{
	M_GET_DRV_DATA();
	return sprintf(buf, "%u\n", sensor->chiplib.fifo_size);
}

ATTR_STORE_FNC(irq)
{
	int ret = -1;
	M_GET_DRV_DATA();

	if (0 != sensor->irqnr || -1 != sensor->irq_gpio) {
		return -EBUSY;
	}

	if (kstrtouint(buf, 0, &sensor->irq_gpio)) {
		return -EINVAL;
	}

	ret = gpio_request(sensor->irq_gpio, AMS_DRV_NAME "-int");
	if (0 == ret) {
		ret = gpio_direction_input(sensor->irq_gpio);
	}

	if (0 == ret) {
		sensor->irqnr = gpio_to_irq(sensor->irq_gpio);
	}

	if (sensor->irqnr < 0) {
		ret = sensor->irqnr;
		sensor->irqnr = 0;
	}

	return (ret == 0) ? count : ret;
}

ATTR_SHOW_FNC(irq)
{
	M_GET_DRV_DATA();
	return sprintf(buf, "%d\n", sensor->irqnr);
}

static unsigned int vital_sensor_misc_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (NULL != sensor) {
		poll_wait(filp, &sensor->chiplib.data_set_wq, wait);

		if (!kfifo_is_empty(&sensor->chiplib.data)) {
			mask |= (POLLIN | POLLRDNORM);
		}
	}

	return mask;
}

ssize_t vital_sensor_misc_read(struct file *filp, char *buff, size_t len,
			       loff_t *off)
{
	int err;
	uint32_t avail, copied;
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (NULL == sensor) {
		return -EFAULT;
	}

	if (kfifo_is_empty(&sensor->chiplib.data)) {
		VITAL_LOG_INFO(sensor->dev,
			       "Skipping read, no data available.");
		return -EAGAIN;
	}

	spin_lock(&sensor->chiplib.lock_kfifo);
	avail = kfifo_len(&sensor->chiplib.data);

	// check, if user buffer is big enough to hold data
	if (len < avail) {
		VITAL_LOG_ERR(
			sensor->dev,
			"User buffer can't hold fifo data. Expected len: %d but is %d",
			avail, len);
		err = -EINVAL;
		goto err_kfifo_out;
	}

	err = kfifo_to_user(&sensor->chiplib.data, buff, avail, &copied);
	if (0 != err) {
		VITAL_LOG_ERR(sensor->dev,
			      "Error copy fifo data to user buffer!");
		goto err_kfifo_out;
	} else {
		err = copied;
	}

err_kfifo_out:
	spin_unlock(&sensor->chiplib.lock_kfifo);
	return err;
}

int vital_sensor_misc_open(struct inode *inode, struct file *filp)
{
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (NULL == sensor) {
		return -EFAULT;
	}

	return vital_sensor_misc_load_chiplib(sensor);
}

int vital_sensor_misc_release(struct inode *inode, struct file *filp)
{
	struct vital_sensor_misc *sensor = dev_get_drvdata(
		((struct miscdevice *)filp->private_data)->this_device);

	if (NULL == sensor) {
		return -EFAULT;
	}

	return vital_sensor_misc_unload_chiplib(sensor);
}

static struct file_operations vital_sensor_misc_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.open = vital_sensor_misc_open,
	.read = vital_sensor_misc_read,
	.release = vital_sensor_misc_release,
	.poll = vital_sensor_misc_poll,
	.unlocked_ioctl = vital_sensor_misc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vital_sensor_misc_compat_ioctl
#endif

};

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/
int vital_sensor_misc_process_of_data(struct vital_sensor_misc *sensor)
{
#ifdef CONFIG_OF
	int ret;
	uint32_t fifo_size;

	// read in the fifo size
	ret = of_property_read_u32(sensor->of_node, "fifo_size", &fifo_size);
	if (0 == ret) {
		sensor->chiplib.fifo_size = fifo_size;
	} else {
		VITAL_LOG_WARN(sensor->dev, "Reading fifo_size failed (%d)!",
			       ret);
	}

#endif
	return 0;
}

int vital_sensor_misc_probe(struct vital_sensor_misc *sensor)
{
	int err;

	sensor->miscdev.minor = MISC_DYNAMIC_MINOR;
	sensor->miscdev.name = AMS_DRV_NAME;
	sensor->miscdev.fops = &vital_sensor_misc_fops;
	err = misc_register(&sensor->miscdev);
	if (err) {
		return err;
	} else {
		sensor->dev = sensor->miscdev.this_device;
		dev_set_drvdata(sensor->dev, sensor);
	}

	// read open firmware information from device tree, if given
	if (sensor->of_node) {
		err = vital_sensor_misc_process_of_data(sensor);
		if (err < 0) {
			VITAL_LOG_ERR(
				sensor->dev,
				"Processing device tree information failed!");
			goto misc_add_failed;
		}
	} else {
		// initialize with default values
		VITAL_LOG_WARN(
			sensor->dev,
			"Reading open firmware data failed. Using default values!");
		sensor->chiplib.fifo_size = 256;
		sensor->irq_gpio = -1;
	}

	// set driver data and setup attributes
	err = sysfs_create_group(&sensor->dev->kobj, &vital_sensor_attr_grp);
	if (err < 0) {
		VITAL_LOG_ERR(sensor->dev, "Setup attributes failed!");
		goto misc_add_failed;
	}

	// allocate kernel fifo which will hold measurement data
	VITAL_LOG_DBG(sensor->dev, "Allocating %d byte for FIFO.",
		      sensor->chiplib.fifo_size);
	err = kfifo_alloc(&sensor->chiplib.data, sensor->chiplib.fifo_size,
			  GFP_KERNEL);
	if (err != 0) {
		VITAL_LOG_ERR(sensor->dev,
			      "Allocating memory for data fifo failed!");
		goto misc_add_failed;
	}

	dev = sensor->dev;
	spin_lock_init(&sensor->chiplib.lock_kfifo);
	init_waitqueue_head(&sensor->chiplib.data_set_wq);
	return 0;

misc_add_failed:
	misc_deregister(&sensor->miscdev);
	return err;
}

int vital_sensor_misc_remove(struct vital_sensor_misc *sensor)
{
	kfifo_free(&sensor->chiplib.data);

	if (0 != sensor->irq_requested) {
		free_irq(sensor->irqnr, sensor);
		sensor->irqnr = 0;
	}

	if (-1 != sensor->irq_gpio) {
		gpio_free(sensor->irq_gpio);
		sensor->irq_gpio = -1;
	}

	misc_deregister(&sensor->miscdev);
	return 0;
}