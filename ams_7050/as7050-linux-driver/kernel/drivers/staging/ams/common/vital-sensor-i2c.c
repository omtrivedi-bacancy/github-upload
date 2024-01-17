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
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "vital-sensor-misc.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/** the name of the character device driver */
#ifndef AMS_DRV_NAME
#pragma GCC error "Please define the name of the vital sensor driver"
#endif

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
#ifdef CONFIG_OF
static const struct of_device_id vital_sensor_of_match[] = {
	{
		.compatible = "ams," AMS_DRV_NAME "-i2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, vital_sensor_of_match);
#endif

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

int vital_sensor_i2c_transfer_data(struct i2c_client *client,
				   const uint8_t address, uint8_t *p_send_data,
				   const uint8_t send_data_size,
				   uint8_t *p_receive_data,
				   const uint8_t receive_data_size)
{
	int nmsgs, err;
	uint8_t i2c_block_data[U8_MAX + 1];
	struct i2c_msg msgs[2];

	nmsgs = 0;
	i2c_block_data[0] = address;
	memcpy(&i2c_block_data[1], p_send_data, send_data_size);

	msgs[nmsgs].addr = client->addr;
	msgs[nmsgs].buf = i2c_block_data;
	msgs[nmsgs].flags = 0;
	msgs[nmsgs++].len = send_data_size + sizeof(address);

	if (receive_data_size > 0) {
		msgs[nmsgs].addr = client->addr;
		msgs[nmsgs].buf = p_receive_data;
		msgs[nmsgs].flags = I2C_M_RD;
		msgs[nmsgs++].len = receive_data_size;
	}

	err = i2c_transfer(client->adapter, msgs, nmsgs);

	if (err < 0) {
		VITAL_LOG_ERR(&client->dev,
			      "I2C-Transfer failed! System error: %d (%s)", err, vital_get_error(err));
	} else if (err >= 0 && err < nmsgs) {
		VITAL_LOG_WARN(
			&client->dev,
			"I2C-Transfer failed! Only %d of %d bytes transferred",
			err, nmsgs);
		err = -ENODATA;
	} else {
		err = 0;
	}

	return err;
}

static int vital_sensor_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int err;
	struct vital_sensor_misc *sensor = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VITAL_LOG_ERR(&client->dev,
			      "Required I2C functionality not supported!");
		return -ENODEV;
	}

	sensor = kzalloc(sizeof(struct vital_sensor_misc), GFP_KERNEL);
	if (!sensor) {
		VITAL_LOG_ERR(&client->dev, "Memory allocation failed!");
		return -ENOMEM;
	}

	sensor->hw_iface = client;
	if (0 != client->irq) {
		sensor->irqnr = client->irq;
	}
	i2c_set_clientdata(client, sensor);

#ifdef CONFIG_OF
	if (of_match_device(vital_sensor_of_match, &client->dev)) {
		sensor->of_node = client->dev.of_node;
	}
#endif

	err = vital_sensor_misc_probe(sensor);
	if (err < 0) {
		VITAL_LOG_ERR(&client->dev,
			      "Initializing sensor device failed!");
		goto deinit;
	}

	VITAL_LOG_INFO(&client->dev, "Successfully configured.");
	return 0;

deinit:
	vital_sensor_misc_remove(sensor);
	kfree(sensor);
	return err;
}

static int vital_sensor_i2c_remove(struct i2c_client *client)
{
	struct vital_sensor_misc *sensor = i2c_get_clientdata(client);
	vital_sensor_misc_remove(sensor);
	kfree(sensor);
	return 0;
}

static const struct i2c_device_id vital_sensor_i2c_id[] = {
	{ .name = AMS_DRV_NAME "-i2c", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, vital_sensor_i2c_id);

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };
static struct i2c_driver vital_sensor_i2c_driver = {
    .class = I2C_CLASS_HWMON,
    .probe = vital_sensor_i2c_probe,
    .remove = vital_sensor_i2c_remove,
    .id_table = vital_sensor_i2c_id,
    .driver =
        {
            .owner = THIS_MODULE,
            .name = AMS_DRV_NAME "-i2c",
#ifdef CONFIG_OF
            .of_match_table = vital_sensor_of_match,
#endif
        },
    .address_list = normal_i2c,
};

module_i2c_driver(vital_sensor_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("ams <sebastian.schwabe@ams.com>");
MODULE_DESCRIPTION(AMS_DRV_NAME "-i2c-driver");