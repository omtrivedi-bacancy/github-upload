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
#include <linux/types.h>
#include <linux/kfifo.h>

#include "vital-sensor-misc.h"
#include "as7050_chiplib.h"
#include "as7050_typedefs.h"
#include "as7050_version.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
struct chiplib_dataset_head {
	err_code_t error;
	as7050_agc_status_t agc_status;
	uint16_t data_size;
};

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/
void as7050_data_callback(err_code_t error, uint8_t *p_data, uint16_t data_num,
			  as7050_agc_status_t *p_agc_status, void *p_cb_param)
{
	uint8_t kfifo_ovfl = 0;
	struct vital_sensor_misc *sensor =
		(struct vital_sensor_misc *)p_cb_param;

	struct chiplib_dataset_head ds_head = { 0 };
	ssize_t data_set_sz = sizeof(ds_head) + data_num * sizeof(uint16_t);

	spin_lock(&sensor->chiplib.lock_kfifo);

	if (kfifo_avail(&sensor->chiplib.data) < data_set_sz) {
		// if no error from device, emulate an overflow error and stop measurement
		if (ERR_SUCCESS == error) {
			kfifo_ovfl = 1;
			ds_head.data_size = kfifo_avail(&sensor->chiplib.data) -
					    sizeof(ds_head);
		}
	} else {
		ds_head.data_size = data_num * sizeof(uint16_t);
	}

	// try to store error and agc status at least
	if (kfifo_avail(&sensor->chiplib.data) < sizeof(ds_head)) {
		VITAL_LOG_ERR(
			sensor->dev,
			"No space left in kfifo to hold status data from sensor!");
	} else {
		if (kfifo_ovfl) {
			error = ERR_OVERFLOW;
			VITAL_LOG_WARN(
				sensor->dev,
				"No space left in kfifo to hold complete data from sensor!");
		}

		ds_head.error = error;

		if (NULL != p_agc_status) {
			memcpy(&ds_head.agc_status, p_agc_status,
			       sizeof(as7050_agc_status_t));
		}

		kfifo_in(&sensor->chiplib.data, &ds_head, sizeof(ds_head));
		if (NULL != p_data) {
			kfifo_in(&sensor->chiplib.data, p_data,
				 ds_head.data_size);
		}
	}
	spin_unlock(&sensor->chiplib.lock_kfifo);
	wake_up_interruptible(&sensor->chiplib.data_set_wq);
}

err_code_t vital_sensor_misc_load_chiplib(struct vital_sensor_misc *sensor)
{
	sensor->chiplib.data_callback_fnc = &as7050_data_callback;
	sensor->ver.major = AS7050_VER_MAJOR;
	sensor->ver.minor = AS7050_VER_MINOR;
	sensor->ver.patch = AS7050_VER_PATCH;

	return ERR_SUCCESS;
}

err_code_t vital_sensor_misc_unload_chiplib(struct vital_sensor_misc *sensor)
{
	as7050_shutdown();
	spin_lock(&sensor->chiplib.lock_kfifo);
	kfifo_reset(&sensor->chiplib.data);
	spin_unlock(&sensor->chiplib.lock_kfifo);

	return ERR_SUCCESS;
}
