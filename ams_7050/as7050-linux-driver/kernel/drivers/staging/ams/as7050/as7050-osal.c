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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include "vital-sensor-misc.h"
#include "as7050_osal_chiplib.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
extern int vital_sensor_i2c_transfer_data(struct i2c_client *client,
					  const uint8_t address,
					  uint8_t *p_send_data,
					  const uint8_t send_data_size,
					  uint8_t *p_receive_data,
					  const uint8_t receive_data_size);

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
static struct vital_sensor_misc *g_pcdev = NULL;
static struct workqueue_struct *wq = 0;

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/
void as7050_work_handler(struct work_struct *w)
{
	err_code_t err;
	if (NULL == g_pcdev) {
		return;
	}

	if (NULL == g_pcdev->chiplib.pisr) {
		return;
	}

	err = ((as7050_osal_interrupt_t)g_pcdev->chiplib.pisr)();
	if (ERR_SUCCESS != err) {
		VITAL_LOG_ERR(
			g_pcdev->dev, 
			"Executing interrupt handler of chiplib failed %d", 
			err);
	}
}
static DECLARE_WORK(as7050_work, as7050_work_handler);

static irqreturn_t as7050_isr(int irq, void *dev_id)
{
	queue_work(wq, &as7050_work);
	return IRQ_HANDLED;
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/
err_code_t as7050_osal_initialize(const char *p_interface_desc)
{
	int ret = 0;
	err_code_t err = ERR_SUCCESS;

	M_CHECK_NULL_POINTER(p_interface_desc);

	if (NULL != g_pcdev) {
		err = ERR_PERMISSION;
		goto osal_init_end;
	}

	g_pcdev = (struct vital_sensor_misc *)p_interface_desc;
	if (0 == g_pcdev->irqnr) {
		VITAL_LOG_WARN(
			g_pcdev->dev,
			"No interrupt line given. Setup of IRQ canceled.");
		err = ERR_SYSTEM_CONFIG;
		goto osal_init_end;
	}

	if (!wq) {
		wq = create_singlethread_workqueue("as7050_work");
	}

	//setup interrupt handler
	if (0 == g_pcdev->irq_requested) {
		ret = request_any_context_irq(g_pcdev->irqnr,
					      (irq_handler_t)as7050_isr,
					      IRQF_TRIGGER_RISING | IRQF_SHARED,
					      AMS_DRV_NAME "-irq", g_pcdev);

		if (ret < 0) {
			VITAL_LOG_ERR(g_pcdev->dev,
				      "Requesting irq %d failed with %d",
				      g_pcdev->irqnr, ret);
			err = ERR_PERMISSION;
		} else {
			g_pcdev->irq_requested = 1;
		}
	}

osal_init_end:
	return err;
}

err_code_t as7050_osal_shutdown(void)
{
	if (NULL != g_pcdev) {
		if (g_pcdev->irq_requested) {
			free_irq(g_pcdev->irqnr, g_pcdev);
			g_pcdev->irq_requested = 0;
		}

		if (wq) {
			destroy_workqueue(wq);
			wq = 0;
		}

		g_pcdev = NULL;
	}

	return ERR_SUCCESS;
}

err_code_t as7050_osal_write_registers(uint8_t address, uint8_t number,
				       uint8_t *p_values)
{
	err_code_t err = ERR_SUCCESS;
	if (NULL == g_pcdev) {
		return ERR_PERMISSION;
	}

	if (vital_sensor_i2c_transfer_data(g_pcdev->hw_iface, address, p_values,
					   number, NULL, 0)) {
		err = ERR_DATA_TRANSFER;
	}

	return err;
}

err_code_t as7050_osal_read_registers(uint8_t address, uint8_t number,
				      uint8_t *p_values)
{
	err_code_t err = ERR_SUCCESS;
	if (NULL == g_pcdev) {
		return ERR_PERMISSION;
	}

	if (vital_sensor_i2c_transfer_data(g_pcdev->hw_iface, address, NULL, 0,
					   p_values, number)) {
		err = ERR_DATA_TRANSFER;
	}

	return err;
}

err_code_t
as7050_osal_register_int_handler(as7050_osal_interrupt_t callback_function)
{
	err_code_t err = ERR_SUCCESS;

	if (NULL != g_pcdev) {
		g_pcdev->chiplib.pisr = (void *)callback_function;
	} else {
		err = ERR_PERMISSION;
	}
	return err;
}

err_code_t as7050_osal_get_ms_tick(uint32_t *p_tick_ms)
{
	*p_tick_ms = (uint32_t)ktime_to_ms(ktime_get());
	return ERR_SUCCESS;
}