/******************************************************************************
 * Copyright by ams AG                                                        *
 * All rights are reserved.                                                   *
 *                                                                            *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING      *
 * THE SOFTWARE.                                                              *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS          *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,      *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT           *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,      *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY      *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT        *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
 ******************************************************************************/

/*!
 * \file      i2c.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup i2c_driver_group
 *
 * \brief Definition for generic I2C master driver interface
 *
 * This interface can be used to initialize I2C periphery and send or receive
 * data. The global device parameters and periphery connections can be commited
 * by the handle-parameter of each function.
 *
 *  @{
 */

#ifndef __I2C_H__
#define __I2C_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief This function must be used to initialize the I2C periphery
 *
 * Following tasks will be done here:
 * - Configuration of the corresponding pins
 * - Configuration of the bus frequency
 *
 * \note This function must be called at first, otherwise all other functions
 * return with error code ::ERR_PERMISSION
 *
 * \param[in] p_handle              Handle to the device configuration
 * \param[in] frequency             Frequency of I2C-bus (100000Hz, 400000Hz und
 *                                  1000000Hz are supported as default)
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            NULL-pointer detected
 * \retval ::ERR_ARGUMENT           An argument is invalid.
 * \retval ::ERR_I2C                Error during I2C initialization
 */
err_code_t i2c_init(void *p_handle, uint32_t frequency);

/*!
 * \brief Transfers a I2C sequence to a specific i2c address
 *
 * \note This function can only called if i2c_init was called before
 *
 * \param[in] p_handle              Handle to the device configuration
 * \param[in] dev_addr_7bit         7bit I2C-address of the slave device
 * \param[in] p_send_data           Pointer to send data buffer
 * \param[in] send_len              Number of bytes to send
 * \param[out] p_recv_data          Pointer to the memory, where received data can be saved
 * \param[in] recv_len              Length of the requested data
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            NULL-pointer detected
 * \retval ::ERR_ARGUMENT           An argument is invalid.
 * \retval ::ERR_PERMISSION         Access to the function is blocked.
 * \retval ::ERR_I2C                Communication error to slave device.
 */
err_code_t i2c_transfer(void *p_handle, uint8_t dev_addr_7bit, uint8_t *p_send_data, uint16_t send_len,
                        uint8_t *p_recv_data, uint16_t recv_len);

/*!
 * \brief Closes the I2C periphery
 *
 * \param[in] p_handle              Handle to the device configuration

 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            NULL-pointer detected
 * \retval ::ERR_ARGUMENT           An argument is invalid.
 * \retval ::ERR_I2C                Error while closing the I2C periphery
 */
err_code_t i2c_shutdown(void *p_handle);

/*! @} */

#endif /*__I2C_H__ */
