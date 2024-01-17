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

#ifndef __AS7050_OSAL_H__
#define __AS7050_OSAL_H__

/*!
 * \file      as7050_osal_chiplib.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup osal_group
 *
 * \brief This is the abstraction layer for the chip library.
 *
 * The functions has dependencies to the operatation system.
 * So the function must be implemented application specific.
 * Following function groups must be implemented:
 * - initialize/shutdown
 * - I2C-transfers
 * - Interrupt-handler
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_std_include.h"
#include "error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
/*!
 * \brief Callback function, which will be called if a new interrupt notification is available.
 *
 * This callback type will be registered via the function ::as7050_osal_register_int_handler.
 *
 */
typedef err_code_t (*as7050_osal_interrupt_t)(void);

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Initialization of the hardware abstraction layer.
 *
 * - Initialization of global parameters.
 * - Activates the ENABLE-pin
 * - Open the interface to the sensor.
 *
 * \note This function must be called at first!
 *
 * \param[in] p_interface_desc      Can be used to transfer special initialization data like interface description.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_ARGUMENT           Argument-content is not supported
 * \retval ::ERR_COM_INTERFACE      The interface to the sensor is faulty.
 */
err_code_t as7050_osal_initialize(const char *p_interface_desc);

/*!
 * \brief Shutdown of the hardware abstraction layer
 *
 * - Closes the interface to the sensor
 * - Deactivates the ENABLE-Pin
 *
 * \note This function must be called for cleanup
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_COM_INTERFACE      The interface to the sensor is faulty
 */
err_code_t as7050_osal_shutdown(void);

/*!
 * \brief Set register values inside the chip
 *
 * \param[in] address               Register address of the chip
 * \param[in] number                Number of register values
 * \param[in] p_values              Pointer to register values
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            If buffer size is unequal 0 and buffer address is zero.
 * \retval ::ERR_DATA_TRANSFER      Data transfer error, like bus error or timeout.
 * \retval ::ERR_PERMISSION         Library was not initialized by ::as7050_osal_initialize
 */
err_code_t as7050_osal_write_registers(uint8_t address, uint8_t number, uint8_t *p_values);

/*!
 * \brief Get register values from the chip
 *
 * \param[in] address               Register address of the chip
 * \param[in] number                Number of register values
 * \param[out] p_values             Pointer to memory, where register values can be saved
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            If buffer size is unequal 0 and buffer address is zero.
 * \retval ::ERR_DATA_TRANSFER      Data transfer error, like bus error or timeout.
 * \retval ::ERR_PERMISSION         Library was not initialized by ::as7050_osal_initialize
 */
err_code_t as7050_osal_read_registers(uint8_t address, uint8_t number, uint8_t *p_values);

/*!
 * \brief Registers and unregisters an interrupt handler.
 *
 * If the function pointer is null, then the old callback will be unregistered.
 *
 * \param[in] callback_function     Pointer to callback function or NULL
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Library was not initialized by ::as7050_osal_initialize
 */
err_code_t as7050_osal_register_int_handler(as7050_osal_interrupt_t callback_function);

/*!
 * \brief Requests an actual time tick (milliseconds) of the system
 *
 * \param[out] p_tick_ms            Pointer to memory, where the actual time tick in milliseconds can be saved
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            If the argument is zero.
 * \retval ::ERR_PERMISSION         Library was not initialized by ::as7050_osal_initialize
 */
err_code_t as7050_osal_get_ms_tick(uint32_t *p_tick_ms);

#ifdef __cplusplus
}
#endif // __cplusplus

/*! @} */

#endif /* __AS7050_OSAL_H__ */
