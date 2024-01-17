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

#ifndef __CLIENT_OSAL_INTERFACE_H__
#define __CLIENT_OSAL_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file      client_osal_interface.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup client_osal_group_rpc
 *
 * \brief This module handles more than one communication interfaces:
 * - UART: RS232 or VCP
 * - UDP: Socket-Connection via Ethernet
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#include "client_osal.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Opens the communication interface to the UART component.
 *
 * \param[out] pp_handle        Pointer to handle of the interface.
 * \param[in] p_description     String with interface COM port: 'COM5'
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_SIZE               Size error for description string
 * \retval ::ERR_NOT_SUPPORTED      Interface description is not supported
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be initialized.
 */
err_code_t client_osal_interface_uart_open(osal_interface_t *pp_handle, const char *p_description);

/*!
 * \brief Writes data to the UART interface (blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[in] p_data    Pointer to send buffer
 * \param[in] size      Buffer size
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_COM_INTERFACE      Got communication interface error.
 */
err_code_t client_osal_interface_uart_write(osal_interface_t p_handle, const uint8_t *p_data, const uint32_t size);

/*!
 * \brief Reads data from the UART interface (blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[out] p_data   Memory for receive data.
 * \param[inout] p_size IN: Maximum buffer size, OUT: received buffer size.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_uart_read(osal_interface_t p_handle, uint8_t *p_data, uint32_t *p_size);

/*!
 * \brief Cancels data read method
 *
 * This functions cancels the data read method of UART if it is in blocking mode.
 *
 * \param[in] p_handle  Handle to the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_uart_abort(osal_interface_t p_handle);

/*!
 * \brief Closes the UART communication interface.
 *
 * \param[in] p_handle  Pointer to handle of the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointer is zero.
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be closed.
 */
err_code_t client_osal_interface_uart_close(osal_interface_t p_handle);

/*!
 * \brief Opens the communication interface to the UDP socket.
 *
 * \param[out] pp_handle        Pointer to handle of the interface.
 * \param[in] p_description     String with host destination address: '192.168.0.1:55555'
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_SIZE               Size error for description string
 * \retval ::ERR_NOT_SUPPORTED      Interface description is not supported
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be initialized.
 */
err_code_t client_osal_interface_udp_open(osal_interface_t *pp_handle, const char *p_description);

/*!
 * \brief Writes data to the UDP socket (blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[in] p_data    Pointer to send buffer
 * \param[in] size      Buffer size
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_COM_INTERFACE      Got communication interface error.
 */
err_code_t client_osal_interface_udp_write(osal_interface_t p_handle, const uint8_t *p_data, const uint32_t size);

/*!
 * \brief Reads data from the UDP socket(blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[out] p_data   Memory for receive data.
 * \param[inout] p_size IN: Maximum buffer size, OUT: received buffer size.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_udp_read(osal_interface_t p_handle, uint8_t *p_data, uint32_t *p_size);

/*!
 * \brief Cancels data read method
 *
 * This functions cancels the data read method of the UDP socket if it is in blocking mode.
 *
 * \param[in] p_handle  Handle to the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_udp_abort(osal_interface_t p_handle);

/*!
 * \brief Closes the UDP socket.
 *
 * \param[in] p_handle  Pointer to handle of the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointer is zero.
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be closed.
 */
err_code_t client_osal_interface_udp_close(osal_interface_t p_handle);

/*! @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __CLIENT_OSAL_INTERFACE_H__ */
