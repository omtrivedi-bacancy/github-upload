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

#ifndef __CLIENT_COMMUNICATION_H__
#define __CLIENT_COMMUNICATION_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file      client_communication.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup client_com_group_rpc
 *
 * \brief This communication handler can be used by the application directly
 *  and has a very easy to use interface.
 *
 * Protocol and periphery will be handled internally.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#include "error_codes.h"
#include "rpc_version.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*!
 * \brief Asynchronous message, which is directly sent by the server.
 *
 * \param[in] target            Spezifies, from which server target the message was sent.
 * \param[in] msg_id            ID of the asynchronous message
 * \param[in] error             Error code of the received message. See ::error_codes
 * \param[in] p_data            Payload data
 * \param[in] size              Size of payload data
 * \param[in] p_callback_data   Additional data for callback
 */
typedef void (*com_client_callback_t)(uint8_t target, uint8_t msg_id, uint8_t error, uint8_t *p_data, uint32_t size,
                                      void *p_callback_data);

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Connect a server and creates a thread which reads cyclic data
 *
 * \param[out] pp_handle        Handle to the device, which is created here.
 * \param[in] p_interface       String with interface description. UART and Ethernet are supported now.
 *                              For further details, please see \ref rpc_osal_sec.
 * \param[in] callback          Pointer to a function which will be called for the given message IDs.
 * \param[in] p_callback_data   Additional data for callback
 * \param[in] p_message_ids     IDs of asynchronous messages which shall be transferred by callback directly.
 * \param[in] num_message_ids   Number of asynchronous message IDs. At maximum 10.
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 * \retval ::ERR_SIZE           Parameter num_message_ids is out of range
 * \retval ::ERR_MEMORY         Memory allocation failed
 */
err_code_t com_client_connect(void **pp_handle, const char *p_interface, const com_client_callback_t callback,
                              void *p_callback_data, uint8_t *p_message_ids, const uint8_t num_message_ids);

/*!
 * \brief Transmits a message
 *
 * \param[in] p_handle                  Handle to the device
 * \param[in] msg_id                    ID of the current message
 * \param[in] target                    Spezifies the target on the server
 * \param[in] p_send_data               Send payload
 * \param[in] send_data_size        	Size of send payload
 * \param[out] p_receive_data           Pointer to memory, where receive data can be saved
 * \param[inout] p_receive_data_size    IN: size of receive data OUT: number of received data
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 * \retval ::ERR_SIZE           Size of receive buffer is too small for incoming message
 * \retval ::ERR_MEMORY         Memory allocation failed
 * \retval ::ERR_PROTOCOL       Get no acknowledge for command
 */
err_code_t com_client_transmit_packet(void *p_handle, const uint8_t msg_id, const uint8_t target, void *p_send_data,
                                      const uint32_t send_data_size, void *p_receive_data,
                                      uint32_t *p_receive_data_size);

/*!
 * \brief Disconnect from server and deletes all allocated resources
 *
 * \param[in] pp_handle         Handle to the device
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 */
err_code_t com_client_disconnect(void **pp_handle);

/*!
 * \brief Returns the version information of the library
 *
 * \returns version information, see \ref rpc_version_t
 */
rpc_version_t com_client_get_version(void);

/*! @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __CLIENT_COMMUNICATION_H__ */
