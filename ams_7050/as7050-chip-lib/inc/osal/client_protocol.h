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

#ifndef __CLIENT_PROTOCOL_H__
#define __CLIENT_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file      client_protocol.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup client_prt_group_rpc
 *
 * \brief This is the protocol handler which can be used on the client
 * to pack the message in a serial stream to send to the server application
 * and backwards.
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

/*! This structure wil be used to manage a message */
typedef struct prt_msg {
    uint8_t sync;          /*!< synchronisation byte to detect start of message (not unique and have not be set by the
                              application) */
    uint8_t msg_id;        /*!< unique message ID to identify the payload */
    uint8_t target;        /*!< describes different targets for same message id (more than one I2C periphery) */
    uint8_t error_code;    /*!< error code will be set on message from server to client */
    uint32_t payload_size; /*!< Size of the following payload data */
    void *p_payload;       /*!< Address of the payload data */
} prt_msg_t;

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Creates a serialized stream from message object.
 *
 * \param[in] p_message         Structure with message information. See ::prt_msg_t.
 * \param[out] pp_output_stream Created byte stream, which can be sent by the interface function.
 * \param[out] p_output_size    Size of the byte stream
 *
 * \note The created byte stream must be deleted by ::prt_client_delete_byte_stream.
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 * \retval ::ERR_MEMORY         Memory allocation failed
 */
err_code_t prt_client_create_byte_stream(const prt_msg_t *p_message, void **pp_output_stream, uint32_t *p_output_size);

/*!
 * \brief Deletes an allocated byte stream
 *
 * \param[in] p_output_stream   Pointer to byte stream, which was created by function ::prt_client_create_byte_stream
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 */
err_code_t prt_client_delete_byte_stream(void *p_output_stream);

/*!
 * \brief Extracts a message from serial stream buffer.
 *
 * \param[in] p_data        Serialized byte stream from interface.
 * \param[inout] p_size     IN: Size of p_data, OUT: Size of the used bytes from p_data.
 * \param[out] pp_message   Created message from buffer.
 *
 * \note The created byte stream must be deleted by ::prt_client_delete_byte_stream.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_SIZE               Minimum size of message was not reached
 * \retval ::ERR_SYNCHRONISATION    Can't find synchronisation byte
 * \retval ::ERR_CHECKSUM           Checksum comparision failed
 * \retval ::ERR_MEMORY             Memory allocation failed
 */
err_code_t prt_client_extract_message(const void *p_data, uint32_t *p_size, prt_msg_t **pp_message);

/*!
 * \brief Deletes an allocated message buffer
 *
 * \param[in] p_message  Pointer to message, which was created by function ::prt_client_extract_message
 *
 * \retval ::ERR_SUCCESS        Function returns without error.
 */
err_code_t prt_client_delete_message(prt_msg_t *p_message);

/*!
 * \brief Returns the version information of the library
 *
 * \returns version information, see \ref rpc_version_t
 */
rpc_version_t prt_client_get_version(void);

/*! @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __CLIENT_PROTOCOL_H__ */
