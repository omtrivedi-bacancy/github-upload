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

#ifndef __CLIENT_OSAL_H__
#define __CLIENT_OSAL_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file      client_osal.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup client_osal_group_rpc
 *
 * \brief This module encapsulates the dependencies of operating systems to a separate module.
 * Therefore, the module Client Communication, which uses this module, can be used without restrictions.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "error_codes.h"
#include <stdint.h>

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#ifndef PVOID
typedef void *PVOID; /*!< PVOID definition if not exists */
#endif
typedef PVOID osal_interface_t; /*!< Handle definition for interface */
typedef PVOID osal_thread_t;    /*!< Handle definition for thread */
typedef PVOID osal_event_t;     /*!< Handle definition for event */
typedef PVOID osal_mutex_t;     /*!< Handle definition for mutex */

#define OSAL_WAIT_INFINITE 0xFFFFFFFF

#define OSAL_MAX_INTERFACE_STR_LEN 140 /*!< maximum interface string size */

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief Opens the communication interface.
 *
 * \param[out] pp_handle        Pointer to handle of the interface.
 * \param[in] p_description     String with interface name, e.g. 'COM:COM5'
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_SIZE               Size error for description string
 * \retval ::ERR_NOT_SUPPORTED      Interface description is not supported
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be initialized.
 */
err_code_t client_osal_interface_open(osal_interface_t *pp_handle, const char *p_description);

/*!
 * \brief Writes data to the communication interface (blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[in] p_data    Pointer to send buffer
 * \param[in] size      Buffer size
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 * \retval ::ERR_COM_INTERFACE      Got communication interface error.
 */
err_code_t client_osal_interface_write(osal_interface_t p_handle, const uint8_t *p_data, const uint32_t size);

/*!
 * \brief Reads data from communication port (blocking mode).
 *
 * \param[in] p_handle  Handle to the interface.
 * \param[out] p_data   Memory for receive data.
 * \param[inout] p_size IN: Maximum buffer size, OUT: received buffer size.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_read(osal_interface_t p_handle, uint8_t *p_data, uint32_t *p_size);

/*!
 * \brief Cancels data read method
 *
 * This functions cancels the data read method if it is in blocking mode.
 *
 * \param[in] p_handle  Handle to the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero.
 */
err_code_t client_osal_interface_cancel_read(osal_interface_t p_handle);

/*!
 * \brief Closes the communication interface.
 *
 * \param[in] pp_handle  Pointer to handle of the interface.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointer is zero.
 * \retval ::ERR_COM_INTERFACE      Communication interface can't be closed.
 */
err_code_t client_osal_interface_close(osal_interface_t *pp_handle);

/*!
 * \brief Starts a function in a separate thread
 *
 * \param[out] pp_handle        Pointer to the handle of the thread.
 * \param[in] thread_function   Pointer to a function, which shall be started in a separate thread.
 * \param[in] p_param           Pointer to a memory area, which can be accessed in thread function.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_THREAD             Creation of thread failed.
 */
err_code_t client_osal_thread_create(osal_thread_t *pp_handle, const void *thread_function, void *p_param);

/*!
 * \brief Deletes the thread.
 *
 * \param[in] pp_handle  Pointer to the handle of the thread.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_THREAD             Deletion of thread failed.
 */
err_code_t client_osal_thread_delete(osal_thread_t *pp_handle);

/*!
 * \brief Creates a new manual reset event.
 *
 * \param[out] pp_handle    Pointer to the handle of the event structure.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_EVENT              Creation of event failed.
 */
err_code_t client_osal_event_create(osal_event_t *pp_handle);

/*!
 * \brief Waits for a event
 *
 * \param[in] p_handle      Handle to the event structure.
 * \param[in] timeout_ms    Wait timeout in ms
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_TIMEOUT            Run into timeout while waiting for event.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_EVENT              Error while waiting for event.
 */
err_code_t client_osal_event_wait(osal_event_t p_handle, const uint32_t timeout_ms);

/*!
 * \brief Sets an event
 *
 * \param[in] p_handle  Handle to the event structure.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_EVENT              Error while set event.
 */
err_code_t client_osal_event_set(osal_event_t p_handle);

/*!
 * \brief Resets an event.
 *
 * \param[in] p_handle  Handle to the event structure
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_EVENT              Error while reset event.
 */
err_code_t client_osal_event_reset(osal_event_t p_handle);

/*!
 * \brief Deletes an event.
 *
 * \param[in] pp_handle  Pointer to the handle of the event structure
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_EVENT              Error while closing event.
 */
err_code_t client_osal_event_delete(osal_event_t *pp_handle);

/*!
 * \brief Creates a new mutex for blocking sections.
 *
 * \param[out] pp_handle            Pointer to the handle of the mutex structure
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_PERMISSION         Pointer in pp_handle is not zero.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_MUTEX              Creation of mutex failed.
 */
err_code_t client_osal_mutex_create(osal_mutex_t *pp_handle);

/*!
 * \brief Lock a mutex
 *
 * \param[in] p_handle              Handle to the mutex structure.
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_MUTEX              Error while waiting for mutex.
 */
err_code_t client_osal_mutex_lock(osal_mutex_t p_handle);

/*!
 * \brief Unlock a mutex.
 *
 * \param[in] p_handle              Handle to the mutex structure
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_MUTEX              Error while releasing mutex.
 */
err_code_t client_osal_mutex_unlock(osal_mutex_t p_handle);

/*!
 * \brief Deletes a mutex.
 *
 * \param[in] pp_handle             Pointer to the handle of the mutex structure
 *
 * \retval ::ERR_SUCCESS            Function returns without error.
 * \retval ::ERR_POINTER            Pointers are zero
 * \retval ::ERR_MUTEX              Error while closing mutex.
 */
err_code_t client_osal_mutex_delete(osal_mutex_t *pp_handle);

/*! @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __CLIENT_OSAL_H__ */
