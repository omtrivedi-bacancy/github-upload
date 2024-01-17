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
 * \file      pio.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup pio_driver_group
 *
 * \brief Definition for generic GPIO driver interface
 *
 * This interface can be used to initialize the GPIOs for input or output purposes.
 * Interrupt handling for inputs is implemented as well. 
 * The global device parameters and periphery connections can be commited 
 * by the handle-parameter of each function.
 *
 *  @{
 */

#ifndef __PIO_H__
#define __PIO_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! List of supported pin configurations */
enum pio_modes {
    PIO_MODE_INPUT_TRIG_NONE    = 0, /*!< normal input pin */
    PIO_MODE_INPUT_TRIG_RISING  = 1, /*!< input pin with interrupt on rising edge */
    PIO_MODE_INPUT_TRIG_FALLING = 2, /*!< input pin with interrupt on falling edge */
    PIO_MODE_INPUT_TRIG_BOTH    = 3, /*!< input pin with interrupt on both edges */
    PIO_MODE_OUTPUT_TYPE_PP     = 4, /*!< push-pull output pin */
    PIO_MODE_OUTPUT_TYPE_OD     = 5, /*!< Open-Drain-Ouput */
    
    PIO_MODE_NUM                = 6  /*!< Number of supported gpio modes */
};
/*! type-definition of ::pio_modes */
typedef enum pio_modes pio_modes_t;

/*! List of supported pull-up and pull-down configurations */
enum pio_pulls {
    PIO_PULL_NONE   = 0, /*!< No pull up/down configured. */
    PIO_PULL_UP     = 1, /*!< Pull-up configured. */
    PIO_PULL_DOWN   = 2, /*!< Pull-down configured. */
    
    PIO_PULL_NUM    = 3  /*!< Number of supported pull configurations */
};
/*! type-definition of ::pio_pulls */
typedef enum pio_pulls pio_pulls_t;

/*! List of supported pin states */
enum pio_states {
    PIO_STATE_RESET     = 0, /*!< Pin is in reset-state (0). */
    PIO_STATE_SET       = 1, /*!< Pin is set (1). */
    PIO_STATE_TOGGLE    = 2, /*!< Pin shall be toggled (1 -> 0, 0 -> 1) */
    
    PIO_STATE_NUM       = 3  /*!< Number of supported pin states */
};
/*! type-definition of ::pio_states */
typedef enum pio_states pio_states_t;

/*!
 * \brief Callback function for interrupt service routine on input pin
 *
 * \param[in] pio_id      Identification number of the pin
 * \param[in] state       Actual pin state, see ::pio_states_t
 */
typedef void (*pio_callback_t)(uint32_t pio_id, pio_states_t state);

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*!
 * \brief This function must be used to initialize the GPIO pin
 *
 * \note This function must be called at first, otherwise all other functions 
 * return with error code ::ERR_PERMISSION
 *
 * \param[in] p_handle      Handle to the device configuration
 * \param[in] mode          GPIO mode see ::pio_modes
 * \param[in] pull          Configuration of pull-down or pull-up, see ::pio_pulls
 * \param[in] p_callback    This parameter can be used to register a callback 
 *                          function for interrupt handling. If not used, set to NULL.
 *
 * \retval ::ERR_SUCCESS    Function returns without error.
 * \retval ::ERR_POINTER    NULL-pointer detected
 * \retval ::ERR_ARGUMENT   An argument is invalid.
 */
err_code_t pio_init(void* p_handle, pio_modes_t mode, pio_pulls_t pull, pio_callback_t p_callback);

/*!
 * \brief Closes the GPIO pin
 *
 * \param[in] p_handle      Handle to the device configuration
 *
 * \retval ::ERR_SUCCESS    Function returns without error.
 * \retval ::ERR_POINTER    NULL-pointer detected
 * \retval ::ERR_ARGUMENT   An argument is invalid.
 */
err_code_t pio_shutdown(void* p_handle);

/*!
 * \brief Sets the state of an output pin
 *
 * \param[in] p_handle      Handle to the device configuration
 * \param[in] state         New state of the pin, see ::pio_states
 *
 * \retval ::ERR_SUCCESS    Function returns without error.
 * \retval ::ERR_POINTER    NULL-pointer detected
 * \retval ::ERR_ARGUMENT   An argument is invalid.
 * \retval ::ERR_PERMISSION Access to the function is blocked, 
 *                          call ::pio_init first.
 */
err_code_t pio_set(void* p_handle, pio_states_t state);

/*!
 * \brief Gets the state of an input pin
 *
 * \param[in] p_handle      Handle to the device configuration
 * \param[out] p_state      Actual state of the pin, see ::pio_states
 *
 * \retval ::ERR_SUCCESS    Function returns without error.
 * \retval ::ERR_POINTER    NULL-pointer detected
 * \retval ::ERR_ARGUMENT   An argument is invalid.
 * \retval ::ERR_PERMISSION Access to the function is blocked, 
 *                          call ::pio_init first.
 */
err_code_t pio_get(void* p_handle, pio_states_t *p_state);

/*! @} */

#endif /* __PIO_H__ */
