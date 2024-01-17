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

#ifndef __RAW_APP_H__
#define __RAW_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \file       raw_app.h
 * \authors    ARIT
 * \copyright  ams
 * \addtogroup raw_app_group Raw Data Streaming
 *
 * \brief This module handles all raw data and pack them together to one data package.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "raw_app_typedefs.h"

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

err_code_t raw_app_initialize(void);

err_code_t raw_app_configure(const void *p_config, uint8_t size);

err_code_t raw_app_start(uint32_t signal_sample_period_us, uint8_t sample_size);

err_code_t raw_app_set_input(const uint8_t *p_fifo_data, uint16_t fifo_data_size,
                             const bio_agc_status_t *p_agc_statuses, uint8_t agc_statuses_num,
                             const vs_acc_data_t *p_acc_data, uint8_t acc_data_num, uint8_t ext_event_num,
                             bio_execution_status_t *p_result);

err_code_t raw_app_execute(bio_output_status_t *p_result);

err_code_t raw_app_get_output(void *p_dest, uint16_t *p_size);

err_code_t raw_app_stop(void);

err_code_t raw_app_shutdown(void);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __RAW_APP_H__ */
