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

#ifndef __RAW_APP_TYPEDEFS_H__
#define __RAW_APP_TYPEDEFS_H__

/*!
 * \file       raw_app_typedefs.h
 * \authors    PKRN
 * \copyright  ams
 * \addtogroup raw_app_group Raw Data Streaming
 *
 * @{
 * The output of the raw app includes FIFO data as received from the AS7050 device, accelerometer samples, and Automatic
 * Gain Correction (AGC) status information. Accelerometer samples are only included in the output when enabled in the
 * ::raw_app_configuration_t. AGC status information is only transmitted when the status information has changed since
 * the last transmission.
 *
 * The output has the following format:
 *
 * \htmlonly
 *      0             7 8            15 16           23
 *     ┌───────────────┬───────────────┬───────────────┐
 *     │  FIFO Sample  │ Accelerometer │  AGC Present  │
 *     │     Count     │ Sample Count  │               │
 *     ├───────────────┴───────────────┴───────────────┤ ────────┐
 *     │                  FIFO Sample                  │         │ Repeats FIFO Sample Count times
 *     │                                               │         │
 *     ├───────────────────────────────┬───────────────┤ ───┐ ───┘
 *     │         Accelerometer         │ Accelerometer │    │
 *     │         Sample X-Axis         │ Sample Y-Axis │    │ Accelerometer Sample
 *     ├───────────────┬───────────────┴───────────────┤    │
 *     │ Accelerometer │         Accelerometer         │    │ Repeats Accelerometer Sample Count times
 *     │ Sample Y-Axis │         Sample Z-Axis         │    │
 *     ├───────────────┴───────────────────────────────┤ ───┘ ───┐
 *     │                                               │         │
 *     │                                               │         │
 *     ├─                                             ─┤         │
 *     │          AGC Status                           │         │ Only present if AGC Present is non-zero
 *     │          Information                          │         │
 *     ├─                              ┌───────────────┘         │
 *     │                               │                         │
 *     │                               │                         │
 *     └───────────────────────────────┘ ────────────────────────┘
 * \endhtmlonly
 *
 * \image latex raw_app_output.pdf "Raw App Output" width=\textwidth
 *
 * The header fields (FIFO Sample Count, Accelerometer Sample Count, AGC Present) each have a size of 1 byte. Each FIFO
 * Sample has a size of 3 bytes. The FIFO Sample field repeats FIFO Sample Count times. An Accelerometer Sample has a
 * total size of 6 bytes. The Accelerometer Sample field repeats Accelerometer Sample Count times. The AGC status
 * information has a size of 8 bytes and is only present if the AGC Present field has a non-zero value.
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "bio_common.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! Configuration structure of the raw app. */
typedef struct {
    uint8_t include_acc; /*!< A non-zero value indicates that accelerometer data shall be included in the output
                              data. */
} raw_app_configuration_t;

/*! @} */

/*******************************************************************************
 *                            TYPE SIZE VALIDATION                            *
 ******************************************************************************/

M_STATIC_ASSERT_TYPE_SIZE(raw_app_configuration_t, 1);

#endif /* __RAW_APP_TYPEDEFS_H__ */
