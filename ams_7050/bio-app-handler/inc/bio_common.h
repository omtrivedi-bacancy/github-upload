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

#ifndef __BIO_COMMON_H__
#define __BIO_COMMON_H__

/*!
 * \file       bio_common.h
 * \authors    ARIT, PKRN
 * \copyright  ams
 * \addtogroup bio_app_group
 *
 * \brief This module contains definitions used by all bio apps.
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

#include "error_codes.h"
#include "std_inc.h"
#include "vital_signs_accelerometer.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! Indicates whether the bio app is executable. */
enum bio_execution_status {
    BIO_EXECUTION_STATUS_NOT_EXECUTABLE = 0, /*!< The bio app has not yet received sufficient amounts of data to be
                                                  executed. */
    BIO_EXECUTION_STATUS_EXECUTABLE,         /*!< The execute function of the bio app can be called as it has received
                                                  sufficient data. */
};

/*! Type definition for ::bio_execution_status. */
typedef uint8_t bio_execution_status_t;

/*! Indicates whether output data is available from the bio app. */
enum bio_output_status {
    BIO_OUTPUT_STATUS_DATA_UNAVAILABLE = 0, /*!< No output data is available. */
    BIO_OUTPUT_STATUS_DATA_AVAILABLE,       /*!< The get output function of the bio app can be called as output data is
                                                 available. */
};

/*! Type definition for ::bio_output_status. */
typedef uint8_t bio_output_status_t;

/*! Indicates the type of sensor signal samples. */
enum bio_signal_samples_type {
    BIO_SIGNAL_SAMPLES_TYPE_U16 = 0, /*!< Data is provided as an unsigned 16-bit integer. */
    BIO_SIGNAL_SAMPLES_TYPE_U32,     /*!< Data is provided as an unsigned 32-bit integer. */
};

/*! Type definition for ::bio_signal_samples_type. */
typedef uint8_t bio_signal_samples_type_t;

/*! Contains a pointer to the samples of sensor signals and the corresponding sample count. */
typedef struct {
    union {
        const uint16_t *p_u16; /*!< Points to unsigned 16-bit signal samples. Use this pointer when the data
                                    type of the samples is ::BIO_SIGNAL_SAMPLES_TYPE_U16. */
        const uint32_t *p_u32; /*!< Points to unsigned 32-bit signal samples. Use this pointer when the data
                                    type of the samples is ::BIO_SIGNAL_SAMPLES_TYPE_U32. */
    } samples;          /*!< Points to the start of an array containing the samples of this sensor signal. This union
                             contains pointers to different types. Check the ::bio_signal_samples_type_t used by the
                             app and use the corresponding union member. */
    unsigned int count; /*!< Contains the number of items in the signal samples array. */
} bio_signal_samples_t;

// TODO: Move AGC definitions to AGC project

enum bio_agc_change {
    BIO_AGC_CHANGE_UNCHANGED = 0,
    BIO_AGC_CHANGE_INCREASED = 1,
    BIO_AGC_CHANGE_DECREASED = 2,
    BIO_AGC_CHANGE_AT_MIN = 3,
    BIO_AGC_CHANGE_AT_MAX = 4,
    BIO_AGC_CHANGE_UNKNOWN = 5,
};

typedef uint8_t bio_agc_change_t;

typedef struct {
    bio_agc_change_t led_change;
    uint8_t led_current;
    bio_agc_change_t pd_offset_change;
    uint8_t pd_offset_current;
} bio_agc_status_t;

/*! @} */

#endif /* __BIO_COMMON_H__ */
