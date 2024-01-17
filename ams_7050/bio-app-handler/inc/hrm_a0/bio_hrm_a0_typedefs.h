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

#ifndef __BIO_HRM_A0_TYPEDEFS_H__
#define __BIO_HRM_A0_TYPEDEFS_H__

/*!
 * \file       bio_hrm_a0_typedefs.h
 * \authors    ARIT, PKRN
 * \copyright  ams
 * \addtogroup bio_hrm_a0_group ams HRM Algorithm
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "bio_common.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! The signal data type required by the SpO2 bio app. */
#define BIO_HRM_A0_SIGNAL_DATA_TYPE BIO_SIGNAL_SAMPLES_TYPE_U16

/*! Maximum number of PRV data items in a single ::bio_hrm_a0_output_t structure. */
#define PRV_DATA_NUM 5

/*! Contains the configuration options of the HRM bio app. */
typedef struct {
    uint8_t enable_prv; /*!< PRV data is included in the output when set to TRUE. */
} bio_hrm_a0_configuration_t;

/*! Describes the output of the HRM bio app. */
typedef struct {
    uint16_t heart_rate;           /*!< Contains the heart rate. The unit is 0.1 bpm. */
    uint8_t quality;               /*!< Contains information about the quality of the heart rate signal. A value of
                                        zero means best quality. */
    uint8_t motion_frequency;      /*!< Contains the detected motion frequency. The unit is bpm. A value of zero means
                                        that no motion has been detected. */
    uint16_t prv_ms[PRV_DATA_NUM]; /*!< Contains PRV data. The unit is milliseconds. */
    uint8_t prv_ms_num;            /*!< Contains the number of prv_ms fields that are used. */
    uint8_t reserved;              /*!< Padding byte. */
} bio_hrm_a0_output_t;

/*! Sensor signals provided to the HRM bio app. */
enum bio_hrm_a0_signal {
    BIO_HRM_A0_SIGNAL_PPG = 0, /*!< PPG signal. */

    BIO_HRM_A0_SIGNAL_NUM /*!< Number of sensor signals provided to the bio app. */
};

/*! @} */

/******************************************************************************
 *                            TYPE SIZE VALIDATION                            *
 ******************************************************************************/

M_STATIC_ASSERT_TYPE_SIZE(bio_hrm_a0_configuration_t, 1);
M_STATIC_ASSERT_TYPE_SIZE(bio_hrm_a0_output_t, 16);

#endif /* __BIO_HRM_A0_TYPEDEFS_H__ */
