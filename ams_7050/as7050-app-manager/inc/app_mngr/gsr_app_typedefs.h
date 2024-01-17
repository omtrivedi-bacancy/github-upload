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

#ifndef __GSR_APP_TYPEDEFS_H__
#define __GSR_APP_TYPEDEFS_H__

/*!
 * \file       gsr_app_typedefs.h
 * \authors    ARIT
 * \copyright  ams
 * \addtogroup gsr_app_group Galvanic Skin Resistance
 *
 * @{
 * The output of the GSR app includes the measured Galvanic Skin Resistance value. For correct GSR measurement, the DAC
 * reference value of the used AS7050 device needs to be determined using the corresponding function of the AS7050 Chip
 * Library. The acquired DAC reference value needs to be provided to the GSR app via the app configuration.
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "bio_common.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! The signal data type required by the GSR app. */
#define GSR_APP_SIGNAL_DATA_TYPE BIO_SIGNAL_SAMPLES_TYPE_U32

/*! Configuration structure of the GSR app. */
typedef struct {
    uint32_t dac_ref; /*!< DAC reference value which must be determined before. */
} gsr_app_configuration_t;

/*! Describes the output of the GSR app. */
typedef struct {
    uint32_t resistor; /*!< Contains the resistance value in Ohm. The output is invalid if the value of this field is
                            0xFFFFFFFF. */
} gsr_app_output_t;

/*! Sensor signals provided to the GSR app. */
enum gsr_app_signal {
    GSR_APP_SIGNAL_ECG = 0, /*!< ECG signal. */

    GSR_APP_SIGNAL_NUM /*!< Number of sensor signals provided to the GSR app. */
};

/*! @} */

/*******************************************************************************
 *                            TYPE SIZE VALIDATION                            *
 ******************************************************************************/

M_STATIC_ASSERT_TYPE_SIZE(gsr_app_configuration_t, 4);
M_STATIC_ASSERT_TYPE_SIZE(gsr_app_output_t, 4);

#endif /* __GSR_APP_TYPEDEFS_H__ */
