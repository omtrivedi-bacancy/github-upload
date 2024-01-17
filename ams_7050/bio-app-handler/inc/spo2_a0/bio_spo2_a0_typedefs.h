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

#ifndef __BIO_SPO2_A0_TYPEDEFS_H__
#define __BIO_SPO2_A0_TYPEDEFS_H__

/*!
 * \file       bio_spo2_a0_typedefs.h
 * \authors    ARIT, PKRN
 * \copyright  ams
 * \addtogroup bio_spo2_a0_group ams SpO2 Algorithm
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
#define BIO_SPO2_A0_SIGNAL_DATA_TYPE BIO_SIGNAL_SAMPLES_TYPE_U16

/*! Contains the configuration options of the SpO2 bio app. */
typedef struct {
    uint16_t a;           /*!< Quadratic correction coefficient a. */
    uint16_t b;           /*!< Quadratic correction coefficient b. */
    uint16_t c;           /*!< Quadratic correction coefficient c. */
    uint16_t dc_comp_red; /*!< DC compensation for the red signal. */
    uint16_t dc_comp_ir;  /*!< DC compensation for the infrared signal. */
} bio_spo2_a0_configuration_t;

/*! Describes the output of the SpO2 bio app. */
typedef struct {
    uint8_t status;       /*!< The value of the field is zero when the structure contains valid SpO2 data. It is one
                               when no result is present in the data. */
    uint8_t quality;      /*!< Contains the quality of the signal. The unit is 1%. */
    uint16_t spo2;        /*!< Contains the SpO2 measurement. The unit is 0.01%. */
    uint16_t heart_rate;  /*!< Contains the heart rate. The unit is 0.1 bpm. */
    uint16_t pi;          /*!< Contains the Perfusion Index measurement. The unit is 0.01%. */
    uint16_t average_r;   /*!< Contains the Average R value. The unit is 10000. */
    uint16_t ac_comp_red; /*!< Contains the AC component of the red PPG signal. It is used for calibration purposes. */
    uint16_t dc_comp_red; /*!< Contains the DC component of the red PPG signal. It is used for calibration purposes. */
    uint16_t ac_comp_ir;  /*!< Contains the AC component of the infrared PPG signal. It is used for calibration
                               purposes. */
    uint16_t dc_comp_ir;  /*!< Contains the DC component of the infrared PPG signal. It is used for calibration
                               purposes. */
} bio_spo2_a0_output_t;

/*! Sensor signals provided to the SpO2 bio app. */
enum bio_spo2_a0_signal {
    BIO_SPO2_A0_SIGNAL_PPG_RED = 0, /*!< Red PPG signal. */
    BIO_SPO2_A0_SIGNAL_PPG_IR = 1,  /*!< Infrared PPG signal. */
    BIO_SPO2_A0_SIGNAL_AMBIENT = 2, /*!< Ambient light signal. */

    BIO_SPO2_A0_SIGNAL_NUM /*!< Number of sensor signals provided to the bio app. */
};

/*! @} */

/******************************************************************************
 *                            TYPE SIZE VALIDATION                            *
 ******************************************************************************/

M_STATIC_ASSERT_TYPE_SIZE(bio_spo2_a0_configuration_t, 10);
M_STATIC_ASSERT_TYPE_SIZE(bio_spo2_a0_output_t, 18);

#endif /* __BIO_SPO2_A0_TYPEDEFS_H__ */
