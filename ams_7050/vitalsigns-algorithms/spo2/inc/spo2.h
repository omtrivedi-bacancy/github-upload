/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   AS70xx SpO2 algorithm
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file spo2.h
 *
 *  \author M. Zimmerman
 *
 *  \brief application specific algorithm for SpO2 calculation
 */

#ifndef SPO2_H
#define SPO2_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */

#include <stdint.h>

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

/* the ADC will deliver a new value every SPO2_ADC_PERIOD_IN_MS milliseconds
   NOTE: 1000 must be divisible by the adc period */
#define SPO2_ADC_PERIOD_IN_MS 50
#define SPO2_ADC_FREQUENCY_IN_HZ (1000 / SPO2_ADC_PERIOD_IN_MS)
#define SPO2_ADC_VALUES_PER_SECOND (1000 / SPO2_ADC_PERIOD_IN_MS)

/* possible return codes from function calls */
#define SPO2_ALGORITHM_OK 0
#define SPO2_ALGORITHM_PARAMETER_ERROR 1

/* possible values for the "afeDriverType" parameter to spo2Initialise() */
#define SPO2_AFE_DRIVER_TYPE_UNKNOWN 0
#define SPO2_AFE_DRIVER_TYPE_AS7000 1
#define SPO2_AFE_DRIVER_TYPE_AS702X_AC_DC 2

/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */

/* calibration values for SpO2 algorithm correction */
/* a,b,c are coefficients of quadratic function used to correct SpO2 algorithm results*/
typedef struct {
    uint16_t a; /* quadratic correction coefficient 1 */
    uint16_t b; /* quadratic correction coefficient 2 */
    uint16_t c; /* quadratic correction coefficient 3 */
    uint16_t dcCompRed;
    uint16_t dcCompIR;
} spo2AlgorithmCalibration_t;

/* data for one sample for one LED */
typedef struct {
    uint16_t absValue;      /* absolute-ADC-DC value */
    uint16_t acValue : 14;  /* AC value */
    uint16_t ledChange : 2; /* LED-change info */
} spo2LedData_t;

/* input data for 1 second - just raw data */
/* AS7000: The redAbs/infraredAbs arrays are used for absolute-ADC data; redAc/infraredAc only used for LED-change info
 */
/* AS702X_AC_DC: The redAbs/infraredAbs are "DC" data; redAc/infraredAc arrays are "AC" data and LED-change info */
/* Each accelerometer axis-value must be scaled to make the LSB = 0.5mg */
typedef struct {
    spo2LedData_t red[SPO2_ADC_VALUES_PER_SECOND];      /* one second of red data */
    spo2LedData_t infrared[SPO2_ADC_VALUES_PER_SECOND]; /* one second of infrared data */
    uint16_t ambient[SPO2_ADC_VALUES_PER_SECOND];       /* one second of ambient light data */
    uint8_t pdOffsetRed[SPO2_ADC_VALUES_PER_SECOND];    /* PD Offset values of the red channel for each sample */
    uint8_t pdOffsetIR[SPO2_ADC_VALUES_PER_SECOND];     /* PD Offset values of the infrared channel for each sample */
} spo2AlgorithmInput_t;

/* result from the last SpO2-algorithm calculation */
typedef struct {
    uint8_t status;        /* 0 for valid SpO2, 1 for no result */
    uint8_t signalQuality; /* quality of the signal/result */
    uint16_t spo2;         /* SpO2 percent * 100 [9812==>98.12%] */
    uint16_t heartrate;    /* heartrate bpm * 10 [815==>81.5bpm] */
    uint16_t pi;           /* PI=Perfusion-Index * 100 [123==>1.23%] */
    uint16_t averageR;     /* average-R value * 10000 [4376==>0.4376] */
    uint16_t acRed;        /* AC for Red-LED */
    uint16_t dcRed;        /* DC for Red-LED */
    uint16_t acInfrared;   /* AC for Infrared-LED */
    uint16_t dcInfrared;   /* DC for Infrared-LED */
} spo2AlgorithmOutput_t;

/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

/*!
 *****************************************************************************
 * \brief Initialise all internal variables of the SpO2 algorithm.
 *
 * Must be called before calling spo2Calculate.
 *
 * \return SPO2_ALGORITHM_OK - everything initialised
 *****************************************************************************
 */
int8_t spo2Initialise(uint8_t afeDriverType, spo2AlgorithmOutput_t *output);

/*!
 *****************************************************************************
 * \brief Apply received calibration values for the SpO2 algorithm.
 *
 * Must be called before calling spo2Calculate if calibration is to be applied.
 *
 * \return SPO2_ALGORITHM_OK - everything initialised
 *****************************************************************************
 */
int8_t spo2UpdateCalibration(spo2AlgorithmCalibration_t *calib);

/*!
 *****************************************************************************
 * \brief Copy all the data into the algorithm handler and prepare it for calculation.
 *
 * Must be called for each second before calling spo2Calculate. Takes
 * the data from the input structure and copies it to its internal buffers.
 *
 * \param input - pointer to an instance of the SpO2 algorithm input
 * for one second.
 *****************************************************************************
 */
void spo2Input(spo2AlgorithmInput_t *input);

/*!
 *****************************************************************************
 * \brief Calculate the new SpO2 based on the data in internal structures.
 *
 * Uses the data (previously stored internally) to calculate the SpO2.
 *
 * \param output - pointer to an instance of the SpO2 algorithm output.
 *
 * \return SPO2_ALGORITHM_OK - output pointer was valid, processing completed
 *         SPO2_ALGORITHM_ERR_PARAM - output pointer was the 0-pointer
 *****************************************************************************
 */
int8_t spo2Calculate(spo2AlgorithmOutput_t *output);

#endif /* SPO2_H */
