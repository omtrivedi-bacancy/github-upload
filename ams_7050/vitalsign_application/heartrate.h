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
 *      PROJECT:   AS7000 firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file heartrate.h
 *
 *  \author M. Arpa
 *  \author G. Wagner
 *  \author M. Zimmerman
 *
 *  \brief application specific algorithm for heartrate calculation
 */

#ifndef HEARTRATE_H
#define HEARTRATE_H

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

/* the ADC will deliver a new value every XX milli-seconds 
   NOTE: 1000 must be divisible by the adc period */
#define HR_ADC_PERIOD_IN_MS             ( 50 )     /* a period of 50 milli-seconds equals a frequency of 20 Hz */ 
#define HR_ADC_FREQUENCY_IN_HZ          ( 1000 / (HR_ADC_PERIOD_IN_MS) )
#define HR_ADC_VALUES_PER_SECOND        ( 1000 / (HR_ADC_PERIOD_IN_MS) )

/* Accelerometer-data must be sampled at a fixed interval, within the range of supported sampling-frequencies [10..20]Hz */
/* The accelerometer sampling-frequency is specified in the "accSampleFreq1000" parameter to heartrateInitialise() */
#define HR_ACC_MIN_FREQUENCY_TIMES_1000    10000 // 10.0Hz
#define HR_ACC_MAX_FREQUENCY_TIMES_1000    20000 // 20.0Hz
#define HR_ACC_MAX_VALUES_PER_SECOND       ((uint8_t)(((HR_ACC_MAX_FREQUENCY_TIMES_1000+999)/1000)+1)) // add one to allow for potential clock-skew

/* possible values for the "afeDriverType" parameter to heartrateInitialise() */
#define HR_AFE_DRIVER_TYPE_UNKNOWN           0
#define HR_AFE_DRIVER_TYPE_NORMAL_POWER      1
#define HR_AFE_DRIVER_TYPE_LOW_POWER         2
#define HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER   3

/* possible values for "cChanged" field of "hrAlgorithmInput" indicating current changes */
#define HR_AFE_CURRENT_UNCHANGED           0    
#define HR_AFE_CURRENT_INCREASED           1
#define HR_AFE_CURRENT_DECREASED           2
#define HR_AFE_CURRENT_UNKNOWN             3    

/* possible return codes from function calls */
#define HEARTRATE_ALGORITHM_OK                0
#define HEARTRATE_ALGORITHM_ERR_PARAM         1


/* The heartrate algorithm calculates a heartrate multiplied by this scale factor
   for precision reasons. So the output from the algorithm has to be scaled back
   to the nominal range of the heartrate which is approximately 30 .. 220 Beats
   Per Minute. */
#define HEARTRATE_ALGORITHM_SCALE_FACTOR    (10)

/* Macro to divide "x" by "y", and round to the nearest integer */
#ifdef DIVIDE_WITH_ROUNDING
#undef DIVIDE_WITH_ROUNDING
#endif
#define DIVIDE_WITH_ROUNDING(x,y)  ( ( (x) + ( (y) / 2 ) ) / (y) )

/* Convert algorithm heartrate output from [units=0.1bpm] to [units=1bpm] */
#define HEARTRATE_CONVERT_TO_BPM( outputFromAlgorithm ) \
            ( DIVIDE_WITH_ROUNDING( (outputFromAlgorithm), HEARTRATE_ALGORITHM_SCALE_FACTOR ) )


/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */


/* result from the last calculation of algorithm */
typedef struct _hrAlgorithmOutput
{
    uint16_t heartrate;         /* heartrate [units=0.1bpm]  [815==>81.5bpm] */
    uint8_t signalQuality;      /* the quality of the heartrate signal */
    uint8_t motionFreq;         /* the detected motion-frequency [units=bpm] (0=no-motion) */
} hrAlgorithmOutput;

/* input data for 1 second - just raw data */
/* Each accelerometer axis-value must be scaled to make the LSB = 0.5mg */
/* Each axis-value should have bit-0 set to indicate that the proper scaling has been performed */
typedef struct _hrAlgorithmInput
{
    uint16_t adcValue[ HR_ADC_VALUES_PER_SECOND ]; /* one second of adc values */
    uint8_t cChanged[ HR_ADC_VALUES_PER_SECOND ]; /* one second of information about the current changes (HR_AFE_CURRENT_*) */
    int16_t accX[ HR_ACC_MAX_VALUES_PER_SECOND ]; /* one second of accelerometer X axis data [LSB=0.5mg] */
    int16_t accY[ HR_ACC_MAX_VALUES_PER_SECOND ]; /* one second of accelerometer Y axis data [LSB=0.5mg] */
    int16_t accZ[ HR_ACC_MAX_VALUES_PER_SECOND ]; /* one second of accelerometer Z axis data [LSB=0.5mg] */
    uint8_t numAccSamples; /* the number of new accelerometer 3-axis samples for this second */
} hrAlgorithmInput;

/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
 
/*!
 *****************************************************************************
 * \brief Initialise all internal variables of the heartrate algorithm.
 * 
 * Must be called before calling heartrateInput or heartrateCalculate.
 *
 * \param afeDriverType - informs the algorithm which AFE is being used (HR_AFE_DRIVER_TYPE_*)
 * \param accSampleFreq1000 - acc-data sampling-frequency [units=0.001Hz]
 *                        range=[HR_ACC_MIN_FREQUENCY..HR_ACC_MAX_FREQUENCY]
 * \param crosstalkLevel64 - crosstalk-level [units=64 adc-counts] (78 ==> 4992-adc-counts)
 * \param output - pointer to an instance of the heartrate algorithm output.
 * Is initialised by the function.
 *
 * \return HEARTRATE_ALGORITHM_OK - output pointer was valid, everything initialised
 *         HEARTRATE_ALGORITHM_ERR_PARAM - output is NULL or accSampleFreq1000 is out of range
 *****************************************************************************
 */
int8_t heartrateInitialise(uint8_t afeDriverType, uint16_t accSampleFreq1000, uint8_t crosstalkLevel64, hrAlgorithmOutput *output);

/*!
 *****************************************************************************
 * \brief Call this directly from the isr where you collect the input data.
 * 
 * Must be called for each second before calling heartrateCalculate. Takes
 * the data from the input structure and copies it to its internal buffers.
 *
 * The accelerometer should be setup for +/-16g range with at least 12-bit
 * data per axis.
 * Each accelerometer axis-value must be scaled to make the LSB = 0.5mg, 
 * for consistency to allow support for many acceleromters.
 * Each axis-value should have bit-0 set to indicate to the algorithm that
 * the proper scaling has been performed (this is needed for legacy reasons).
 * Overwriting the LSB does not imact the performance of the algorithm, as
 * 0.5mg is not significant to the algorithm.
 *
 * \param input - pointer to an instance of the heartrate algorithm input
 * for one second.
 *****************************************************************************
 */
void heartrateInput(hrAlgorithmInput *input, uint8_t minHeartrateInput, uint8_t maxHeartrateInput, uint8_t activityInput);

/*!
 *****************************************************************************
 * \brief Calculate the new heartrate based on the data in internal 
 * structures.  Uses data stored internally to calculate the heartrate.
 * The update-interval can be used to adjust for clock skew.
 *
 * \param updateInterval100Us - the actual update-interval, units=100us.
 *                              [nominal=10000 ==> 1.0000 seconds]
 *                              [use "0" if update-interval is not computed]
 * \param output - pointer to an instance of the heartrate algorithm output.
 *
 * \return HEARTRATE_ALGORITHM_OK - output pointer was valid, everything initialised
 *         HEARTRATE_ALGORITHM_ERR_PARAM - output pointer was the 0-pointer
 *****************************************************************************
 */
 int8_t heartrateCalculate(uint16_t updateInterval100Us, hrAlgorithmOutput *output);
 

#endif /* HEARTRATE_H */
