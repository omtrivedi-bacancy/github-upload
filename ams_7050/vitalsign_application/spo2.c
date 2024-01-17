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

/*! \file spo2.c
 *
 *  \author M. Zimmerman
 *
 *  \brief implementation for SpO2 algorithm.
 *
 *
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include <string.h>

#include "adcData.h"
#include "spo2.h"
#include "spo2_private.h"
#include "stdio.h"

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */
#define SPO2_INTERVAL_BUFFER_SIZE 15

/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */

static spo2AlgorithmCalibration_t g_calib = {0, 0, 0, 0, 0};
static uint32_t g_spo2SecondCounter = 0;
static uint8_t g_spo2AfeDriverType = SPO2_AFE_DRIVER_TYPE_UNKNOWN;
static AdcData_t g_spo2RedAdcData;
static AdcData_t g_spo2InfraredAdcData;
static uint16_t g_ambient[SPO2_INTERVAL_BUFFER_SIZE];
static uint8_t g_lastPdOffset = 255;
static spo2Interval_t g_spo2Interval[SPO2_INTERVAL_BUFFER_SIZE];
static uint8_t g_spo2BuffSize;

/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

void array_sort(uint16_t *array, uint8_t n);
uint16_t find_median(uint16_t array[], uint8_t n);

///////////////////////////////////////////////////////////////////////////////
// spo2UpdateCalibration
///////////////////////////////////////////////////////////////////////////////
int8_t spo2UpdateCalibration(spo2AlgorithmCalibration_t *newCalib)
{
    g_calib.a = newCalib->a;
    g_calib.b = newCalib->b;
    g_calib.c = newCalib->c;
    g_calib.dcCompRed = newCalib->dcCompRed;
    g_calib.dcCompIR = newCalib->dcCompIR;

    return SPO2_ALGORITHM_OK;
}

///////////////////////////////////////////////////////////////////////////////
// spo2Initialise
///////////////////////////////////////////////////////////////////////////////
int8_t spo2Initialise(uint8_t afeDriverType, spo2AlgorithmOutput_t *output)
{
    int16_t i;

    if (!output) {
        return SPO2_ALGORITHM_PARAMETER_ERROR;
    }

    g_spo2SecondCounter = 0;
    g_spo2AfeDriverType = afeDriverType;
    adcDataSetAfeDriverType(afeDriverType);

    output->status = 1; // start with "no-result"
    output->signalQuality = 0;
    output->spo2 = 0;
    output->heartrate = 0;
    output->pi = 0;
    output->averageR = 0;
    output->acRed = 0;
    output->dcRed = 0;
    output->acInfrared = 0;
    output->dcInfrared = 0;

    // algorithm init
    adcDataInit(&g_spo2RedAdcData, 1);
    adcDataInit(&g_spo2InfraredAdcData, 0);
    for (i = 0; i < SPO2_INTERVAL_BUFFER_SIZE; i++) {
        g_ambient[i] = 0;
        g_spo2Interval[i].interval = 0;
        g_spo2Interval[i].endTime = 0;
        g_spo2Interval[i].acRed = 0;
        g_spo2Interval[i].dcRed = 0;
        g_spo2Interval[i].acInfrared = 0;
        g_spo2Interval[i].dcInfrared = 0;
        g_spo2Interval[i].dcAmbient = 0;
    }

    memset(&g_calib, 0, sizeof(spo2AlgorithmCalibration_t));
    g_lastPdOffset = 255;
    g_spo2BuffSize = 0;

    return SPO2_ALGORITHM_OK;
}

///////////////////////////////////////////////////////////////////////////////
// spo2Input
///////////////////////////////////////////////////////////////////////////////
void spo2Input(spo2AlgorithmInput_t *input)
{
    int i;

    if (g_lastPdOffset == 255) {
        g_lastPdOffset = input->pdOffset[0];
    }

    // copy new data to algorithm-internal buffer
    adcDataBufferNewSecondOfData(&g_spo2RedAdcData, input->red);
    adcDataBufferNewSecondOfData(&g_spo2InfraredAdcData, input->infrared);

    for (i = 0; i < SPO2_INTERVAL_BUFFER_SIZE; i++) {
        g_ambient[i] = input->ambient[i];

        if (g_lastPdOffset != input->pdOffset[i]) {
            g_lastPdOffset = input->pdOffset[i];
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// spo2RemoveOldestBufferedResult
///////////////////////////////////////////////////////////////////////////////
static void spo2RemoveOldestBufferedResult(void)
{
    int16_t i;

    g_spo2BuffSize--;
    for (i = 0; i < g_spo2BuffSize; i++) {
        g_spo2Interval[i] = g_spo2Interval[i + 1];
    }
    g_spo2Interval[g_spo2BuffSize].interval = 0;
    g_spo2Interval[g_spo2BuffSize].endTime = 0;
    g_spo2Interval[g_spo2BuffSize].acRed = 0;
    g_spo2Interval[g_spo2BuffSize].dcRed = 0;
    g_spo2Interval[g_spo2BuffSize].acInfrared = 0;
    g_spo2Interval[g_spo2BuffSize].dcInfrared = 0;
    g_spo2Interval[g_spo2BuffSize].dcAmbient = 0;
}

///////////////////////////////////////////////////////////////////////////////
// spo2Calculate
///////////////////////////////////////////////////////////////////////////////
int8_t spo2Calculate(spo2AlgorithmOutput_t *output)
{
    int16_t i, j;
    int8_t redIntervalFound;
    int8_t infraredIntervalFound;
    uint16_t acRed, dcRed;
    uint16_t acInfrared, dcInfrared;
    uint16_t dcAmbient;
    uint32_t R;
    uint8_t intervalInSamples;
    int8_t intervalEndInSamples;
    uint16_t intervalSum;
    uint16_t quality;
    uint32_t dcIRAdj;
    uint32_t dcRedAdj;
    uint16_t medianWorkBuffer[SPO2_INTERVAL_BUFFER_SIZE];

    g_spo2SecondCounter++;

    // shift out old buffered results before processing new data
    if (g_spo2SecondCounter >= SPO2_INTERVAL_BUFFER_SIZE) {
        while (g_spo2BuffSize > 0) {
            if (g_spo2Interval[0].endTime >= (g_spo2SecondCounter - SPO2_INTERVAL_BUFFER_SIZE))
                break;
            spo2RemoveOldestBufferedResult();
        }
    }

    // process new data to generate new K-results
    i = 0;
    do {
        i++; // used to limit number of iterations
        redIntervalFound = adcDataFindInterval(&g_spo2RedAdcData);
        infraredIntervalFound = adcDataFindInterval(&g_spo2InfraredAdcData);
        if (redIntervalFound && infraredIntervalFound) {
            // check for mismatched red/infrared intervals
            if ((g_spo2RedAdcData.firstMaxPeakIdx < (g_spo2InfraredAdcData.firstMaxPeakIdx - 2)) ||
                (g_spo2RedAdcData.minPeakIdx < (g_spo2InfraredAdcData.minPeakIdx - 4)) ||
                (g_spo2RedAdcData.secondMaxPeakIdx < (g_spo2InfraredAdcData.secondMaxPeakIdx - 2))) {
                // red is too much before infrared -- remove red
                spo2DebugIntervalRemoved(g_spo2RedAdcData.red, g_spo2SecondCounter, g_spo2RedAdcData.secondMaxPeakIdx);
                adcDataCalculateAndRemoveInterval(&g_spo2RedAdcData, &acRed, &dcRed);
                continue;
            }
            if ((g_spo2RedAdcData.firstMaxPeakIdx > (g_spo2InfraredAdcData.firstMaxPeakIdx + 2)) ||
                (g_spo2RedAdcData.minPeakIdx > (g_spo2InfraredAdcData.minPeakIdx + 4)) ||
                (g_spo2RedAdcData.secondMaxPeakIdx > (g_spo2InfraredAdcData.secondMaxPeakIdx + 2))) {
                // infrared is too much before red -- remove infrared
                spo2DebugIntervalRemoved(g_spo2InfraredAdcData.red, g_spo2SecondCounter,
                                         g_spo2InfraredAdcData.secondMaxPeakIdx);
                adcDataCalculateAndRemoveInterval(&g_spo2InfraredAdcData, &acInfrared, &dcInfrared);
                continue;
            }

            // calculate R for this interval
            intervalInSamples = (uint8_t)g_spo2InfraredAdcData.secondMaxPeakIdx - g_spo2InfraredAdcData.firstMaxPeakIdx;
            intervalEndInSamples = g_spo2InfraredAdcData.secondMaxPeakIdx;
            redIntervalFound = adcDataCalculateAndRemoveInterval(&g_spo2RedAdcData, &acRed, &dcRed);
            infraredIntervalFound = adcDataCalculateAndRemoveInterval(&g_spo2InfraredAdcData, &acInfrared, &dcInfrared);
            if ((!redIntervalFound || !infraredIntervalFound) || (acInfrared == 0) || (dcRed == 0)) {
                // one of the matched intervals is invalid -- do not use results
                spo2DebugIntervalRemoved(g_spo2InfraredAdcData.red, g_spo2SecondCounter,
                                         g_spo2InfraredAdcData.firstMaxPeakIdx);
                spo2DebugIntervalRemoved(g_spo2InfraredAdcData.red, g_spo2SecondCounter,
                                         g_spo2InfraredAdcData.firstMaxPeakIdx);
                continue;
            }

            dcAmbient = 0;

            for (j = g_spo2InfraredAdcData.firstMaxPeakIdx + 1; j <= g_spo2InfraredAdcData.secondMaxPeakIdx; j++) {
                dcAmbient += g_ambient[j];
            }
            dcAmbient /= intervalInSamples;

            spo2DebugIntervalResult(g_spo2SecondCounter, intervalEndInSamples, 0, intervalInSamples);
            // add new interval to buffer
            if (g_spo2BuffSize >= SPO2_INTERVAL_BUFFER_SIZE) {
                spo2RemoveOldestBufferedResult();
            }
            g_spo2Interval[g_spo2BuffSize].interval = intervalInSamples;
            if (intervalEndInSamples >= ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND))
                g_spo2Interval[g_spo2BuffSize].endTime = g_spo2SecondCounter;
            else
                g_spo2Interval[g_spo2BuffSize].endTime = g_spo2SecondCounter - 1;
            g_spo2Interval[g_spo2BuffSize].acRed = acRed;
            g_spo2Interval[g_spo2BuffSize].dcRed = dcRed;
            g_spo2Interval[g_spo2BuffSize].acInfrared = acInfrared;
            g_spo2Interval[g_spo2BuffSize].dcInfrared = dcInfrared;
            g_spo2Interval[g_spo2BuffSize].dcAmbient = dcAmbient;
            g_spo2BuffSize++;
        }

    } while ((redIntervalFound && infraredIntervalFound) && (i < 4));

    // return the new results
    if (g_spo2BuffSize > 0) {
        output->status = 0; // result
        // compute average for each AC/DC value
        for (i = 0; i < g_spo2BuffSize; i++)
            medianWorkBuffer[i] = g_spo2Interval[i].acRed;
        output->acRed = find_median(medianWorkBuffer, g_spo2BuffSize);
        for (i = 0; i < g_spo2BuffSize; i++)
            medianWorkBuffer[i] = g_spo2Interval[i].dcRed;
        output->dcRed = find_median(medianWorkBuffer, g_spo2BuffSize);
        for (i = 0; i < g_spo2BuffSize; i++)
            medianWorkBuffer[i] = g_spo2Interval[i].acInfrared;
        output->acInfrared = find_median(medianWorkBuffer, g_spo2BuffSize);
        for (i = 0; i < g_spo2BuffSize; i++)
            medianWorkBuffer[i] = g_spo2Interval[i].dcInfrared;
        output->dcInfrared = find_median(medianWorkBuffer, g_spo2BuffSize);
        for (i = 0; i < g_spo2BuffSize; i++)
            medianWorkBuffer[i] = g_spo2Interval[i].dcAmbient;
        dcAmbient = find_median(medianWorkBuffer, g_spo2BuffSize);

        // compute the R based on average values

        // adjusted dc values for pd offset correction factors
        dcIRAdj = (uint32_t)output->dcInfrared + ((uint32_t)g_lastPdOffset * g_calib.dcCompIR) - dcAmbient;
        dcRedAdj = (uint32_t)output->dcRed + ((uint32_t)g_lastPdOffset * g_calib.dcCompRed) - dcAmbient;

        if ((dcRedAdj != 0) && (output->acInfrared != 0)) {
            R = (((dcIRAdj * 10000) / output->acInfrared) * output->acRed) / dcRedAdj;
        } else {
            // division by 0 --> set max R value
            R = 40000;
        }

        output->averageR = (uint16_t)R;
        // compute PI from the average IR AC/DC ratio
        if (output->dcInfrared == 0)
            output->pi = 0; // set to 0% for unexpected DC=0
        else if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS702X_AC_DC)
            output->pi =
                (uint16_t)(((uint32_t)output->acInfrared * 10000 / 1) / dcIRAdj); // adjust for gain (if needed)
        else // if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS7000)
            output->pi = (uint16_t)(((uint32_t)output->acInfrared * 10000) / dcIRAdj);
        // compute the SpO2 from the average-R: 110 - 25*R
        output->spo2 = (uint16_t)((((uint32_t)g_calib.c * 10000) - (uint32_t)g_calib.b * R -
                                   (uint32_t)g_calib.a * (((uint32_t)R * R) / 10000)) /
                                  10000);
        //if(output->spo2 > 10000)
        //{
        //  output->spo2 = 10000;
        //}
        // compute the Heartrate from the average-interval
        intervalSum = 0;
        for (i = 0; i < g_spo2BuffSize; i++)
            intervalSum += g_spo2Interval[i].interval;
        if (intervalSum == 0)
            output->heartrate = 0; // set to 0 for unexpected intervalSum=0
        else
            output->heartrate = (uint16_t)((12000 * (uint32_t)g_spo2BuffSize) / intervalSum);
        // compute the quality value based on amount of data used to make estimate
        quality = intervalSum / 2;
        if (quality > 100)
            quality = 100;
        output->signalQuality = (uint8_t)quality; // Q=100 ==> at least 10 seconds of data used to make estimate
    } else {
        output->status = 1; // no result
        output->signalQuality = 0;
        output->spo2 = 0;
        output->heartrate = 0;
        output->pi = 0;
        output->averageR = 0;
        output->acRed = 0;
        output->dcRed = 0;
        output->acInfrared = 0;
        output->dcInfrared = 0;
    }
    return SPO2_ALGORITHM_OK;
}

// function to sort the array in ascending order
void array_sort(uint16_t *array, uint8_t n)
{
    uint16_t i = 0, j = 0, temp = 0;

    for (i = 0; i < n; i++) {
        for (j = 0; j < n - 1; j++) {
            if (array[j] > array[j + 1]) {
                temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

// function to calculate the median of the array
uint16_t find_median(uint16_t array[], uint8_t n)
{
    uint16_t median = 0;
    uint8_t idx = 0;
    uint8_t idxStart, meanSize;
    uint32_t sum = 0;
    uint8_t i;

    array_sort(array, n);

    idx = n / 2;

    if (n < 7) {
        // if number of elements are even
        if (n % 2 == 0) {
            idxStart = idx - 1;
            meanSize = 2;
        }
        // if number of elements are odd
        else {
            idxStart = idx;
            meanSize = 1;
        }
    } else if (n < 12) {
        // increase size of mean to 4 bins
        idxStart = idx - 2;
        // if number of elements are even
        if (n % 2 == 0) {
            meanSize = 4;
        }
        // if number of elements are odd
        else {
            meanSize = 5;
        }
    } else {
        // increase size of mean to 7 bins
        idxStart = idx - 3;
        // if number of elements are even
        if (n % 2 == 0) {
            meanSize = 6;
        }
        // if number of elements are odd
        else {
            meanSize = 7;
        }
    }

    for (i = idxStart; i < (idxStart + meanSize); i++) {
        sum += (uint32_t)array[i];
    }
    median = (uint16_t)(sum / (uint32_t)meanSize);

    return median;
}
