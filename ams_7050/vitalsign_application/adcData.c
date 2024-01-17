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

/*! \file adcData.c
 *
 *  \author M. Zimmerman
 *
 *  \brief implementation for SpO2 algorithm.
 *
 *
 */

#include "adcData.h"
#include "spo2.h"
#include "spo2_private.h"

///////////////////////////////////////////////////////////////////////////////
// global parameters
///////////////////////////////////////////////////////////////////////////////

static uint8_t g_spo2AfeDriverType = SPO2_AFE_DRIVER_TYPE_UNKNOWN;

///////////////////////////////////////////////////////////////////////////////
// local functions
///////////////////////////////////////////////////////////////////////////////

static int8_t adcDataFindMaxPeak(AdcData_t *adcData);
static int8_t adcDataFindMinPeak(AdcData_t *adcData);

///////////////////////////////////////////////////////////////////////////////
// adcDataSetAfeDriverType
///////////////////////////////////////////////////////////////////////////////
void adcDataSetAfeDriverType(uint8_t spo2AfeDriverType)
{
    g_spo2AfeDriverType = spo2AfeDriverType;
}

///////////////////////////////////////////////////////////////////////////////
// adcDataInit
///////////////////////////////////////////////////////////////////////////////
void adcDataInit(AdcData_t *adcData, uint8_t red)
{
    int16_t i;

    for (i = 0; i < ADC_DATA_BUFFER_SIZE; i++) {
        adcData->sample[i] = 0;
        adcData->dcData[i] = 0;
    }
    adcData->startIdx = ADC_DATA_BUFFER_SIZE;
    adcData->firstMaxPeakIdx = -1;
    adcData->minPeakIdx = -1;
    adcData->secondMaxPeakIdx = -1;
    adcData->red = red;
}

///////////////////////////////////////////////////////////////////////////////
// adcDataBufferNewSecondOfData
///////////////////////////////////////////////////////////////////////////////
void adcDataBufferNewSecondOfData(AdcData_t *adcData, spo2LedData_t *newData)
{
    int16_t i;

    // shift all data by one second
    for (i = 0; i < ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND); i++) {
        adcData->sample[i] = adcData->sample[i + SPO2_ADC_VALUES_PER_SECOND];
        adcData->dcData[i] = adcData->dcData[i + SPO2_ADC_VALUES_PER_SECOND];
    }
    // adjust all indeces for the shift
    if (adcData->startIdx >= SPO2_ADC_VALUES_PER_SECOND)
        adcData->startIdx -= SPO2_ADC_VALUES_PER_SECOND;
    else
        adcData->startIdx = 0;
    if (adcData->firstMaxPeakIdx >= SPO2_ADC_VALUES_PER_SECOND) {
        // the first-Max peak is still valid -- keep all peaks (and interval-found)
        adcData->firstMaxPeakIdx -= SPO2_ADC_VALUES_PER_SECOND;
        if (adcData->minPeakIdx >= SPO2_ADC_VALUES_PER_SECOND)
            adcData->minPeakIdx -= SPO2_ADC_VALUES_PER_SECOND;
        else
            adcData->minPeakIdx = -1;
        if (adcData->secondMaxPeakIdx >= SPO2_ADC_VALUES_PER_SECOND)
            adcData->secondMaxPeakIdx -= SPO2_ADC_VALUES_PER_SECOND;
        else
            adcData->secondMaxPeakIdx = -1;
    } else {
        // the first-Max peak is not found or too old -- reset all peaks to not found
        adcData->firstMaxPeakIdx = -1;
        adcData->minPeakIdx = -1;
        adcData->secondMaxPeakIdx = -1;
    }
    // copy new data into buffer
    for (i = 0; i < SPO2_ADC_VALUES_PER_SECOND; i++) {
        if (newData[i].ledChange == 0) // no LED-change
        {
            if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS702X_AC_DC) {
                adcData->sample[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] = newData[i].acValue;  // AC
                adcData->dcData[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] = newData[i].absValue; // DC
            } else // if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS7000)
            {
                adcData->sample[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] =
                    newData[i].absValue; // Absolute-ADC
                adcData->dcData[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] =
                    0; // [DC is part of Absolute-ADC]
            }
        } else {
            // LED-change -- reset the peak detection
            adcData->sample[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] = 0;
            adcData->dcData[i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND)] = 0;
            adcData->startIdx = i + ((ADC_DATA_SECONDS - 1) * SPO2_ADC_VALUES_PER_SECOND) + 1;
            adcData->firstMaxPeakIdx = -1;
            adcData->minPeakIdx = -1;
            adcData->secondMaxPeakIdx = -1;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// adcDataFindMaxPeak
// Returns the index of the maxima, or (-1) if no minima found.
///////////////////////////////////////////////////////////////////////////////
static int8_t adcDataFindMaxPeak(AdcData_t *adcData)
{
    int8_t i;
    uint16_t value;
    uint16_t *data;

    i = adcData->startIdx;
    if (i < 4)
        i = 4; // the first few samples cannot be a maxima (cannot compare to prior samples)
    data = adcData->sample;

    // find the first maxima (if there is one)
    while (i < (ADC_DATA_BUFFER_SIZE - 4)) {
        // find next sample that is higher than its previous 4 samples
        value = data[i];
        while ((value <= data[i - 1]) || (value <= data[i - 2]) || (value <= data[i - 3]) || (value <= data[i - 4]) ||
               (data[i - 1] == 0) || (data[i - 2] == 0) || (data[i - 3] == 0) || (data[i - 4] == 0) ||
               (data[i + 1] == 0) || (data[i + 2] == 0) || (data[i + 3] == 0) || (data[i + 4] == 0)) {
            adcData->startIdx++;
            i++;
            if (i >= (ADC_DATA_BUFFER_SIZE - 4))
                return -1; // maxima not found
            value = data[i];
        }
        // determine if the current sample is a maxima (next 4 samples are lower)
        if ((data[i + 1] <= value) && (data[i + 2] <= value) && (data[i + 3] <= value) && (data[i + 4] <= value)) {
            // return the index of the maxima
            adcData->startIdx += 2;
            return i; // maxima found
        }
        adcData->startIdx++;
        i++;
    }
    return -1; // maxima not found
}

///////////////////////////////////////////////////////////////////////////////
// adcDataFindMinPeak
// Returns the index of the minima, or (-1) if no minima found.
///////////////////////////////////////////////////////////////////////////////
static int8_t adcDataFindMinPeak(AdcData_t *adcData)
{
    int8_t i;
    uint16_t value;
    uint16_t *data;

    i = adcData->startIdx;
    if (i < 4)
        i = 4; // the first few samples cannot be a minima (cannot compare to prior samples)
    data = adcData->sample;

    // find the first minima (if there is one)
    while (i < (ADC_DATA_BUFFER_SIZE - 4)) {
        // find next sample that is lower than its previous 4 samples
        value = data[i];
        while ((value == 0) || (value >= data[i - 1]) || (value >= data[i - 2]) || (value >= data[i - 3]) ||
               (value >= data[i - 4])) {
            adcData->startIdx++;
            i++;
            if (i >= (ADC_DATA_BUFFER_SIZE - 4))
                return -1; // minima not found
            value = data[i];
        }
        // determine if the current sample is a minima (next 4 samples are higher)
        if ((data[i + 1] >= value) && (data[i + 2] >= value) && (data[i + 3] >= value) && (data[i + 4] >= value)) {
            // return the index of the minima
            adcData->startIdx += 2;
            return i; // minima found
        }
        adcData->startIdx++;
        i++;
    }
    return -1; // minima not found
}

///////////////////////////////////////////////////////////////////////////////
// adcDataFindInterval
// Returns 1 if an interval is found, or 0 if no interval found.
///////////////////////////////////////////////////////////////////////////////
int8_t adcDataFindInterval(AdcData_t *adcData)
{
    // get first max peak
    if (adcData->firstMaxPeakIdx == -1) // not found
    {
        adcData->firstMaxPeakIdx = adcDataFindMaxPeak(adcData);
        if (adcData->firstMaxPeakIdx == -1) // still not found
            return 0;                       // no first max peak -- no interval found
        spo2DebugPeakFound(adcData->red, 1, spo2SecondCounter, adcData->firstMaxPeakIdx);
    }

    // get min peak
    if (adcData->minPeakIdx == -1) // not found
    {
        adcData->minPeakIdx = adcDataFindMinPeak(adcData);
        if (adcData->minPeakIdx == -1) // still not found
            return 0;                  // no min peak -- no interval found
        spo2DebugPeakFound(adcData->red, 0, spo2SecondCounter, adcData->minPeakIdx);
    }

    // get second max peak
    if (adcData->secondMaxPeakIdx == -1) // not found
    {
        adcData->secondMaxPeakIdx = adcDataFindMaxPeak(adcData);
        if (adcData->secondMaxPeakIdx == -1) // still not found
            return 0;                        // no second max peak -- no interval found
        spo2DebugPeakFound(adcData->red, 1, spo2SecondCounter, adcData->secondMaxPeakIdx);
    }

    spo2DebugIntervalFound(adcData->red, spo2SecondCounter, adcData->firstMaxPeakIdx, adcData->minPeakIdx,
                           adcData->secondMaxPeakIdx);
    return 1; // interval found
}

///////////////////////////////////////////////////////////////////////////////
// adcDataCalculateAndRemoveInterval
///////////////////////////////////////////////////////////////////////////////
int8_t adcDataCalculateAndRemoveInterval(AdcData_t *adcData, uint16_t *ac, uint16_t *dc)
{
    int32_t diff;
    int32_t adjust;

    *ac = 0;
    *dc = 0;
    if ((adcData->firstMaxPeakIdx == -1) || (adcData->minPeakIdx == -1) || (adcData->secondMaxPeakIdx == -1)) {
        // no interval -- prepare for next interval
        adcData->firstMaxPeakIdx = adcData->secondMaxPeakIdx;
        adcData->minPeakIdx = -1;
        adcData->secondMaxPeakIdx = -1;
        return 0; // no interval
    }
    if ((adcData->secondMaxPeakIdx - adcData->firstMaxPeakIdx) == 0) {
        // invalid interval -- prepare for next interval
        adcData->firstMaxPeakIdx = adcData->secondMaxPeakIdx;
        adcData->minPeakIdx = -1;
        adcData->secondMaxPeakIdx = -1;
        return 0; // invalid interval
    }

    // compute the AC value for the interval
    diff = (int32_t)adcData->sample[adcData->secondMaxPeakIdx] - adcData->sample[adcData->firstMaxPeakIdx];
    adjust = diff * (adcData->minPeakIdx - adcData->firstMaxPeakIdx) /
             (adcData->secondMaxPeakIdx - adcData->firstMaxPeakIdx);
    *ac = adcData->sample[adcData->firstMaxPeakIdx] + (uint16_t)adjust - adcData->sample[adcData->minPeakIdx];

    // compute (or select) the DC value for the interval
    if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS702X_AC_DC)
        *dc = adcData->dcData[adcData->minPeakIdx];
    else // if (g_spo2AfeDriverType == SPO2_AFE_DRIVER_TYPE_AS7000)
        *dc = adcData->sample[adcData->minPeakIdx] + (*ac / 2);

    if ((*ac == 0) || (*dc == 0)) {
        // invalid interval -- prepare for next interval
        adcData->firstMaxPeakIdx = adcData->secondMaxPeakIdx;
        adcData->minPeakIdx = -1;
        adcData->secondMaxPeakIdx = -1;
        return 0; // invalid interval
    }
//#define spo2DebugIntervalData(red, time, sampleIndex, ac, dc, max1, max2, maxAdj, min)
    // log the interval data
    spo2DebugIntervalData(adcData->red, spo2SecondCounter, adcData->secondMaxPeakIdx, *ac, *dc,
                          adcData->sample[adcData->firstMaxPeakIdx], adcData->sample[adcData->secondMaxPeakIdx],
                          adcData->sample[adcData->firstMaxPeakIdx] + (uint16_t)adjust,
                          adcData->sample[adcData->minPeakIdx]);

    // prepare for next interval
    adcData->firstMaxPeakIdx = adcData->secondMaxPeakIdx;
    adcData->minPeakIdx = -1;
    adcData->secondMaxPeakIdx = -1;
    return 1; // valid interval
}
