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
 *      PROJECT:   AS7000 heartrate algorithm
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file acc_processing.c
 *
 *  \brief Accelerometer data processing
 *
 *
 */

#include "acc_processing.h"
#include "fft.h"
#include "heartrate.h"
#include "heartrate_private.h"
#include <string.h>
#include <math.h>


/*
 *****************************************************************************
 * DEFINES  
 *****************************************************************************
 */


/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */
    
// storage for the combined accelerometer-data (packed)
static uint8_t accDataBuffer[ACC_DATA_BUFFER_SIZE_BYTES];
static uint8_t accDataCount;
static uint8_t accMaxDataCount;

static uint16_t accSamplePeriod100us; // the sample-period in units=100us [500==>50000us==>20Hz]
static uint8_t accLisDataUnscaled;
static uint16_t accDataScale;

    
/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
static uint16_t accConvertFreqToBin(uint16_t frequency);
    

/*!
 *****************************************************************************
 * \brief Initialize accelerometer data-processing
 * \param sampleFreq1000 - accelerometer-data sample-frequency [units=0.001Hz]
 *****************************************************************************
 */
void accProcessingInit(uint16_t sampleFreq1000)
{
    uint32_t maxData;

    memset(accDataBuffer, 0, sizeof(accDataBuffer));
    accDataCount = 0;
    accLisDataUnscaled = 0;
    if ((sampleFreq1000 < ACC_MIN_FREQUENCY_TIMES_1000) || (sampleFreq1000 > ACC_MAX_FREQUENCY_TIMES_1000))
        sampleFreq1000 = ACC_MAX_FREQUENCY_TIMES_1000; // change to valid range -- this should not be needed
    // calculate the sample-period in units of 100us
    accSamplePeriod100us = (uint16_t)(10000000L / sampleFreq1000);
    // determine the maximum data to be stored (ACC_DATA_BUFFER_SECONDS seconds)
    // round((sampleFreq1000 * ACC_DATA_BUFFER_SECONDS) / 1000)
    maxData = ((sampleFreq1000 * ACC_DATA_BUFFER_SECONDS) + 500) / 1000;
    if (maxData > ACC_DATA_BUFFER_MAX_SAMPLES)
        accMaxDataCount = ACC_DATA_BUFFER_MAX_SAMPLES;
    else
        accMaxDataCount = (uint8_t)maxData;
}

/*!
 *****************************************************************************
 * \brief Set the specified index in the accDataBuffer with the new value
 * \param index - index in the accDataBuffer to be set
 * \param value - the new value
 *****************************************************************************
 */
void accDataBufferSetIndex(uint16_t index, int16_t value)
{
    uint8_t temp;
    if (index >= ACC_DATA_BUFFER_MAX_SAMPLES)
        return;
    if ((index & 1) == 0)
    {
        accDataBuffer[(index * 3) / 2] = (uint8_t)(value >> 8);
        temp = accDataBuffer[((index * 3) / 2) + 1] & 0xF0;
        accDataBuffer[((index * 3) / 2) + 1] = temp | (uint8_t)((value >> 4) & 0x0F);
    }
    else
    {
        temp = accDataBuffer[(index * 3) / 2] & 0x0F;
        accDataBuffer[(index * 3) / 2] = temp | (uint8_t)((value >> 8) & 0xF0);
        accDataBuffer[((index * 3) / 2) + 1] = (uint8_t)(value >> 4);
    }
}

/*!
 *****************************************************************************
 * \brief Get the value for the specified index in the accDataBuffer
 * \param index - index in the accDataBuffer of the value to get
 * \return the value of the specified index in the accDataBuffer
 *****************************************************************************
 */
int16_t accDataBufferGetIndex(uint16_t index)
{
    if (index >= ACC_DATA_BUFFER_MAX_SAMPLES)
        return 0;
    if ((index & 1) == 0)
    {
        return (int16_t)((accDataBuffer[(index * 3) / 2] << 8) | ((accDataBuffer[((index * 3) / 2) + 1] & 0x0F) << 4));
    }
    else
    {
        return (int16_t)(((accDataBuffer[(index * 3) / 2] &0xF0) << 8) | ((accDataBuffer[((index * 3) / 2) + 1]) << 4));
    }
}

/*!
 *****************************************************************************
 * \brief Combine the data for the 3 accelerometer-axes and store result
 *
 * \param *accX - accelerometer-X data
 * \param *accY - accelerometer-Y data
 * \param *accZ - accelerometer-Z data
 *****************************************************************************
 */
void accStoreCombinedAxisData(int16_t *accX, int16_t *accY, int16_t *accZ, uint8_t numSamples)
{
    int16_t idx;
    int16_t temp;
    int32_t accVal[3];
    uint32_t sumOfSq;
    int32_t value;

    if (numSamples > HR_ACC_MAX_VALUES_PER_SECOND)
        numSamples = HR_ACC_MAX_VALUES_PER_SECOND;

    // shift the previous acc-data to make room for the new data (oldest data is discarded)
    for (idx=0; idx<(accMaxDataCount-numSamples); idx++)
    {
        temp = accDataBufferGetIndex(numSamples+idx);
        accDataBufferSetIndex(idx, temp);
    }
    if ((accDataCount + numSamples) > accMaxDataCount)
        accDataCount = accMaxDataCount;
    else
        accDataCount += numSamples;

    // check data from first second to see if data is from LIS accelerometer and properly scaled
    if (secondCounter == 1)
    {
        // The LIS accelerometer does not use the lower 4-bits.
        // The ADXL accelerometer does not use the lower 3-bits.
        // Check the lower bits of each axis for each sample...
        value = 0;
        for (idx=0; idx<numSamples; idx++)
        {
            value |= accX[idx] & 0x000F;
            value |= accY[idx] & 0x000F;
            value |= accZ[idx] & 0x000F;
        }
        if (value == 0)
        {
            // bit-0 was never set -- no indication of proper scaling from the application
            // bit-3 was never set -- LIS accelerometer detected
            accLisDataUnscaled = 1;
        }
    }

    // convert all samples to 1.0mg/LSB with a range of [-16384..16383]
    if (accLisDataUnscaled != 0)
    {
        // compensate for LIS-accelerometer -- convert from 0.75mg/LSB to 1.0mg/LSB
        for (idx=0; idx<numSamples; idx++)
        {
            accX[idx] = (accX[idx] / 4) * 3;
            accY[idx] = (accY[idx] / 4) * 3;
            accZ[idx] = (accZ[idx] / 4) * 3;
        }
    }
    else
    {
        // convert from 0.5mg/LSB to 1.0mg/LSB
        for (idx=0; idx<numSamples; idx++)
        {
            accX[idx] = accX[idx] / 2;
            accY[idx] = accY[idx] / 2;
            accZ[idx] = accZ[idx] / 2;
        }
    }

    // combine the accelerometer-axes for each new sample: sqrt(x^2 + y^2 + z^2)
    // - each axis value has a range of [-16384..16383] (LSB=1.0mg)
    // - each axis value is shifted to [0..32767]
    // - the sum-of-squares of the axes has a range of [0..3,221,028,867] (fits in uint32_t)
    // - the sqrt has a range of [0..56754] (LSB=1.0mg) (fits in a uint16_t)
    // - the value is shifted to range of [-28377..28377] (fits in an int16_t)
    for (idx=0; idx<numSamples; idx++)
    {
        accVal[ACC_X] = (int32_t)accX[idx] + 16384;
        accVal[ACC_Y] = (int32_t)accY[idx] + 16384;
        accVal[ACC_Z] = (int32_t)accZ[idx] + 16384;
        sumOfSq = (uint32_t)(accVal[ACC_X] * accVal[ACC_X]) + (uint32_t)(accVal[ACC_Y] * accVal[ACC_Y]) + (uint32_t)(accVal[ACC_Z] * accVal[ACC_Z]);
        value = (int32_t)sqrtf((float)sumOfSq);
        value -= 28377;
        accDataBufferSetIndex(accMaxDataCount-numSamples+idx, (int16_t)value);
    }
}

/*!
 *****************************************************************************
 * \brief Get the buffered acc-data
 *
 * \param *accData - buffer to copy acc-data to
 *****************************************************************************
 */
void accGetData(int16_t *accData)
{
    uint16_t dstOffset;
    int16_t accValue;
    int16_t *dstPtr;
    uint16_t idx;
    uint16_t adjIdx;
    int16_t *ptr;
    int16_t min;
    int16_t max;
    int16_t midValue;
    int32_t maxAdj;
    int32_t repeatValue;
    uint32_t invDstOffset;
    int32_t fract;

    // copy the data to the specified buffer
    dstOffset = (ACC_FFT_BINS - accDataCount) / 2; // center the data in the FFT input-buffer
    dstPtr = accData + dstOffset;
    min = 32767;
    max = -32768;
    for (idx=0; idx<accDataCount; idx++)
    {
        accValue = accDataBufferGetIndex(accMaxDataCount - accDataCount + idx);
        dstPtr[idx] = accValue;
        if (accValue < min)
            min = accValue;
        if (accValue > max)
            max = accValue;
    }

    // shift data by the mid-point of the data-range
    midValue = (max >> 1) + (min >> 1);
    ptr = dstPtr;
    for (idx=0; idx<accDataCount; idx++)
    {
        *ptr++ -= midValue;
    }
    // scale the data
    max = max - midValue;
    min = min - midValue;
    maxAdj = (int32_t)max;
    if ((int32_t)(-min) > maxAdj)
        maxAdj = (int32_t)(-min);
    accDataScale = 1;
    if (maxAdj < 128)
        accDataScale = 256;
    else if (maxAdj <= 32767)
        accDataScale = 32767 / maxAdj;
    ptr = dstPtr;
    for (idx=0; idx<accDataCount; idx++)
    {
        *ptr++ *= accDataScale;
    }

    // make the buffer end-points meet at 0
    // fill the buffer prior to first sample used -- this reduces FFT artifacts
    repeatValue = (int32_t)dstPtr[0];
    invDstOffset = 2147483648UL / dstOffset; // 2^31 / dstOffset
    ptr = accData;
    for (idx=0; idx<dstOffset; idx++)
    {
        fract = (int32_t)idx * invDstOffset; // result <= 2^31
        fract >>= 16; // result <= 2^15
        *ptr++ = (int16_t)((repeatValue * fract) >> 15);
    }
    // fill the buffer after the last sample used -- this reduces FFT artifacts
    repeatValue = (int32_t)dstPtr[accDataCount-1];
    invDstOffset = 2147483648UL / (dstOffset+1); // 2^31 / (dstOffset+1)
    ptr = &dstPtr[accDataCount];
    for (idx=0; idx<(ACC_FFT_BINS-dstOffset-accDataCount); idx++)
    {
        fract = ((int32_t)dstOffset-idx) * invDstOffset; // result <= 2^31
        fract >>= 16; // result <= 2^15
        *ptr++ = (int16_t)((repeatValue * fract) >> 15);
    }

    // apply window
    dstOffset += 5; // make the window-slope start a few samples from the edge of the input-data
    if (dstOffset >= (ACC_FFT_BINS/2))
        dstOffset = (ACC_FFT_BINS/2) - 1;
    invDstOffset = 2147483648UL / dstOffset; // 2^31 / dstOffset
    for (idx=0; idx<dstOffset; idx++)
    {
        fract = (int32_t)idx * invDstOffset; // result <= 2^31
        fract >>= 16; // result <= 2^15
        accData[idx] = (int16_t)(((int32_t)accData[idx] * fract) >> 15);
    }
    for (idx=(ACC_FFT_BINS-dstOffset+1); idx<ACC_FFT_BINS; idx++)
    {
        adjIdx = ACC_FFT_BINS - idx;
        fract = (int32_t)adjIdx * invDstOffset; // result <= 2^31
        fract >>= 16; // result <= 2^15
        accData[idx] = (int16_t)(((int32_t)accData[idx] * fract) >> 15);
    }
}

/*!
 *****************************************************************************
 * \brief Find the best peaks in the magnitude array
 *
 * \param mag - magnitude buffer
 * \param peakValue - value for each peak found
 * \param peakBin - bin for each peak found
 * \param peakStartBin - start-bin for each peak found
 * \param peakStopBin - stop-bin for each peak found
 * \param numPeaks - number of peaks to return
 *****************************************************************************
 */
void accFindFftPeaks(uint16_t *mag, uint32_t *peakValue, uint8_t *peakBin, uint8_t *peakStartBin, uint8_t *peakStopBin, uint8_t numPeaks)
{
    uint8_t peak;
    uint8_t bin;
    uint8_t lowestBin;
    uint8_t highestBin;
    uint32_t sum;
    uint8_t peaksFound;
    
    // Search for peak between 30 and 220 BPM [include the 2nd harmonic (2x base-frequency)]
    lowestBin = (uint8_t)accConvertFreqToBin(30);
    highestBin = (uint8_t)accConvertFreqToBin(220);

    // Find peaks
    peaksFound = 0;
    for (bin=lowestBin; bin<=highestBin; bin++)
    {
        if ((mag[bin] >= mag[bin-1]) && (mag[bin] > mag[bin+1]))
        {
            // there is a peak at this bin
            sum = mag[bin-1] + mag[bin] + mag[bin+1];

            // place the peak into the peak-list, if it fits
            for (peak=peaksFound; peak>0; peak--)
            {
                if (sum <= peakValue[peak-1])
                    break; // insert position found
                // shift the lower-peak down one in the list to maintain order (unless this is the lowest peak in a full list)
                if (peak < numPeaks)
                {
                    peakValue[peak] = peakValue[peak-1];
                    peakBin[peak] = peakBin[peak-1];
                    peakStartBin[peak] = peakStartBin[peak-1];
                    peakStopBin[peak] = peakStopBin[peak-1];
                }
                else
                {
                    peaksFound--;
                }
            }
            if (peak < numPeaks)
            {
                peakValue[peak] = sum;
                peakBin[peak] = bin;
                // find the edges of the peak
                peakStartBin[peak] = bin - 1;
                while ((peakStartBin[peak] > lowestBin) && (mag[peakStartBin[peak]-1] <= mag[peakStartBin[peak]]))
                    peakStartBin[peak]--;
                peakStopBin[peak] = bin + 1;
                while ((peakStopBin[peak] < highestBin) && (mag[peakStopBin[peak]+1] < mag[peakStopBin[peak]]))
                    peakStopBin[peak]++;
                bin = peakStopBin[peak];
                peaksFound++;
            }
        }
    }
    // fill in non-peaks
    for (peak=peaksFound; peak<numPeaks; peak++)
    {
        peakValue[peak] = 0;
        peakBin[peak] = 0;
        peakStartBin[peak] = 0;
        peakStopBin[peak] = 0;
    }
}

/*!
 *****************************************************************************
 * \brief Finds harmonics in the peaks
 *
 * \param peakValue - value for each peak
 * \param freq - frequency for each peak (scaled)
 * \param numPeaks - number of peaks to return
 * \param minPeakValue - minimum value for using a peak for harmonic-detection
 *****************************************************************************
 */
int8_t accFindHarmonic_0_5(uint32_t *peakValue, uint16_t *freq, uint8_t numPeaks, uint32_t minPeakValue)
{
    uint8_t base;
    uint8_t idx;

    base = 0;

    // harmonics must be within 12% of expected frequency (+/-12% ==> [22/25..28/25])
    for (idx=0; idx<numPeaks; idx++)
    {
        if ((idx == base) || (peakValue[idx] < minPeakValue) || (peakValue[idx] < (peakValue[base]/2)))
            continue;
        // check for 2:1 relationships
        if (freq[base] > freq[idx])
        {
            if (((freq[idx]*22/25) <= (freq[base]/2)) && ((freq[idx]*28/25) >= (freq[base]/2)))
            {
                return idx;
            }
        }
    }
    return -1;
}

/*!
 *****************************************************************************
 * \brief Finds harmonics in the peaks
 *
 * \param peakValue - value for each peak
 * \param freq - frequency for each peak (scaled)
 * \param numPeaks - number of peaks to return
 * \param minPeakValue - minimum value for using a peak for harmonic-detection
 *****************************************************************************
 */
int8_t accFindHarmonic_2_0(uint32_t *peakValue, uint16_t *freq, uint8_t numPeaks, uint32_t minPeakValue)
{
    uint8_t base;
    uint8_t idx;

    base = 0;

    // harmonics must be within 12% of expected frequency (+/-12% ==> [22/25..28/25])
    for (idx=0; idx<numPeaks; idx++)
    {
        if ((idx == base) || (peakValue[idx] < minPeakValue) || (peakValue[idx] < (peakValue[base]/2)))
            continue;
        // check for 1:2 relationships
        if (freq[base] < freq[idx])
        {
            if (((freq[idx]*22/25) <= (freq[base]*2)) && ((freq[idx]*28/25) >= (freq[base]*2)))
            {
                return idx;
            }
        }
    }
    return -1;
}

/*!
 *****************************************************************************
 * \brief Finds harmonics in the peaks
 *
 * \param peakValue - value for each peak
 * \param freq - frequency for each peak (scaled)
 * \param numPeaks - number of peaks to return
 * \param minPeakValue - minimum value for using a peak for harmonic-detection
 *****************************************************************************
 */
int8_t accFindHarmonic_1_5(uint32_t *peakValue, uint16_t *freq, uint8_t numPeaks, uint32_t minPeakValue)
{
    uint8_t base;
    uint8_t idx;

    base = 0;

    // harmonics must be within 12% of expected frequency (+/-12% ==> [22/25..28/25])
    for (idx=0; idx<numPeaks; idx++)
    {
        if ((idx == base) || (peakValue[idx] < minPeakValue) || (peakValue[idx] < (peakValue[base]/5)))
            continue;
        // check for 2:3 relationships
        if (freq[base] < freq[idx])
        {
            if (((freq[idx]*22/25) <= (freq[base]*3/2)) && ((freq[idx]*28/25) >= (freq[base]*3/2)))
            {
                return idx;
            }
        }
    }
    return -1;
}

/*!
 *****************************************************************************
 * \brief Finds harmonics in the peaks
 *
 * \param peakValue - value for each peak
 * \param freq - frequency for each peak (scaled)
 * \param harmonic - harmonic for each peak [0 if not found]
 * \param numPeaks - number of peaks to return
 * \param minPeakValue - minimum value for using a peak for harmonic-detection
 *****************************************************************************
 */
void accFindHarmonics(uint32_t *peakValue, uint16_t *freq, uint8_t *harmonic, uint8_t numPeaks, uint32_t minPeakValue)
{
    uint8_t base;
    int8_t harm_0_5;
    int8_t harm_2_0;
    int8_t harm_1_5;

    for (base=0; base<numPeaks; base++)
        harmonic[base] = 0; // init to "not a harmonic"
    base = 0;

    harm_0_5 = accFindHarmonic_0_5(peakValue, freq, numPeaks, minPeakValue);
    harm_2_0 = accFindHarmonic_2_0(peakValue, freq, numPeaks, minPeakValue);
    harm_1_5 = accFindHarmonic_1_5(peakValue, freq, numPeaks, minPeakValue*2);

    if (harm_0_5 > 0)
    {
        harmonic[base] = 2;
        harmonic[harm_0_5] = 1;
    }
    if (harm_1_5 > 0)
    {
        if (harmonic[base] == 0)
            harmonic[base] = 2;
        harmonic[harm_1_5] = harmonic[base] * 3 / 2;
    }
    if (harm_2_0 > 0)
    {
        if (harmonic[base] == 0)
            harmonic[base] = 1;
        harmonic[harm_2_0] = harmonic[base] * 2;
    }

    if (harmonic[base] == 0)
    {
        harm_1_5 = accFindHarmonic_1_5(peakValue, freq, 3, minPeakValue*3/2);
        if (harm_1_5 > 0)
        {
            harmonic[base] = 2;
            harmonic[harm_1_5] = harmonic[base] * 3 / 2;
        }
    }
}

/*!
 *****************************************************************************
 * \brief Converts a frequency to an FFT-bin.
 *
 * \param frequency - frequency (units=bpm) to be converted to FFT-bin
 * \return FFT-bin for the input frequency (rounded to nearest FFT-bin)
 *****************************************************************************
 */
static uint16_t accConvertFreqToBin(uint16_t frequency)
{
    uint32_t numerator;
    uint32_t denominator;

    numerator = (uint32_t)frequency * ACC_FFT_BINS * accSamplePeriod100us;
    denominator = (uint32_t)60 * 10000; // 60 seconds
    return (uint16_t)DIVIDE_WITH_ROUNDING(numerator, denominator);
}

/*!
 *****************************************************************************
 * \brief Converts an FFT-bin to a scaled-frequency.
 *
 * \param value - values for FFT-result
 * \param maxIdx - FFT-bin selected (to be converted)
 * \return frequency (units=0.1bpm) for FFT-bin
 *****************************************************************************
 */
static uint16_t accConvertBinToScaledFreq(uint16_t bin)
{
    uint32_t numerator;
    uint32_t denominator;

    // "bpm" requires 60 seconds/minute factor
    numerator = (uint32_t)bin * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10000;
    denominator = (uint32_t)ACC_FFT_BINS * accSamplePeriod100us;
    if (denominator == 0)
        return 0;
    return (uint16_t)DIVIDE_WITH_ROUNDING(numerator, denominator);
}

/*!
 *****************************************************************************
 * \brief Converts an FFT-bin to a frequency.
 *
 * \param value - values for FFT-result
 * \param maxIdx - FFT-bin selected (to be converted)
 * \return frequency (units=0.1bpm) for FFT-bin adjusted for adjacent-bin weighting
 *****************************************************************************
 */
uint16_t accConvertAdjustedBinToScaledFreq(uint16_t *value, uint8_t maxIdx)
{
    int16_t diff;
    int32_t sum;
    int32_t numerator;
    int32_t denominator;
    int32_t binOffsetFreq;
    int32_t binFract; // scaled fraction [1000==>1.000]

    if (maxIdx == 0)
        return 0; // avoid indexing below 0

    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER)
    {
        // Use one bin on each side of the selected-bin to compute a weighted fractional-bin offset for the frequency
        diff = value[maxIdx+1] - value[maxIdx-1];
        sum = (int32_t)value[maxIdx+1] + value[maxIdx] + value[maxIdx-1];
        // Convert the fractional-bin offset to a frequency-offset
        binOffsetFreq = 0;
        if (sum != 0)
        {
            // offsetFreq = ((diff * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR) / (sum * ACC_FFT_BINS)) * (10000 / accSamplePeriod100us)
            // offsetFreq = (diff * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10000) / (sum * ACC_FFT_BINS * accSamplePeriod100us)
            // offsetFreq = (diff * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10000 / 128) / (sum * ACC_FFT_BINS * accSamplePeriod100us / 128)
            // "bpm" requires 60 seconds/minute factor
            numerator = ((int32_t)60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10000) >> 7;
            numerator *= (int32_t)diff;
            denominator = sum * (ACC_FFT_BINS >> 7) * accSamplePeriod100us;
            binOffsetFreq = DIVIDE_WITH_ROUNDING(numerator, denominator);
        }
    }
    else
    {
        // Use one bin on each side of the selected-bin to compute a weighted fractional-bin offset for the frequency
        if (value[maxIdx+1] == value[maxIdx-1])
        {
            binFract = 0; // peak is centered -- no adjustment
        }
        else if ((value[maxIdx-1] > value[maxIdx]) || (value[maxIdx+1] > value[maxIdx]))
        {
            // "peak" index is not the peak-value (may be between double-peak)
            binFract = ((int32_t)value[maxIdx+1] - value[maxIdx-1]) * 1000 / ((int32_t)value[maxIdx+1] + value[maxIdx] + value[maxIdx-1]);
        }
        else if (value[maxIdx-1] > value[maxIdx+1])
        {
            // left side of peak is higher than right side -- compute adjustment
            binFract = ((((int32_t)value[maxIdx] - value[maxIdx-1]) * 1000 / (value[maxIdx] - value[maxIdx+1])) + 1000) / 2;
            binFract = binFract - 1000;
        }
        else
        {
            // right side of peak is higher than left side -- compute adjustment
            binFract = ((((int32_t)value[maxIdx] - value[maxIdx+1]) * 1000 / (value[maxIdx] - value[maxIdx-1])) + 1000) / 2;
            binFract = 1000 - binFract;
        }
        // Convert the fractional-bin offset to a frequency-offset
        // offsetFreq = ((binFract * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR) / (1000 * ACC_FFT_BINS)) * (10000 / accSamplePeriod100us)
        // offsetFreq = (binFract * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10000) / (1000 * ACC_FFT_BINS * accSamplePeriod100us)
        // offsetFreq = (binFract * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10) / (ACC_FFT_BINS * accSamplePeriod100us)
        // offsetFreq = (binFract * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10 / 16) / (ACC_FFT_BINS * accSamplePeriod100us / 16)
        // "bpm" requires 60 seconds/minute factor
        numerator = ((int32_t)60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * 10) >> 4;
        numerator *= binFract;
        denominator = (ACC_FFT_BINS * accSamplePeriod100us) >> 4;
        binOffsetFreq = DIVIDE_WITH_ROUNDING(numerator, denominator);
    }

    // Return the peak's frequency, with the frequency-offset applied
    return (accConvertBinToScaledFreq(maxIdx) + (int16_t)binOffsetFreq);
}

/*!
 *****************************************************************************
 * \brief Uses the FFT-result of the acc-data to determine the motion-frequency.
 *
 * \param accComplexMag - complex-magnitude (FFT-result)
 * \param accMotion - structure containing results
 *****************************************************************************
 */
void accDetermineMotionFreq(int16_t *accComplexMag, accMotionResults_t *accMotion)
{
    accProcessingWorkBuffer_t *accProcessingWorkBuffer;
    int16_t idx;
    uint16_t *magnitude;
    uint16_t *adjMag;
    accProcessingResults_t *accResults;
    uint16_t offset;
    uint16_t adjustedMag;
    uint8_t selectedIdx;

    // Setup pointers into work-buffer
    accProcessingWorkBuffer = (accProcessingWorkBuffer_t *)accComplexMag;
    magnitude = accProcessingWorkBuffer->mag;
    adjMag = accProcessingWorkBuffer->adjMag;
    accResults = &accProcessingWorkBuffer->accResults;

    // Calculate the Complex-Magnitude at each bin
    complexMagnitude(accComplexMag, magnitude, ACC_FFT_POST_PROCESSING_BINS);
    // Adjust the Complex-Magnitude to remove FFT artifacts
    accResults->magnitudeAdjustBase = magnitude[1];
    for (idx=0; idx<5; idx++)
        adjMag[idx] = magnitude[idx];
    for (; idx<ACC_FFT_POST_PROCESSING_BINS; idx++)
    {
        offset = (uint16_t)(accResults->magnitudeAdjustBase / idx);
        if (magnitude[idx] > offset)
            adjustedMag = magnitude[idx] - offset;
        else
            adjustedMag = 0;
        adjMag[idx] = adjustedMag;
    }
    // Compute the total of all bins (except bin-0 == DC)
    accResults->totalMagnitude = 0;
    for (idx=1; idx<ACC_FFT_POST_PROCESSING_BINS; idx++)
        accResults->totalMagnitude += magnitude[idx];

    // Search for frequency peaks
    accFindFftPeaks(adjMag, accResults->peakValue, accResults->peakBin, accResults->peakStartBin, accResults->peakStopBin, ACC_NUM_PEAKS);

    // Convert bin to frequency
    accResults->maxScore = 0;
    for (idx=0; idx<ACC_NUM_PEAKS; idx++)
    {
        accResults->freqAtPeak[idx] = accConvertAdjustedBinToScaledFreq(adjMag, accResults->peakBin[idx]);

        // Adjust the peak-value for scale (data may have been upscaled prior to FFT)
        accResults->peakValue[idx] = accResults->peakValue[idx] / accDataScale;

        // Compute the quality (based on peak-value, adjusted for number of acc-samples)
        {
            uint32_t quality=0;
            if (accDataCount != 0)
                quality = accResults->peakValue[idx] * 25 / accDataCount;
            if (quality > 100)
                quality = 100;
            accResults->quality[idx] = (uint8_t)quality;
        }
    }

    // Select the frequency to use
    // Check for harmonics
    accFindHarmonics(accResults->peakValue, accResults->freqAtPeak, accResults->harmonic, ACC_NUM_PEAKS, ACC_MIN_PEAK_VALUE);
    // use the first-peak, or the 2X harmonic (if there is one)
    selectedIdx = 0;
    accResults->freqQuality = accResults->quality[0];
    for (idx=0; idx<ACC_NUM_PEAKS; idx++)
    {
        if (accResults->harmonic[idx] == 2)
        {
            selectedIdx = (uint8_t)idx;
            if ((accResults->harmonic[0] != 1) || (accResults->quality[selectedIdx] > accResults->freqQuality))
                accResults->freqQuality = accResults->quality[selectedIdx];
            break;
        }
    }

    if (accResults->harmonic[selectedIdx] < 2)
    {
        accResults->freqResult = accResults->freqAtPeak[selectedIdx];
    }
    else
    {
        accResults->freqResult = accResults->freqAtPeak[selectedIdx] / accResults->harmonic[selectedIdx];
        // override the harmonic if amplitude is too strong for the reduced frequency
        if (accResults->freqQuality >= 50)
        {
            if (accResults->freqResult < 500)
                accResults->freqResult *= 2;
        }
        else if (accResults->freqQuality >= 40)
        {
            if (accResults->freqResult < 400)
                accResults->freqResult *= 2;
        }
        else if (accResults->freqQuality >= 15)
        {
            if (accResults->freqResult < 300)
                accResults->freqResult *= 2;
        }
    }

    // adjust the frequency, if necessary
    if (accResults->harmonic[selectedIdx] == 0)
    {
        if ((accResults->freqResult >= 900) && (accResults->freqQuality <= 15))
            accResults->freqResult /= 2;
        if ((accResults->freqResult >= 1000) && (accResults->freqQuality <= 30))
            accResults->freqResult /= 2;
    }
    if (accResults->freqQuality < 5)
    {
        accResults->freqResult = 0;
    }
    else if (accResults->freqQuality <= 7)
    {
        if (accResults->freqResult >= 600)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 10)
    {
        if (accResults->freqResult >= 650)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 15)
    {
        if (accResults->freqResult >= 700)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 20)
    {
        if (accResults->freqResult >= 750)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 25)
    {
        if (accResults->freqResult >= 850)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 30)
    {
        if (accResults->freqResult >= 950)
            accResults->freqResult /= 2;
    }
    else if (accResults->freqQuality <= 35)
    {
        if (accResults->freqResult >= 1000)
            accResults->freqResult /= 2;
    }
    else
    {
        if (accResults->freqResult >= 1100)
            accResults->freqResult /= 2;
    }

    // place results into caller structure
    accMotion->selectedIdx = selectedIdx;
    accMotion->freqResult = accResults->freqResult;
    accMotion->freqQuality = accResults->freqQuality;
    for (idx=0; idx<ACC_NUM_PEAKS; idx++)
    {
        accMotion->freqAtPeak[idx] = accResults->freqAtPeak[idx];
        accMotion->quality[idx] = accResults->quality[idx];
    }
    accResults->scale = accDataScale;

    //debugOutputAccFFT(magnitude, adjMag, accResults);
}

/*!
 *****************************************************************************
 * \brief Computes the FFT of the acc-data and returns the motion-frequency.
 * NOTE: The FFT library-function requires a large amount of stack, so this
 * function should have minimal stack-usage.  That is the reason for breaking
 * out the majority of this function's code into subroutines.
 *
 * \param accData - working buffer (un-init) [must be 512 16-bit elements for FFT]
 * \param accMotion - structure containing results
 *****************************************************************************
 */
void accFindMotionFreq(int16_t *accData, accMotionResults_t *accMotion)
{
    // copy acc-data into the working-buffer for processing (data is returned packed)
    accGetData(accData);

    // FFT processing
    ams_rfft_int16_512(accData);

    // Find and return the motion frequency
    accDetermineMotionFreq(accData, accMotion);  
}
