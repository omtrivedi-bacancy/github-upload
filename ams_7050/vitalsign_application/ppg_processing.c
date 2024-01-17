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

/*! \file ppg_processing.c
 *
 *  \brief PPG data processing
 *
 *
 */

#include "ppg_processing.h"
#include "fft.h"
#include "heartrate.h"
#include "heartrate_private.h"
#include <string.h>


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
    
// storage for the PPG-data
static uint16_t ppgBuffer[PPG_DATA_SIZE];
static uint8_t ppgSecondsBuffered;
static uint8_t ppgCrosstalkLevel64; // [units = 64 adc-counts]
// statistics for the buffered PPG-data
static ppgRawDataStats_t ppgRawDataStats;


    
/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
static void ppgRawClearBuffer(void);



/*!
 *****************************************************************************
 * \brief Initialize PPG data-processing
 *****************************************************************************
 */
void ppgProcessingInit(uint8_t crosstalkLevel64)
{
    ppgRawClearBuffer();
    ppgCrosstalkLevel64 = crosstalkLevel64;
}


/*!
 *****************************************************************************
 * \brief Clear the PPG data buffer
 *****************************************************************************
 */
static void ppgRawClearBuffer(void)
{
    int16_t second;

    memset(ppgBuffer, 0, sizeof(ppgBuffer));
    ppgSecondsBuffered = 0;
    memset(&ppgRawDataStats, 0, sizeof(ppgRawDataStats));
    ppgRawDataStats.scale = 1;
    for (second=0; second<PPG_BUFFER_SECONDS; second++)
    {
        ppgRawDataStats.minSamplePerSecond[second] = 65535;
        ppgRawDataStats.maxSamplePerSecond[second] = 0;
        ppgRawDataStats.excludeSecond |= (1 << second);
        ppgRawDataStats.userNotPresent |= (1 << second);
    }
}

/*!
 *****************************************************************************
 * \brief Store the new second of PPG data
 *
 * \param *ppgInput - PPG input data
 * \param *currentChange - LED-current change data
 *****************************************************************************
 */
void ppgRawStoreData(uint16_t *ppgInput, uint8_t *currentChange)
{
    int16_t idx;
    uint8_t second;
    uint16_t *ptr;
    uint16_t min;
    uint16_t max;
    int32_t diff;
    uint32_t total;
    uint8_t firstIdx;
    uint8_t width;
    uint8_t savedWidth;

    if (ppgSecondsBuffered < PPG_BUFFER_SECONDS)
        ppgSecondsBuffered++;

    // shift the previous PPG-data by one-second to make room for the new data
    for (idx=0; idx<(PPG_BUFFER_SECONDS-1) * PPG_VALUES_PER_SECOND; idx++)
    {
        ppgBuffer[idx] = ppgBuffer[PPG_VALUES_PER_SECOND+idx];
    }
    // store the new data
    ptr = &ppgBuffer[(PPG_BUFFER_SECONDS-1) * PPG_VALUES_PER_SECOND];
    for (idx=0; idx<PPG_VALUES_PER_SECOND; idx++)
		{
			*ptr++ = ppgInput[idx];
		}
    // shift the previous statistics to match the data shift
    for (second=0; second<(PPG_BUFFER_SECONDS-1); second++)
    {
        ppgRawDataStats.sampleDeltaMaxPerSecond[second] = ppgRawDataStats.sampleDeltaMaxPerSecond[second+1];
        ppgRawDataStats.minSamplePerSecond[second] = ppgRawDataStats.minSamplePerSecond[second+1];
        ppgRawDataStats.maxSamplePerSecond[second] = ppgRawDataStats.maxSamplePerSecond[second+1];
        ppgRawDataStats.rangePerSecond[second] = ppgRawDataStats.rangePerSecond[second+1];
    }
    ppgRawDataStats.excludeSecond >>= 1;
    ppgRawDataStats.userNotPresent >>= 1;

    ///////////////////////////////////////////////////////////////////////////
    // calculate statistics for the new data
    second = (PPG_BUFFER_SECONDS - 1); // new data is here
    ptr = &ppgBuffer[second*PPG_VALUES_PER_SECOND];
    ppgRawDataStats.sampleDeltaMaxPerSecond[second] = 0;
    if (ppgSecondsBuffered != 1) // there was data prior to the new data
    {
        diff = (int32_t)(*ptr) - (*(ptr-1));
        if (diff < 0)
            diff = -diff;
        ppgRawDataStats.sampleDeltaMaxPerSecond[second] = (uint16_t)diff;
    }
    max = *ptr++;
    min = max;
    for (idx=1; idx<PPG_VALUES_PER_SECOND; idx++)
    {
        if (*ptr > max)
            max = *ptr;
        if (*ptr < min)
            min = *ptr;
        diff = (int32_t)(*ptr) - (*(ptr-1));
        if (diff < 0)
            diff = -diff;
        if ((uint16_t)diff > ppgRawDataStats.sampleDeltaMaxPerSecond[second])
            ppgRawDataStats.sampleDeltaMaxPerSecond[second] = (uint16_t)diff;
        ptr++;
    }
    ppgRawDataStats.minSamplePerSecond[second] = min;
    ppgRawDataStats.maxSamplePerSecond[second] = max;
    ppgRawDataStats.rangePerSecond[second] = (uint16_t)((int32_t)max - min);

    ///////////////////////////////////////////////////////////////////////////
    // calculate statistics for the buffered data
    second = (PPG_BUFFER_SECONDS - ppgSecondsBuffered);
    min = ppgRawDataStats.minSamplePerSecond[second];
    max = ppgRawDataStats.maxSamplePerSecond[second];
    for (second++; second<PPG_BUFFER_SECONDS; second++)
    {
        if (ppgRawDataStats.minSamplePerSecond[second] < min)
            min = ppgRawDataStats.minSamplePerSecond[second];
        if (ppgRawDataStats.maxSamplePerSecond[second] > max)
            max = ppgRawDataStats.maxSamplePerSecond[second];
    }
    ppgRawDataStats.minSampleValueAll = min;
    ppgRawDataStats.maxSampleValueAll = max;

    ppgRawDataStats.sampleDeltaMax = 0;
    total = 0;
    for (idx=((PPG_BUFFER_SECONDS-ppgSecondsBuffered)*PPG_VALUES_PER_SECOND)+1; idx<PPG_DATA_SIZE; idx++)
    {

        diff = (int32_t)ppgBuffer[idx] - ppgBuffer[idx-1];
        if (diff < 0)
            diff = -diff;
        if ((uint16_t)diff > ppgRawDataStats.sampleDeltaMax)
            ppgRawDataStats.sampleDeltaMax = (uint16_t)diff;
        total += (uint32_t)diff;
    }
    ppgRawDataStats.sampleDeltaAvg = total / ((ppgSecondsBuffered*PPG_VALUES_PER_SECOND)-1);
    ppgRawDataStats.rangeAll = max - min;

    // calculate statistics for the last-2-seconds (used for detecting discontinuities)
    second = (PPG_BUFFER_SECONDS - 1); // new data is here
    min = ppgRawDataStats.minSamplePerSecond[second];
    if (ppgRawDataStats.minSamplePerSecond[second-1] < min)
        min = ppgRawDataStats.minSamplePerSecond[second-1];
    max = ppgRawDataStats.maxSamplePerSecond[second];
    if (ppgRawDataStats.maxSamplePerSecond[second-1] > max)
        max = ppgRawDataStats.maxSamplePerSecond[second-1];
    ppgRawDataStats.rangeLast2 = max - min;
    // calculate statistics for data before the last-second (used for detecting discontinuities)
    min = ppgRawDataStats.minSamplePerSecond[0];
    max = ppgRawDataStats.maxSamplePerSecond[0];
    for (second=1; second<(PPG_BUFFER_SECONDS-1); second++)
    {
        if (ppgRawDataStats.minSamplePerSecond[second] < min)
            min = ppgRawDataStats.minSamplePerSecond[second];
        if (ppgRawDataStats.maxSamplePerSecond[second] > max)
            max = ppgRawDataStats.maxSamplePerSecond[second];
    }
    ppgRawDataStats.rangeBeforeLast = max - min;

    ///////////////////////////////////////////////////////////////////////////
    // determine if the new second of data (or the previous second) should be excluded
    ppgRawDataStats.excludeSecond &= ~(1 << (PPG_BUFFER_SECONDS-1)); // clear the bit representing the new second

    // For LP only, exclude first 3 seconds (unstable data due to LED-adjustment and HW-filtering)
    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER)
    {
        if (timeSinceUserPresent <= 3)
            ppgRawDataStats.excludeSecond |= (1 << (PPG_BUFFER_SECONDS-1)); // set the bit representing the new second
    }
    // For ULP only, exclude any second with an LED-change
    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        total = 0;
        for (idx=0; idx<PPG_VALUES_PER_SECOND; idx++)
        {
            if ((currentChange[idx] == HR_AFE_CURRENT_INCREASED) || (currentChange[idx] == HR_AFE_CURRENT_DECREASED))
                total++;
        }
        if (total > 0)
            ppgRawDataStats.excludeSecond |= (1 << (PPG_BUFFER_SECONDS-1)); // set the bit representing the new second
    }
    // For non-ULP, automatically exclude data with specific charactreristics...
    if (hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        second = (PPG_BUFFER_SECONDS - 2); // previous second is here (not the new second of data)
        if (ppgRawDataStats.rangeLast2 > (ppgRawDataStats.rangePerSecond[second] + 8000))
        {
            // large range-change caused by new data
            ppgRawDataStats.excludeSecond |= (1 << (second+1)); // set the bit
        }
        if ((ppgRawDataStats.rangePerSecond[second+1] + 8000) < ppgRawDataStats.rangeLast2)
        {
            // new data has much lower range than previous second
            ppgRawDataStats.excludeSecond |= (1 << (second)); // set the bit
        }

        if ((ppgRawDataStats.sampleDeltaMaxPerSecond[second+1] > 10000) && 
            (ppgRawDataStats.sampleDeltaMaxPerSecond[second+1] > (ppgRawDataStats.sampleDeltaMaxPerSecond[second] * 4)))
        {
            // large (or much larger) sample-to-sample variation
            ppgRawDataStats.excludeSecond |= (1 << (second+1)); // set the bit
        }
    }
    // Exclude data with a large range-change
    second = (PPG_BUFFER_SECONDS - 1); // the new second of data
    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        if ((timeSinceUserPresent > 20) && (ppgRawDataStats.rangeAll > (ppgRawDataStats.rangeBeforeLast * 2)))
        {
            // large range-change caused by new data
            ppgRawDataStats.excludeSecond |= (1 << (second)); // set the bit
        }
    }
    else
    {
        if ((timeSinceUserPresent >= 3) && (ppgRawDataStats.rangeAll > (ppgRawDataStats.rangeBeforeLast * 2)))
        {
            // large range-change caused by new data
            ppgRawDataStats.excludeSecond |= (1 << (second)); // set the bit
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // check the new second of data for "no user present"
    ppgRawDataStats.userNotPresent &= ~(1 << (PPG_BUFFER_SECONDS-1)); // clear the bit representing the new second
    // For ULP only, check for user-not-present
    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        total = 0;
        for (idx=0; idx<PPG_VALUES_PER_SECOND; idx++)
        {
            // ADC-value is below the threshold for user-presence
            // Only allow declaration of "user not present" if current is already at maximum
            if (ppgInput[idx] < ((uint16_t)ppgCrosstalkLevel64 << 6))
            {
                if (currentChange[idx] != HR_AFE_CURRENT_INCREASED)
                    total++;
            }
        }
        if (total >= 2)
        {
            // At least 2 values in the new second were too low for user-present
            ppgRawDataStats.userNotPresent |= (1 << (PPG_BUFFER_SECONDS-1)); // set the bit representing the new second
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // determine which data should be used for processing
    width = 0;
    savedWidth = 0;
    firstIdx = 0;
    // find the first index after any "not-present" index
    for (second=0; second<PPG_BUFFER_SECONDS; second++)
    {
        if ((ppgRawDataStats.userNotPresent & (1 << second)) != 0)
            firstIdx = second + 1;
    }

    // find first non-excluded index
    for (second=firstIdx; second<PPG_BUFFER_SECONDS; second++)
    {
        if ((ppgRawDataStats.excludeSecond & (1 << second)) == 0)
        {
            firstIdx = second;
            width = 1;
            break;
        }
    }
    // init to no non-excluded seconds of data -- indicate that no data is to be used
    ppgRawDataStats.firstSecondIdx = PPG_BUFFER_SECONDS; // indicates no data to be used
    ppgRawDataStats.lastSecondIdx = (PPG_BUFFER_SECONDS-1);
    if (width != 0)
    {
        // find the width of the non-excluded range
        for (second++; second<PPG_BUFFER_SECONDS; second++)
        {
            if ((ppgRawDataStats.excludeSecond & (1 << second)) == 0)
            {
                width++;
            }
            else
            {
                // the non-excluded range has ended -- check if it should be saved
                if (width >= savedWidth)
                {
                    // save the non-excluded range
                    savedWidth = width;
                    ppgRawDataStats.firstSecondIdx = firstIdx;
                    ppgRawDataStats.lastSecondIdx = second - 1;
                }
                width = 0;
                // find the next non-excluded index
                for (; second<PPG_BUFFER_SECONDS; second++)
                {
                    if ((ppgRawDataStats.excludeSecond & (1 << second)) == 0)
                    {
                        firstIdx = second;
                        width = 1;
                        break;
                    }
                }
            }
        }
        // check if the final non-excluded range (if it exists) should be saved
        if (width >= savedWidth)
        {
            // save the non-excluded range
            ppgRawDataStats.firstSecondIdx = firstIdx;
            ppgRawDataStats.lastSecondIdx = second - 1;
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    // calculate statistics for the used data
    second = ppgRawDataStats.firstSecondIdx;
    min = 65535;
    max = 0;
    for (second=ppgRawDataStats.firstSecondIdx; second<=ppgRawDataStats.lastSecondIdx; second++)
    {
        if (ppgRawDataStats.minSamplePerSecond[second] < min)
            min = ppgRawDataStats.minSamplePerSecond[second];
        if (ppgRawDataStats.maxSamplePerSecond[second] > max)
            max = ppgRawDataStats.maxSamplePerSecond[second];
    }
    ppgRawDataStats.minSampleValueUsed = min;
    ppgRawDataStats.maxSampleValueUsed = max;

    //debugOutputPpgIn(ppgInput, PPG_VALUES_PER_SECOND);
}

/*!
 *****************************************************************************
 * \brief Returns "user-present" state
 *
 * \return 0 = user-not-present, 1 = user-present
 * \param samples - number of samples in the buffer to filter
 *****************************************************************************
 */
uint8_t ppgRawIsUserPresent(void)
{
    return ((ppgRawDataStats.userNotPresent & (1 << (PPG_BUFFER_SECONDS-1))) == 0);
}

#define HIGH_PASS_FILTER_MULTIPLIER      7
#define HIGH_PASS_FILTER_ADJ_MULTIPLIER  9
#define HIGH_PASS_FILTER_DIVISOR         8
#define HIGH_PASS_FILTER_DIVISOR_BITS    3
/*!
 *****************************************************************************
 * \brief Apply high-pass filter to the data
 *
 * \param *ppgData - buffer to filter (in/out - filter in place)
 * \param samples - number of samples in the buffer to filter
 *****************************************************************************
 */
void ppgRawDataHighPassFilter(int16_t *ppgData, uint16_t samples)
{
    int16_t *ptr;
    int16_t inPrev;
    int32_t out;
    uint16_t idx;

    // initialize the filter by going backwards through the first section of data
    idx = PPG_VALUES_PER_SECOND - 1;
    if (samples < PPG_VALUES_PER_SECOND)
        idx = samples - 1;
    ptr = &ppgData[idx];
    out = 0; // [S16.3]
    inPrev = *ptr; // x(i-1) initialized to x(0)
    ptr--;
    for (; idx>0; idx--)
    {
        // y(i) = (y(i-1) + x(i) - x(i-1)) * (HIGH_PASS_FILTER_MULTIPLIER/HIGH_PASS_FILTER_DIVISOR)
        out = ((out + ((int32_t)(*ptr - inPrev) * HIGH_PASS_FILTER_ADJ_MULTIPLIER)) * HIGH_PASS_FILTER_MULTIPLIER) >> HIGH_PASS_FILTER_DIVISOR_BITS; // [S16.3]
        inPrev = *ptr; // x(i-1) for next iteration
        ptr--;
    }

    // process the data using the calculated initial value
    ptr = ppgData;
    inPrev = *ptr; // x(i-1) initialized to x(0)
    // out was determined above // [S16.3]
    *ptr++ = (int16_t)(out >> HIGH_PASS_FILTER_DIVISOR_BITS); // y(0) = result of reverse-data-filtering [S16.0]
    for (idx=1; idx<samples; idx++)
    {
        // y(i) = (y(i-1) + x(i) - x(i-1)) * (HIGH_PASS_FILTER_MULTIPLIER/HIGH_PASS_FILTER_DIVISOR)
        out = ((out + ((int32_t)(*ptr - inPrev) * HIGH_PASS_FILTER_ADJ_MULTIPLIER)) * HIGH_PASS_FILTER_MULTIPLIER) >> HIGH_PASS_FILTER_DIVISOR_BITS; // [S16.3]
        inPrev = *ptr; // x(i-1) for next iteration
        *ptr++ = (int16_t)(out >> HIGH_PASS_FILTER_DIVISOR_BITS); // y(i) = result [S16.0]
    }
}

/*!
 *****************************************************************************
 * \brief Get the raw PPG data for FFT processing
 *
 * \param *ppgData - buffer to copy PPG-data to
 *****************************************************************************
 */
void ppgRawGetDataFft(int16_t *ppgData)
{
    uint16_t samplesToCopy;
    uint16_t dstOffset;
    uint16_t *srcPtr;
    int16_t *dstPtr;
    uint16_t idx;
    int32_t repeatValue;
    int16_t *ptr;
    int16_t midValue;
    int16_t min;
    int16_t max;
    int32_t maxAdj;
    uint16_t adjIdx;
    uint32_t invDstOffset;
    int32_t fract;

    // check for no-data case
    if (ppgRawDataStats.firstSecondIdx > ppgRawDataStats.lastSecondIdx)
    {
        ptr = ppgData;
        for (idx=0; idx<PPG_FFT_BINS; idx++)
            *ptr++ = 0;
        ppgRawDataStats.scale = 1;
        //debugOutputPpgFftIn(ppgBuffer, ppgData, &ppgRawDataStats);
        return;
    }

    // copy the data to the specified buffer -- convert from range [0..65535] to [-32768..32767]
    samplesToCopy = ((ppgRawDataStats.lastSecondIdx + 1) - ppgRawDataStats.firstSecondIdx) * PPG_VALUES_PER_SECOND;
    dstOffset = (PPG_FFT_BINS - samplesToCopy) / 2; // center the data in the FFT input-buffer
    srcPtr = ppgBuffer + (ppgRawDataStats.firstSecondIdx * PPG_VALUES_PER_SECOND); // start of buffered-data being used
    dstPtr = ppgData + dstOffset;
    for (idx=0; idx<samplesToCopy; idx++)
        dstPtr[idx] = (int16_t)srcPtr[idx] - 32768;

    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        // apply a high-pass filter to reduce near-DC artifacts
        ppgRawDataHighPassFilter(dstPtr, samplesToCopy); // filters in place

        // determine the new min/max
        min = 32767;
        max = -32768;
        for (idx=0; idx<samplesToCopy; idx++)
        {
            if (dstPtr[idx] < min)
                min = dstPtr[idx];
            if (dstPtr[idx] > max)
                max = dstPtr[idx];
        }
    }
    else
    {
        max = (int16_t)ppgRawDataStats.maxSampleValueUsed - 32768;
        min = (int16_t)ppgRawDataStats.minSampleValueUsed - 32768;
    }

    // shift data by the mid-point of the data-range
    midValue = (max >> 1) + (min >> 1);
    ptr = dstPtr;
    for (idx=0; idx<samplesToCopy; idx++)
    {
        *ptr++ -= midValue;
    }
    max = max - midValue;
    min = min - midValue;
    // scale the data
    maxAdj = (int32_t)max;
    if ((int32_t)(-min) > maxAdj)
        maxAdj = (int32_t)(-min);
    if (maxAdj == 0)
        ppgRawDataStats.scale = 1;
    else if (maxAdj < 128)
        ppgRawDataStats.scale = 256;
    else
        ppgRawDataStats.scale = 32767 / maxAdj;
    ptr = dstPtr;
    for (idx=0; idx<samplesToCopy; idx++)
    {
        *ptr++ *= ppgRawDataStats.scale;
    }

    if ((hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER) || (accMotion.freqQuality != 0))
    {
        // make the buffer end-points meet at 0
        // fill the buffer prior to first sample used -- this reduces FFT artifacts
        repeatValue = (int32_t)dstPtr[0];
        invDstOffset = 2147483648UL / dstOffset; // 2^31 / dstOffset
        ptr = ppgData;
        for (idx=0; idx<dstOffset; idx++)
        {
            fract = (int32_t)idx * invDstOffset; // result <= 2^31
            fract >>= 16; // result <= 2^15
            *ptr++ = (int16_t)((repeatValue * fract) >> 15);
        }
        // fill the buffer after the last sample used -- this reduces FFT artifacts
        repeatValue = (int32_t)dstPtr[samplesToCopy-1];
        invDstOffset = 2147483648UL / (dstOffset+1); // 2^31 / (dstOffset+1)
        ptr = &dstPtr[samplesToCopy];
        for (idx=0; idx<(PPG_FFT_BINS-dstOffset-samplesToCopy); idx++)
        {
            fract = ((int32_t)dstOffset-idx) * invDstOffset; // result <= 2^31
            fract >>= 16; // result <= 2^15
            *ptr++ = (int16_t)((repeatValue * fract) >> 15);
        }

        // apply triangle window (width=#-FFT-bins)  -- this reduces FFT artifacts
        if (dstOffset < PPG_WINDOW_MIN_OFFSET)
            dstOffset = PPG_WINDOW_MIN_OFFSET;
        invDstOffset = 2147483648UL / dstOffset; // 2^31 / dstOffset
        for (idx=0; idx<dstOffset; idx++)
        {
            fract = (int32_t)idx * invDstOffset; // result <= 2^31
            fract >>= 16; // result <= 2^15
            ppgData[idx] = (int16_t)(((int32_t)ppgData[idx] * fract) >> 15);
        }
        for (idx=(PPG_FFT_BINS-dstOffset+1); idx<PPG_FFT_BINS; idx++)
        {
            adjIdx = PPG_FFT_BINS - idx;
            fract = (int32_t)adjIdx * invDstOffset; // result <= 2^31
            fract >>= 16; // result <= 2^15
            ppgData[idx] = (int16_t)(((int32_t)ppgData[idx] * fract) >> 15);
        }
    }
    else // ULP with no-motion
    {
        // fill the buffer prior to first sample used -- this reduces FFT artifacts
        ptr = ppgData;
        for (idx=0; idx<dstOffset; idx++)
        {
            *ptr++ = 0;
        }
        // fill the buffer after the last sample used -- this reduces FFT artifacts
        ptr = &dstPtr[samplesToCopy];
        for (idx=0; idx<(PPG_FFT_BINS-dstOffset-samplesToCopy); idx++)
        {
            *ptr++ = 0;
        }

        // apply triangle window (width=data-size) -- this reduces FFT artifacts
        invDstOffset = 2147483648UL / (samplesToCopy/2); // 2^31 / (samplesToCopy/2)
        for (idx=0; idx<(samplesToCopy/2); idx++)
        {
            fract = (int32_t)(idx+1) * invDstOffset; // result <= 2^31
            fract >>= 16; // result <= 2^15
            ppgData[dstOffset+idx] = (int16_t)(((int32_t)ppgData[dstOffset+idx] * fract) >> 15);
            ppgData[dstOffset+samplesToCopy-idx-1] = (int16_t)(((int32_t)ppgData[dstOffset+samplesToCopy-idx-1] * fract) >> 15);
        }
    }

    //debugOutputPpgFftIn(ppgBuffer, ppgData, &ppgRawDataStats);
}

/*!
 *****************************************************************************
 * \brief Get the raw PPG data for PPI processing
 *
 * \param *ppgData - buffer to copy PPG-data to
 *****************************************************************************
 */
void ppgRawGetDataPpi(uint16_t *ppgData)
{
    uint16_t idx;
    uint16_t repeatValue;
    uint16_t *ptr;

    // check for no-data case
    if (ppgRawDataStats.firstSecondIdx > ppgRawDataStats.lastSecondIdx)
    {
        ptr = ppgData;
        for (idx=0; idx<PPG_DATA_SIZE; idx++)
            *ptr++ = 0;
        return;
    }

    // copy the data to the specified buffer
    for (idx=0; idx<PPG_DATA_SIZE; idx++)
        ppgData[idx] = ppgBuffer[idx];

    // back-fill buffer with first-sample used
    repeatValue = ppgData[ppgRawDataStats.firstSecondIdx*PPG_VALUES_PER_SECOND];
    ptr = ppgData;
    for (idx=0; idx<(ppgRawDataStats.firstSecondIdx*PPG_VALUES_PER_SECOND); idx++)
        *ptr++ = repeatValue;

    // fill-in remainder of buffer with last-sample used
    repeatValue = ppgData[((ppgRawDataStats.lastSecondIdx+1)*PPG_VALUES_PER_SECOND)-1];
    ptr = &ppgData[(ppgRawDataStats.lastSecondIdx+1)*PPG_VALUES_PER_SECOND];
    for (idx=(ppgRawDataStats.lastSecondIdx+1)*PPG_VALUES_PER_SECOND; idx<PPG_DATA_SIZE; idx++)
        *ptr++ = repeatValue;
}

/*!
 *****************************************************************************
 * \brief Find the best peaks in the magnitude array
 *
 * \param mag - magnitude buffer
 * \param lowestBin - lowest-bin to search
 * \param highestBin - highest-bin to search
 * \param peakValue - value for each peak found
 * \param peakBin - bin for each peak found
 * \param peakStartBin - start-bin for each peak found
 * \param peakStopBin - stop-bin for each peak found
 * \param numPeaks - number of peaks to return
 *****************************************************************************
 */
static void ppgFindFftPeaks(uint16_t *mag, uint8_t lowestBin, uint8_t highestBin, uint32_t *peakValue, uint8_t *peakBin, uint8_t *peakStartBin, uint8_t *peakStopBin, uint8_t numPeaks)
{
    uint8_t peak;
    uint8_t bin;
    uint32_t sum;
    uint8_t peaksFound;

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
 * \brief Adjust magnitude for spectral noise
 *
 * \param magnitude - magnitude
 * \param adjMag - adjusted-magnitude
 * \param ppgResults - results structure
 *****************************************************************************
 */
void ppgAdjustMagnitude(uint16_t *magnitude, uint16_t *adjMag, ppgProcessingResults_t *ppgResults)
{
    uint8_t bin;
    uint16_t offset;

    if (timeSinceUserPresent <= 20)
    {
        // use a different adjustment method at startup to prevent losing low-level signals during fast-lock
        if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
        {
            if (accMotion.freqQuality == 0)
            {
                for (bin=0; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
                {
                    adjMag[bin] = magnitude[bin];
                }
                // Remove low-frequency artifacts with a decay (inverse) adjustment across all bins
                // Find the maximum adjustment using bins [4..10]
                ppgResults->magnitudeAdjustBase = 0;
                for (bin=4; bin<=10; bin++)
                {
                    if (((uint32_t)adjMag[bin] * bin) > ppgResults->magnitudeAdjustBase)
                        ppgResults->magnitudeAdjustBase = (uint32_t)adjMag[bin] * bin;
                }
                for (; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
                {
                    uint16_t adjustedMag;
                    offset = ppgResults->magnitudeAdjustBase / bin;
                    if ((uint32_t)adjMag[bin] > offset)
                        adjustedMag = (uint32_t)adjMag[bin] - offset;
                    else
                        adjustedMag = 0;
                    adjMag[bin] = adjustedMag;
                }
            }
            else // (accMotion.freqQuality != 0)
            {
                for (bin=0; bin<8; bin++)
                {
                    adjMag[bin] = magnitude[bin];
                }
                // Remove the local noise floor
                ppgResults->magnitudeAdjustBase = 0; // not used here
                offset = magnitude[bin];
                for (; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
                {
                    if (magnitude[bin] < offset)
                        offset = magnitude[bin]; // maintain the minimum magnitude found
                    adjMag[bin] = magnitude[bin] - offset;
                }
                // Reduce the early bins
                for (bin=8; bin<50; bin++)
                {
                    uint8_t adjBin = bin;
                    if (bin < 17)
                        adjBin = 17;
                    offset = (uint16_t)((uint32_t)adjMag[bin] * (50 - adjBin) * (50 - adjBin) / 1600);
                    adjMag[bin] = adjMag[bin] - offset;
                }
            }
        }
        else // (hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
        {
            for (bin=0; bin<10; bin++)
            {
                adjMag[bin] = magnitude[bin];
            }
            // Remove the local noise floor
            ppgResults->magnitudeAdjustBase = 0; // not used here
            offset = magnitude[bin];
            for (; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
            {
                if (magnitude[bin] < offset)
                    offset = magnitude[bin]; // maintain the minimum magnitude found
                adjMag[bin] = magnitude[bin] - offset;
            }
            // Reduce the early bins
            for (bin=10; bin<50; bin++)
            {
                offset = (uint16_t)((uint32_t)adjMag[bin] * (50 - bin) * (50 - bin) / 1600);
                adjMag[bin] = adjMag[bin] - offset;
            }
        }
    }
    else // not fast-lock
    {
        for (bin=0; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
        {
            adjMag[bin] = magnitude[bin];
        }
        // Remove low-frequency artifacts with a decay (inverse) adjustment across all bins
        // Find the maximum adjustment using bins [4..10]
        ppgResults->magnitudeAdjustBase = 0;
        for (bin=4; bin<=10; bin++)
        {
            if (((uint32_t)adjMag[bin] * bin) > ppgResults->magnitudeAdjustBase)
                ppgResults->magnitudeAdjustBase = (uint32_t)adjMag[bin] * bin;
        }
        for (; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
        {
            uint16_t adjustedMag;
            offset = ppgResults->magnitudeAdjustBase / bin;
            if ((uint32_t)adjMag[bin] > offset)
                adjustedMag = (uint32_t)adjMag[bin] - offset;
            else
                adjustedMag = 0;
            adjMag[bin] = adjustedMag;
        }
    }
}

/*!
 *****************************************************************************
 * \brief Converts an FFT-bin to a frequency.
 *
 * \param value - values for FFT-result
 * \param maxIdx - FFT-bin selected (to be converted)
 * \return frequency for FFT-bin (scaled) adjusted for adjacent-bin weighting
 *****************************************************************************
 */
uint16_t ppgConvertBinToFreq(uint16_t *value, uint8_t maxIdx)
{
    int16_t diff;
    int32_t sum;
    int32_t binOffsetFreq;
    int32_t binFract; // scaled fraction [1000==>1.000]

    if (maxIdx == 0)
        return 0; // avoid indexing below 0

    if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER)
    {
        // Use one bin on each side of the selected-bin to compute a weighted fractional-bin offset for the frequency
        diff = value[maxIdx+1] - value[maxIdx-1];
        sum = (int32_t)value[maxIdx+1] + value[maxIdx] + value[maxIdx-1];
        if (sum == 0)
            binOffsetFreq = 0;
        else
            binOffsetFreq = DIVIDE_WITH_ROUNDING(((int32_t)diff * PPG_FFT_FREQ_RANGE_SCALED), (sum * PPG_FFT_BINS));
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
        binOffsetFreq = DIVIDE_WITH_ROUNDING((binFract * PPG_FFT_FREQ_RANGE_SCALED), (1000 * PPG_FFT_BINS));
    }

    // Return the peak's frequency, with the frequency-offset applied
    return (PPG_FREQUENCY_SCALED_FROM_FFT_BIN(maxIdx) + (int16_t)binOffsetFreq);
}

/*!
 *****************************************************************************
 * \brief Uses the FFT-result of the PPG-data to find dominant frequencies.
 *
 * \param ppgComplexMag - complex-magnitude (FFT-result)
 * \param ppgResultsHnd - handle for pointer to results structure
 *****************************************************************************
 */
void ppgRawFindPeakFFT(int16_t *ppgComplexMag, ppgProcessingResults_t **ppgResultsHnd)
{
    ppgProcessingWorkBuffer_t *ppgProcessingWorkBuffer;
    uint16_t *magnitude;
    uint16_t *adjMag;
    ppgProcessingResults_t *ppgResults;
    uint8_t bin;
    uint8_t peak;
    uint8_t minProcBin;
    uint8_t maxProcBin;
    uint8_t samples;

    // Setup pointers into work-buffer
    ppgProcessingWorkBuffer = (ppgProcessingWorkBuffer_t *)ppgComplexMag;
    magnitude = ppgProcessingWorkBuffer->mag;
    adjMag = ppgProcessingWorkBuffer->adjMag;
    ppgResults = &ppgProcessingWorkBuffer->ppgResults;
    *ppgResultsHnd = ppgResults;

    // Calculate the Complex-Magnitude at each bin
    complexMagnitude(ppgComplexMag, magnitude, PPG_FFT_POST_PROCESSING_BINS);
    // Adjust the Complex-Magnitude to remove FFT artifacts
    ppgAdjustMagnitude(magnitude, adjMag, ppgResults);

    // Compute the total of all bins (except bin-0 == DC)
    ppgResults->totalMagnitude = 0;
    for (bin=1; bin<PPG_FFT_POST_PROCESSING_BINS; bin++)
        ppgResults->totalMagnitude += magnitude[bin];

    // Search for frequency peaks in the range [HEARTRATE_PROC_SCALED_FREQUENCY_MIN_* .. HEARTRATE_PROC_SCALED_FREQUENCY_MAX] bpm
    if (hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
    {
        if (minHeartrate == 0)
        {
            if (accMotion.freqQuality == 0)
                ppgResults->minHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_MIN_STATIC;
            else
                ppgResults->minHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_MIN_DYNAMIC;
        }
        else
        {
            ppgResults->minHrScaled = minHeartrate * 10 * .98;
        }
        if (maxHeartrate == 0)
            ppgResults->maxHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_MAX;
        else
            ppgResults->maxHrScaled = maxHeartrate * 10 * 1.02;
    }
    else
    {
        if (minHeartrate == 0)
        {
            if (accMotion.freqQuality == 0)
                ppgResults->minHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MIN_STATIC;
            else
                ppgResults->minHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MIN_DYNAMIC;
        }
        else
        {
            ppgResults->minHrScaled = minHeartrate * 10 * .96;
        }
        if (maxHeartrate == 0)
            ppgResults->maxHrScaled = HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MAX;
        else
            ppgResults->maxHrScaled = maxHeartrate * 10 * 1.04;
    }
    minProcBin = (uint8_t)PPG_FFT_BIN_FROM_SCALED_FREQUENCY(ppgResults->minHrScaled);
    maxProcBin = (uint8_t)PPG_FFT_BIN_FROM_SCALED_FREQUENCY(ppgResults->maxHrScaled);
    if (timeSinceUserPresent <= 20)
    {
        maxProcBin = (uint8_t)PPG_FFT_BIN_FROM_SCALED_FREQUENCY(4000); // do not go above 400bpm
        if (maxProcBin > (PPG_FFT_POST_PROCESSING_BINS - 2))
            maxProcBin = PPG_FFT_POST_PROCESSING_BINS - 2;
    }
    ppgFindFftPeaks(adjMag, minProcBin, maxProcBin, ppgResults->peakValue, ppgResults->peakBin, ppgResults->peakStartBin, ppgResults->peakStopBin, PPG_NUM_PEAKS);

    // Convert bin to frequency
    for (peak=0; peak<PPG_NUM_PEAKS; peak++)
    {
        ppgResults->freqAtPeak[peak] = ppgConvertBinToFreq(adjMag, ppgResults->peakBin[peak]);
        ppgResults->peakScore[peak] = 0;

        // Compute the score, value, and quality for each peak
        if (hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
        {
            // Compute the peak-score (adjust to be relative to total)
            if (ppgResults->totalMagnitude != 0)
                ppgResults->peakScore[peak] = ppgResults->peakValue[peak] * PPG_SCORE_NORMALIZER / ppgResults->totalMagnitude;

            // Adjust the peak-value for scale (data may have been upscaled prior to FFT)
            ppgResults->peakValue[peak] = ppgResults->peakValue[peak] / ppgRawDataStats.scale;

            // Compute the quality
            if (ppgResults->peakScore[peak] > (PPG_QUALITY_DIVISOR*100))
                ppgResults->quality[peak] = 100;
            else 
                ppgResults->quality[peak] = (ppgResults->peakScore[peak] / PPG_QUALITY_DIVISOR);
        }
        else
        {
            // Compute the peak-score (adjust to be relative to total)
            if (ppgResults->totalMagnitude != 0)
                ppgResults->peakScore[peak] = ppgResults->peakValue[peak] * PPG_ULP_SCORE_NORMALIZER / (ppgResults->totalMagnitude);

            // Adjust the peak-value for scale (data may have been upscaled prior to FFT)
            ppgResults->peakValue[peak] = ppgResults->peakValue[peak] * PPG_ULP_VALUE_MAG / ((uint32_t)ppgRawDataStats.scale * PPG_ULP_VALUE_DIVISOR);

            // Compute the quality
            if (ppgResults->peakScore[peak] > (PPG_ULP_QUALITY_DIVISOR*100))
                ppgResults->quality[peak] = 100;
            else 
                ppgResults->quality[peak] = (ppgResults->peakScore[peak] / PPG_ULP_QUALITY_DIVISOR);
        }
        if ((ppgResults->peakScore[peak] > 0) && (ppgResults->quality[peak] == 0))
            ppgResults->quality[peak] = 1;
    }

    ppgResults->scale = ppgRawDataStats.scale;
    ppgResults->firstSecondIdx = ppgRawDataStats.firstSecondIdx;
    ppgResults->lastSecondIdx = ppgRawDataStats.lastSecondIdx;

    samples = (ppgRawDataStats.lastSecondIdx - ppgRawDataStats.firstSecondIdx + 1) * PPG_VALUES_PER_SECOND;
    if (samples == 0)
        samples = 1;
    //debugOutputPpgFftOut(magnitude, adjMag, ppgResults, &ppgRawDataStats, (2.0f * PPG_FFT_BINS / samples));
}

/*!
 *****************************************************************************
 * \brief Computes the FFT of the PPG-data and returns the PPG-frequency.
 * NOTE: The FFT library-function requires a large amount of stack, so this
 * function should have minimal stack-usage.  That is the reason for breaking
 * out the majority of this function's code into subroutines.
 *
 * \param ppgData - working buffer (un-init) [must be 512 16-bit elements for FFT]
 * \param ppgResultsHnd - handle for pointer to results structure
 *****************************************************************************
 */
void ppgRawProcessFft(int16_t *ppgData, ppgProcessingResults_t **ppgResultsHnd)
{
    // copy PPG-data into the working-buffer for processing (data is returned packed)
    ppgRawGetDataFft(ppgData);

    ams_rfft_int16_512(ppgData);

    // Find and return the motion frequency
    ppgRawFindPeakFFT(ppgData, ppgResultsHnd);  
}


/*!
 *****************************************************************************
 * \brief Searches for minima in raw-ppg-data.
 * This function is intended for fast-lock.
 *
 * \param inBuf - input-data buffer
 * \param bufferSize - number of elements in the input-buffer
 * \param peakValueBuf - output-buffer for peak-values (minimas)
 * \param peakIndexBuf - output-buffer for peak-indeces
 * \param maxPeaks - the maximum number of peaks to return (limit of output-buffers)
 * \return number of minimum peaks found and minimas with indices in buffers
 *****************************************************************************
 */
uint8_t ppgRawFindMinPeaks(uint16_t *inBuf, int16_t bufferSize, uint16_t *peakValueBuf, int16_t *peakIndexBuf, int16_t maxPeaks)
{
    int32_t idx;              // current index in buffer
    uint16_t currentValue;    // minimum value found in buffer, 
    int8_t minimaFound = 0;   // number of minima found in fft_bufer
    
    // the first samples cannot be a minima (cannot compare to prior samples)
    idx = 4;
    while ((idx < (bufferSize-4)) && (minimaFound < maxPeaks))
    {
        // find next sample that is lower than its previous 4 samples
        while ((inBuf[idx] >= inBuf[idx-1]) || (inBuf[idx] >= inBuf[idx-2]) || (inBuf[idx] >= inBuf[idx-3]) || (inBuf[idx] >= inBuf[idx-4]))
        {
            idx++;
            if (idx >= (bufferSize-4))
                return minimaFound;
        }
        // Determine if the current sample is a minima (next 4 samples are higher)
        currentValue = inBuf[idx];
        if (inBuf[idx+1] >= currentValue)
        {
            if (inBuf[idx+2] >= currentValue)
            {
                if (inBuf[idx+3] >= currentValue)
                {
                    if (inBuf[idx+4] > currentValue)
                    {
                        // save data and increase minimaFound
                        *peakValueBuf++ = currentValue;
                        *peakIndexBuf++ = idx;
                        minimaFound++;
                        idx = idx + 5;
                    }
                    else
                    {
                        idx = idx + 4;
                    }
                }
                else
                {
                    idx = idx + 3;
                }
            }
            else
            {
                idx = idx + 2;
            }
        }
        else
        {
            idx = idx + 1;
        }
    }    
    return minimaFound;
}

/*!
 *****************************************************************************
 * \brief Return a new heartrate PPI-result based on raw-ppg-data.
 * This function is intended for fast-lock.
 *
 * \param ppgData - ptr to work-buffer
 * \param ppiHr - ptr to PPI-HR result (scaled)
 * \param ppiQuality - ptr to PPI-Quality result [0..100]                         
 *****************************************************************************
 */
void ppgRawProcessPpi(uint16_t *ppgData, uint16_t *ppiHr, uint8_t *ppiQuality)
{
    uint8_t minPeaksCnt;
    ppgPpiWorkBuffer_t *ppiWorkBuffer;
    uint8_t numberIntervals;
    uint8_t *truePeaks;
    uint16_t *bbiDeltas;
    uint8_t *bbiBins;
    uint16_t bbi;
    uint8_t bbiCount;
    uint8_t numIntervalsOutOfRange;
    uint8_t numIntervalsInRange;
    uint8_t countAtMedian;
    uint8_t medianBin;
    uint8_t idx;
    uint8_t firstBin;
    uint8_t lastBin;
    uint16_t bbiSum;

    ppiWorkBuffer = (ppgPpiWorkBuffer_t *)ppgData;
    ppgRawGetDataPpi(ppgData);
    minPeaksCnt = ppgRawFindMinPeaks(&ppgData[(PPG_BUFFER_SECONDS-ppgSecondsBuffered)*PPG_VALUES_PER_SECOND], (ppgSecondsBuffered*PPG_VALUES_PER_SECOND), ppiWorkBuffer->peakValue, ppiWorkBuffer->peakIndex, PPG_DATA_SIZE/2);

    truePeaks = ppiWorkBuffer->truePeak;
    bbiDeltas = ppiWorkBuffer->bbiDelta;
    bbiBins = ppiWorkBuffer->bbiBin;

    // treat all peaks as true-peaks (for now)
    for (idx=0; idx<minPeaksCnt; idx++)
        truePeaks[idx] = 1;

    // compute all BBIs and keep track of count-per-interval (bins)
    if (minPeaksCnt > 0)
        numberIntervals = minPeaksCnt - 1;
    else
        numberIntervals = 0;
    for (idx=0; idx<=(HEARTRATE_PPG_BBI_LIMIT_MAX+1); idx++)
        bbiBins[idx] = 0;
    for (idx=0; idx<numberIntervals; idx++)
    {
        bbi = ppiWorkBuffer->peakIndex[idx+1] - ppiWorkBuffer->peakIndex[idx];
        bbiDeltas[idx] = bbi;
        if (bbi > HEARTRATE_PPG_BBI_LIMIT_MAX)
            bbi = HEARTRATE_PPG_BBI_LIMIT_MAX+1;
        bbiBins[bbi]++;
    }

    // compute the time-based heartrate estimate
    // eliminate intervals that are out-of-range
    numIntervalsOutOfRange = 0;
    for (idx=0; idx<HEARTRATE_PPG_BBI_LIMIT_MIN; idx++)
        numIntervalsOutOfRange += bbiBins[idx];
    numIntervalsOutOfRange += bbiBins[HEARTRATE_PPG_BBI_LIMIT_MAX+1];
    numIntervalsInRange = numberIntervals - numIntervalsOutOfRange;
    if (numIntervalsInRange == 0)
    {
        *ppiHr = 0;
        *ppiQuality = 0;
    }
    else
    {
        // find median-bin of intervals-in-range
        countAtMedian = (numIntervalsInRange+1) / 2;
        bbiCount = 0;
        medianBin = HEARTRATE_PPG_BBI_LIMIT_MAX;
        for (idx=HEARTRATE_PPG_BBI_LIMIT_MIN; idx<=HEARTRATE_PPG_BBI_LIMIT_MAX; idx++)
        {
            bbiCount += bbiBins[idx];
            if (bbiCount >= countAtMedian)
            {
                medianBin = idx;
                break;
            }
        }

        // keep intervals within 3/8 of median-bin-index
        firstBin = medianBin * 5 / 8;
        lastBin = medianBin + (medianBin - firstBin);
        if (firstBin < HEARTRATE_PPG_BBI_LIMIT_MIN)
            firstBin = HEARTRATE_PPG_BBI_LIMIT_MIN;
        if (lastBin > HEARTRATE_PPG_BBI_LIMIT_MAX)
            lastBin = HEARTRATE_PPG_BBI_LIMIT_MAX;
        numIntervalsInRange = 0;
        for (idx=firstBin; idx<=lastBin; idx++)
            numIntervalsInRange += bbiBins[idx];

        // find median-bin of remaining intervals-in-range
        countAtMedian = (numIntervalsInRange+1) / 2;
        bbiCount = 0;
        medianBin = lastBin;
        for (idx=firstBin; idx<=lastBin; idx++)
        {
            bbiCount += bbiBins[idx];
            if (bbiCount >= countAtMedian)
            {
                medianBin = idx;
                break;
            }
        }

        // keep intervals within 3/8 of median-bin-index
        firstBin = medianBin * 5 / 8;
        lastBin = medianBin + (medianBin - firstBin);
        if (firstBin < HEARTRATE_PPG_BBI_LIMIT_MIN)
            firstBin = HEARTRATE_PPG_BBI_LIMIT_MIN;
        if (lastBin > HEARTRATE_PPG_BBI_LIMIT_MAX)
            lastBin = HEARTRATE_PPG_BBI_LIMIT_MAX;

        // calculate the average BBI of remaining intervals-in-range and convert to frequency (BPM-scaled)
        bbiCount = 0;
        bbiSum = 0;
        for (idx=firstBin; idx<=lastBin; idx++)
        {
            bbiCount += bbiBins[idx];
            bbiSum += (uint16_t)bbiBins[idx] * idx;
        }
        if (bbiCount == 0)
        {
            *ppiHr = 0;
            *ppiQuality = 0;
        }
        else
        {
            *ppiHr = (uint16_t)DIVIDE_WITH_ROUNDING( (PPG_VALUES_PER_SECOND * 60 * HEARTRATE_ALGORITHM_SCALE_FACTOR * (uint32_t)bbiCount ), bbiSum );
            *ppiQuality = (uint8_t)bbiSum / 2; // bbiSum is 200 max -- rescale to [0..100]
        }
    }

    //debugOutputPpgPeaks(ppiWorkBuffer->peakIndex, ppiWorkBuffer->peakValue, truePeaks, minPeaksCnt, bbiDeltas, numberIntervals);
}
