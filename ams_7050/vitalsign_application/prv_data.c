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
 *      PROJECT:   AS70xx PRV algorithm
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file prv_data.c
 *
 *  \author M. Zimmerman
 *
 *  \brief implementation for PRV algorithm. 
 *
 *
 */

#include "prv_data.h"
#include "prv_private.h"


PrvData_t prvData;

#define PRV_DATA_SAMPLE_RADIUS       (prvData.samplesPerSecond*3/16)
#define PRV_DATA_MAX_SAMPLE_INTERVAL (2*prvData.samplesPerSecond) // 2 second interval (30bpm)


///////////////////////////////////////////////////////////////////////////////
// prvDataReset
///////////////////////////////////////////////////////////////////////////////
static void prvDataReset(void)
{
    prvData.startup = 0;
    prvData.startAge = 0;
    prvData.searchForMaxima = 1;
    prvData.foundValue = 0;
    prvData.foundAge = 0;
    prvData.oppositeValue = 0;
    prvData.oppositeAge = 0;
    prvData.maximaAge = -1;
    prvData.minimaAge = -1;
    prvData.secondMaximaAge = -1;
}

///////////////////////////////////////////////////////////////////////////////
// prvDataInit
///////////////////////////////////////////////////////////////////////////////
uint8_t prvDataInit(uint8_t afeDriverType, uint16_t samplesPerSecond)
{
    if ((afeDriverType != PRV_AFE_DRIVER_TYPE_NORMAL_POWER) && (afeDriverType != PRV_AFE_DRIVER_TYPE_LOW_POWER))
        return 0;
    if (samplesPerSecond > 4000)
        return 0;
    prvData.afeDriverType = afeDriverType;
    prvData.samplesPerSecond = samplesPerSecond;
    prvDataReset();
    prvData.startup = 1;
    prvData.startAge = prvData.samplesPerSecond; // wait one full second before using data at startup
    return 1;
}

///////////////////////////////////////////////////////////////////////////////
// prvDataProcessNewSample
///////////////////////////////////////////////////////////////////////////////
void prvDataProcessNewSample(uint16_t sample)
{
    // manage the sample-buffer
    if (prvData.startup == 1)
    {
        prvData.startAge--;
        if (prvData.startAge == 0)
        {
            // startup period is over -- begin normal operation
            prvData.startup = 0;
            prvDebugMsg(prvSampleCount, "Startup complete -- begin using data");
        }
        return;
    }
    if (sample == 0)
    {
        // a value of 0 indicates a discontinuity (e.g. led-change) or underflow (clipping)
        // reset peak-detection -- remove all buffered data
        prvDataReset();
        prvDebugMsg(prvSampleCount, "Reset: Sample=0");
        return;
    }
    if (sample == 16383)
    {
        // a value of 16383 indicates overflow (clipping)
        // reset peak-detection -- remove all buffered data
        prvDataReset();
        prvDebugMsg(prvSampleCount, "Reset: Sample=16383");
        return;
    }
    
    if (prvData.foundValue == 0)
    {
        // this is the first sample since reset
        prvData.foundValue = sample;
        prvData.foundAge = 0;
        prvData.oppositeValue = sample;
        prvData.oppositeAge = 0;
        return;
    }
    
    // adjust ages for the new sample
    if (prvData.startAge < 1023)
        prvData.startAge++;
    prvData.foundAge++;
    prvData.oppositeAge++;
    if (prvData.maximaAge >= 0)
        prvData.maximaAge++;
    if (prvData.minimaAge >= 0)
        prvData.minimaAge++;
    if (prvData.maximaAge > (PRV_DATA_MAX_SAMPLE_INTERVAL + PRV_DATA_SAMPLE_RADIUS + 1))
    {
        // the found maxima is too old -- reset interval detection
        prvDataReset();
        prvDebugMsg(prvSampleCount, "Reset: Maxima too old");
    }
    
    // compare new value with found/opposite values
    if (prvData.searchForMaxima)
    {
        if (sample > prvData.foundValue)
        {
            prvData.foundValue = sample;
            prvData.foundAge = 0;
            prvData.oppositeValue = sample;
            prvData.oppositeAge = 0;
        }
        else if (sample < prvData.oppositeValue)
        {
            prvData.oppositeValue = sample;
            prvData.oppositeAge = 0;
        }
    }
    else
    {
        if (sample < prvData.foundValue)
        {
            prvData.foundValue = sample;
            prvData.foundAge = 0;
            prvData.oppositeValue = sample;
            prvData.oppositeAge = 0;
        }
        else if (sample > prvData.oppositeValue)
        {
            prvData.oppositeValue = sample;
            prvData.oppositeAge = 0;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// prvDataFindInterval
// Returns 1 if an interval is found, or 0 if no interval found.
///////////////////////////////////////////////////////////////////////////////
uint8_t prvDataFindInterval(void)
{
    if (prvData.maximaAge == -1) // not found
    {
        // find first maxima
        if (prvData.searchForMaxima != 1)
        {
            prvDataReset();
            prvDebugMsg(prvSampleCount, "Reset: Search in wrong state (maxima)");
            return 0;
        }
        if (prvData.foundAge >= PRV_DATA_SAMPLE_RADIUS)
        {
            if (prvData.startAge < (PRV_DATA_SAMPLE_RADIUS*2+1))
            {
                // not enough data since reset
                prvDataReset();
                prvDebugMsg(prvSampleCount, "Reset: Maxima too soon after reset");
                return 0;
            }
            // save the found maxima
            prvData.maximaAge = prvData.foundAge;
#ifdef DEBUG_LOGGING
            prvData.maximaValue = prvData.foundValue;
            prvDebugPeakFound(prvSampleCount-prvData.maximaAge, 1, prvData.maximaValue);
#endif // DEBUG_LOGGING
            // switch to finding minima
            prvData.searchForMaxima = 0;
            prvData.foundAge = prvData.oppositeAge;
            prvData.foundValue = prvData.oppositeValue;
            return 0; // interval not complete
        }
    }
    else if (prvData.minimaAge == -1) // not found
    {
        // find minima
        if (prvData.searchForMaxima != 0)
        {
            prvDataReset();
            prvDebugMsg(prvSampleCount, "Reset: Search in wrong state (minima)");
            return 0;
        }
        if (prvData.foundAge >= PRV_DATA_SAMPLE_RADIUS)
        {
            // save the found minima
            prvData.minimaAge = prvData.foundAge;
#ifdef DEBUG_LOGGING
            prvData.minimaValue = prvData.foundValue;
            prvDebugPeakFound(prvSampleCount-prvData.minimaAge, 0, prvData.minimaValue);
#endif // DEBUG_LOGGING
            // switch to finding maxima
            prvData.searchForMaxima = 1;
            prvData.foundAge = prvData.oppositeAge;
            prvData.foundValue = prvData.oppositeValue;
            return 0; // interval not complete
        }
    }
    else
    {
        // find second maxima
        if (prvData.searchForMaxima != 1)
        {
            prvDataReset();
            prvDebugMsg(prvSampleCount, "Reset: Search in wrong state (second maxima)");
            return 0;
        }
        if (prvData.foundAge >= PRV_DATA_SAMPLE_RADIUS)
        {
            // save the found maxima
            prvData.secondMaximaAge = prvData.foundAge;
#ifdef DEBUG_LOGGING
            prvData.secondMaximaValue = prvData.foundValue;
            prvDebugPeakFound(prvSampleCount-prvData.secondMaximaAge, 1, prvData.secondMaximaValue);
            prvDebugIntervalFound(prvSampleCount, prvSampleCount-prvData.maximaAge, prvSampleCount-prvData.minimaAge, prvSampleCount-prvData.secondMaximaAge,
                (uint16_t)((uint32_t)(prvData.maximaAge-prvData.minimaAge) * 1000 / prvData.samplesPerSecond),
                (uint16_t)((uint32_t)(prvData.minimaAge-prvData.secondMaximaAge) * 1000 / prvData.samplesPerSecond));
#endif // DEBUG_LOGGING
            return 1; // interval found
        }
    }
    return 0; // interval not complete
}

///////////////////////////////////////////////////////////////////////////////
// prvDataRemoveInterval
///////////////////////////////////////////////////////////////////////////////
static void prvDataRemoveInterval(void)
{
    // switch to finding minima
    prvData.searchForMaxima = 0;
    prvData.foundAge = prvData.oppositeAge;
    prvData.foundValue = prvData.oppositeValue;
    prvData.maximaAge = prvData.secondMaximaAge;
#ifdef DEBUG_LOGGING
    prvData.maximaValue = prvData.secondMaximaValue;
#endif // DEBUG_LOGGING
    prvData.minimaAge = -1;
    prvData.secondMaximaAge = -1;
}

///////////////////////////////////////////////////////////////////////////////
// prvDataCalculateAndRemoveInterval
///////////////////////////////////////////////////////////////////////////////
uint8_t prvDataCalculateAndRemoveInterval(uint16_t *intervalMs)
{
    uint16_t intervalSamples;
#ifdef DEBUG_LOGGING
    uint16_t peakHeight;
    int32_t diff;
    int32_t adjust;
#endif // DEBUG_LOGGING

    *intervalMs = 0;
    if ((prvData.maximaAge == -1) || (prvData.minimaAge == -1) || (prvData.secondMaximaAge == -1))
    {
        // no interval -- prepare for next interval
        prvDataRemoveInterval();
        return 0; // no interval
    }
    if (prvData.secondMaximaAge >= prvData.maximaAge)
    {
        // invalid interval -- prepare for next interval
        prvDataRemoveInterval();
        return 0; // invalid interval
    }

    // compute the interval time
#ifdef PRV_AS7000_DEVICE // this should be defined for AS7000-device build only
    // check for valid trim settings, for AS7000-device only
    // [note: the handling of non-AS7000 devices is elsewhere]
    *(volatile uint32_t *)(&CCU->CCU_DEVICEID.reg) = 0x9c632e37;
    if (CCU->CCU_DEVICEID.bit.device_id == 0x1b58)
    {
        uint32_t trimWord0;
        uint32_t trimWord2;
        *(volatile uint32_t *)0x30008010 = 0x2e37639c;
        *(volatile uint32_t *)0x30008018 = 0x9c63372e;
        trimWord0 = *(volatile uint32_t *)0x30008010;
        trimWord2 = *(volatile uint32_t *)0x30008018;
        // if "intervalMs" is not in range for AS7000-RAM or trim indicates PRV-disabled, report "no-interval"
        if ((((uint32_t)intervalMs & 0xFFFFF000) != 0x20000000) ||
            ((((trimWord0 & 0x10000) >> 8) != (trimWord2 & 0x100)) && ((trimWord0 & 0xFFFFFF00) != 0)))
        {
            prvDataRemoveInterval();
            return 0; // invalid interval
        }
    }
#endif // PRV_AS7000_DEVICE
    intervalSamples = prvData.maximaAge - prvData.secondMaximaAge;
    *intervalMs = (uint16_t)((uint32_t)intervalSamples * 1000 / prvData.samplesPerSecond);
    
#ifdef DEBUG_LOGGING
    // compute the peak height (minima to maxima)
    diff = (int32_t)prvData.secondMaximaValue - prvData.maximaValue;
    adjust = diff * (prvData.minimaAge - prvData.maximaAge) / (prvData.secondMaximaAge - prvData.maximaAge);
    peakHeight = prvData.maximaValue + (uint16_t)adjust - prvData.minimaValue;
#endif // DEBUG_LOGGING

#ifdef DEBUG_LOGGING
    // log the interval data
    prvDebugIntervalData(prvSampleCount-prvData.secondMaximaAge, *intervalMs, intervalSamples, peakHeight,
                         prvData.maximaValue, prvData.secondMaximaValue,
                         prvData.maximaValue + (uint16_t)adjust, prvData.minimaValue);
#endif // DEBUG_LOGGING

    // prepare for next interval
    prvDataRemoveInterval();

    // validate the interval
    if ((*intervalMs == 0) || (*intervalMs < 360))
    {
        // invalid interval -- prepare for next interval
        prvDebugMsg(prvSampleCount, "Invalid interval");
        *intervalMs = 0;
        return 0; // invalid interval
    }
    return 1; // valid interval
}
