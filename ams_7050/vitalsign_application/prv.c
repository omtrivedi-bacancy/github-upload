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
 *      PROJECT:   AS7000 heartrate application
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file prv.c
 *
 *  \author M. Zimmerman
 *
 *  \brief Implementation for Pulse-Rate Variability (PRV) algorithm
 */
#include "prv.h"
#include "prv_data.h"
#include "prv_private.h"


#ifdef DEBUG_LOGGING

#include <stdio.h>
#include <string.h>

uint32_t prvSampleCount;
//void prvDebugMsg(uint32_t sampleIdx, char *str);
//void prvDebugPeakFound(uint32_t sampleIdx, uint8_t maxima, uint16_t sampleValue);
//void prvDebugIntervalFound(uint32_t sampleIdx, uint32_t firstMaxSampleIdx, uint32_t minSampleIdx, uint32_t secondMaxSampleIdx, uint16_t maxToMinMs, uint16_t minToMaxMs);
//void prvDebugIntervalData(uint32_t sampleIdx, uint16_t intervalMs, uint16_t intervalSamples, uint16_t intervalHeight, uint16_t max1, uint16_t max2, uint16_t maxAdj, uint16_t min);
//void prvDebugIntervalReported(uint32_t sampleIdx, uint16_t intervalMs);

//void prvDebugMsg(uint32_t sampleIdx, char *str)
//{
//	printf("prv msg: %d %s\r\n", sampleIdx, str);
//}

//void prvDebugPeakFound(uint32_t sampleIdx, uint8_t maxima, uint16_t sampleValue)
//{
//	printf("prv maxima found: %d %d %d\r\n", sampleIdx, maxima, sampleValue);
//}

//void prvDebugIntervalFound(uint32_t sampleIdx, uint32_t firstMaxSampleIdx, uint32_t minSampleIdx, uint32_t secondMaxSampleIdx, uint16_t maxToMinMs, uint16_t minToMaxMs)
//{
//	printf("prv interval found: %d %d %d %d %d %d\r\n", sampleIdx, firstMaxSampleIdx, minSampleIdx, secondMaxSampleIdx, maxToMinMs, minToMaxMs);
//}

//void prvDebugIntervalData(uint32_t sampleIdx, uint16_t intervalMs, uint16_t intervalSamples, uint16_t intervalHeight, uint16_t max1, uint16_t max2, uint16_t maxAdj, uint16_t min)
//{
//	printf("prv interval data: %d %d %d %d %d %d %d %d\r\n", sampleIdx, intervalMs, intervalSamples, intervalHeight, max1, max2, maxAdj, min);
//}

//void prvDebugIntervalReported(uint32_t sampleIdx, uint16_t intervalMs)
//{
//	printf("prv interval reported: %d %d\r\n", sampleIdx, intervalMs);
//}

#endif // DEBUG_LOGGING


/*!
 *****************************************************************************
 * \brief PRV algorithm initialization.
 * Must be called before calling prvCalculate.
 * \param afeDriverType - informs the algorithm which AFE is being used (PRV_AFE_DRIVER_TYPE_*)
 * \param samplesPerSecond - rate of samples being provided to the PRV algorithm 
 * \return PRV_ALGORITHM_OK - parameters were valid
 *         PRV_ALGORITHM_ERR_PARAM - invalid parameter
 *****************************************************************************
 */
uint8_t prvInitialise(uint8_t afeDriverType, uint16_t samplesPerSecond)
{
    uint8_t ret;

    ret = prvDataInit(afeDriverType, samplesPerSecond);
    if (ret == 0)
        return PRV_ALGORITHM_ERR_PARAM;
#ifdef DEBUG_LOGGING
    prvSampleCount = 0;
#endif // DEBUG_LOGGING
    return PRV_ALGORITHM_OK;
}

/*!
 *****************************************************************************
 * \brief Process recent ADC samples to find a new PRV result
 * \param adcValue - new ADC-value being provided to the PRV algorithm 
 * \param output - pointer to the result of the PRV algorithm 
 * \return PRV_ALGORITHM_VALUE_AVAILABLE - a new interval was found
 *         PRV_ALGORITHM_VALUE_NOT_AVAILABLE - no new interval found
 *         PRV_ALGORITHM_ERR_* - various PRV errors
 *****************************************************************************
 */
uint8_t prvCalculate(uint16_t adcValue, prvAlgorithmOutput *output)
{
    uint8_t result;
    uint16_t intervalMs;
  //  printf("##############  inside prvcalculate \n");
    
    if (output == 0)
        return PRV_ALGORITHM_ERR_PARAM;
    
#ifdef DEBUG_LOGGING
    prvSampleCount++;
#endif // DEBUG_LOGGING
    
    // process this sample
    prvDataProcessNewSample(adcValue);
    
    // check for complete interval    
    result = prvDataFindInterval();
    if (result == 1)
    {
        result = prvDataCalculateAndRemoveInterval(&intervalMs);
        if (result == 1)
        {
            /* new peak-to-peak interval found */
		    output->peakToPeakIntervalMs = intervalMs;
#ifdef DEBUG_LOGGING
            prvDebugIntervalReported(prvSampleCount, intervalMs);
#endif // DEBUG_LOGGING
            return PRV_ALGORITHM_VALUE_AVAILABLE;
        }
    }

    /* no new peak-to-peak interval */            
    output->peakToPeakIntervalMs = 0;
   // printf("##############  inside prvcalculate %d\n",result);
    return PRV_ALGORITHM_VALUE_NOT_AVAILABLE;

}
