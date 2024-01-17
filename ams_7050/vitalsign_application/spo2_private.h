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

/*! \file spo2_private.h
 *
 *  \author M. Zimmerman
 *
 *  \brief defines, types, functions that must not be exported to the public
 * interface
 */

#ifndef SPO2_PRIVATE_H
#define SPO2_PRIVATE_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include "spo2.h"


/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

extern uint32_t spo2SecondCounter;
extern uint8_t spo2AfeDriverType;


/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */
 
typedef struct
{
    uint8_t interval;
    uint32_t endTime;
    uint16_t acRed;
    uint16_t dcRed;
    uint16_t acInfrared;
    uint16_t dcInfrared;
    uint16_t dcAmbient;
} spo2Interval_t;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

//#define SPO2_DEBUG_LOGGING
#ifdef SPO2_DEBUG_LOGGING
void spo2DebugPeakFound(uint8_t red, uint8_t maxima, uint32_t time, int8_t sampleIndex);
void spo2DebugIntervalFound(uint8_t red, uint32_t time, int8_t firstMaxPeakIdx, int8_t minPeakIdx, int8_t secondMaxPeakIdx);
void spo2DebugIntervalData(uint8_t red, uint32_t time, int8_t sampleIndex, uint16_t ac, uint16_t dc, uint16_t max1, uint16_t max2, uint16_t maxAdj, uint16_t min);
void spo2DebugIntervalRemoved(uint8_t red, uint32_t time, int8_t sampleIndex);
void spo2DebugIntervalResult(uint32_t time, int8_t sampleIndex, uint16_t k, uint8_t samples);
#else
#define spo2DebugPeakFound(red, maxima, time, sampleIndex)
#define spo2DebugIntervalFound(red, time, firstMaxPeakIdx, minPeakIdx, secondMaxPeakIdx)
#define spo2DebugIntervalData(red, time, sampleIndex, ac, dc, max1, max2, maxAdj, min)
#define spo2DebugIntervalRemoved(red, time, sampleIndex)
#define spo2DebugIntervalResult(time, sampleIndex, k, samples)
#endif


#endif /* SPO2_PRIVATE_H */
