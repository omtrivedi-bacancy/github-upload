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

/*! \file prv_private.h
 *
 *  \author M. Zimmerman
 *
 *  \brief defines, types, funcations that must not be exported to the public
 * interface
 */

#ifndef PRV_PRIVATE_H
#define PRV_PRIVATE_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include "prv.h"
#include <stdint.h>
#ifdef PRV_AS7000_DEVICE // this should be defined for AS7000-device build only
#include "AS7000.h" // needed for AS7000-device builds only
#endif

/*
 *****************************************************************************
 * GLOBALS
 *****************************************************************************
 */
#ifdef DEBUG_LOGGING
extern uint32_t prvSampleCount;
#endif // DEBUG_LOGGING


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
#ifdef DEBUG_LOGGING
// The following functions must be defined by the application for debugging only
void prvDebugMsg(uint32_t sampleIdx, char *str);
void prvDebugPeakFound(uint32_t sampleIdx, uint8_t maxima, uint16_t sampleValue);
void prvDebugIntervalFound(uint32_t sampleIdx, uint32_t firstMaxSampleIdx, uint32_t minSampleIdx, uint32_t secondMaxSampleIdx, uint16_t maxToMinMs, uint16_t minToMaxMs);
void prvDebugIntervalData(uint32_t sampleIdx, uint16_t intervalMs, uint16_t intervalSamples, uint16_t intervalHeight, uint16_t max1, uint16_t max2, uint16_t maxAdj, uint16_t min);
void prvDebugIntervalReported(uint32_t sampleIdx, uint16_t intervalMs);
#else // DEBUG_LOGGING
#define prvDebugMsg(sampleIdx, str)
#define prvDebugPeakFound(sampleIdx, maxima, sampleValue)
#define prvDebugIntervalFound(sampleIdx, firstMaxSampleIdx, minSampleIdx, secondMaxSampleIdx, maxToMinMs, minToMaxMs)
#define prvDebugIntervalData(sampleIdx, intervalMs, intervalSamples, intervalHeight, max1, max2, maxAdj, min)
#define prvDebugIntervalReported(sampleIdx, intervalMs)
#endif // DEBUG_LOGGING


#endif /* PRV_PRIVATE_H */
