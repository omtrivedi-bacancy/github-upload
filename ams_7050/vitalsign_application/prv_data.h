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

/*! \file prv_data.h
 *
 *  \author M. Zimmerman
 *
 *  \brief application specific algorithm for PRV calculation
 */

#ifndef PRV_DATA_H
#define PRV_DATA_H

#include "prv.h"
#include "prv_private.h"


typedef struct
{
    uint16_t afeDriverType    : 4;
    uint16_t samplesPerSecond : 12;
    uint16_t startup           : 1; // 1=intial-startup
    uint16_t searchForMaxima   : 1; // 1=maxima, 0=minima
    uint16_t startAge          : 14;
    uint16_t foundValue;
    uint16_t foundAge;
    uint16_t oppositeValue; // opposite since found-value
    uint16_t oppositeAge;

    int16_t maximaAge;  // (-1)==>no peak
    int16_t minimaAge;  // (-1)==>no peak
    int16_t secondMaximaAge;  // (-1)==>no peak
#ifdef DEBUG_LOGGING
    uint16_t maximaValue;
    uint16_t minimaValue;
    uint16_t secondMaximaValue;
#endif // DEBUG_LOGGING
} PrvData_t;


uint8_t prvDataInit(uint8_t afeDriverType, uint16_t samplesPerSecond);
void prvDataProcessNewSample(uint16_t sample);
uint8_t prvDataFindInterval(void);
uint8_t prvDataCalculateAndRemoveInterval(uint16_t *intervalMs);


#endif // PRV_DATA_H
