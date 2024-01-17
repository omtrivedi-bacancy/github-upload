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

/*! \file adcData.h
 *
 *  \author M. Zimmerman
 *
 *  \brief application specific algorithm for SpO2 calculation
 */

#ifndef ADC_DATA_H
#define ADC_DATA_H

#include "spo2.h"

#define ADC_DATA_SECONDS 3
#define ADC_DATA_BUFFER_SIZE (ADC_DATA_SECONDS * SPO2_ADC_VALUES_PER_SECOND)

typedef struct {
    uint16_t sample[ADC_DATA_BUFFER_SIZE];
    uint16_t dcData[ADC_DATA_BUFFER_SIZE];
    int8_t startIdx;
    int8_t firstMaxPeakIdx;  // (-1)==>no peak
    int8_t minPeakIdx;       // (-1)==>no peak
    int8_t secondMaxPeakIdx; // (-1)==>no peak
    uint8_t red;             // 1=red, 0=infrared
} AdcData_t;

void adcDataSetAfeDriverType(uint8_t spo2AfeDriverType);
void adcDataInit(AdcData_t *adcData, uint8_t red);
void adcDataBufferNewSecondOfData(AdcData_t *adcData, spo2LedData_t *newData);
int8_t adcDataFindInterval(AdcData_t *adcData);
int8_t adcDataCalculateAndRemoveInterval(AdcData_t *adcData, uint16_t *ac, uint16_t *dc);

#endif // ADC_DATA_H
