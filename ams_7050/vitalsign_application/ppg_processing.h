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

/*! \file ppg_processing.h
 *
 *  \brief Header for PPG data processing
 *
 *
 */

#ifndef PPG_PROCESSING_H
#define PPG_PROCESSING_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include "heartrate.h"


/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

#define PPG_BUFFER_SECONDS     10
#define PPG_VALUES_PER_SECOND  HR_ADC_VALUES_PER_SECOND
#define PPG_DATA_SIZE          (PPG_VALUES_PER_SECOND * PPG_BUFFER_SECONDS)

// FFT size defines
#define PPG_FFT_BINS                   512    // number of bins
#define PPG_FFT_POST_PROCESSING_BINS   192    // number of bins used for post-processing
#define PPG_FFT_BIN_SIZE_BYTES         4      // each bin is a complex number (real/imaginary, 16-bit each)
#define PPG_FFT_BUFFER_SIZE_BYTES      ((PPG_FFT_BINS/2) * PPG_FFT_BIN_SIZE_BYTES)

#define PPG_WINDOW_MIN_OFFSET    240
#define PPG_QUALITY_DIVISOR      16
#define PPG_SCORE_NORMALIZER     4000

#define PPG_ULP_VALUE_MAG        32
#define PPG_ULP_VALUE_DIVISOR    7
#define PPG_ULP_QUALITY_DIVISOR  32
#define PPG_ULP_SCORE_NORMALIZER 8000


// The frequency range of the FFT == 60 (one minute) * scaling * PPG_VALUES_PER_SECOND (to be divided by number of fft bins)
#define PPG_FFT_FREQ_RANGE_SCALED	((int32_t)60*HEARTRATE_ALGORITHM_SCALE_FACTOR*PPG_VALUES_PER_SECOND)

/* this macro returns the frequency for a given FFT bin index */
#define PPG_FREQUENCY_SCALED_FROM_FFT_BIN(index) ((int16_t)(DIVIDE_WITH_ROUNDING(((int32_t)index * PPG_FFT_FREQ_RANGE_SCALED), PPG_FFT_BINS)))

/* This macro returns the FFT-bin index for a given frequency (round to nearest FFT-bin) */
#define PPG_FFT_BIN_FROM_FREQUENCY(frequency) ((int16_t)(DIVIDE_WITH_ROUNDING(((int32_t)frequency*HEARTRATE_ALGORITHM_SCALE_FACTOR*PPG_FFT_BINS), PPG_FFT_FREQ_RANGE_SCALED)))
#define PPG_FFT_BIN_FROM_SCALED_FREQUENCY(frequency) ((int16_t)(DIVIDE_WITH_ROUNDING(((int32_t)frequency*PPG_FFT_BINS), PPG_FFT_FREQ_RANGE_SCALED)))



#define PPG_NUM_PEAKS  10
typedef struct
{
    uint32_t peakValue[PPG_NUM_PEAKS];
    uint8_t peakBin[PPG_NUM_PEAKS];
    uint8_t peakStartBin[PPG_NUM_PEAKS];
    uint8_t peakStopBin[PPG_NUM_PEAKS];
    uint8_t quality[PPG_NUM_PEAKS];
    uint16_t freqAtPeak[PPG_NUM_PEAKS];
    uint32_t peakScore[PPG_NUM_PEAKS];
    uint32_t magnitudeAdjustBase;
    uint32_t totalMagnitude;
    uint16_t scale;
    uint8_t firstSecondIdx;  // [0..(PPG_BUFFER_SECONDS-1),PPG_BUFFER_SECONDS]
    uint8_t lastSecondIdx;   // [0..(PPG_BUFFER_SECONDS-1)]
    uint16_t minHrScaled;
    uint16_t maxHrScaled;
} ppgProcessingResults_t;

typedef struct
{
    uint16_t minSampleValueAll;
    uint16_t maxSampleValueAll;
    uint16_t sampleDeltaMax;
    uint16_t sampleDeltaMaxPerSecond[PPG_BUFFER_SECONDS];
    uint16_t sampleDeltaAvg;
    uint16_t rangeAll;
    uint16_t minSamplePerSecond[PPG_BUFFER_SECONDS];
    uint16_t maxSamplePerSecond[PPG_BUFFER_SECONDS];
    uint16_t rangePerSecond[PPG_BUFFER_SECONDS];
    uint16_t rangeLast2;
    uint16_t rangeBeforeLast;
    uint16_t excludeSecond;  // for each bit [0..(PPG_BUFFER_SECONDS-1)]: 1 means this second of data is excluded
    uint16_t userNotPresent; // for each bit [0..(PPG_BUFFER_SECONDS-1)]: 1 means this second of data has no user present
    uint8_t firstSecondIdx;  // [0..(PPG_BUFFER_SECONDS-1),PPG_BUFFER_SECONDS]
    uint8_t lastSecondIdx;   // [0..(PPG_BUFFER_SECONDS-1)]
    uint16_t minSampleValueUsed;
    uint16_t maxSampleValueUsed;
    uint16_t scale;
} ppgRawDataStats_t;

typedef struct
{
    uint16_t mag[PPG_FFT_POST_PROCESSING_BINS];
    uint16_t adjMag[PPG_FFT_POST_PROCESSING_BINS];
    ppgProcessingResults_t ppgResults;
} ppgProcessingWorkBuffer_t;

typedef struct
{
    uint16_t peakValue[PPG_DATA_SIZE/2]; // overlaps the input-data [0..99]
    uint16_t bbiDelta[PPG_DATA_SIZE/2]; // overlaps the input-data [100..199]
    int16_t peakIndex[PPG_DATA_SIZE/2];
    uint8_t truePeak[PPG_DATA_SIZE/2];
    uint8_t bbiBin[PPG_DATA_SIZE/2];
} ppgPpiWorkBuffer_t;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

/* Initialize PPG data-processing */
void ppgProcessingInit(uint8_t crosstalkLevel64);

/* Store the new second of PPG data */
void ppgRawStoreData(uint16_t *ppgInput, uint8_t *currentChange);

/* Returns the frequency result after processing the Raw-PPG data using an FFT */
void ppgRawProcessFft(int16_t *ppgData, ppgProcessingResults_t **ppgResultsHnd);

/* Return a new heartrate PPI-result */
void ppgRawProcessPpi(uint16_t *ppgData, uint16_t *ppiHr, uint8_t *ppiQuality);

/* Return "user-present" state (0 = user-not-present) */
uint8_t ppgRawIsUserPresent(void);

#endif /* PPG_PROCESSING_H */
