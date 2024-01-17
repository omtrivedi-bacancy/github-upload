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

/*! \file acc_processing.h
 *
 *  \brief Header for accelerometer data processing
 *
 *
 */

#ifndef ACC_PROCESSING_H
#define ACC_PROCESSING_H

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

#define ACC_MIN_FREQUENCY_TIMES_1000      10000 // 10.0Hz
#define ACC_MAX_FREQUENCY_TIMES_1000      20000 // 20.0Hz

#define ACC_DATA_BUFFER_SECONDS      10
#define ACC_DATA_BUFFER_MAX_SAMPLES  ((uint16_t)(ACC_MAX_FREQUENCY_TIMES_1000 * ACC_DATA_BUFFER_SECONDS / 1000))
#define ACC_DATA_BUFFER_SIZE_BYTES   (((ACC_DATA_BUFFER_MAX_SAMPLES * 12) + 7) / 8) // 12-bits per sample

// FFT size defines
#define ACC_FFT_BINS                   512    // number of bins
#define ACC_FFT_POST_PROCESSING_BINS   192    // number of bins used for post-processing
#define ACC_FFT_BIN_SIZE_BYTES         4      // each bin is a complex number (real/imaginary, 16-bit each)
#define ACC_FFT_BUFFER_SIZE_BYTES      ((ACC_FFT_BINS/2) * ACC_FFT_BIN_SIZE_BYTES)


#define ACC_NUM_PEAKS  5
typedef struct
{
    uint8_t peakBin[ACC_NUM_PEAKS];
    uint8_t peakStartBin[ACC_NUM_PEAKS];
    uint8_t peakStopBin[ACC_NUM_PEAKS];
    uint32_t peakValue[ACC_NUM_PEAKS];
    uint16_t freqAtPeak[ACC_NUM_PEAKS];
    uint8_t quality[ACC_NUM_PEAKS];
    uint8_t harmonic[ACC_NUM_PEAKS];
    uint32_t magnitudeAdjustBase;
    uint32_t totalMagnitude;
    uint16_t scale;
    uint32_t maxScore;
    uint8_t selectedIdx;
    uint8_t freqQuality;
    uint16_t freqResult;
} accProcessingResults_t;

typedef struct
{
    uint16_t mag[ACC_FFT_POST_PROCESSING_BINS];
    uint16_t adjMag[ACC_FFT_POST_PROCESSING_BINS];
    accProcessingResults_t accResults;
} accProcessingWorkBuffer_t;

typedef struct
{
    uint16_t freqAtPeak[ACC_NUM_PEAKS];
    uint8_t quality[ACC_NUM_PEAKS];
    uint8_t selectedIdx;
    uint8_t freqQuality; // [0..100]
    uint16_t freqResult; // (scaled)
} accMotionResults_t;

#define ACC_MIN_PEAK_VALUE 75

/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

/* Initialize accelerometer data-processing */
void accProcessingInit(uint16_t sampleFreq1000);

/* Combine the data for the 3 accelerometer-axes and store result */
void accStoreCombinedAxisData(int16_t *accX, int16_t *accY, int16_t *accZ, uint8_t numSamples);

/* returns the motion-frequency of the accelerometer-data */
void accFindMotionFreq(int16_t *accData, accMotionResults_t *accMotion);


#endif /* ACC_PROCESSING_H */
