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

/*! \file track_hr.h
 *
 *  \brief Header for Track-HR processing
 *
 *
 */

#ifndef TRACK_HR_H
#define TRACK_HR_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include "heartrate.h"
#include "acc_processing.h"
#include "ppg_processing.h"

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

// Structure used for Track-HR results (global -- maintained across multiple seconds)
typedef struct
{
    uint16_t hrResult; // [scaled]
    int8_t selectedTrackIdx;     // (-1 = no-selection)
    uint8_t selectedTrackTime;
} trackHrResults_t;

// Structure used for an individual node of a track
#define TRACK_NODE_MAX_SCORE  2000

typedef struct
{
    uint16_t freq;            // [scaled]
    uint16_t score      : 11; // [0..TRACK_NODE_MAX_SCORE] (saturated)
    uint16_t motionHarm : 3;  // [0 if not a motion-harmonic]
    uint16_t unused     : 2;
} trackNode_t;

// Structure used for a track
#define TRACK_NUM_NODES  10
#define TRACK_COUNT      16
typedef struct
{
    trackNode_t node[TRACK_NUM_NODES]; // most recent data is in [0]
    uint16_t freq;
    uint8_t age             : 7; // (limit to 100)
    uint8_t selectionBranch : 1; // set if this track is a branch of the tree of selection-tracks (set when created or selected)
    int8_t harmTrackIdx     : 5; // index of track that is 2X match for this harmonic (-1 if harmonic not found)
    uint8_t motionHarm      : 3; // [0 if not a motion-harmonic]
} track_t;


// Structure used for Track-HR info (temporary-buffer -- used to reduce stack usage)
#define TRACK_HR_NUM_PPG_PEAKS  5  // must not be greater than PPG_NUM_PEAKS
typedef struct
{
    // PPI-processing results (input to Track-HR)
    uint16_t ppiHr;
    uint8_t ppiQuality;

    // Track-HR processing (temporary-info)
    uint8_t motionFreqQual;
    uint16_t motionFrequency;
    uint16_t previousHr;
    uint16_t expectedHr;
    uint16_t minExpectedHr;
    uint16_t maxExpectedHr;

    uint16_t fftFreq;
    uint8_t fftQual;
    uint16_t adjFftFreq;
    uint16_t fraction;

    // Results
    uint16_t resultHr;
    uint8_t resultSqi;
    uint8_t reportedSqi;

    // the following fields are used to find the best PPG peak (temporary-info)
    uint16_t peakDistanceHr[TRACK_HR_NUM_PPG_PEAKS];
    uint32_t peakDistanceScore[TRACK_HR_NUM_PPG_PEAKS];
    uint8_t peakUsed[TRACK_HR_NUM_PPG_PEAKS];
    int8_t closestPeak[TRACK_HR_NUM_PPG_PEAKS]; // peak-index [0..(TRACK_HR_NUM_PPG_PEAKS-1)]
    int8_t closestNonHarmonicIdx;
    int8_t closestHarmonicIdx; // H-2 only
    int8_t selectedPeakIdx;

    // the following fields are used to match PPG/Acc peaks (temporary-info)
    int8_t accMatchingPeak[PPG_NUM_PEAKS]; // index of acc-peak that matches this ppg-peak (-1 if no match)
    uint8_t motionHarmonic[PPG_NUM_PEAKS]; // the motion-harmonic of this ppg-peak (0 if no motion-harmonic)

    // the following fields are used to maintain tracks (temporary-info)
    uint16_t trackScoreRaw[TRACK_COUNT]; // raw-score
    uint16_t trackLastFreq[TRACK_COUNT]; // last match
    uint16_t trackScoreNonMH[TRACK_COUNT]; // recent consecutive non-MH nodes
    uint16_t trackScoreFinal[TRACK_COUNT];
    uint16_t trackIsNew; // one-bit per track
    uint16_t peakMatchedToTrack; // one-bit per peak

    uint16_t maxScoreNonMH;
    uint16_t maxScoreMH2X;
    uint16_t maxScoreFinal;
    uint16_t maxFastLockScoreNonMH;
    uint16_t maxFastLockScoreFinal;
    uint16_t selScoreFinal;
    int8_t trackIdxMaxScoreNonMH; // the best non-MH track (-1 = no-selection)
    int8_t trackIdxMaxScoreMH2X;  // the best MH2X track (-1 = no-selection)
    int8_t trackIdxMaxScoreFinal; // the best track overall (-1 = no-selection)
    int8_t trackIdxMaxFastLockScoreNonMH; // the best non-MH track (-1 = no-selection)
    int8_t trackIdxMaxFastLockScoreFinal; // the best track during selection (-1 = no-selection)
    int8_t selectedTrackIdx;      // the selected-track (-1 = no-selection) -- previous-selection may be chosen

    // the following fields are used for peak/track matching (temporary-info)
    int8_t selPeakBestTrackIdx;  // the track with the best score with the selected-peak
    uint8_t selTrackPeakMatches;  // number of nodes with peaks in the selected-track
    uint8_t selTrackPeakMatchesNonMH;  // number of nodes with non-motion peaks in the selected-track
    uint8_t selTrackSelPeakMatchFound; // true if selected-track is matched with selected-peak

    uint8_t switchToSelTrack; // true if selected-track should be used instead of selected-peak

} trackHrInfo_t;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

/* Initialize track-hr processing */
void trackHrInit(void);

/* Returns the final frequency result after processing new Motion/FFT results */
void trackHrProcess(accMotionResults_t *accMotion, ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo);

#endif /* TRACK_HR_H */
