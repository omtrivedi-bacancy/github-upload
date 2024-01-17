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

/*! \file track_hr.c
 *
 *  \brief Track-HR processing
 *
 *
 */

#include "track_hr.h"
#include "ppg_processing.h"
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
    
// storage for the Track-HR history/results
static trackHrResults_t trackHrResults;

// multi-track storage (file-global -- maintained across multiple seconds)
static track_t tracks[TRACK_COUNT];


    
/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
static void trackInit(track_t *track);
static void trackNodeInit(trackNode_t *trackNode);
    
    

/*!
 *****************************************************************************
 * \brief Initialize Track-HR results
 *****************************************************************************
 */
void trackHrInit(void)
{
    int8_t track;

    memset(&trackHrResults, 0, sizeof(trackHrResults));
    trackHrResults.hrResult = DEFAULT_HR_VALUE;
    trackHrResults.selectedTrackIdx = -1;
    trackHrResults.selectedTrackTime = 0;
    memset(&tracks, 0, sizeof(tracks));
    for (track=0; track<TRACK_COUNT; track++)
    {
        trackInit(&tracks[track]);
    }
}


/*!
 *****************************************************************************
 * \brief Initialize the track
 *****************************************************************************
 */
static void trackInit(track_t *track)
{
    int8_t node;

    track->freq = 0;
    track->age = 0;
    track->harmTrackIdx = -1;
    track->selectionBranch = 0;
    track->motionHarm = 0;
    for (node=0; node<TRACK_NUM_NODES; node++)
    {
        trackNodeInit(&track->node[node]);
    }
}

/*!
 *****************************************************************************
 * \brief Initialize the track-node
 *****************************************************************************
 */
static void trackNodeInit(trackNode_t *trackNode)
{
    trackNode->freq = 0;
    trackNode->score = 0;
    trackNode->motionHarm = 0;
}


/*!
 *****************************************************************************
 * \brief Update the frequency for the track
 *
 * \param track - the track to update
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackUpdateFreq(track_t *track, trackHrInfo_t *trackHrInfo)
{
    uint32_t freq;
    uint16_t fraction;
    uint16_t maxScore;
    int8_t node;
    uint32_t base;

    // Compute the Expected-Freq based on Prev-Frequencies and motion
    freq = track->freq;
    if (freq == 0)
    {
        freq = track->node[0].freq;
    }
    else
    {
        // compute the expected-freq
        fraction = 0;
        if (trackHrInfo->motionFrequency > 300) // 30bpm
        {
            if (((uint32_t)trackHrInfo->motionFrequency*2) > freq) // expecting increase
            {
                fraction = (trackHrInfo->motionFrequency - 300) / 40; // higher motion-freq ==> faster HR increase
                freq = ((freq * (1000-fraction)) + (((uint32_t)(trackHrInfo->motionFrequency * 2) * fraction)) + 500) / 1000;
            }
            else // expecting decrease
            {
                fraction = 0;
            }
        }
        // adjust the freq based on new freq-data
        if (track->node[0].freq != 0)
        {
            maxScore = 0;
            for (node=0; node<TRACK_NUM_NODES; node++)
            {
                if (track->node[node].score > maxScore)
                    maxScore = track->node[node].score;
            }
            base = maxScore * 3 / 2; // increase the weight for consistently low-level signals
            if (base > 1000)
                base = 1000;
            fraction = track->node[0].score;
            freq = ((freq * (base - fraction)) + (track->node[0].freq * fraction) + (base/2)) / base;
        }
    }
    track->freq = (uint16_t)freq;
}

/*!
 *****************************************************************************
 * \brief Maintain tracks using the new PPG-results
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrMaintainTracks(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t track;
    int8_t node;
    int8_t ppgPeak;
    int8_t newTrackIdx;

    // prepare for matching peaks to tracks
    for (track=0; track<TRACK_COUNT; track++)
    {
        // shift the nodes to prepare for new ones
        for (node=TRACK_NUM_NODES-1; node>0; node--)
            tracks[track].node[node] = tracks[track].node[node-1];
        // clear the "new" node
        trackNodeInit(&tracks[track].node[0]);
        // determine the current track-score
        trackHrInfo->trackScoreRaw[track] = 0;
        for (node=1; node<TRACK_NUM_NODES; node++)
            trackHrInfo->trackScoreRaw[track] += tracks[track].node[node].score;
        // determine the last frequency matched (if any)
        trackHrInfo->trackLastFreq[track] = 0;
        for (node=1; node<TRACK_NUM_NODES; node++)
        {
            if (tracks[track].node[node].freq != 0)
            {
                trackHrInfo->trackLastFreq[track] = tracks[track].node[node].freq;
                break;
            }
        }
        // Check for expired track
        if (trackHrInfo->trackLastFreq[track] == 0)
        {
            trackInit(&tracks[track]);
            trackHrInfo->trackScoreRaw[track] = 0;
        }
    }
    trackHrInfo->trackIsNew = 0; // one-bit per track
    trackHrInfo->peakMatchedToTrack = 0; // one-bit per peak

    // match peaks to tracks
    for (ppgPeak=0; ppgPeak<PPG_NUM_PEAKS; ppgPeak++)
    {
        if (ppgResults->freqAtPeak[ppgPeak] == 0)
            break;
        // find all matching tracks
        for (track=0; track<TRACK_COUNT; track++)
        {
            if (tracks[track].freq == 0)
                continue;
            if ((trackHrInfo->trackIsNew & (1 << track)) != 0) // new-track
                continue; // do not match to brand-new track
            if (((ppgResults->freqAtPeak[ppgPeak] >= (tracks[track].freq - 100)) && (ppgResults->freqAtPeak[ppgPeak] <= (tracks[track].freq + 100))) ||
                ((ppgResults->freqAtPeak[ppgPeak] >= (trackHrInfo->trackLastFreq[track] - 100)) && (ppgResults->freqAtPeak[ppgPeak] <= (trackHrInfo->trackLastFreq[track] + 100))))
            {
                uint8_t needDuplicate;
                needDuplicate = 0;
                if (tracks[track].node[0].freq != 0)
                {
                    // the track was already matched -- create a duplicate (branch)
                    needDuplicate = 1;
                }
                else if (track == trackHrResults.selectedTrackIdx)
                {
                    // the matching track is the previous-selected-track
                    // create a duplicate if the peak is a motion-harmonic and the track was not
                    if ((tracks[track].motionHarm == 0) && (trackHrInfo->motionHarmonic[ppgPeak] != 0))
                        needDuplicate = 1;
                }

                if (needDuplicate == 0) // add node to existing track
                {
                    tracks[track].node[0].freq = ppgResults->freqAtPeak[ppgPeak];
                    if (ppgResults->peakValue[ppgPeak] == 0)
                        tracks[track].node[0].score = 1;
                    else if (ppgResults->peakValue[ppgPeak] > TRACK_NODE_MAX_SCORE)
                        tracks[track].node[0].score = TRACK_NODE_MAX_SCORE;
                    else
                        tracks[track].node[0].score = ppgResults->peakValue[ppgPeak];
                    tracks[track].node[0].motionHarm = trackHrInfo->motionHarmonic[ppgPeak];
                    trackHrInfo->trackScoreRaw[track] += tracks[track].node[0].score;
                    trackHrInfo->peakMatchedToTrack |= (1 << ppgPeak);
                }
                else // create a duplicate (branch)
                {
                    // find an unused track (if there is one)
                    for (newTrackIdx=0; newTrackIdx<TRACK_COUNT; newTrackIdx++)
                    {
                        if (tracks[newTrackIdx].freq == 0)
                            break;
                    }
                    if (newTrackIdx < TRACK_COUNT) // an unused track was found
                    {
                        // copy the track
                        tracks[newTrackIdx] = tracks[track];
                        trackHrInfo->trackScoreRaw[newTrackIdx] = trackHrInfo->trackScoreRaw[track];
                        trackHrInfo->trackScoreRaw[newTrackIdx] -= tracks[newTrackIdx].node[0].score; // remove score from previous match
                        // place this peak into the copy
                        tracks[newTrackIdx].node[0].freq = ppgResults->freqAtPeak[ppgPeak];
                        if (ppgResults->peakValue[ppgPeak] == 0)
                            tracks[newTrackIdx].node[0].score = 1;
                        else if (ppgResults->peakValue[ppgPeak] > TRACK_NODE_MAX_SCORE)
                            tracks[newTrackIdx].node[0].score = TRACK_NODE_MAX_SCORE;
                        else
                            tracks[newTrackIdx].node[0].score = ppgResults->peakValue[ppgPeak];
                        tracks[newTrackIdx].node[0].motionHarm = trackHrInfo->motionHarmonic[ppgPeak];
                        trackHrInfo->trackScoreRaw[newTrackIdx] += tracks[newTrackIdx].node[0].score;
                        trackHrInfo->peakMatchedToTrack |= (1 << ppgPeak);
                        trackHrInfo->trackIsNew += (1 << newTrackIdx);
                    }
                }
            }
        }

        // create new track if peak was not matched
        if ((trackHrInfo->peakMatchedToTrack & (1 << ppgPeak)) == 0) // unmatched
        {
            // find an unused track (if there is one)
            for (newTrackIdx=0; newTrackIdx<TRACK_COUNT; newTrackIdx++)
            {
                if (tracks[newTrackIdx].freq == 0)
                    break;
            }
            if (newTrackIdx < TRACK_COUNT) // an unused track was found
            {
                tracks[newTrackIdx].node[0].freq = ppgResults->freqAtPeak[ppgPeak];
                if (ppgResults->peakValue[ppgPeak] == 0)
                    tracks[newTrackIdx].node[0].score = 1;
                else if (ppgResults->peakValue[ppgPeak] > TRACK_NODE_MAX_SCORE)
                    tracks[newTrackIdx].node[0].score = TRACK_NODE_MAX_SCORE;
                else
                    tracks[newTrackIdx].node[0].score = ppgResults->peakValue[ppgPeak];
                tracks[newTrackIdx].node[0].motionHarm = trackHrInfo->motionHarmonic[ppgPeak];
                tracks[newTrackIdx].age = 0;
                tracks[newTrackIdx].freq = tracks[newTrackIdx].node[0].freq;
                trackHrInfo->trackScoreRaw[newTrackIdx] = tracks[newTrackIdx].node[0].score;
                trackHrInfo->trackIsNew += (1 << newTrackIdx);
            }
        }
    }

    // update all tracks
    for (track=0; track<TRACK_COUNT; track++)
    {
        // update the frequency for the track
        trackUpdateFreq(&tracks[track], trackHrInfo);
        // increment the age of the track
        if (trackHrInfo->trackScoreRaw[track] != 0)
        {
            if (tracks[track].age < 100)
                tracks[track].age++;
        }
        // determine the last frequency matched (if any) -- needs to be refreshed here
        trackHrInfo->trackLastFreq[track] = 0;
        for (node=0; node<TRACK_NUM_NODES; node++)
        {
            if (tracks[track].node[node].freq != 0)
            {
                trackHrInfo->trackLastFreq[track] = tracks[track].node[node].freq;
                break;
            }
        }
    }

    // eliminate very poor tracks and duplicate tracks
    for (track=0; track<TRACK_COUNT; track++)
    {
        uint16_t consecutiveMissedMatches;
        if (tracks[track].freq == 0)
            continue; // not active
        // eliminate tracks with very low scores
        consecutiveMissedMatches = 0;
        for (node=0; node<TRACK_NUM_NODES; node++)
        {
            if (tracks[track].node[node].freq != 0)
                break;
            consecutiveMissedMatches++;
        }
        if (trackHrInfo->trackScoreRaw[track] < (consecutiveMissedMatches*10))
        {
            // remove the track
            trackInit(&tracks[track]);
            trackHrInfo->trackScoreRaw[track] = 0;
        }
        // eliminate duplicate tracks (the tracks can merge over time)
        for (newTrackIdx=track+1; newTrackIdx<TRACK_COUNT; newTrackIdx++)
        {
            if (tracks[newTrackIdx].freq == 0)
                continue; // not active
            // check for duplicate tracks:  within 2.3bpm OR 1% of each other, AND equal "LastFreq"
            if ((((tracks[track].freq >= (tracks[newTrackIdx].freq - 23)) && (tracks[track].freq <= (tracks[newTrackIdx].freq + 23))) ||
                 ((tracks[track].freq >= (tracks[newTrackIdx].freq - (tracks[newTrackIdx].freq/100))) && (tracks[track].freq <= (tracks[newTrackIdx].freq + (tracks[newTrackIdx].freq/100))))) &&
                (trackHrInfo->trackLastFreq[track] == trackHrInfo->trackLastFreq[newTrackIdx]))
            {
                // select which track to remove: the newer track (or lower-score)
                int8_t removedTrackIdx;
                int8_t keepTrackIdx;
                removedTrackIdx = newTrackIdx;
                if ((tracks[track].age >= 10) && (tracks[newTrackIdx].age >= 10))
                {
                    if (trackHrInfo->trackScoreRaw[track] < trackHrInfo->trackScoreRaw[newTrackIdx])
                        removedTrackIdx = track;
                }
                else if (tracks[track].age < tracks[newTrackIdx].age)
                    removedTrackIdx = track;
                else if (tracks[track].age > tracks[newTrackIdx].age)
                    removedTrackIdx = newTrackIdx;
                else if (trackHrInfo->trackScoreRaw[track] < trackHrInfo->trackScoreRaw[newTrackIdx])
                    removedTrackIdx = track;
                else if (trackHrInfo->trackScoreRaw[track] > trackHrInfo->trackScoreRaw[newTrackIdx])
                    removedTrackIdx = newTrackIdx;
                else // same age and score -- just pick one
                    removedTrackIdx = newTrackIdx;
                if (removedTrackIdx == trackHrResults.selectedTrackIdx)
                    continue; // do not remove the previously-selected track
                keepTrackIdx = track;
                if (removedTrackIdx == track)
                    keepTrackIdx = newTrackIdx;
                // preserve "selectionBranch" from either track
                tracks[keepTrackIdx].selectionBranch |= tracks[removedTrackIdx].selectionBranch;
                // clear the removed track
                trackInit(&tracks[removedTrackIdx]);
                trackHrInfo->trackScoreRaw[removedTrackIdx] = 0;
            }
        }
    }

    // find motion-harmonics (determine if each track is a motion-harmonic)
    for (track=0; track<TRACK_COUNT; track++)
    {
        // use the latest matched-node (or the second-to-last matched-node)
        tracks[track].motionHarm = 0;
        for (node=0; node<TRACK_NUM_NODES; node++)
        {
            if (tracks[track].node[node].freq != 0)
            {
                tracks[track].motionHarm = (uint8_t)tracks[track].node[node].motionHarm;
                break;
            }
        }
        if (tracks[track].motionHarm == 0)
        {
            for (node++; node<TRACK_NUM_NODES; node++)
            {
                if (tracks[track].node[node].freq != 0)
                {
                    tracks[track].motionHarm = (uint8_t)tracks[track].node[node].motionHarm;
                    break;
                }
            }
        }
        // generate non-MH score for each track (recent consecutive non-MH nodes)
        trackHrInfo->trackScoreNonMH[track] = 0;
        for (node=0; node<TRACK_NUM_NODES; node++)
        {
            if (tracks[track].node[node].motionHarm != 0)
                break;
            trackHrInfo->trackScoreNonMH[track] += tracks[track].node[node].score;
        }
    }

    // find track-harmonics (find index of 2X match)
    for (track=0; track<TRACK_COUNT; track++)
    {
        tracks[track].harmTrackIdx = -1;
        if (tracks[track].freq == 0)
            continue;
        for (newTrackIdx=0; newTrackIdx<TRACK_COUNT; newTrackIdx++)
        {
            if ((newTrackIdx == track) || (tracks[newTrackIdx].freq == 0))
                continue;
            if (tracks[newTrackIdx].motionHarm != 0) // do not associate motion-harmonic tracks
                continue;
            if ((trackHrInfo->trackScoreRaw[track] < (trackHrInfo->trackScoreRaw[newTrackIdx] / 4)) && (timeSinceUserPresent > 4))
                continue;
            if (trackHrInfo->trackScoreRaw[track] < (trackHrInfo->trackScoreRaw[newTrackIdx] / 8))
                continue; // relax the minimum-relative-score requirement for the first few seconds
            if ((((tracks[newTrackIdx].freq / 2) >= (tracks[track].freq - 50)) && ((tracks[newTrackIdx].freq / 2) <= (tracks[track].freq + 50))) ||
                (((tracks[newTrackIdx].freq / 2) >= (trackHrInfo->trackLastFreq[track] - 50)) && ((tracks[newTrackIdx].freq / 2) <= (trackHrInfo->trackLastFreq[track] + 50))) ||
                (((trackHrInfo->trackLastFreq[newTrackIdx] / 2) >= (tracks[track].freq - 50)) && ((trackHrInfo->trackLastFreq[newTrackIdx] / 2) <= (tracks[track].freq + 50))) ||
                (((trackHrInfo->trackLastFreq[newTrackIdx] / 2) >= (trackHrInfo->trackLastFreq[track] - 50)) && ((trackHrInfo->trackLastFreq[newTrackIdx] / 2) <= (trackHrInfo->trackLastFreq[track] + 50))))
            {
                if (tracks[track].harmTrackIdx >= 0) // a harmonic has previously been found
                {
                    // check if the new pairing has a higher score
                    if (tracks[track].motionHarm == 0)
                    {
                        if ((trackHrInfo->trackScoreNonMH[track] + trackHrInfo->trackScoreNonMH[tracks[track].harmTrackIdx]) > (trackHrInfo->trackScoreNonMH[track] + trackHrInfo->trackScoreNonMH[newTrackIdx]))
                            continue; // the previously found match has a higher score -- just continue
                    }
                    else
                    {
                        if ((trackHrInfo->trackScoreRaw[track] + trackHrInfo->trackScoreRaw[tracks[track].harmTrackIdx]) > (trackHrInfo->trackScoreRaw[track] + trackHrInfo->trackScoreRaw[newTrackIdx]))
                            continue; // the previously found match has a higher score -- just continue
                    }
                }
                tracks[track].harmTrackIdx = newTrackIdx; // [newTrackIdx] is a 2X harmonic of [track]
            }
        }
    }

    // generate final-score for each track
    trackHrInfo->maxScoreNonMH = 0;
    trackHrInfo->trackIdxMaxScoreNonMH = -1;
    trackHrInfo->maxScoreMH2X = 0;
    trackHrInfo->trackIdxMaxScoreMH2X = -1;
    trackHrInfo->maxScoreFinal = 0;
    trackHrInfo->trackIdxMaxScoreFinal = -1;
    for (track=0; track<TRACK_COUNT; track++)
    {
        if (tracks[track].motionHarm == 0)
        {
            trackHrInfo->trackScoreFinal[track] = trackHrInfo->trackScoreNonMH[track];
            if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER) // only add harmonic-scores for LP
            {
                // check for harmonic
                if (tracks[track].harmTrackIdx >= 0)
                {
                    newTrackIdx = tracks[track].harmTrackIdx;
                    trackHrInfo->trackScoreFinal[track] += trackHrInfo->trackScoreNonMH[newTrackIdx];
                    // check for harmonic of the harmonic
                    if (tracks[newTrackIdx].harmTrackIdx >= 0)
                        trackHrInfo->trackScoreFinal[track] += trackHrInfo->trackScoreNonMH[tracks[newTrackIdx].harmTrackIdx];
                }
            }
        }
        else
        {
            trackHrInfo->trackScoreFinal[track] = trackHrInfo->trackScoreRaw[track];
            if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER) // only add harmonic-scores for LP
            {
                // check for harmonic
                if (tracks[track].harmTrackIdx >= 0)
                {
                    newTrackIdx = tracks[track].harmTrackIdx;
                    trackHrInfo->trackScoreFinal[track] += trackHrInfo->trackScoreRaw[newTrackIdx];
                    // check for harmonic of the harmonic
                    if (tracks[newTrackIdx].harmTrackIdx >= 0)
                        trackHrInfo->trackScoreFinal[track] += trackHrInfo->trackScoreRaw[tracks[newTrackIdx].harmTrackIdx];
                }
            }
        }

        if (trackHrInfo->trackScoreNonMH[track] > trackHrInfo->maxScoreNonMH)
        {
            trackHrInfo->maxScoreNonMH = trackHrInfo->trackScoreNonMH[track];
            trackHrInfo->trackIdxMaxScoreNonMH = track;
        }
        if ((tracks[track].motionHarm == 2) && (trackHrInfo->trackScoreFinal[track] > trackHrInfo->maxScoreMH2X))
        {
            trackHrInfo->maxScoreMH2X = trackHrInfo->trackScoreFinal[track];
            trackHrInfo->trackIdxMaxScoreMH2X = track;
        }
        if (trackHrInfo->trackScoreFinal[track] > trackHrInfo->maxScoreFinal)
        {
            trackHrInfo->maxScoreFinal = trackHrInfo->trackScoreFinal[track];
            trackHrInfo->trackIdxMaxScoreFinal = track;
        }
    }

    // clear the final-selection info
    trackHrInfo->maxFastLockScoreNonMH = 0;
    trackHrInfo->trackIdxMaxFastLockScoreNonMH = -1;
    trackHrInfo->maxFastLockScoreFinal = 0;
    trackHrInfo->trackIdxMaxFastLockScoreFinal = -1;
    trackHrInfo->selScoreFinal = 0;
}

/*!
 *****************************************************************************
 * \brief Select the track to use for the fast-lock result
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrFastLockSelect(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t track;

    // select the track to use for fast-lock
    if (trackHrInfo->motionFrequency == 0)
    {
        // static...
        for (track=0; track<TRACK_COUNT; track++)
        {
            if (trackHrInfo->trackScoreFinal[track] > trackHrInfo->maxFastLockScoreFinal)
            {
                // do not select tracks with frequency above the max supported
                if ((tracks[track].freq >= ppgResults->maxHrScaled) || (tracks[track].freq <= ppgResults->minHrScaled))
                    continue;
                // do not select high-frequencies with low-scores during first few seconds of operation
                if ((timeSinceUserPresent <= 2) && (tracks[track].freq >= 1000) && (trackHrInfo->trackScoreRaw[track] < 200))
                    continue;
                if ((timeSinceUserPresent <= 4) && (tracks[track].freq >= 1300) && (trackHrInfo->trackScoreRaw[track] < 200))
                    continue;
                trackHrInfo->maxFastLockScoreFinal = trackHrInfo->trackScoreFinal[track];
                trackHrInfo->trackIdxMaxFastLockScoreFinal = track;
            }
        }
        // select the track with the max-score
        trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxFastLockScoreFinal;
    }
    else
    {
        // dynamic...
        for (track=0; track<TRACK_COUNT; track++)
        {
            // do not select tracks with frequency above the max supported
            if ((tracks[track].freq >= ppgResults->maxHrScaled) || (tracks[track].freq <= ppgResults->minHrScaled))
                continue;
            // do not select tracks with frequency below the motion-freq
            if (tracks[track].freq < trackHrInfo->motionFrequency)
                continue;
            if (trackHrInfo->trackScoreFinal[track] > trackHrInfo->maxFastLockScoreFinal)
            {
                trackHrInfo->maxFastLockScoreFinal = trackHrInfo->trackScoreFinal[track];
                trackHrInfo->trackIdxMaxFastLockScoreFinal = track;
            }
            if ((tracks[track].motionHarm == 0) && (trackHrInfo->trackScoreFinal[track] > trackHrInfo->maxFastLockScoreNonMH))
            {
                trackHrInfo->maxFastLockScoreNonMH = trackHrInfo->trackScoreFinal[track];
                trackHrInfo->trackIdxMaxFastLockScoreNonMH = track;
            }
        }
        // select a track
        if (timeSinceUserPresent <= 8)
        {
            // re-evaluate track-selection for first few seconds (need sufficient input-data)
            trackHrResults.selectedTrackIdx = -1;
        }
        if ((trackHrResults.selectedTrackIdx >= 0) && (trackHrResults.selectedTrackIdx == trackHrInfo->trackIdxMaxFastLockScoreNonMH))
        {
            // the previous-selected-track still has the max-score of non-MH tracks
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxFastLockScoreNonMH; // no change
        }
        else if (trackHrResults.selectedTrackIdx == trackHrInfo->trackIdxMaxFastLockScoreFinal)
        {
            // the previous-selected-track still has the max-score (even if no track was selected)
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxFastLockScoreFinal; // no change
        }
        else if ((trackHrInfo->trackIdxMaxFastLockScoreNonMH != -1) && (trackHrInfo->maxFastLockScoreNonMH > 100))
        {
            // a non-MH track above the threshold is available
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxFastLockScoreNonMH;
        }
        else if (trackHrInfo->trackIdxMaxScoreMH2X != -1)
        {
            // a 2X-MH track is available
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxScoreMH2X;
        }
        else
        {
            // any non-MH track (may be none)
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxFastLockScoreNonMH;
        }
    }
}

/*!
 *****************************************************************************
 * \brief Select the track to use for the non-fast-lock result
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrSelect(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t track;

    // select the track to use for non-fast-lock
    trackHrInfo->selectedTrackIdx = -1;
    if (trackHrInfo->motionFrequency == 0)
    {
        // static...
        for (track=0; track<TRACK_COUNT; track++)
        {
            if (trackHrInfo->trackScoreFinal[track] > trackHrInfo->selScoreFinal)
            {
                // do not select tracks with frequency above the max supported
                if ((tracks[track].freq >= ppgResults->maxHrScaled) || (tracks[track].freq <= ppgResults->minHrScaled))
                    continue;
                trackHrInfo->selScoreFinal = trackHrInfo->trackScoreFinal[track];
                trackHrInfo->selectedTrackIdx = track;
            }
        }
    }
    else
    {
        // dynamic...
        uint16_t maxScoreNonMH=0;
        int8_t trackIdxMaxScoreNonMH=-1;
        for (track=0; track<TRACK_COUNT; track++)
        {
            // do not select tracks with frequency above the max supported
            if ((tracks[track].freq >= ppgResults->maxHrScaled) || (tracks[track].freq <= ppgResults->minHrScaled))
                continue;
            // do not select tracks with frequency below the motion-freq
            if (tracks[track].freq < trackHrInfo->motionFrequency)
                continue;
            if ((tracks[track].motionHarm == 0) && (trackHrInfo->trackScoreFinal[track] > maxScoreNonMH))
            {
                maxScoreNonMH = trackHrInfo->trackScoreFinal[track];
                trackIdxMaxScoreNonMH = track;
            }
        }
        // select a track
        if ((trackIdxMaxScoreNonMH >= 0) && (maxScoreNonMH > 100))
        {
            // a non-MH track above the threshold is available
            trackHrInfo->selectedTrackIdx = trackIdxMaxScoreNonMH;
        }
        else if (trackHrInfo->trackIdxMaxScoreMH2X != -1)
        {
            // a 2X-MH track is available
            trackHrInfo->selectedTrackIdx = trackHrInfo->trackIdxMaxScoreMH2X;
        }
    }
}

/*!
 *****************************************************************************
 * \brief Maintain the branch-info
 *
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrMaintainBranchInfo(trackHrInfo_t *trackHrInfo)
{
    int8_t track;

    // maintain the branch info
    if (trackHrInfo->selectedTrackIdx >= 0)
    {
        if (tracks[trackHrInfo->selectedTrackIdx].selectionBranch != 1)
        {
            // clear the "selectionBranch" from all tracks, then set just this track
            for (track=0; track<TRACK_COUNT; track++)
                tracks[track].selectionBranch = 0;
            tracks[trackHrInfo->selectedTrackIdx].selectionBranch = 1;
        }
    }
}

/*!
 *****************************************************************************
 * \brief Find motion-harmonics in PPG peaks
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrFindMotionHarmonics(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t ppgPeak;
    uint8_t harm;
    uint16_t motionFrequency;
    uint8_t motionFreqQual;

    // determine which PPG-peaks are motion-harmonics
    motionFrequency = trackHrInfo->motionFrequency;
    motionFreqQual = trackHrInfo->motionFreqQual;
    for (ppgPeak=0; ppgPeak<PPG_NUM_PEAKS; ppgPeak++)
    {
        trackHrInfo->motionHarmonic[ppgPeak] = 0;
        if (motionFrequency > 300) // over 30bpm
        {
            // check each motion harmonic
            for (harm=1; harm<=7; harm++)
            {
                uint16_t notchRadius; // scaled-freq
                if ((motionFreqQual < 10) && (harm > 4)) // harmonics above 4X need a higher-Q
                {
                    continue; // need a minimum motion quality (mag)
                }
                if ((motionFrequency < 400) && (motionFreqQual < 12) && (harm >= 3) && ((harm&1) == 1)) // odd harmonics need a higher-Q
                {
                    continue; // need a minimum motion quality (mag)
                }
                // notch-radius of 2.5bpm * harmonic (minimum 5.0bpm)
                notchRadius = harm * 25;
                if (notchRadius < 50)
                    notchRadius = 50;
                if (((motionFrequency*harm) > (ppgResults->freqAtPeak[ppgPeak]-notchRadius)) && ((motionFrequency*harm) < (ppgResults->freqAtPeak[ppgPeak]+notchRadius)))
                {
                    // this peak is too close to the motion-harmonic
                    trackHrInfo->motionHarmonic[ppgPeak] = harm;
                }
            }
        }
    }
}

/*!
 *****************************************************************************
 * \brief Matches the Acc and PPG peaks
 *
 * \param accMotion - motion-results
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrMatchAccPpgPeaks(accMotionResults_t *accMotion, ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t ppgPeak;
    int8_t accPeak;

    // match each PPG-peak with an Acc-peak (if there is a match) -- only use Acc-peaks with good quality
    for (ppgPeak=0; ppgPeak<PPG_NUM_PEAKS; ppgPeak++)
    {
        trackHrInfo->accMatchingPeak[ppgPeak] = -1;
        for (accPeak=0; accPeak<ACC_NUM_PEAKS; accPeak++)
        {
            if (accMotion->quality[accPeak] >= 5)
            {
                if ((ppgResults->freqAtPeak[ppgPeak] >= (accMotion->freqAtPeak[accPeak] - 50)) &&
                    (ppgResults->freqAtPeak[ppgPeak] <= (accMotion->freqAtPeak[accPeak] + 50)))
                {
                    trackHrInfo->accMatchingPeak[ppgPeak] = accPeak;
                    break;
                }
            }
        }
    }
}

/*!
 *****************************************************************************
 * \brief Return a new heartrate estimate for fast-lock case.
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for results/processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrFastLockResult(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    uint8_t peak;

    if (trackHrInfo->selectedTrackIdx < 0) // no track selected
    {
        trackHrInfo->resultHr = trackHrInfo->previousHr; // just use previous result
        trackHrInfo->fftFreq = 0;
        trackHrInfo->fftQual = 0;
    }
    else
    {
        trackHrInfo->resultHr = tracks[trackHrInfo->selectedTrackIdx].freq; // use selected-track result
        trackHrInfo->fftFreq = tracks[trackHrInfo->selectedTrackIdx].node[0].freq;
        trackHrInfo->fftQual = tracks[trackHrInfo->selectedTrackIdx].node[0].score / 10;
    }

    // return SQI
    trackHrInfo->resultSqi = 0;
    trackHrInfo->selectedPeakIdx = -1;
    if (trackHrInfo->selectedTrackIdx >= 0)
    {
        uint16_t initialSqiScore;
        uint16_t sqiScore;
        // SQI: start with the final score for this track
        initialSqiScore = trackHrInfo->trackScoreFinal[trackHrInfo->selectedTrackIdx];
        sqiScore = initialSqiScore;
        if ((trackHrInfo->motionFrequency == 0) && (sqiScore < 400))
            sqiScore = 400; // minimum starting score for selected track (static)
        if ((trackHrInfo->motionFrequency != 0) && (sqiScore < 300))
            sqiScore = 300; // minimum starting score for selected track (dynamic)
        // find the ppgPeak that is part of the selected-track
        for (peak=0; peak<PPG_NUM_PEAKS; peak++)
        {
            if (ppgResults->freqAtPeak[peak] == trackHrInfo->fftFreq)
                trackHrInfo->selectedPeakIdx = peak;
        }
        // check the new matching peak for this track
        if (trackHrInfo->selectedPeakIdx == 0)
        {
            // the new matching peak for this track was the best-peak -- increase SQI
            sqiScore *= 2;
        }
        if (sqiScore > 1000)
            sqiScore = 1000; // maximum score
        if (trackHrInfo->selectedPeakIdx == -1)
        {
            // no peak matched this track -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if ((tracks[trackHrInfo->selectedTrackIdx].node[1].freq == 0) || (tracks[trackHrInfo->selectedTrackIdx].node[2].freq == 0))
        {
            // this track is missing a match in the most recent 3 seconds -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }

        if (tracks[trackHrInfo->selectedTrackIdx].node[0].score < 15)
        {
            // this track's most recent node-score is too low -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if (initialSqiScore < 30)
        {
            // this track's final score is too low -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if ((tracks[trackHrInfo->selectedTrackIdx].freq < (tracks[trackHrInfo->selectedTrackIdx].node[0].freq - 23)) ||
            (tracks[trackHrInfo->selectedTrackIdx].freq > (tracks[trackHrInfo->selectedTrackIdx].node[0].freq + 23)))
        {
            // this track's frequency has not converged (track-freq differs from last node by more than 2.3bpm) -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }

        // check the data being processed
        if (ppgResults->lastSecondIdx < (ppgResults->firstSecondIdx + 2))
        {
            // less than 3 seconds of data were processed -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if (ppgResults->lastSecondIdx != (PPG_BUFFER_SECONDS-1))
        {
            // the latest second of data was not used for processing -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if (trackHrInfo->motionFrequency == 0)
        {
            // static restrictions...
            if (tracks[trackHrInfo->selectedTrackIdx].age < 3)
            {
                // the track is not old enough -- reduce SQI
                sqiScore = (sqiScore * 3) / 4;
            }
            else if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
            {
                if ((tracks[trackHrInfo->selectedTrackIdx].age == 3) && (initialSqiScore < 100))
                {
                    // the track is not old enough -- reduce SQI
                    sqiScore = (sqiScore * 3) / 4;
                }
                else if ((tracks[trackHrInfo->selectedTrackIdx].age == 4) && (initialSqiScore < 50))
                {
                    // the track is not old enough -- reduce SQI
                    sqiScore = (sqiScore * 3) / 4;
                }
            }
        }
        else
        {
            // dynamic restrictions...
            if (ppgResults->lastSecondIdx < (ppgResults->firstSecondIdx + 4))
            {
                // less than 5 seconds of data were processed -- reduce SQI
                sqiScore = (sqiScore * 3) / 4;
            }
            if (tracks[trackHrInfo->selectedTrackIdx].age < 5)
            {
                // the track is not old enough -- reduce SQI
                sqiScore = (sqiScore * 3) / 4;
            }
            else if (hrAfeDriverType == HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER)
            {
                if ((tracks[trackHrInfo->selectedTrackIdx].age == 5) && (initialSqiScore < 150))
                {
                    // the track is not old enough -- reduce SQI
                    sqiScore = (sqiScore * 3) / 4;
                }
                else if ((tracks[trackHrInfo->selectedTrackIdx].age == 6) && (initialSqiScore < 100))
                {
                    // the track is not old enough -- reduce SQI
                    sqiScore = (sqiScore * 3) / 4;
                }
            }
            if ((tracks[trackHrInfo->selectedTrackIdx].node[3].freq == 0) || (tracks[trackHrInfo->selectedTrackIdx].node[4].freq == 0))
            {
                // this track is missing a match in the most recent 5 seconds-- reduce SQI
                sqiScore = (sqiScore * 3) / 4;
            }
        }
        trackHrInfo->resultSqi = sqiScore / 10;
    }

    // set result-info
    trackHrInfo->expectedHr = 0;
    trackHrInfo->minExpectedHr = 0;
    trackHrInfo->maxExpectedHr = 0;
    for (peak=0; peak<TRACK_HR_NUM_PPG_PEAKS; peak++)
    {
        trackHrInfo->peakDistanceHr[peak] = 0;
        trackHrInfo->peakDistanceScore[peak] = 0;
        trackHrInfo->peakUsed[peak] = 0;
        trackHrInfo->closestPeak[peak] = -1;
    }
    trackHrInfo->closestNonHarmonicIdx = -1;
    trackHrInfo->closestHarmonicIdx = -1;
    trackHrInfo->adjFftFreq = trackHrInfo->ppiHr;
    trackHrInfo->fraction = trackHrInfo->ppiQuality;
}

/*!
 *****************************************************************************
 * \brief Find the track for the peak that was selected.
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for results/processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrFindPeakTrack(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    int8_t track;
    uint8_t node;
    uint32_t maxScore;
    uint16_t minNodeScoreThresh = 30;
    uint8_t scoreMultThresh4 = 4;
    uint8_t scoreMultThresh3 = 3;
    uint8_t scoreMultThresh2 = 2;
	
    if (activitySetting == 1) //typing
    {
        minNodeScoreThresh = 500;
        scoreMultThresh4 = 5;
        scoreMultThresh3 = 5;
        scoreMultThresh2 = 5;
        
    }

    trackHrInfo->selPeakBestTrackIdx = -1;
    trackHrInfo->selTrackPeakMatches = 0;
    trackHrInfo->selTrackPeakMatchesNonMH = 0;
    trackHrInfo->selTrackSelPeakMatchFound = 0;
    trackHrInfo->switchToSelTrack = 0;
    if (trackHrInfo->selectedPeakIdx < 0)
        return; // no selection to compare

    // Find the highest-scoring track with the selected-peak as its most recent node
    trackHrInfo->selPeakBestTrackIdx = -1;
    maxScore = 0;
    for (track=0; track<TRACK_COUNT; track++)
    {
        if (tracks[track].node[0].freq == ppgResults->freqAtPeak[trackHrInfo->selectedPeakIdx])
        {
            if (trackHrInfo->trackScoreNonMH[track] > maxScore)
            {
                maxScore = trackHrInfo->trackScoreNonMH[track];
                trackHrInfo->selPeakBestTrackIdx = track;
            }
        }
    }

    // Check if selected-peak is in the selected-track...
    if ((trackHrInfo->selectedTrackIdx < 0) || (trackHrInfo->selPeakBestTrackIdx < 0))
        return; // no selection to compare

    // A track was selected AND a track was found with the selected-peak...
    for (node=0; node<TRACK_NUM_NODES; node++)
    {
        if (tracks[trackHrInfo->selectedTrackIdx].node[node].freq != 0)
        {
            trackHrInfo->selTrackPeakMatches++;
            if (tracks[trackHrInfo->selectedTrackIdx].node[node].motionHarm == 0)
                trackHrInfo->selTrackPeakMatchesNonMH++;
        }
    }
    if (tracks[trackHrInfo->selectedTrackIdx].node[0].freq == ppgResults->freqAtPeak[trackHrInfo->selectedPeakIdx])
    {
        // The selected-track uses the selected-peak
        trackHrInfo->selTrackSelPeakMatchFound = 1; // use this to increase the reported quality
        return; // match found -- just return
    }

    // The selected-track does NOT use the selected-peak...
    // Decide if the selected-track should be used instead
    if (trackHrInfo->motionFrequency != 0)
        return; // do not switch tracks if there is motion

    if ((tracks[trackHrInfo->selectedTrackIdx].node[0].score < minNodeScoreThresh) || (tracks[trackHrInfo->selectedTrackIdx].node[1].score < minNodeScoreThresh) ||
        (tracks[trackHrInfo->selectedTrackIdx].node[2].score < minNodeScoreThresh))
    {
        return; // recent selected-track nodes must have a non-trivial score
    }

    if ((tracks[trackHrInfo->selectedTrackIdx].age >= 5) && (tracks[trackHrInfo->selectedTrackIdx].age < 8) && 
        (trackHrInfo->selTrackPeakMatchesNonMH == tracks[trackHrInfo->selectedTrackIdx].age) &&
        (tracks[trackHrInfo->selectedTrackIdx].node[0].score >= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].score * 3)))
    {
        if ((trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] > (trackHrInfo->trackScoreNonMH[trackHrInfo->selPeakBestTrackIdx] * 5)) &&
            (trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] >= 500))
        {
            // switch to selected-track
            trackHrInfo->switchToSelTrack = 1;
        }
    }
    else if ((tracks[trackHrInfo->selectedTrackIdx].age >= 8) && (tracks[trackHrInfo->selectedTrackIdx].age < 10) && 
             (trackHrInfo->selTrackPeakMatchesNonMH == tracks[trackHrInfo->selectedTrackIdx].age) &&
             (tracks[trackHrInfo->selectedTrackIdx].node[0].score >= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].score * 3)))
    {
        if ((trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] > (trackHrInfo->trackScoreNonMH[trackHrInfo->selPeakBestTrackIdx] * scoreMultThresh4)) &&
            (trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] >= 400))
        {
            // switch to selected-track
            trackHrInfo->switchToSelTrack = 1;
        }
    }
    else if (((tracks[trackHrInfo->selectedTrackIdx].age >= 10) && (trackHrInfo->selTrackPeakMatchesNonMH == 10)) && // (TRACK_NUM_NODES>=10)
             (tracks[trackHrInfo->selectedTrackIdx].node[0].score >= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].score * 3)))
    {
        if ((trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] > (trackHrInfo->trackScoreNonMH[trackHrInfo->selPeakBestTrackIdx] * scoreMultThresh3)) &&
            (trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] >= 300))
        {
            // switch to selected-track
            trackHrInfo->switchToSelTrack = 1;
        }
    }
    if (trackHrInfo->switchToSelTrack != 0)
        return;
    if (((tracks[trackHrInfo->selectedTrackIdx].age >= 12) && (trackHrInfo->selTrackPeakMatchesNonMH == 10)) && // (TRACK_NUM_NODES>=10)
         (tracks[trackHrInfo->selectedTrackIdx].node[0].score >= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].score * 2)))
    {
        // allow a 1:2 harmonic ratio to switch with a lower threshold
        if ((trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] > (trackHrInfo->trackScoreNonMH[trackHrInfo->selPeakBestTrackIdx] * scoreMultThresh2)) &&
            (trackHrInfo->trackScoreNonMH[trackHrInfo->selectedTrackIdx] >= 300) &&
            ((tracks[trackHrInfo->selectedTrackIdx].freq * 2) <= (tracks[trackHrInfo->selPeakBestTrackIdx].freq + 50)) &&
            ((tracks[trackHrInfo->selectedTrackIdx].freq * 2) >= (tracks[trackHrInfo->selPeakBestTrackIdx].freq - 50)) &&
            ((tracks[trackHrInfo->selectedTrackIdx].node[0].freq * 2) <= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].freq + 50)) &&
            ((tracks[trackHrInfo->selectedTrackIdx].node[0].freq * 2) >= (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].freq - 50)))
        {
            // switch to selected-track, unless one node comparison fails
            trackHrInfo->switchToSelTrack = 1;
            for (node=0; node<TRACK_NUM_NODES; node++)
            {
                if (tracks[trackHrInfo->selectedTrackIdx].node[node].score < (tracks[trackHrInfo->selPeakBestTrackIdx].node[node].score))
                    trackHrInfo->switchToSelTrack = 0;
            }
        }
    }
}

/*!
 *****************************************************************************
 * \brief Return a new heartrate estimate for non-fast-lock case.
 *
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for results/processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrResult(ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    uint8_t idx;
    uint8_t peak;
    uint32_t maxScore;
    uint16_t baseHr;
    uint32_t fraction;
    uint32_t hr;
    uint8_t maxFraction = 100;
    uint8_t expectedHrRange = 30;
    uint8_t hrDriftRate = 10;
    
    if (activitySetting == 1) 
    { //typing activity selected
        maxFraction = 10;
        expectedHrRange = 10;
        hrDriftRate = 1;
    }

    // Compute the Expected-HR based on Prev-HR and motion
    if (trackHrInfo->motionFrequency == 0)
    {
        // no motion -- just use Prev-HR
        trackHrInfo->expectedHr = trackHrInfo->previousHr;
    }
    else
    {
        // motion -- drift toward 2X motion-freq
        if ((trackHrInfo->motionFrequency*2) > trackHrInfo->previousHr)
            fraction = 15;
        else
            fraction = 5;
        fraction = fraction * trackHrInfo->motionFreqQual / 100;
        trackHrInfo->expectedHr = (uint16_t)((((uint32_t)trackHrInfo->previousHr * (1000-fraction)) + ((uint32_t)(trackHrInfo->motionFrequency * 2) * fraction)) / 1000);
    }
    // Compute the expected-range (min/max) [+/3.0bpm]
    trackHrInfo->minExpectedHr = trackHrInfo->expectedHr - expectedHrRange;
    trackHrInfo->maxExpectedHr = trackHrInfo->expectedHr + expectedHrRange;
    baseHr = trackHrInfo->expectedHr;

    // Calculate the peak-distance from the Expected-HR
    for (peak=0; peak<TRACK_HR_NUM_PPG_PEAKS; peak++)
    {
        if (ppgResults->freqAtPeak[peak] >= baseHr)
            trackHrInfo->peakDistanceHr[peak] = ppgResults->freqAtPeak[peak] - baseHr;
        else
            trackHrInfo->peakDistanceHr[peak] = baseHr - ppgResults->freqAtPeak[peak];
        trackHrInfo->peakDistanceScore[peak] = ppgResults->peakScore[peak] * 1000 / (trackHrInfo->peakDistanceHr[peak] + 1);
        trackHrInfo->peakUsed[peak] = 0;
        if (ppgResults->freqAtPeak[peak] == 0)
            trackHrInfo->peakUsed[peak] = 1;
    }

    // Order the peaks -- closest first (just freq, no weighting)
    for (idx=0; idx<TRACK_HR_NUM_PPG_PEAKS; idx++)
    {
        maxScore = 0;
        trackHrInfo->closestPeak[idx] = -1;
        for (peak=0; peak<TRACK_HR_NUM_PPG_PEAKS; peak++)
        {
            if ((trackHrInfo->peakUsed[peak] == 0) && (trackHrInfo->peakDistanceScore[peak] > maxScore))
            {
                maxScore = trackHrInfo->peakDistanceScore[peak];
                trackHrInfo->closestPeak[idx] = peak;
            }
        }
        if (trackHrInfo->closestPeak[idx] != -1)
            trackHrInfo->peakUsed[trackHrInfo->closestPeak[idx]] = 1;
    }

    // Find the closest non-harmonic peak (if there is one)
    trackHrInfo->closestNonHarmonicIdx = -1; // no selection
    for (idx=0; idx<TRACK_HR_NUM_PPG_PEAKS; idx++)
    {
        if (trackHrInfo->closestPeak[idx] != -1)
        {
            if (trackHrInfo->motionHarmonic[trackHrInfo->closestPeak[idx]] == 0)
            {
                trackHrInfo->closestNonHarmonicIdx = trackHrInfo->closestPeak[idx];
                break;
            }
        }
    }
    // Find the closest harmonic peak (if there is one)
    trackHrInfo->closestHarmonicIdx = -1; // no selection
    for (idx=0; idx<TRACK_HR_NUM_PPG_PEAKS; idx++)
    {
        if (trackHrInfo->closestPeak[idx] != -1)
        {
            if (trackHrInfo->motionHarmonic[trackHrInfo->closestPeak[idx]] == 2)
            {
                trackHrInfo->closestHarmonicIdx = trackHrInfo->closestPeak[idx];
                break;
            }
        }
    }

    // If a non-harmonic peak was found, check that it is within 10.0bpm of Expected-HR
    trackHrInfo->selectedPeakIdx = trackHrInfo->closestNonHarmonicIdx;
    if (trackHrInfo->selectedPeakIdx != -1)
    {
        peak = trackHrInfo->selectedPeakIdx;
        if ((ppgResults->freqAtPeak[peak] < (baseHr-100)) || (ppgResults->freqAtPeak[peak] > (baseHr+100)))
            trackHrInfo->selectedPeakIdx = -1; // no selection
    }

    // If no selection has been made, check for nearby harmonics (within 10.0bpm of Expected-HR)
    if (trackHrInfo->selectedPeakIdx == -1)
    {
        if (trackHrInfo->closestHarmonicIdx != -1)
        {
            trackHrInfo->selectedPeakIdx = trackHrInfo->closestHarmonicIdx;
            peak = trackHrInfo->selectedPeakIdx;
            if ((ppgResults->freqAtPeak[peak] < (baseHr-100)) || (ppgResults->freqAtPeak[peak] > (baseHr+100)))
            {
                trackHrInfo->selectedPeakIdx = -1; // no selection
            }
            else if (trackHrInfo->closestNonHarmonicIdx != -1)
            {
                if (trackHrInfo->peakDistanceScore[trackHrInfo->closestNonHarmonicIdx] > trackHrInfo->peakDistanceScore[trackHrInfo->closestHarmonicIdx])
                    trackHrInfo->selectedPeakIdx = trackHrInfo->closestNonHarmonicIdx;
            }
        }
    }

    // If no selection has been made, use the closest non-harmonic as the default
    if (trackHrInfo->selectedPeakIdx == -1)
    {
        trackHrInfo->selectedPeakIdx = trackHrInfo->closestNonHarmonicIdx;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Compute the initial HR-result using the selected-peak and previous-HR
    if (trackHrInfo->selectedPeakIdx == -1)
    {
        // no peak was selected -- force no change to HR
        trackHrInfo->fftFreq = 0;
        trackHrInfo->fftQual = 0;
        trackHrInfo->adjFftFreq = 0;
        trackHrInfo->fraction = 0;
    }
    else
    {
        // Use the selected-peak...
        // Get the info for the selected-peak
        trackHrInfo->fftFreq = ppgResults->freqAtPeak[trackHrInfo->selectedPeakIdx];
        trackHrInfo->fftQual = ppgResults->quality[trackHrInfo->selectedPeakIdx];

        // Compute an adjusted-FFT-HR (limit the difference allowed tp 10.0bpm)
        trackHrInfo->adjFftFreq = trackHrInfo->fftFreq;
        if (trackHrInfo->motionFrequency != 0)
        {
            // motion -- limit the change each second to 10.0bpm
            if (trackHrInfo->adjFftFreq > (baseHr + 100))
                trackHrInfo->adjFftFreq = baseHr + 100;
            else if (trackHrInfo->adjFftFreq < (baseHr - 100))
                trackHrInfo->adjFftFreq = baseHr - 100;
        }

        // Compute the weighting (fraction) of the adjusted-FFT
        trackHrInfo->fraction = trackHrInfo->fftQual;
        if (trackHrInfo->fraction > maxFraction)
            trackHrInfo->fraction = maxFraction;

        if (trackHrInfo->motionFrequency != 0)
        {
            // motion detected
            if (trackHrInfo->fftFreq < trackHrInfo->previousHr)
            {
                // FFT-result is lower than previous estimate
                // do not use FFT-result if...
                // - it is less than 1.5X motion-freq AND
                // - the FFT-result is less than 0.9X PrevResult AND
                // - the FFT-Q is low
                if ((trackHrInfo->fftFreq < (trackHrInfo->motionFrequency * 3 / 2)) &&
                    (trackHrInfo->fftFreq < (baseHr * 9 / 10)) &&
                    (trackHrInfo->fftQual < 75))
                {
                    trackHrInfo->fraction = 0;
                }
            }

            // If expectedHR within 5bpm of motionFreq(2), increase the weighting for expectedHR
            if (((trackHrInfo->motionFrequency*2) > (baseHr-50)) &&
                ((trackHrInfo->motionFrequency*2) < (baseHr+50)))
            {
                trackHrInfo->fraction /= 2;
            }
            // If FFT is increasing toward 2X motion-freq, increase the weighting for FFT
            if (((trackHrInfo->motionFrequency*2) > trackHrInfo->previousHr) &&
                (trackHrInfo->adjFftFreq > (baseHr + 10)))
            {
                // increase the drift toward 2X motion-freq
                trackHrInfo->fraction *= 2;
                if (trackHrInfo->fraction > 100)
                    trackHrInfo->fraction = 100;
                if (trackHrInfo->fraction < 10)
                    trackHrInfo->fraction = 10;
            }
        }
    }

    if (trackHrInfo->fraction == 0)
    {
        // the FFT-result is not being used
        if (trackHrInfo->motionFrequency != 0)
        {
            // motion detected -- drift toward 2X motion-freq
            trackHrInfo->adjFftFreq = trackHrInfo->expectedHr;
            if ((trackHrInfo->motionFrequency*2) > trackHrInfo->previousHr)
                trackHrInfo->fraction = 4;
            else
                trackHrInfo->fraction = 1;
        }
        else
        {
            // no motion -- drift
            trackHrInfo->adjFftFreq = trackHrInfo->expectedHr;
        }
    }

    // Adjust the base-HR in the direction of the adjusted-FFT-HR
    if (trackHrInfo->adjFftFreq > trackHrInfo->maxExpectedHr)
        baseHr += hrDriftRate;
    else if (trackHrInfo->adjFftFreq < trackHrInfo->minExpectedHr)
        baseHr -= hrDriftRate;

    // apply the weighting to generate a new result
    hr = (((uint32_t)baseHr * (100 - trackHrInfo->fraction)) + ((uint32_t)trackHrInfo->adjFftFreq * trackHrInfo->fraction)) / 100;
    trackHrInfo->resultHr = (uint16_t)hr;

    // Find the track with the selected peak
    trackHrFindPeakTrack(ppgResults, trackHrInfo);
    trackHrInfo->resultSqi = 0;
    if ((trackHrInfo->switchToSelTrack != 0) && (hrAfeDriverType != HR_AFE_DRIVER_TYPE_NORMAL_POWER))
    {
        // switch to use the selected-track info
        trackHrInfo->resultHr = tracks[trackHrInfo->selectedTrackIdx].freq;
        return;
    }
    if (trackHrInfo->selectedPeakIdx != -1)
    {
        uint32_t sqiScore;
        if (ppgResults->totalMagnitude == 0)
            sqiScore = 0;
        else
            sqiScore = ppgResults->peakValue[trackHrInfo->selectedPeakIdx] * ppgResults->scale * 200 / ppgResults->totalMagnitude;
        if (sqiScore > 100)
            sqiScore = 100;
        trackHrInfo->resultSqi = (uint8_t)sqiScore;
    }

    // If selected-peak is in the selected-track, increase the reported-quality
    if (trackHrInfo->selTrackSelPeakMatchFound != 0)
    {
        uint16_t sqiScore;
        sqiScore = (uint16_t)trackHrInfo->resultSqi;
        if (tracks[trackHrInfo->selectedTrackIdx].age >= 8)
        {
            sqiScore *= 2;
            if (sqiScore > 100)
                sqiScore = 100;
        }
        if ((tracks[trackHrInfo->selectedTrackIdx].freq < (tracks[trackHrInfo->selectedTrackIdx].node[0].freq - 23)) ||
            (tracks[trackHrInfo->selectedTrackIdx].freq > (tracks[trackHrInfo->selectedTrackIdx].node[0].freq + 23)))
        {
            // this track's frequency has not converged (track-freq differs from last node by more than 2.3bpm) -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if ((tracks[trackHrInfo->selectedTrackIdx].freq < (hr - 23)) ||
            (tracks[trackHrInfo->selectedTrackIdx].freq > (hr + 23)))
        {
            // the HR-estimate has not converged with the matching track (track-freq differs by more than 2.3bpm) -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if (sqiScore > 100)
            trackHrInfo->resultSqi = 100;
        trackHrInfo->resultSqi = (uint8_t)sqiScore;
    }
    else if (trackHrInfo->selPeakBestTrackIdx >= 0)
    {
        uint16_t sqiScore;
        sqiScore = (uint16_t)trackHrInfo->resultSqi;
        if ((tracks[trackHrInfo->selPeakBestTrackIdx].freq < (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].freq - 23)) ||
            (tracks[trackHrInfo->selPeakBestTrackIdx].freq > (tracks[trackHrInfo->selPeakBestTrackIdx].node[0].freq + 23)))
        {
            // this track's frequency has not converged (track-freq differs from last node by more than 2.3bpm) -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if ((tracks[trackHrInfo->selPeakBestTrackIdx].freq < (hr - 23)) ||
            (tracks[trackHrInfo->selPeakBestTrackIdx].freq > (hr + 23)))
        {
            // the HR-estimate has not converged with the matching track (track-freq differs by more than 2.3bpm) -- reduce SQI
            sqiScore = (sqiScore * 3) / 4;
        }
        if (sqiScore > 100)
            trackHrInfo->resultSqi = 100;
        trackHrInfo->resultSqi = (uint8_t)sqiScore;
    }
}

/*!
 *****************************************************************************
 * \brief Return a new heartrate estimate.
 *
 * \param accMotion - motion-results
 * \param ppgResults - latest PPG-raw FFT-results
 * \param trackHrInfo - structure used for results/processing (reduces stack-usage)
 *****************************************************************************
 */
void trackHrProcess(accMotionResults_t *accMotion, ppgProcessingResults_t *ppgResults, trackHrInfo_t *trackHrInfo)
{
    // Save the previous HR-result for comparison
    trackHrInfo->previousHr = trackHrResults.hrResult;

    // Determine the motion-frequency and quality to use
    trackHrInfo->motionFrequency = accMotion->freqResult;
    trackHrInfo->motionFreqQual = accMotion->freqQuality;
#ifdef HRM_AS7000_DEVICE // this should be defined for AS7000-device build only
    // if not an AS7000-device when expected, disable motion-compensatation
    *(volatile uint32_t *)(&CCU->CCU_DEVICEID.reg) = 0x639c2e37;
    if (CCU->CCU_DEVICEID.bit.device_id != 0x1b58)
        trackHrInfo->motionFreqQual >>= 5;
#endif
    // Do not use motion-frequency if quality is low
    if (trackHrInfo->motionFreqQual < 5)
        trackHrInfo->motionFrequency = 0;

    // Find the motion-harmonics in the PPG peaks
    trackHrFindMotionHarmonics(ppgResults, trackHrInfo);

    // Match the accelerometer and PPG peaks
    trackHrMatchAccPpgPeaks(accMotion, ppgResults, trackHrInfo);

    // Match the new PPG peaks with existing tracks
    trackHrMaintainTracks(ppgResults, trackHrInfo);

    // Determine the new HR-result
    if (timeSinceUserPresent <= 20) // fast-lock
    {
        trackHrFastLockSelect(ppgResults, trackHrInfo);
        trackHrFastLockResult(ppgResults, trackHrInfo);
    }
    else
    {
        trackHrSelect(ppgResults, trackHrInfo);
        trackHrResult(ppgResults, trackHrInfo);
    }

    // Update track-information based on the selected-track
    trackHrMaintainBranchInfo(trackHrInfo);

    // Keep results for comparison on next iteration
    trackHrResults.hrResult = trackHrInfo->resultHr;
    if (trackHrResults.selectedTrackIdx == trackHrInfo->selectedTrackIdx)
    {
        if (trackHrResults.selectedTrackTime < 100)
            trackHrResults.selectedTrackTime++;
    }
    else
    {
        trackHrResults.selectedTrackIdx = trackHrInfo->selectedTrackIdx;
        trackHrResults.selectedTrackTime = 1;
    }

    //debugOutputTracks(tracks, trackHrInfo);
    //debugOutputTrackHr(accMotion, ppgResults, trackHrInfo);
}
