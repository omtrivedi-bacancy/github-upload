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

/*! \file heartrate.c
 *
 *  \author M. Arpa
 *  \author F. Haas
 *  \author G. Wagner
 *  \author M. Zimmerman
 *
 *  \brief Implementation for heartrate algorithm. 
 *
 *
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include "heartrate.h"
#include "heartrate_private.h"
#include "acc_processing.h"
#include "ppg_processing.h"
#include "track_hr.h"


/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */
#if (ACC_FFT_BUFFER_SIZE_BYTES > PPG_FFT_BUFFER_SIZE_BYTES)
#define WORK_BUFFER_SIZE_BYTES ACC_FFT_BUFFER_SIZE_BYTES
#else
#define WORK_BUFFER_SIZE_BYTES PPG_FFT_BUFFER_SIZE_BYTES
#endif

/*
 *****************************************************************************
 * VARIABLES
 *****************************************************************************
 */
    
// main scratch work buffer - content is overwritten each time the algorithm runs
static uint32_t workBuffer[WORK_BUFFER_SIZE_BYTES/4]; // force 32-bit alignment

// how long the algorithm has run
uint32_t secondCounter;

// time since user-present, in seconds
uint8_t timeSinceUserPresent;

// the AFE-type used to generate the data
uint8_t hrAfeDriverType; // (HR_AFE_TYPE_*)

// accelerometer motion-processing results
accMotionResults_t accMotion;

uint8_t minHeartrate = 0;
uint8_t maxHeartrate = 0;
uint8_t activitySetting = 0;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */

    
/*!
 *****************************************************************************
 * \brief Initialise all internal variables of the heartrate algorithm.
 * 
 * Must be called before calling heartrateInput() or heartrateCalculate().
 *
 * \param afeDriverType - informs the algorithm which AFE is being used (HR_AFE_DRIVER_TYPE_*)
 * \param accSampleFreq1000 - acc-data sampling-frequency [units=0.001Hz]
 *                        range=[HR_ACC_MIN_FREQUENCY..HR_ACC_MAX_FREQUENCY]
 * \param crosstalkLevel64 - crosstalk-level [units=64 adc-counts] (78 ==> 4992 adc-counts)
 * \param output - pointer to an instance of the heartrate algorithm output.
 * Is initialised by the function.
 *
 * \return HEARTRATE_ALGORITHM_OK - output pointer was valid, everything initialised
 *         HEARTRATE_ALGORITHM_ERR_PARAM - output is NULL or accSampleFreq1000 is out of range
 *****************************************************************************
 */
int8_t heartrateInitialise(uint8_t afeDriverType, uint16_t accSampleFreq1000, uint8_t crosstalkLevel64, hrAlgorithmOutput *output)
{
    if (!output)
        return HEARTRATE_ALGORITHM_ERR_PARAM;
    if ((accSampleFreq1000 < HR_ACC_MIN_FREQUENCY_TIMES_1000) || (accSampleFreq1000 > HR_ACC_MAX_FREQUENCY_TIMES_1000))
        return HEARTRATE_ALGORITHM_ERR_PARAM;

    secondCounter = 0;
    timeSinceUserPresent = 0;
    if ((afeDriverType == HR_AFE_DRIVER_TYPE_UNKNOWN) || (afeDriverType == HR_AFE_DRIVER_TYPE_NORMAL_POWER))
        hrAfeDriverType = HR_AFE_DRIVER_TYPE_NORMAL_POWER;
    else if (afeDriverType == HR_AFE_DRIVER_TYPE_LOW_POWER)
        hrAfeDriverType = HR_AFE_DRIVER_TYPE_LOW_POWER;
    else
        hrAfeDriverType = HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER;

    output->heartrate = DEFAULT_HR_VALUE;
    output->signalQuality = 255; // unknown signal quality at the beginning
    output->motionFreq = 0;

    
    accProcessingInit(accSampleFreq1000);
    ppgProcessingInit(crosstalkLevel64);
    trackHrInit();

    return HEARTRATE_ALGORITHM_OK;
}


/*!
 *****************************************************************************
 * \brief Pass one second of input data to the algorithm.
 *
 * Call this function directly from the ISR where the input data is generated.
 * This function performs an immediate copy of the input data to internal buffers.
 * 
 * For each second of new data, this function must be called before calling
 * heartrateCalculate().
 *
 * \param input - pointer to an instance of the heartrate algorithm input
 * for one second.
 *****************************************************************************
 */
void heartrateInput(hrAlgorithmInput *input, uint8_t minHeartrateInput, uint8_t maxHeartrateInput, uint8_t activityInput)
{
    hrAlgorithmInput *scratchBuffer = (hrAlgorithmInput *)workBuffer;
    uint16_t * const endDst = (uint16_t *)(scratchBuffer + 1);
    uint16_t *dst;
    uint16_t *src;

    src = (uint16_t *)input;
    dst = (uint16_t *)scratchBuffer;
    while (dst < endDst)
    {
        *dst++ = *src++;
    }
    
    if ((minHeartrateInput < maxHeartrateInput) || (maxHeartrateInput == 0) )
    {
        minHeartrate = minHeartrateInput;
        maxHeartrate = maxHeartrateInput;
    }
    
    if (activityInput == 1)
        activitySetting = 1;
    else
        activitySetting = 0;
    
}

/*!
 *****************************************************************************
 * \brief Compensate for any sampling-rate variance from expected
 *
 * Uses the actual sample-interval to correct the input frequency.
 *
 * \param rawFreq - raw frequency to compensate
 * \param actualUpdateInterval100Us - the actual update-interval [units=100us]
 * \returns adjusted frequency
 *****************************************************************************
 */
static uint16_t frequencyCompensation(uint16_t rawFreq, uint16_t actualUpdateInterval100Us)
{
    uint32_t adjustedResult;
    
    if (actualUpdateInterval100Us == DEFAULT_UPDATE_INTERVAL_100US)
        return rawFreq;
    adjustedResult = ((uint32_t)rawFreq * DEFAULT_UPDATE_INTERVAL_100US) / actualUpdateInterval100Us;
    return (uint16_t)adjustedResult;
}

/*!
 *****************************************************************************
 * \brief Calculates a new heartrate
 * The following steps are executed...
 * 1) Accelerometer data is combined and stored
 * 2) PPG data is stored
 * 3) Fundamental motion frequency is calculated
 * 4) PPI-calculation for PPG data is performed (if needed)
 * 5) FFT-calculation for PPG data is performed
 * 6) The new heartrate result is determined based on previous calculations
 *
 * \param[in] updateInterval100Us - the actual update-interval, units=100us.
 *                              [nominal=10000 ==> 1.0000 seconds]
 *                              [use "0" if update-interval is not computed]
 * \param[inout] *output - pointer to an instance of the heartrate algorithm output.
 * \returns HEARTRATE_ALGORITHM_OK in case of no error, any other value indicates an error
 *****************************************************************************
 */
int8_t heartrateCalculate(uint16_t updateInterval100Us, hrAlgorithmOutput *output)
{
    uint16_t ppiHr;
    uint8_t ppiQuality;
    ppgProcessingResults_t *ppgResults;
    trackHrInfo_t *trackHrInfo;
    hrAlgorithmInput *scratchBuffer = (hrAlgorithmInput *)workBuffer;

    if (!output)
        return HEARTRATE_ALGORITHM_ERR_PARAM;
    
    // Initialize calculation
    secondCounter++;
    timeSinceUserPresent++;
    if (timeSinceUserPresent > 100)
        timeSinceUserPresent = 100; // avoid overflow
    if ((updateInterval100Us == 0) || (hrAfeDriverType != HR_AFE_DRIVER_TYPE_ULTRA_LOW_POWER))
        updateInterval100Us = DEFAULT_UPDATE_INTERVAL_100US; // use default
    
    // Acc and PPG input-data was placed into the work-buffer before this function was called.
    // This data needs to be managed prior to using the work-buffer for processing.
    // Combine and store the Acc-data
    accStoreCombinedAxisData(scratchBuffer->accX, scratchBuffer->accY, scratchBuffer->accZ, scratchBuffer->numAccSamples);

    
    // Store the raw-PPG data
    ppgRawStoreData(scratchBuffer->adcValue, scratchBuffer->cChanged);
    if (ppgRawIsUserPresent() == 0) // user not present
    {
        timeSinceUserPresent = 0; // reset the time since user present
        trackHrInit(); // reset HR-tracking
    }
    
    // Compute FFT of acc-data and find motion-frequency (0 if no significant motion found)
    accFindMotionFreq((int16_t *)workBuffer, &accMotion);

    // Perform PPI-processing on the PPG data (if needed)
    ppiHr = 0;
    ppiQuality = 0;
    if (accMotion.freqQuality < 10)
    {
        // Perform PPI-processing on the raw-PPG data
        ppgRawProcessPpi((uint16_t *)workBuffer, &ppiHr, &ppiQuality);
    }

    // Perform FFT processing on the raw-PPG data
    ppgRawProcessFft((int16_t *)workBuffer, &ppgResults);
   
    // Compute the final result for this second of data
    trackHrInfo = (trackHrInfo_t *)workBuffer; // this must be coordinated with "ppgResults"
    trackHrInfo->ppiHr = ppiHr;
    trackHrInfo->ppiQuality = ppiQuality;
    trackHrProcess(&accMotion, ppgResults, trackHrInfo);
    
    // Convert internal SQI rating to external-format.
    // Internal: [0..100] [80..100]=very-good, 0=very-bad
    // External: [0..12,254,255] 0=very-good, 12=very-bad, 254=user-not-present, 255=unknown(at-start)
    if (timeSinceUserPresent == 0)
    {
        trackHrInfo->reportedSqi = 254; // user not present
        trackHrInfo->resultHr = 0; // no processing -- just report 0
    }
    else if (timeSinceUserPresent <= 3)
    {
        trackHrInfo->reportedSqi = 255; // unknown signal quality at the beginning
    }
    else
    {
        if (trackHrInfo->resultSqi > 100)
            trackHrInfo->reportedSqi = 0;
        else
            trackHrInfo->reportedSqi = (100 - trackHrInfo->resultSqi) / 8;
        if (trackHrInfo->reportedSqi <= 2) // fast-lock complete
            timeSinceUserPresent = 100;
    }

    // Setup output to caller
    output->heartrate = frequencyCompensation(trackHrInfo->resultHr, updateInterval100Us);
    if ((output->heartrate < HEARTRATE_RESULT_SCALED_FREQUENCY_MIN) && (output->heartrate != 0))
        output->heartrate = HEARTRATE_RESULT_SCALED_FREQUENCY_MIN;
    if (output->heartrate > HEARTRATE_RESULT_SCALED_FREQUENCY_MAX)
        output->heartrate = HEARTRATE_RESULT_SCALED_FREQUENCY_MAX;
    output->signalQuality = trackHrInfo->reportedSqi;
    output->motionFreq = HEARTRATE_CONVERT_TO_BPM(frequencyCompensation(accMotion.freqResult, updateInterval100Us));


    // Debug output
		/*
    debugLogValue(DEBUG_OUT_HEARTRATE, frequencyCompensation(accMotion.freqResult, updateInterval100Us) );
    debugLogValue(DEBUG_OUT_HEARTRATE, 0 );
    debugLogValue(DEBUG_OUT_HEARTRATE, 0 );
    debugLogValue(DEBUG_OUT_HEARTRATE, 0 );
    debugLogValue(DEBUG_OUT_HEARTRATE, output->heartrate );
    debugLogValue(DEBUG_OUT_HEARTRATE, frequencyCompensation(trackHrInfo->fftFreq, updateInterval100Us) );
    debugLogValue(DEBUG_OUT_HEARTRATE, frequencyCompensation(ppiHr, updateInterval100Us) );
    debugLogValue(DEBUG_OUT_HEARTRATE, trackHrInfo->reportedSqi );
    debugLogValue(DEBUG_OUT_HEARTRATE, 0 );
    debugLogValue(DEBUG_OUT_HEARTRATE, HEARTRATE_CONVERT_TO_BPM( frequencyCompensation(accMotion.freqResult, updateInterval100Us) ) );
    debugLogEndEntry();
    debugOutputDetailedLog(&accMotion, ppgResults, trackHrInfo);
    debugOutputHeartrateInfo(output->heartrate,
                             frequencyCompensation(trackHrInfo->fftFreq, updateInterval100Us),
                             frequencyCompensation(ppiHr, updateInterval100Us));

    // Debug timing (used for execution-time analysis)
    debugTimerRead(&debugTime.tick[5]);
    debugTimerDiffUs(debugTime.tick[0], debugTime.tick[1], 0);
    debugTimerDiffUs(debugTime.tick[1], debugTime.tick[2], 1);
    debugTimerDiffUs(debugTime.tick[2], debugTime.tick[3], 2);
    debugTimerDiffUs(debugTime.tick[3], debugTime.tick[4], 3);
    debugTimerDiffUs(debugTime.tick[4], debugTime.tick[5], 4);
    debugTimerDiffUs(debugTime.tick[0], debugTime.tick[5], 5);
    */
    return HEARTRATE_ALGORITHM_OK;
}
