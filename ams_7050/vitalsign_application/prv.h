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

/*! \file prv.h
 *
 *  \author M. Zimmerman
 *
 *  \brief Interface to Pulse-Rate Variability (PRV) algorithm
 */

#ifndef PRV_H
#define PRV_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include <stdint.h>

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

/* possible values for the "afeDriverType" parameter to prvInitialise() */
#define PRV_AFE_DRIVER_TYPE_UNKNOWN           0
#define PRV_AFE_DRIVER_TYPE_NORMAL_POWER      1
#define PRV_AFE_DRIVER_TYPE_LOW_POWER         2
#define PRV_AFE_DRIVER_TYPE_ULTRA_LOW_POWER   3

/* possible return codes from function calls */
#define PRV_ALGORITHM_OK                      0
#define PRV_ALGORITHM_ERR_NO_DATA             1
#define PRV_ALGORITHM_ERR_PARAM               2
#define PRV_ALGORITHM_VALUE_AVAILABLE         3
#define PRV_ALGORITHM_VALUE_NOT_AVAILABLE     4


/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */

/* result from the last calculation of algorithm */
typedef struct
{
    uint16_t peakToPeakIntervalMs; /* the interval between detected peaks (units=ms) */
} prvAlgorithmOutput;


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */
 
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
uint8_t prvInitialise(uint8_t afeDriverType, uint16_t samplesPerSecond);

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
uint8_t prvCalculate(uint16_t adcValue, prvAlgorithmOutput *output);


#endif /* PRV_H */
