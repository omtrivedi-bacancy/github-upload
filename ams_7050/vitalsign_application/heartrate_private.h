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

/*! \file heartrate_private.h
 *
 *  \author M. Arpa
 *  \author G. Wagner
 *
 *  \brief defines, types, functions that must not be exported to the public
 * interface
 */

#ifndef HEARTRATE_PRIVATE_H
#define HEARTRATE_PRIVATE_H

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */

#include "heartrate.h"
#include "acc_processing.h"
#ifdef HRM_AS7000_DEVICE // this should be defined for AS7000-device build only
#include "AS7000.h" // needed for AS7000-device builds only
#endif

/*
 *****************************************************************************
 * DEFINES
 *****************************************************************************
 */

#define DEFAULT_HR_VALUE                    (800) 

#define DEFAULT_UPDATE_INTERVAL_100US       (10000)  // this is 1.0000 seconds


/* Maximum Heartrate has as a lower bbi limit: 5bbi == 240 bpm */
/* Minimum Heartrate has as a upper bbi limit: 40bbi == 30 bpm */
#define HEARTRATE_PPG_BBI_LIMIT_MIN     5
#define HEARTRATE_PPG_BBI_LIMIT_MAX     40

/* Heartrate range = [30.0 .. 240.0] bpm */
/* Heartrate-processing range is [40.0 .. 240.0] bpm for dynamic cases */
#define HEARTRATE_PROC_SCALED_FREQUENCY_MIN_STATIC    (294)  //  29.4bpm: allow for 2% clock-skew
#define HEARTRATE_PROC_SCALED_FREQUENCY_MIN_DYNAMIC   (392)  //  39.2bpm: allow for 2% clock-skew
#define HEARTRATE_PROC_SCALED_FREQUENCY_MAX           (2448) // 244.8bpm: allow for 2% clock-skew
#define HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MIN_STATIC    (288)  //  28.8bpm: allow for 4% total clock-error (ULP)
#define HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MIN_DYNAMIC   (384)  //  38.4bpm: allow for 4% total clock-error (ULP)
#define HEARTRATE_PROC_SCALED_FREQUENCY_ULP_MAX           (2496)  // 249.6bpm: allow for 4% total clock-error (ULP)
#define HEARTRATE_RESULT_SCALED_FREQUENCY_MIN   (295)  //  29.5bpm (rounds to 30)
#define HEARTRATE_RESULT_SCALED_FREQUENCY_MAX   (2404) // 240.4bpm (rounds to 240)


/* each axis has an index */
#define ACC_X       0
#define ACC_Y       1
#define ACC_Z       2


extern uint32_t secondCounter;
extern uint8_t timeSinceUserPresent;
extern uint8_t hrAfeDriverType; // (HR_AFE_DRIVER_TYPE_*)
extern accMotionResults_t accMotion;
extern uint8_t minHeartrate;
extern uint8_t maxHeartrate;
extern uint8_t activitySetting;


/*
 *****************************************************************************
 * TYPES
 *****************************************************************************
 */


/*
 *****************************************************************************
 * FUNCTIONS
 *****************************************************************************
 */


#endif /* HEARTRATE_PRIVATE_H */
