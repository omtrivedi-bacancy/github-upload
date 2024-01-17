/******************************************************************************
 * Copyright by ams AG                                                        *
 * All rights are reserved.                                                   *
 *                                                                            *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING      *
 * THE SOFTWARE.                                                              *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS          *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,      *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT           *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,      *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY      *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT        *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
 ******************************************************************************/

#ifndef __AS7050_APP_MANAGER_TYPEDEFS_H__
#define __AS7050_APP_MANAGER_TYPEDEFS_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*!
 * \file       as7050_app_manager_typedefs.h
 * \authors    PKRN
 * \copyright  ams
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_typedefs.h"
#include "std_inc.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! Chip-specific type representing the identifier of a channel. */
typedef uint8_t as7050_appmgr_channel_id_t;

/*!
 * \brief Chip-specific chip status information type, which may include information such as interrupt status. This
 *        chip status is provided to ::as7050_appmgr_set_input and is intended to be forwarded to the raw data
 *        application.
 *
 * This structure is unused as the output of the raw data application does not include such fields.
 */
typedef struct {

} as7050_appmgr_chip_status_t;

/*!
 * \addtogroup AS7050_APPMGR_APPS Supported Vital Signs Applications
 *
 * The Application Manager supports four Vital Signs Applications:
 *  - \ref raw_app_group
 *  - \ref bio_hrm_a0_group
 *  - \ref bio_spo2_a0_group
 *  - \ref gsr_app_group
 *
 * The Application Manager provides a common API to all supported Vital Signs Applications. Each application is assigned
 * an ::as7050_appmgr_app_id identifier, which is used in the Application Manager APIs.
 *
 * @{ */

/*! Identifiers of Vital Signs Applications. */
enum as7050_appmgr_app_id {
    AS7050_APPMGR_APP_ID_RAW = 0, /*!< Identifies the \ref raw_app_group application. */
    AS7050_APPMGR_APP_ID_HRM_A0,  /*!< Identifies the \ref bio_hrm_a0_group application. */
    AS7050_APPMGR_APP_ID_SPO2_A0, /*!< Identifies the \ref bio_spo2_a0_group application. */
    AS7050_APPMGR_APP_ID_GSR,     /*!< Identifies the \ref gsr_app_group application. */

    AS7050_APPMGR_APP_ID_NUM, /*!< Number of Vital Signs Applications. */
};

/*! Type for ::as7050_appmgr_app_id. */
typedef uint8_t as7050_appmgr_app_id_t;

/*! @} */

/*!
 * \ingroup AS7050_APPMGR_APPS
 * \defgroup AS7050_APPMGR_APP_FLAG Application Flags
 *
 * These definitions are used by the ::as7050_appmgr_enable_apps, ::as7050_appmgr_set_input, and ::as7050_appmgr_execute
 * functions. Use these definitions with bitwise operators to set, clear, or read the flag bit corresponding to a given
 * Vital Signs Application.
 *
 * @{ */

/*! Macro to create a application flag bitmask value with the bit for the given ::as7050_appmgr_app_id_t set. */
#define M_AS7050_APPMGR_APP_FLAG(app) (1 << (app))

/*! Application flag bitmask value with the bit for ::AS7050_APPMGR_APP_ID_RAW set. */
#define AS7050_APPMGR_APP_FLAG_RAW M_AS7050_APPMGR_APP_FLAG(AS7050_APPMGR_APP_ID_RAW)

/*! Application flag bitmask value with the bit for ::AS7050_APPMGR_APP_ID_HRM_A0 set. */
#define AS7050_APPMGR_APP_FLAG_HRM_A0 M_AS7050_APPMGR_APP_FLAG(AS7050_APPMGR_APP_ID_HRM_A0)

/*! Application flag bitmask value with the bit for ::AS7050_APPMGR_APP_ID_SPO2_A0 set. */
#define AS7050_APPMGR_APP_FLAG_SPO2_A0 M_AS7050_APPMGR_APP_FLAG(AS7050_APPMGR_APP_ID_SPO2_A0)

/*! Application flag bitmask value with the bit for ::AS7050_APPMGR_APP_ID_GSR set. */
#define AS7050_APPMGR_APP_FLAG_GSR M_AS7050_APPMGR_APP_FLAG(AS7050_APPMGR_APP_ID_GSR)

/*! @} */

/*! Describes the version of the Application Manager. */
typedef struct {
    uint8_t major; /*!< Major version */
    uint8_t minor; /*!< Minor version */
    uint8_t patch; /*!< Patch version */
} as7050_appmgr_version_t;

/******************************************************************************
 *                            TYPE SIZE VALIDATION                            *
 ******************************************************************************/

M_STATIC_ASSERT_TYPE_SIZE(as7050_appmgr_channel_id_t, 1);
M_STATIC_ASSERT_TYPE_SIZE(as7050_appmgr_app_id_t, 1);
M_STATIC_ASSERT_TYPE_SIZE(as7050_appmgr_version_t, 3);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* __AS7050_APP_MANAGER_TYPEDEFS_H__ */
