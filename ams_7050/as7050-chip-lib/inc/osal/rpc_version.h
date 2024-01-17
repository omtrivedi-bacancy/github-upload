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

#ifndef __RPC_PROTOCOL_H__
#define __RPC_PROTOCOL_H__

/*!
 * \file      rpc_version.h
 * \authors   ARIT
 * \copyright ams
 * \addtogroup version_group_rpc
 *
 * \brief Version information for RPC server
 *
 * @{
 */

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include <stdint.h>

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/*! version structure for the RPC-library */
typedef struct {
    uint8_t major; /*!< Major version information */
    uint8_t minor; /*!< Minor version information */
    uint8_t patch; /*!< Patch level of library */
} rpc_version_t;

#define PROJECT_NAME "RPC PROTOCOL" /*!< Application name */
#define RPC_VER_MAJOR 3             /*!< Major version number */
#define RPC_VER_MINOR 0             /*!< Minor version number */
#define RPC_VER_PATCH 0             /*!< Patch version number */

/******************************************************************************
 *                                 FUNCTIONS                                  *
 ******************************************************************************/

/*! @} */

#endif /* __RPC_PROTOCOL_H__ */
