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

#ifndef __AS7050_STD_INCLUDE_H__
#define __AS7050_STD_INCLUDE_H__

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/
#include <stdarg.h>

#ifndef __KERNEL__
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#else
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/types.h>

#include <asm/string.h>
#endif

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef __KERNEL__
#define DIV64_S64(s64dividend, s64divisor) div64_s64(s64dividend, s64divisor)
#define DIV64_U64(u64dividend, u64divisor) div64_u64(u64dividend, u64divisor)
#define UINT8_MAX U8_MAX
#define UINT16_MAX U16_MAX
#define UINT32_MAX U32_MAX
#define UINT64_MAX U64_MAX
#else
#define DIV64_S64(s64dividend, s64divisor) (s64dividend / s64divisor)
#define DIV64_U64(u64dividend, u64divisor) (u64dividend / u64divisor)
#endif

#endif //__AS7050_STD_INCLUDE_H__
