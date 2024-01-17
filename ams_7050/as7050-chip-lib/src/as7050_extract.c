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

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "as7050_extract.h"
#include "as7050_std_include.h"
#include "as7050_typedefs.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

static uint32_t get_bit_count(uint32_t val)
{
    // see http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
    uint32_t bit_count;
    for (bit_count = 0; val; bit_count++)
        val &= val - 1;
    return bit_count;
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/

err_code_t as7050_ifce_extract_samples(uint16_t fifo_map, uint8_t sample_size, as7050_channel_flags_t chan,
                                       const uint8_t *p_fifo_data, uint16_t fifo_data_num, uint32_t *p_chan_data,
                                       uint16_t *p_chan_data_num)
{
    uint32_t i, j;
    uint32_t sample;
    uint8_t sample_type;
    uint8_t old_ppg_channel = 0xFF;
    as7050_channel_flags_t actual_sample_type;
    uint16_t maximum_num;
    uint16_t fifo_map_without_gsr = fifo_map & (~AS7050_CHANNEL_FLAG_GSR);

    M_CHECK_NULL_POINTER(p_fifo_data);
    M_CHECK_NULL_POINTER(p_chan_data);
    M_CHECK_NULL_POINTER(p_chan_data_num);

    if ((3 != sample_size) && (4 != sample_size)) {
        return ERR_CONFIG;
    }
    if (0 == *p_chan_data_num) {
        return ERR_ARGUMENT;
    }

    if ((0 == fifo_map_without_gsr) || (AS7050_CHANNEL_FLAG_STATUS < fifo_map_without_gsr)) {
        return ERR_CONFIG;
    }
    if (0 == (fifo_map_without_gsr & chan)) {
        return ERR_ARGUMENT;
    }

    if (0 == fifo_data_num) {
        return ERR_ARGUMENT;
    }

    if (fifo_data_num % sample_size) {
        return ERR_ARGUMENT;
    }

    maximum_num = *p_chan_data_num;
    *p_chan_data_num = 0;

    for (i = 0; i < fifo_data_num; i += sample_size) {
        sample = p_fifo_data[i] >> 3;
        sample |= p_fifo_data[i + 1] << 5;
        sample |= p_fifo_data[i + 2] << 13;

        if ((4 == sample_size) && (0 != p_fifo_data[i + 3])) {
            return ERR_ARGUMENT;
        }

        sample_type = p_fifo_data[i] & 0x03;
        if (FIFO_DATA_MARKER_ECG == sample_type) {
            actual_sample_type = AS7050_CHANNEL_FLAG_ECG;
        } else if (FIFO_DATA_MARKER_PPG_1 == sample_type) {
            actual_sample_type = AS7050_CHANNEL_FLAG_PPG_1;
            old_ppg_channel = 0;
        } else if (FIFO_DATA_MARKER_STATUS == sample_type) {
            actual_sample_type = AS7050_CHANNEL_FLAG_STATUS;
        } else { /* FIFO_DATA_MARKER_PPG_2_8 */

            /* Data starts with any other ppg channel than PPG1.
               We must search for the next available unique data marker */
            if (0xFF == old_ppg_channel) {

                uint32_t num_ppg_channels = get_bit_count(fifo_map_without_gsr & 0xFF);

                for (j = i; j < fifo_data_num; j += sample_size) {
                    sample_type = p_fifo_data[j] & 0x03;
                    if (sample_type != FIFO_DATA_MARKER_PPG_2_8) {
                        /* found new marker, finished */
                        old_ppg_channel = num_ppg_channels - (j / sample_size) - 1;
                        break;
                    }
                }
            }

            if (0xFF == old_ppg_channel) {
                /* We can't analyse which channel was the first ppg channel, need more data */
                return ERR_ARGUMENT;
            }
            old_ppg_channel++;
            if (7 < old_ppg_channel) {
                return ERR_ARGUMENT;
            }
            actual_sample_type = (as7050_channel_flags_t)((int)AS7050_CHANNEL_FLAG_PPG_1 << old_ppg_channel);
        }

        if (actual_sample_type == chan) {
            if (maximum_num > *p_chan_data_num) {
                p_chan_data[(*p_chan_data_num)++] = sample;
            } else {
                return ERR_SIZE;
            }
        }
    }

    return ERR_SUCCESS;
}
