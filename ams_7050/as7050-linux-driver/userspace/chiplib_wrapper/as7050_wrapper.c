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
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

#include "as7050-fcntl.h"
#include "as7050_chiplib.h"
#include "as7050_extract.h"
#include "as7050_interface.h"
#include "as7050_wrapper.h"
#include "as7050_wrapper_version.h"
#include "error_codes.h"

/******************************************************************************
 *                                DEFINITIONS                                 *
 ******************************************************************************/
/*! The default path to the device node of the underlying driver */
#ifndef SENSOR_DEVICE_PATH
#define SENSOR_DEVICE_PATH "/dev/as7050"
#endif

/*! A single read from driver will return a data set with this head, following data and item(s) */
struct chiplib_data_set_head {
    err_code_t error;
    as7050_agc_status_t agc_status;
    uint16_t data_size;
};

/*! Holds the information about an opened connection to the underlying driver */
struct chiplib_info {
    int fd;                          /*!< The file descriptor to the device node of the driver */
    pthread_t datathread;            /*!< Handle to the data read thread */
    uint8_t runDataThread;           /*!< Flag signalizing, that data read thread shall run */
    as7050_callback_t user_data_fnc; /*!< User callback function, transfering measurement results to the application. */
    void *p_cp_param;                /*!< Additional parameter which will be given to the User callback function */
    uint32_t fifo_size;              /*!< Size of the kernel fifo*/
    uint8_t run_meas;                /*!< Flag indicates if measurement is running*/
};

/*! Holds the version information of the wrapper library */
struct wrapper_version {
    uint8_t major; /*!< Major number */
    uint8_t minor; /*!< Minor number */
    uint8_t patch; /*!< Patch level */
};

/******************************************************************************
 *                                  GLOBALS                                   *
 ******************************************************************************/
static struct chiplib_info chiplib = {0};
static struct wrapper_version version = {
    .major = AS7050_WRAPPER_VER_MAJOR, .minor = AS7050_WRAPPER_VER_MINOR, .patch = AS7050_WRAPPER_VER_PATCH};

/******************************************************************************
 *                               LOCAL FUNCTIONS                              *
 ******************************************************************************/

/*!
 * \brief Check the return code of an ioctl call
 *
 * \param[in] err       The return code of an ioctl call
 *
 * \retval err_code_t   Returns the translated internal error code
 */
err_code_t check_ioctl_call(int err)
{
    err_code_t result = ERR_NOT_SUPPORTED;
    if (-1 == err) {
        fprintf(stderr, "AS7050_WRAPPER: (%s) calling ioctl failed with %d - %s\n", SENSOR_DEVICE_PATH, err,
                strerror(errno));
    } else {
        result = (err_code_t)err;
    }
    return result;
}

/*!
 * \brief Thread which reads data from a device node
 *
 * The opened device node is polled continuously for new data. If new data is available,
 * the complete data set is read and the registered user callback is called with it.
 *
 * \param[in] arg   Thread argument holding valid sensor information
 */
static void *__sensor_read_thread(void *arg)
{
    int err, bytes;
    fd_set rfds;
    struct timeval tmo;
    struct chiplib_data_set_head ds_head;

    struct chiplib_info *pci = arg;
    uint8_t *pData = malloc(pci->fifo_size);
    uint8_t *pDataStart = pData;
    while (pci->runDataThread) {
        FD_ZERO(&rfds);
        FD_SET(pci->fd, &rfds);
        tmo.tv_sec = 0;
        tmo.tv_usec = 1 * 1000;

        // poll the underlying device node
        err = select(pci->fd + 1, &rfds, NULL, NULL, &tmo);
        if (err < 0) {
            break;
        } else if (err > 0) {
            if (!FD_ISSET(pci->fd, &rfds)) {
                break;
            }

            // new data is available, try to read it
            pData = pDataStart;
            memset(pData, 0, pci->fifo_size);
            err = read(pci->fd, pData, pci->fifo_size);
            if (err > 0) {
                bytes = err;
                // read new data and call the registered user callback while datasets are available
                while (bytes > 0) {
                    ds_head = *(struct chiplib_data_set_head *)pData;
                    if (pci->user_data_fnc && pci->run_meas) {

                        if (ERR_OVERFLOW == ds_head.error) {
                            pci->run_meas = 0;
                            check_ioctl_call(ioctl(pci->fd, AS7050_STOP_MEASUREMENT));
                        }

                        pci->user_data_fnc(ds_head.error, (uint8_t *)(pData + sizeof(ds_head)),
                                           ds_head.data_size / sizeof(uint16_t), &ds_head.agc_status, pci->p_cp_param);
                    }
                    bytes -= sizeof(ds_head) + ds_head.data_size;
                    pData = (uint8_t *)(pData + sizeof(ds_head) + ds_head.data_size);
                };
            }
        } else {
            // timeout - don't care
            continue;
        }
    }

    free(pDataStart);
    return (void *)pthread_self();
}

/******************************************************************************
 *                             GLOBAL FUNCTIONS                               *
 ******************************************************************************/
err_code_t as7050_initialize(const as7050_callback_t p_callback, const void *p_cb_param, const char *p_interface_descr)
{
    int result;
    err_code_t err = ERR_SYSTEM_CONFIG;
    M_UNUSED_PARAM(p_interface_descr);

    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    if (0 != chiplib.fd) {
        // call the underlying function of the driver
        err = check_ioctl_call(ioctl(chiplib.fd, AS7050_INITIALIZE));

        if (err == ERR_SUCCESS) {
            err = check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_FIFO_SIZE, &chiplib.fifo_size));
        }

        // create thread which waits for new data and calls user callback in case of data
        if (err == ERR_SUCCESS) {
            chiplib.user_data_fnc = p_callback;
            chiplib.p_cp_param = (void *)p_cb_param;
            chiplib.runDataThread = 1;

            result = pthread_create(&chiplib.datathread, NULL, &__sensor_read_thread, (void *)&chiplib);
            if (0 != result) {
                close(chiplib.fd);
                chiplib.fd = 0;
                err = ERR_THREAD;
            }
        }
    }

    return err;
}

err_code_t as7050_shutdown(void)
{
    err_code_t err = ERR_SUCCESS;
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    err = check_ioctl_call(ioctl(chiplib.fd, AS7050_SHUTDOWN));

    // stop the data read thread
    if (0 != chiplib.runDataThread) {
        chiplib.runDataThread = 0;
        pthread_join(chiplib.datathread, NULL);
    }

    close(chiplib.fd);
    chiplib.fd = 0;
    return err;
}

err_code_t as7050_set_reg_group(enum as7050_reg_group_ids id, const uint8_t *p_data, const uint8_t size)
{
    struct as7050_reg_group reg_group;

    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    reg_group.id = id;
    reg_group.data = (uint8_t *)p_data;
    reg_group.size = size;

    // call the underlying function of the driver
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_SET_REG_GROUP, &reg_group));
}

err_code_t as7050_get_reg_group(const enum as7050_reg_group_ids id, uint8_t *p_data, uint8_t *p_size)
{
    err_code_t err = ERR_SUCCESS;
    struct as7050_reg_group reg_group;

    M_CHECK_NULL_POINTER(p_size);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    reg_group.id = id;
    reg_group.data = p_data;
    reg_group.size = *p_size;

    // call the underlying function of the driver
    err = check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_REG_GROUP, &reg_group));
    if (err == ERR_SUCCESS) {
        *p_size = reg_group.size;
    }

    return err;
}

err_code_t as7050_set_agc_config(const as7050_agc_config_t *p_agc_config)
{
    M_CHECK_NULL_POINTER(p_agc_config);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_SET_AGC_CONFIG, p_agc_config));
}

err_code_t as7050_get_agc_config(as7050_agc_config_t *p_agc_config)
{
    M_CHECK_NULL_POINTER(p_agc_config);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_AGC_CONFIG, p_agc_config));
}

err_code_t as7050_write_register(uint8_t reg_addr, uint8_t reg_val)
{
    struct as7050_reg_pair reg_pair;

    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    reg_pair.addr = reg_addr;
    reg_pair.val = reg_val;

    // call the underlying function of the driver
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_WRITE_REGISTER, &reg_pair));
}

err_code_t as7050_read_register(uint8_t reg_addr, uint8_t *p_reg_val)
{
    struct as7050_reg_pair reg_pair;
    err_code_t err = ERR_SUCCESS;

    M_CHECK_NULL_POINTER(p_reg_val);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }
    reg_pair.addr = reg_addr;

    // call the underlying function of the driver
    err = check_ioctl_call(ioctl(chiplib.fd, AS7050_READ_REGISTER, &reg_pair));

    if (ERR_SUCCESS == err) {
        *p_reg_val = reg_pair.val;
    }

    return err;
}

err_code_t as7050_get_measurement_config(as7050_meas_config_t *p_meas_config)
{
    M_CHECK_NULL_POINTER(p_meas_config);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_MEASUREMENT_CONFIG, p_meas_config));
}

err_code_t as7050_extract_samples(as7050_channel_flags_t chan, const uint8_t *p_fifo_data, uint16_t fifo_data_num,
                                  uint32_t *p_chan_data, uint16_t *p_chan_data_num)
{
    err_code_t result;
    as7050_meas_config_t meas_config;

    result = as7050_get_measurement_config(&meas_config);

    if (ERR_SUCCESS == result) {
        result = as7050_ifce_extract_samples(meas_config.fifo_map, meas_config.sample_size, chan, p_fifo_data,
                                             fifo_data_num, p_chan_data, p_chan_data_num);
    }

    return result;
}

err_code_t as7050_start_measurement(void)
{
    err_code_t result;
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    result = check_ioctl_call(ioctl(chiplib.fd, AS7050_START_MEASUREMENT));
    if (ERR_SUCCESS == result) {
        chiplib.run_meas = 1;
    }

    return result;
}

err_code_t as7050_stop_measurement(void)
{
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    // call the underlying function of the driver
    chiplib.run_meas = 0;
    return check_ioctl_call(ioctl(chiplib.fd, AS7050_STOP_MEASUREMENT));
}

err_code_t as7050_get_version(as7050_version_t *p_version)
{
    M_CHECK_NULL_POINTER(p_version);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    return check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_VERSION, p_version));
}

err_code_t as7050_wrapper_get_version(uint8_t *p_major, uint8_t *p_minor, uint8_t *p_patch)
{
    M_CHECK_NULL_POINTER(p_major);
    M_CHECK_NULL_POINTER(p_minor);
    M_CHECK_NULL_POINTER(p_patch);

    *p_major = version.major;
    *p_minor = version.minor;
    *p_patch = version.patch;

    return ERR_SUCCESS;
}

err_code_t as7050_calculate_dac_reference_value(dac_ref_control_t control, uint16_t average_cnt,
                                                dac_ref_status_t *p_status, uint32_t *p_dac_ref)
{
    struct as7050_dac_ref_val dac_val;
    err_code_t err = ERR_SUCCESS;

    M_CHECK_NULL_POINTER(p_status);
    M_CHECK_NULL_POINTER(p_dac_ref);
    if (0 == chiplib.fd) {
        chiplib.fd = open(SENSOR_DEVICE_PATH, O_RDWR);
    }

    dac_val.control = control;
    dac_val.average_cnt = average_cnt;

    // call the underlying function of the driver
    err = check_ioctl_call(ioctl(chiplib.fd, AS7050_GET_DAC_REF_VAL, &dac_val));

    if (ERR_SUCCESS == err) {
        *p_status = dac_val.status;
        *p_dac_ref = dac_val.dac_ref;
    }

    return err;
}
