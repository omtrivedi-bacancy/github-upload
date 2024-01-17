 #
 #############################################################################
 # Copyright by ams AG                                                       #
 # All rights are reserved.                                                  #
 #                                                                           #
 # IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     #
 # THE SOFTWARE.                                                             #
 #                                                                           #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       #
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         #
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         #
 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  #
 # OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     #
 # SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          #
 # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     #
 # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     #
 # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       #
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     #
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      #
 #############################################################################
 #/


# project definitions
PROJECT_NAME     := as7050_sample_code
BUILD_DIR        ?= ./build

# definitions for cross compiling
TARGET_ARCH      ?= x86-64
TARGET_OS        ?= win
TARGET_OS_LIST   := win linux
TARGET_ARCH_LIST := x86-64 armv7-a

# compiler settings
CC               ?= gcc
AS               ?= as
OPT              ?= -O0
CFLAGS           ?= -Wall -Wextra -Werror -std=gnu99 $(OPT)

# extra definitions for preprocessor
DEFINES := \
	-DHOST_INTERFACE

# check if target OS is supported
ifneq ($(filter $(TARGET_OS),$(TARGET_OS_LIST)),$(TARGET_OS))
$(error Given target OS not supported. Please choose out of "$(TARGET_OS_LIST)")
endif

# check if target ARCH is supported
ifneq ($(filter $(TARGET_ARCH),$(TARGET_ARCH_LIST)),$(TARGET_ARCH))
$(error Given suffix for architecture not supported. Please choose out of "$(TARGET_ARCH_LIST)")
endif

ifeq ($(TARGET_OS),win)
TARGET_HRM      ?= $(PROJECT_NAME)_hrm.exe
TARGET_SPO2     ?= $(PROJECT_NAME)_spo2.exe
TARGET_CHIP_LIB ?= $(PROJECT_NAME)_chip_lib.exe
TARGET_GSR      ?= $(PROJECT_NAME)_gsr.exe
else
TARGET_HRM      ?= $(PROJECT_NAME)_hrm
TARGET_SPO2     ?= $(PROJECT_NAME)_spo2
TARGET_CHIP_LIB ?= $(PROJECT_NAME)_chip_lib
TARGET_GSR      ?= $(PROJECT_NAME)_gsr
endif

# sources
# to keep this Makefile simple, all sample code targets build the same set of library sources, even
# though not all sources are needed by all sample code targets
ifeq ($(TARGET_OS),win)
SRCS := \
	as7050-chip-lib/src/as7050_chiplib.c \
	as7050-chip-lib/src/as7050_interface.c \
	as7050-chip-lib/src/as7050_extract.c \
	as7050-chip-lib/src/as7050_agc.c \
	as7050-app-manager/src/app_mngr/as7050_app_manager.c \
	as7050-app-manager/src/app_mngr/gsr_app.c \
	as7050-app-manager/src/app_mngr/raw_app.c \
	css-sw-utilities/c/i2c_drv/lis2dh12/src/lis2dh12.c \
	css-sw-utilities/lib/st_driver_lis2dh12/src/lis2dh12_reg.c \
	vital-signs-acc/src/vital_signs_accelerometer.c
else
ifeq ($(TARGET_ARCH),armv7-a)
SRCS := \
	as7050-linux-driver/userspace/chiplib_wrapper/as7050_wrapper.c \
	as7050-linux-driver/userspace/chiplib_wrapper/as7050_extract.c \
	as7050-linux-driver/userspace/accelerometer/vital_signs_accelerometer_emulated.c \
	as7050-app-manager/src/app_mngr/as7050_app_manager.c \
	as7050-app-manager/src/app_mngr/gsr_app.c \
	as7050-app-manager/src/app_mngr/raw_app.c
else
$(error Building target for $(TARGET_OS) $(TARGET_ARCH) not supported!)
endif
endif

SRCS_HRM := sample_code/src/sample_code_hrm.c

SRCS_SPO2 := sample_code/src/sample_code_spo2.c

SRCS_CHIP_LIB := sample_code/src/sample_code_chip_lib.c

SRCS_GSR := sample_code/src/sample_code_gsr.c

# include directories
# to keep this Makefile simple, all sample code targets use the same set of include paths, even
# though not all include paths are needed by all sample code targets
ifeq ($(TARGET_OS),win)
INCLUDES += \
	as7050-chip-lib/inc \
	as7050-chip-lib/inc/osal \
	bio-app-handler/inc \
	bio-app-handler/inc/hrm_a0 \
	bio-app-handler/inc/spo2_a0 \
	as7050-app-manager/inc/app_mngr \
	as7050-app-manager/inc/accelerometer \
	css-sw-utilities/c/error_codes/inc \
	css-sw-utilities/c/i2c_drv/lis2dh12/inc \
	css-sw-utilities/c/std_inc/inc \
	css-sw-utilities/lib/st_driver_lis2dh12/inc \
	vital-signs-acc/inc
else
INCLUDES += \
	as7050-linux-driver/userspace/chiplib_wrapper \
	as7050-linux-driver/userspace/accelerometer \
	bio-app-handler/inc \
	bio-app-handler/inc/hrm_a0 \
	bio-app-handler/inc/spo2_a0 \
	as7050-app-manager/inc/app_mngr \
	css-sw-utilities/c/error_codes/inc \
	css-sw-utilities/c/std_inc/inc
endif

# libraries to use
# to keep this Makefile simple, all sample code targets link against the same set of libraries,
# even though not all libraries are needed by all sample code targets
LIBDIRS += \
	bio-app-handler/lib/hrm_a0/$(TARGET_OS)_$(TARGET_ARCH) \
	bio-app-handler/lib/spo2_a0/$(TARGET_OS)_$(TARGET_ARCH) \

ifeq ($(TARGET_OS),win)
LIBDIRS += \
	as7050-chip-lib/osal/win_x86-64
LIBS += \
	as7050_osal_xenon_core \
	rpc_client_comm \
	rpc_client_protocol \
	rpc_client_osal \
	bio_hrm_a0 \
	hrm_algorithm \
	prv_algorithm \
	bio_spo2_a0 \
	spo2_algorithm \
	ws2_32
LIB_FLAGS := $(addprefix -l,$(LIBS))
else
LIBS += \
	bio_hrm_a0 \
	hrm_algorithm \
	prv_algorithm \
	bio_spo2_a0 \
	spo2_algorithm
LIB_FLAGS := $(addprefix -l,$(LIBS)) -lm -lpthread
endif

INC_FLAGS += $(addprefix -I,$(INCLUDES))
LIBDIR_FLAGS += $(addprefix -L,$(LIBDIRS))

# objects to build
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
OBJS_HRM := $(SRCS_HRM:%=$(BUILD_DIR)/%.o)
OBJS_SPO2 := $(SRCS_SPO2:%=$(BUILD_DIR)/%.o)
OBJS_CHIP_LIB := $(SRCS_CHIP_LIB:%=$(BUILD_DIR)/%.o)
OBJS_GSR := $(SRCS_GSR:%=$(BUILD_DIR)/%.o)

all: $(TARGET_HRM) $(TARGET_SPO2) $(TARGET_CHIP_LIB) $(TARGET_GSR)
print-%:
	@echo $*=$($*)
# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CFLAGS) $(INC_FLAGS) $(DEFINES) -c $< -o $@

$(TARGET_HRM): $(OBJS) $(OBJS_HRM)
	$(CC) $(OBJS) $(OBJS_HRM) $(LIBDIR_FLAGS) $(LIB_FLAGS) -o $@

$(TARGET_SPO2): $(OBJS) $(OBJS_SPO2)
	$(CC) $(OBJS) $(OBJS_SPO2) $(LIBDIR_FLAGS) $(LIB_FLAGS) -o $@

$(TARGET_CHIP_LIB): $(OBJS) $(OBJS_CHIP_LIB)
	$(CC) $(OBJS) $(OBJS_CHIP_LIB) $(LIBDIR_FLAGS) $(LIB_FLAGS) -o $@

$(TARGET_GSR): $(OBJS) $(OBJS_GSR)
	$(CC) $(OBJS) $(OBJS_GSR) $(LIBDIR_FLAGS) $(LIB_FLAGS) -o $@

$(BUILD_DIR):
	mkdir $@

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

MKDIR_P ?= mkdir -p