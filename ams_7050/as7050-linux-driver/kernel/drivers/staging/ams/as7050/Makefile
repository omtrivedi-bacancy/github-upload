
#
# Makefile for ams as7050 i2c driver.
#

obj-$(CONFIG_AMS_AS7050) += ams-as7050-i2c.o

ams-as7050-i2c-y += ./as7050-dev.o
ams-as7050-i2c-y += ./as7050-fcntl.o
ams-as7050-i2c-y += ./as7050-osal.o

ams-as7050-i2c-y += ./../common/vital-sensor-misc.o
ams-as7050-i2c-y += ./../common/vital-sensor-error.o
ams-as7050-i2c-y += ./../common/vital-sensor-i2c.o

ams-as7050-i2c-y += ./../lib/as7050-chip-lib/src/chiplib/as7050_agc.o
ams-as7050-i2c-y += ./../lib/as7050-chip-lib/src/chiplib/as7050_chiplib.o
ams-as7050-i2c-y += ./../lib/as7050-chip-lib/src/chiplib/as7050_extract.o
ams-as7050-i2c-y += ./../lib/as7050-chip-lib/src/chiplib/as7050_interface.o

KBUILD_CFLAGS += -I$(src)
KBUILD_CFLAGS += -I$(src)/../common
KBUILD_CFLAGS += -I$(src)/../lib/as7050-chip-lib/inc/
KBUILD_CFLAGS += -I$(src)/../lib/css-sw-utilities/c/error_codes/inc/
KBUILD_CFLAGS += -DAMS_DRV_NAME='"as7050"'
KBUILD_CFLAGS += -std=gnu99
