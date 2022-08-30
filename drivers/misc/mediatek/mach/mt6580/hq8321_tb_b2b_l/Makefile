
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifneq ($(CONFIG_ARM64), y)
obj-y   +=  camera/camera/
endif
obj-y   +=  flashlight/constant_flashlight/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-$(CONFIG_MTK_ACCDET)   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)

ifeq ($(CONFIG_MTK_BMA222E_NEW),y)
obj-y   +=  accelerometer/bma222E-new/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  accelerometer/KXTJ2_1009-new/
endif

ifeq ($(CONFIG_MTK_BMA250E),y)
obj-y   +=  accelerometer/bma250e/
endif

ifeq ($(CONFIG_MTK_S62X),y)
obj-y	+=  magnetometer/s62x/
endif
endif
