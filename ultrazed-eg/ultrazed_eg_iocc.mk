#
# Copyright (C) 2017 Mentor Graphics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Inherit device specific configurations
$(call inherit-product, device/xilinx/ultrazed-eg/device-common.mk)
$(call inherit-product, device/xilinx/ultrazed-eg/ultrazed_eg_iocc/device-iocc.mk)

# Inherit full base product
$(call inherit-product, $(SRC_TARGET_DIR)/product/full_base.mk)

# Include vendor binaries
$(call inherit-product-if-exists, vendor/xilinx/zynqmp/device-vendor.mk)

# Specify product details
PRODUCT_NAME := ultrazed_eg_iocc
PRODUCT_DEVICE := ultrazed_eg_iocc
PRODUCT_BRAND := Android
PRODUCT_MODEL := AOSP on Avnet UltraZed-EG with IO Carrier Card
PRODUCT_MANUFACTURER := Avnet

# Specify variables for kernel build
KERNEL_SRC_DIR ?= linux-xlnx
KERNEL_CFG_NAME ?= xilinx_zynqmp_android_defconfig
KERNEL_DTS_NAMES ?= \
	zynqmp-ultrazed-eg-iocc.dts

# Check for availability of kernel source
ifneq ($(wildcard $(KERNEL_SRC_DIR)/Makefile),)
	# Give precedence to TARGET_PREBUILT_KERNEL
	ifeq ($(TARGET_PREBUILT_KERNEL),)
		TARGET_KERNEL_BUILT_FROM_SOURCE := true
	endif
endif

# Include gralloc for Mali GPU
PRODUCT_PACKAGES += \
	gralloc.zynqmp
