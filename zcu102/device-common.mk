#
# Copyright (C) 2016 The Android Open-Source Project
# Copyright (C) 2016 Mentor Graphics Inc.
# Copyright (C) 2016 Xilinx Inc.
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

# Common device configuration.

# Inherit 64-bit device configuration
$(call inherit-product, $(SRC_TARGET_DIR)/product/core_64_bit.mk)

# Adjust the dalvik heap to be appropriate for a tablet.
$(call inherit-product, frameworks/native/build/tablet-10in-xhdpi-2048-dalvik-heap.mk)

# Build and run only ART
OVERRIDE_RUNTIMES := runtime_libart_default

# Set custom settings for the framework
DEVICE_PACKAGE_OVERLAYS += \
    device/xilinx/zcu102/overlay

# Copy supported hardware features config file(s)
PRODUCT_COPY_FILES +=  \
    frameworks/native/data/etc/android.hardware.screen.landscape.xml:system/etc/permissions/android.hardware.screen.landscape.xml \
    frameworks/native/data/etc/android.hardware.ethernet.xml:system/etc/permissions/android.hardware.ethernet.xml \
    frameworks/native/data/etc/android.software.app_widgets.xml:system/etc/permissions/android.software.app_widgets.xml \
    frameworks/native/data/etc/android.software.backup.xml:system/etc/permissions/android.software.backup.xml \
    frameworks/native/data/etc/android.hardware.usb.accessory.xml:system/etc/permissions/android.hardware.usb.accessory.xml \
    frameworks/native/data/etc/android.hardware.usb.host.xml:system/etc/permissions/android.hardware.usb.host.xml

# Add libion for graphics
PRODUCT_PACKAGES += \
	libion \
	libdrm

# Include libs for SW graphics
PRODUCT_PACKAGES += libGLES_android

# Graphics HAL
PRODUCT_PACKAGES += \
    android.hardware.graphics.allocator@2.0-impl \
    android.hardware.graphics.mapper@2.0-impl \
    android.hardware.graphics.composer@2.1-impl \

PRODUCT_PACKAGES += \
    android.hardware.audio@2.0-impl \
	android.hardware.audio.effect@2.0-impl \

# Keymaster HAL
PRODUCT_PACKAGES += \
    android.hardware.keymaster@3.0-impl

TARGET_USES_HWC2 := true
BOARD_USES_DRM_HWCOMPOSER := true
PRODUCT_PACKAGES += hwcomposer.drm
SF_START_GRAPHICS_ALLOCATOR_SERVICE := true
