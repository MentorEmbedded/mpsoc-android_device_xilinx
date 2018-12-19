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

# Copy basic config files
PRODUCT_COPY_FILES += \
    device/xilinx/zcu106/fstab.common:root/fstab.zcu106 \
    device/xilinx/zcu106/zcu106_vcu/init.zcu106_vcu.rc:root/init.zcu106.rc \
    device/xilinx/zcu106/init.common.usb.rc:root/init.zcu106.usb.rc \
    device/xilinx/zcu106/zcu106_vcu/ueventd.zcu106_vcu.rc:root/ueventd.zcu106.rc

# Copy VCU firmware
PRODUCT_COPY_FILES += \
    hardware/xilinx/vcu/vcu-firmware/1.0.0/lib/firmware/al5d.fw:system/etc/firmware/al5d.fw \
    hardware/xilinx/vcu/vcu-firmware/1.0.0/lib/firmware/al5d_b.fw:system/etc/firmware/al5d_b.fw \
    hardware/xilinx/vcu/vcu-firmware/1.0.0/lib/firmware/al5e.fw:system/etc/firmware/al5e.fw \
    hardware/xilinx/vcu/vcu-firmware/1.0.0/lib/firmware/al5e_b.fw:system/etc/firmware/al5e_b.fw

# Copy HDMI RX EDID firmware
PRODUCT_COPY_FILES += \
	device/xilinx/zcu106/zcu106_vcu/xilinx-hdmi-rx-edid.bin:system/etc/firmware/xilinx/xilinx-hdmi-rx-edid.bin \

# Copy media_codec config
PRODUCT_COPY_FILES += \
    frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml:system/etc/media_codecs_google_audio.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video.xml:system/etc/media_codecs_google_video.xml \
    device/xilinx/zcu106/zcu106_vcu/media_codecs.xml:system/etc/media_codecs.xml

# Copy camera configs
PRODUCT_COPY_FILES +=  \
    frameworks/native/data/etc/android.hardware.camera.xml:system/etc/android.hardware.camera.xml \
    device/xilinx/zcu106/zcu106_vcu/media_profiles.xml:system/etc/media_profiles.xml

# Copy bootloader envs
PRODUCT_COPY_FILES += \
   $(LOCAL_PATH)/uEnv.txt:boot/uEnv.txt

# Copy prebuilt BOOT.BIN if it exists
PRODUCT_COPY_FILES += $(call add-to-product-copy-files-if-exists,\
    $(LOCAL_PATH)/BOOT.BIN:boot/BOOT.BIN)

# Copy FPGA firmware if it exists
PRODUCT_COPY_FILES += $(call add-to-product-copy-files-if-exists,\
    $(LOCAL_PATH)/bitstream.bit:boot/bitstream.bit)

# Include packages for VCU
PRODUCT_PACKAGES += \
    liballegro_decode \
    liballegro_encode \
    al_decoder \
    al_encoder \
    libOMX.allegro.core \
    libOMX.allegro.video_decoder \
    libOMX.allegro.video_encoder \
    omx_decoder \
    omx_encoder \
    libstagefrighthw \
    recordvideo \
    stagefright

# Install required kernel modules
KERNEL_MODULES += \
    drivers/soc/xilinx/xlnx_vcu.ko \
    drivers/soc/xilinx/xlnx_vcu_core.ko \
    drivers/soc/xilinx/xlnx_vcu_clk.ko

# Default OMX service to non-Treble
PRODUCT_PROPERTY_OVERRIDES += \
    persist.media.treble_omx=false

# Add camera-related packages
PRODUCT_PACKAGES += \
	android.hardware.camera.provider@2.4-impl \
	camera.device@3.2-impl \
	TestingCamera2 \
	camera.zynqmp

PRODUCT_PROPERTY_OVERRIDES += ro.hardware.camera=zynqmp
