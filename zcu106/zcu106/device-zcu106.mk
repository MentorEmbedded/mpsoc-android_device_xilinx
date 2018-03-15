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
    device/xilinx/zcu106/init.common.rc:root/init.zcu106.rc \
    device/xilinx/zcu106/init.common.usb.rc:root/init.zcu106.usb.rc \
    device/xilinx/zcu106/ueventd.common.rc:root/ueventd.zcu106.rc

# Copy bootloader envs
PRODUCT_COPY_FILES += \
   $(LOCAL_PATH)/uEnv.txt:boot/uEnv.txt

# Copy media codec settings
PRODUCT_COPY_FILES +=  \
    frameworks/av/media/libstagefright/data/media_codecs_google_audio.xml:system/etc/media_codecs_google_audio.xml \
    frameworks/av/media/libstagefright/data/media_codecs_google_video.xml:system/etc/media_codecs_google_video.xml \
    device/xilinx/common/media_codecs.xml:system/etc/media_codecs.xml

# Copy prebuilt BOOT.BIN if it exists
PRODUCT_COPY_FILES += $(call add-to-product-copy-files-if-exists,\
	$(LOCAL_PATH)/BOOT.BIN:boot/BOOT.BIN)

# Copy FPGA firmware if it exists
PRODUCT_COPY_FILES += $(call add-to-product-copy-files-if-exists,\
	$(LOCAL_PATH)/bitstream.bit:boot/bitstream.bit)
