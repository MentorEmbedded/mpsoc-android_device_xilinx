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

# Primary Arch
TARGET_ARCH := arm64
TARGET_ARCH_VARIANT := armv8-a
TARGET_CPU_VARIANT := generic
TARGET_CPU_ABI := arm64-v8a

# Secondary Arch
TARGET_2ND_ARCH := arm
TARGET_2ND_ARCH_VARIANT := armv7-a-neon
TARGET_2ND_CPU_VARIANT := cortex-a15
TARGET_2ND_CPU_ABI := armeabi-v7a
TARGET_2ND_CPU_ABI2 := armeabi

# Set board platform name
TARGET_BOARD_PLATFORM := zynqmp

# Use 64bit Binder
TARGET_USES_64_BIT_BINDER := true

# enable to use the CPUSETS feature
ENABLE_CPUSETS := true

USE_OPENGL_RENDERER := true
USE_CAMERA_STUB := true
BOARD_USES_GENERIC_AUDIO := true
TARGET_NO_BOOTLOADER := true
TARGET_NO_RADIOIMAGE := true

TARGET_USERIMAGES_USE_EXT4 := true
BOARD_SYSTEMIMAGE_PARTITION_SIZE := 1073741824
BOARD_FLASH_BLOCK_SIZE := 131072
TARGET_USERIMAGES_SPARSE_EXT_DISABLED := true

ifeq ($(BOARD_WLAN_VENDOR),TI)
WPA_SUPPLICANT_VERSION := VER_0_8_X
BOARD_WPA_SUPPLICANT_DRIVER := NL80211
BOARD_HOSTAPD_DRIVER := NL80211
CONFIG_DRIVER_NL80211 := y
BOARD_WPA_SUPPLICANT_PRIVATE_LIB := lib_driver_cmd_wl12xx
BOARD_WLAN_DEVICE                := wl12xx_mac80211
WIFI_DRIVER_MODULE_NAME          := "wl12xx"
WIFI_DRIVER_MODULE_PATH          := "/vendor/modules/wl12xx.ko"
BOARD_HOSTAPD_PRIVATE_LIB        := lib_driver_cmd_wl12xx
BOARD_SOFTAP_DEVICE              := wl12xx_mac80211
USES_TI_MAC80211                 := true
COMMON_GLOBAL_CFLAGS             += -DUSES_TI_MAC80211
endif

ifeq ($(HOST_OS),linux)
    ifeq ($(WITH_DEXPREOPT),)
      WITH_DEXPREOPT := true
    endif
endif
