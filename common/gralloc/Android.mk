# Copyright (C) 2010 ARM Limited. All rights reserved.
# Copyright (C) 2008 The Android Open Source Project
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

ifneq ($(filter zynqmp%, $(TARGET_BOARD_PLATFORM)),)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

SHARED_MEM_LIBS := libion libhardware libsync
LOCAL_SHARED_LIBRARIES := liblog libcutils libGLESv1_CM $(SHARED_MEM_LIBS)
LOCAL_C_INCLUDES := system/core/include/
LOCAL_CFLAGS := -DLOG_TAG=\"gralloc\" -DGRALLOC_32_BITS \
	-DSTANDARD_LINUX_SCREEN -DPLATFORM_SDK_VERSION=$(PLATFORM_SDK_VERSION)

LOCAL_SRC_FILES := \
	gralloc_module.cpp \
	alloc_device.cpp \
	framebuffer_device.cpp

# Workaround:
# ION ABI has changed in kernel since 4.12
# Android libion is temporary using internal ion_4.12.h header
# for a new interface.
# Include libion's folder to have access to the header with the
# new interface
LOCAL_C_INCLUDES += system/core/libion

LOCAL_C_INCLUDES += system/core/libsync/include

LOCAL_MODULE := gralloc.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH := hw

ifeq ($(BOARD_USES_DRM_HWCOMPOSER), true)
LOCAL_CFLAGS += -DDISABLE_FRAMEBUFFER_HAL
endif

include $(BUILD_SHARED_LIBRARY)
endif
