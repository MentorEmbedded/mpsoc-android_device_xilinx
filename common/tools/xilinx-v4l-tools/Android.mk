# Copyright (C) 2018 Mentor Graphics Inc.
# Copyright (C) 2018 Xilinx Inc.
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

ifeq ($(TARGET_USES_XILINX_VCU),true)

LOCAL_PATH := $(call my-dir)

### Test Pattern Generator configuration example ###
include $(CLEAR_VARS)
LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils \
	libmediactl \
	libv4l2subdev

LOCAL_C_INCLUDES := \
	external/v4l-utils/utils/media-ctl/ \
	$(LOCAL_PATH)/include

LOCAL_CFLAGS := -DLOG_TAG=\"configure-tpg\"

LOCAL_SRC_FILES := \
	configure-tpg.c

LOCAL_MODULE := configure-tpg
LOCAL_MODULE_TAGS := optional
include $(BUILD_EXECUTABLE)

### Console tool to capture from v4l2 device ###
include $(CLEAR_VARS)
LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils \
	libmediactl \
	libv4l2subdev

LOCAL_C_INCLUDES := \
	external/v4l-utils/utils/media-ctl/ \
	$(LOCAL_PATH)/include

LOCAL_CFLAGS := -DLOG_TAG=\"capture-example\"

LOCAL_SRC_FILES := \
	capture-example.c

LOCAL_MODULE := capture-example
LOCAL_MODULE_TAGS := optional
include $(BUILD_EXECUTABLE)

### HDMI RX configuration example ###
include $(CLEAR_VARS)
LOCAL_SHARED_LIBRARIES := \
	liblog \
	libcutils \
	libmediactl \
	libv4l2subdev

LOCAL_C_INCLUDES := \
	external/v4l-utils/utils/media-ctl/ \
	$(LOCAL_PATH)/include

LOCAL_CFLAGS := -DLOG_TAG=\"configure-hdmi\"

LOCAL_SRC_FILES := \
	configure-hdmi.c

LOCAL_MODULE := configure-hdmi
LOCAL_MODULE_TAGS := optional
include $(BUILD_EXECUTABLE)
endif
