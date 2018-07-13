#
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
#

HDMI_MODULES_SRC_DIR := $(ANDROID_BUILD_TOP)/hardware/xilinx/hdmi/hdmi-modules
HDMI_MODULES_OUT_DIR := $(TARGET_OUT_INTERMEDIATES)/HDMI_MODULES_OBJ
HDMI_CROSS_COMP := $(notdir $(TARGET_TOOLS_PREFIX))

HDMI_BUILD_FLAGS := \
	ARCH=$(TARGET_ARCH) \
	CROSS_COMPILE=$(HDMI_CROSS_COMP) \
	O=$(ANDROID_BUILD_TOP)/$(KERNEL_OUT_DIR) \
	CFLAGS_MODULE="-fno-pic" \
	ANDROID_TOOLCHAIN_FLAGS="-mno-android -Werror"
	
hdmi_modules: install_hdmi_modules

$(HDMI_MODULES_OUT_DIR): $(HDMI_MODULES_SRC_DIR)
	@$(info Copying directory $(HDMI_MODULES_SRC_DIR) to $(HDMI_MODULES_OUT_DIR))
	@rm -rf $(HDMI_MODULES_OUT_DIR)
	@mkdir -p $(HDMI_MODULES_OUT_DIR)
	@cp -rfl $(HDMI_MODULES_SRC_DIR)/* $(HDMI_MODULES_OUT_DIR)/

build_hdmi_modules: build_kernel $(HDMI_MODULES_OUT_DIR)
	KERNEL_SRC=$(KERNEL_SRC_DIR) $(HDMI_BUILD_FLAGS) $(MAKE) -C $(HDMI_MODULES_OUT_DIR)

install_hdmi_modules: build_hdmi_modules
	$(info Copying kernel HDMI modules to $(KERNEL_MODULES_OUT))
	$(hide) mkdir -p $(KERNEL_MODULES_OUT)
	@cp $(HDMI_MODULES_OUT_DIR)/hdmi/xilinx-vphy.ko $(KERNEL_MODULES_OUT)
	@cp $(HDMI_MODULES_OUT_DIR)/hdmi/xilinx-hdmi-tx.ko $(KERNEL_MODULES_OUT)
	@cp $(HDMI_MODULES_OUT_DIR)/hdmi/xilinx-hdmi-rx.ko $(KERNEL_MODULES_OUT)
	@cp $(HDMI_MODULES_OUT_DIR)/clk/si5324.ko $(KERNEL_MODULES_OUT)
	@cp $(HDMI_MODULES_OUT_DIR)/misc/dp159.ko $(KERNEL_MODULES_OUT)

.PHONY: hdmi_modules
droidcore: hdmi_modules
