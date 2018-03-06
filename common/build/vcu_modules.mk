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

VCU_MODULES_SRC_DIR := $(ANDROID_BUILD_TOP)/hardware/xilinx/vcu/vcu-modules
VCU_MODULES_OUT_DIR := $(TARGET_OUT_INTERMEDIATES)/VCU_MODULES_OBJ
VCU_CROSS_COMP := $(notdir $(TARGET_TOOLS_PREFIX))

VCU_BUILD_FLAGS := \
	ARCH=$(TARGET_ARCH) \
	CROSS_COMPILE=$(VCU_CROSS_COMP) \
	O=$(ANDROID_BUILD_TOP)/$(KERNEL_OUT_DIR) \
	CFLAGS_MODULE="-fno-pic" \
	ANDROID_TOOLCHAIN_FLAGS="-mno-android -Werror"
	
vcu_modules: install_vcu_modules

$(VCU_MODULES_OUT_DIR): $(VCU_MODULES_SRC_DIR)
	@$(info Copying directory $(VCU_MODULES_SRC_DIR) to $(VCU_MODULES_OUT_DIR))
	@rm -rf $(VCU_MODULES_OUT_DIR)
	@mkdir -p $(VCU_MODULES_OUT_DIR)
	@cp -rfl $(VCU_MODULES_SRC_DIR)/* $(VCU_MODULES_OUT_DIR)/

build_vcu_modules: build_kernel $(VCU_MODULES_OUT_DIR)
	KERNEL_SRC=$(KERNEL_SRC_DIR) $(VCU_BUILD_FLAGS) $(MAKE) -C $(VCU_MODULES_OUT_DIR)

install_vcu_modules: build_vcu_modules
	$(info Copying kernel VCU modules to $(KERNEL_MODULES_OUT))
	$(hide) mkdir -p $(KERNEL_MODULES_OUT)
	@cp $(VCU_MODULES_OUT_DIR)/common/allegro.ko $(KERNEL_MODULES_OUT)
	@cp $(VCU_MODULES_OUT_DIR)/al5d/al5d.ko $(KERNEL_MODULES_OUT)
	@cp $(VCU_MODULES_OUT_DIR)/al5e/al5e.ko $(KERNEL_MODULES_OUT)

.PHONY: vcu_modules
droidcore: vcu_modules
