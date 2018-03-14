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

# include makefile for Xilinx kernel
# variables for kernel build should be set in
# product makefile
include device/xilinx/common/build/kernel.mk

# include makefile for Xilinx VCU kernel modules
# it should be after kernel.mk since it uses some
# kernel build variables
include device/xilinx/common/build/vcu_modules.mk

# include makefile for Xilinx U-Boot
# variables for U-Boot build should be set in
# product makefile
include device/xilinx/common/build/u-boot.mk
