#
# Copyright (C) 2025 Xiaomi Corporation
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

if(CONFIG_AURIX_MCMCAN)

set(INCDIRS
          ${CMAKE_CURRENT_LIST_DIR}/inc)

set_property(
TARGET nuttx
APPEND
PROPERTY NUTTX_INCLUDE_DIRECTORIES ${INCDIRS})

set(AUTOMID_CAN_DRIVER
    ${NUTTX_APPS_DIR}/vendor/infineon/chips/aurix
    ${NUTTX_APPS_DIR}/vendor/infineon/boards/aurix/tc4d9_evb/src
    ${NUTTX_DIR}/arch/tricore/src/common
    )
nuttx_add_kernel_library(can_filter_cfg)

set(SRCS_CAN_FILTER src/can_filter.c)
target_include_directories(can_filter_cfg PRIVATE ${AUTOMID_CAN_DRIVER})
target_sources(can_filter_cfg PRIVATE ${SRCS_CAN_FILTER})

endif()
