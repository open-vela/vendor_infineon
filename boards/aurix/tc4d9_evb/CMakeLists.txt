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

add_subdirectory(src)

if(CONFIG_BUILD_PROTECTED)
  add_subdirectory(kernel)
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/tc4dx" EXCLUDE_FROM_ALL
                 scripts)

file(GLOB EXTRA_LIBS
     ${NUTTX_BOARD_ABS_DIR}/libs/${CONFIG_ARCH_BOARD_CUSTOM_NAME}/*.a)

set_property(GLOBAL APPEND PROPERTY NUTTX_EXTRA_LIBRARIES ${EXTRA_LIBS})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../configurations/CMakeLists.txt)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../configurations" EXCLUDE_FROM_ALL
                   exclude_common_code)
endif()
