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

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  set(toolchain "Tasking")
else()
  set(toolchain "Gnuc")
endif()

if(CONFIG_TRICORE_BL)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/Lcf_${toolchain}_Tricore_Tc.lsl.bl")
elseif(CONFIG_TRICORE_CSRM)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/Lcf_${toolchain}_Tricore_Tc.lsl.corecs")
else()
  set_property(
    GLOBAL PROPERTY LD_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/Lcf_${toolchain}_Tricore_Tc.lsl.kernel")
  if(CONFIG_BUILD_PROTECTED)
    set_property(
      GLOBAL PROPERTY LD_SCRIPT_USER
      "${CMAKE_CURRENT_SOURCE_DIR}/Lcf_${toolchain}_Tricore_Tc.lsl.user")
  endif()
endif()

add_custom_target(
  nuttx_post_build
  DEPENDS nuttx
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_command(
  TARGET nuttx_post_build
  DEPENDS nuttx
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND cp nuttx vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}.elf
  COMMAND cp nuttx.map vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}.mapfile
  COMMAND cp System.map vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}.SystemMap)

set(HEX_COMMAND_ARGS
    nuttx.hex -intel -crop 0x80000000 0x81000000
    nuttx.hex -intel -crop 0xA0000000 0xA1000000)

if(CONFIG_BUILD_PROTECTED)
  add_custom_command(
    TARGET nuttx_post_build
    DEPENDS nuttx_user
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp nuttx_user vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}_user.elf
    COMMAND cp nuttx_user.map vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}_user.mapfile
    COMMAND cp nuttx_user.hex vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}_user.hex
    COMMAND cp User.map vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}_user.SystemMap)

  list(APPEND HEX_COMMAND_ARGS
       nuttx_user.hex -intel -crop 0x80000000 0x81000000
       nuttx_user.hex -intel -crop 0xA0000000 0xA1000000)
endif()

add_custom_command(
  TARGET nuttx_post_build
  DEPENDS nuttx
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND srec_cat ${HEX_COMMAND_ARGS}
  -o vela_${CONFIG_ARCH_BOARD_CUSTOM_NAME}.hex -intel)
