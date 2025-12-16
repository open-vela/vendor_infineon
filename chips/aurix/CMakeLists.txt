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

set(SRCS aurix_board.c)

if(CONFIG_AURIX_UART)
  list(APPEND SRCS aurix_uart.c)
endif()

if(CONFIG_AURIX_EGTM)
  list(APPEND SRCS aurix_egtm.c)
endif()

if(CONFIG_ARCH_CHIP_AURIX_TC3XX)
  list(APPEND SRCS aurix_gtm.c)
endif()

if(CONFIG_AURIX_GTM_CAPTURE)
  list(APPEND SRCS aurix_gtm_capture.c)
endif()

if(CONFIG_AURIX_EGTM_CAPTURE)
  list(APPEND SRCS aurix_egtm_capture.c)
endif()

if(CONFIG_AURIX_IOEXPANDER)
  list(APPEND SRCS aurix_ioexpander.c)
endif()

if(CONFIG_AURIX_I2C)
  list(APPEND SRCS aurix_i2c.c)
endif()

if(CONFIG_AURIX_LIN)
  list(APPEND SRCS aurix_lin.c)
endif()

if(CONFIG_AURIX_MCMCAN)
  if(CONFIG_AURIX_MCMCAN_CHARDRIVER)
    list(APPEND SRCS aurix_mcmcan.c)
  else()
    if(CONFIG_ARCH_CHIP_AURIX_TC3XX)
      list(APPEND SRCS aurix_mcmcan_net_tc3.c)
    else()
      list(APPEND SRCS aurix_mcmcan_net.c)
    endif()
  endif()
endif()

if(CONFIG_AURIX_EGTM_ATOM_TIMER)
  list(APPEND SRCS aurix_egtm_atom_timer.c)
endif()

if(CONFIG_AURIX_EGTM_TOM_TIMER)
  list(APPEND SRCS aurix_egtm_tom_timer.c)
endif()

if(CONFIG_AURIX_GTM_ATOM_TIMER)
  list(APPEND SRCS aurix_gtm_atom_timer.c)
endif()

if(CONFIG_AURIX_TMADC)
  list(APPEND SRCS aurix_tmadc.c)
endif()

if(CONFIG_AURIX_ENET)
  if(CONFIG_ARCH_CHIP_AURIX_TC3XX)
    list(APPEND SRCS aurix_enet_tc3.c)
  else()
    list(APPEND SRCS aurix_enet.c)
  endif()
endif()

if(CONFIG_AURIX_I2S)
  list(APPEND SRCS aurix_i2s.c)
endif()

if(CONFIG_AURIX_I2S_SIM)
  list(APPEND SRCS aurix_i2s_sim.c)
endif()

if(CONFIG_AURIX_EVADC)
  list(APPEND SRCS aurix_evadc.c)
endif()

if(CONFIG_ARCH_CHIP_AURIX_TC3XX)
  if(CONFIG_ARCH_CHIP_AURIX_TC39X)
    include(illd_tc397.cmake)
  endif()
  list(APPEND SRCS tc3xx.c)
else()
  if(CONFIG_ARCH_CHIP_AURIX_TC4DX)
    include(illd_tc4da.cmake)
  elseif(CONFIG_ARCH_CHIP_AURIX_TC48X)
    include(illd_tc48x.cmake)
  endif()
  list(APPEND SRCS tc4xx.c)
endif()

if(CONFIG_AURIX_GTM_TOM_PWM)
  list(APPEND SRCS aurix_gtm_pwm.c)
endif()

if(CONFIG_AURIX_GTM_ATOM_PWM)
  list(APPEND SRCS aurix_gtm_atom_pwm.c)
endif()

if(CONFIG_AURIX_EGTM_TOM_PWM)
  list(APPEND SRCS aurix_egtm_pwm.c)
endif()

if(CONFIG_AURIX_EGTM_ATOM_PWM)
  list(APPEND SRCS aurix_egtm_atom_pwm.c)
endif()

if(CONFIG_AURIX_SPI)
  list(APPEND SRCS aurix_spi.c)
endif()

if(CONFIG_AURIX_QSPI)
  if(CONFIG_ARCH_CHIP_AURIX_TC3XX)
    list(APPEND SRCS aurix_qspi_tc3.c)
  else()
    list(APPEND SRCS aurix_qspi.c)
  endif()
endif()

if(CONFIG_AURIX_QSPI_SLAVE)
  list(APPEND SRCS aurix_qspi_slave.c)
endif()

if(CONFIG_AURIX_MTD_FLASH)
  list(APPEND SRCS aurix_mtd_flash.c)
  list(APPEND SRCS aurix_mtd_partition.c)
endif()

if(CONFIG_AURIX_WDG)
  list(APPEND SRCS aurix_wdg.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS aurix_userspace.c)
endif()

if(CONFIG_AURIX_RPMSG)
  list(APPEND SRCS aurix_rpmsg.c)
endif()

if(CONFIG_AURIX_RNG)
  list(APPEND SRCS aurix_rng.c)
endif()

if(CONFIG_AURIX_UCB)
  list(APPEND SRCS aurix_ucb.c)
endif()

if(CONFIG_AURIX_PMS)
  list(APPEND SRCS aurix_pms.c)
endif()

if(CONFIG_AURIX_PMS_RTC)
  list(APPEND SRCS aurix_tc4dx_pms_rtc.c)
  list(APPEND SRCS aurix_tc4dx_pms_rtc_lowerhalf.c)
endif()

if(CONFIG_AURIX_SCR)
  list(APPEND SRCS aurix_scr.c)
  add_subdirectory(scr)
endif()

if(CONFIG_AURIX_QBVSCH)
  list(APPEND SRCS aurix_qbv_schedu.c)
endif()

if(CONFIG_AURIX_CRYPTO)
  list(APPEND SRCS aurix_crypto.c)
  list(APPEND SRCS aurix_crypto_css.c)
  if(CONFIG_AURIX_PKC)
    list(APPEND SRCS aurix_crypto_pkc.c)
  endif()
endif()

if(CONFIG_AURIX_HARDWARE_KEYMANAGEMENT)
  list(APPEND SRCS aurix_crypto_keym.c)
endif()

if(CONFIG_COREDUMP)
  list(APPEND SRCS aurix_coredump.c)
endif()

if(CONFIG_AURIX_MCAL)
  add_subdirectory(mcal)
  target_include_directories(arch PRIVATE ${CMAKE_CURRENT_LIST_DIR})
endif()

target_sources(arch PRIVATE ${SRCS})

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  nuttx_add_kernel_library(c_fpu)
  target_sources(c_fpu PRIVATE aurix_libc.c)
  target_link_options(
    nuttx PRIVATE
    --library-directory=${CMAKE_BINARY_DIR}/arch/tricore/src/exclude_chip)
  if(CONFIG_BUILD_PROTECTED)
    target_link_options(
      nuttx_user PRIVATE
      --library-directory=${CMAKE_BINARY_DIR}/arch/tricore/src/exclude_chip)
  endif()
else()
  nuttx_add_kernel_library(libos)
  target_sources(libos PRIVATE aurix_dummy.c)
endif()
