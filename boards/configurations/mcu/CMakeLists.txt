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

if(CONFIG_GPIO_LOWER_HALF OR CONFIG_AURIX_CAPTURE
   OR CONFIG_AURIX_PWM OR CONFIG_AURIX_I2C
   OR CONFIG_AURIX_QSPI OR CONFIG_AURIX_TMADC OR CONFIG_AURIX_EVADC OR CONFIG_AURIX_UART0)
set(INCDIRS ${CMAKE_CURRENT_LIST_DIR}/inc)

set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_INCLUDE_DIRECTORIES ${INCDIRS})
set(INCDIR ${NUTTX_APPS_DIR}/vendor/infineon/chips/aurix/
           ${NUTTX_DIR}/arch/tricore/src/common/)


if(CONFIG_GPIO_LOWER_HALF)
  nuttx_add_kernel_library(zcu_aurix_gpio_cfg)
  target_sources(zcu_aurix_gpio_cfg PRIVATE src/aurix_gpio_cfg.c)
  target_include_directories(zcu_aurix_gpio_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_CAPTURE)
  nuttx_add_kernel_library(zcu_aurix_capture_cfg)
  target_sources(zcu_aurix_capture_cfg PRIVATE src/aurix_capture_cfg.c)
  target_include_directories(zcu_aurix_capture_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_PWM)
  nuttx_add_kernel_library(zcu_aurix_pwm_cfg)
  target_sources(zcu_aurix_pwm_cfg PRIVATE src/aurix_pwm_cfg.c)
  target_include_directories(zcu_aurix_pwm_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_I2C)
  nuttx_add_kernel_library(zcu_aurix_i2c_cfg)
  target_sources(zcu_aurix_i2c_cfg PRIVATE src/aurix_i2c_cfg.c)
  target_include_directories(zcu_aurix_i2c_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_QSPI)
  nuttx_add_kernel_library(zcu_aurix_qspi_cfg)
  target_sources(zcu_aurix_qspi_cfg PRIVATE src/aurix_qspi_cfg.c)
  target_include_directories(zcu_aurix_qspi_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_TMADC)
  nuttx_add_kernel_library(zcu_aurix_tmadc_cfg)
  target_sources(zcu_aurix_tmadc_cfg PRIVATE src/aurix_tmadc_cfg.c)
  target_include_directories(zcu_aurix_tmadc_cfg PRIVATE ${INCDIR})
endif()
if(CONFIG_AURIX_EVADC)
  nuttx_add_kernel_library(zcu_aurix_evadc_cfg)
  target_sources(zcu_aurix_evadc_cfg PRIVATE src/aurix_evadc_cfg.c)
  target_include_directories(zcu_aurix_evadc_cfg PRIVATE ${INCDIR})
endif()
if(NOT CONFIG_ARCH_CHIP_AURIX_TC39X)
  if(CONFIG_AURIX_UART0)
    nuttx_add_kernel_library(zcu_aurix_uart_cfg)
    target_sources(zcu_aurix_uart_cfg PRIVATE src/aurix_uart_cfg.c)
    target_include_directories(zcu_aurix_uart_cfg PRIVATE ${INCDIR})
  endif()
endif()
endif()
