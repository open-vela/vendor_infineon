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

set(ILLD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/illd/tc4x/Libraries)
set(PATCHES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/illd/tc4x/patches)

file(GLOB PATCH_FILES "${PATCHES_DIR}/*.patch")

foreach(PATCH ${PATCH_FILES})
  execute_process(
    COMMAND git apply --reverse --check ${PATCH}
    WORKING_DIRECTORY ${ILLD_PATH}
    RESULT_VARIABLE PATCH_NEEDED
    OUTPUT_QUIET
    ERROR_QUIET
  )

  if(PATCH_NEEDED EQUAL 0)
    message(STATUS "Patch ${PATCH} already applied, skipping.")
  else()
    execute_process(
      COMMAND git apply --whitespace=nowarn ${PATCH}
      WORKING_DIRECTORY ${ILLD_PATH}
      RESULT_VARIABLE APPLY_RESULT
    )
    if(NOT APPLY_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to apply patch ${PATCH}!")
    else()
      message(STATUS "Successfully applied patch ${PATCH}.")
    endif()
  endif()
endforeach()

set(ILLD_DIR "illd/tc4x/Libraries/src")
set(CONFIG_DIR "illd/tc4x/Configurations")

set(SDK_INCDIR
    ${CMAKE_CURRENT_LIST_DIR}/${CONFIG_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/${CONFIG_DIR}/Configurations/Ssw
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/ArcEV
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap/TC4Dx
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Ap
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Ap/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Asclin/Asc
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Asclin/Lin
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Clock/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Atom
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Tom
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Fce/Crc
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Fce/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Src/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Port/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Scr
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap/TC4Dx
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Can/Can
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Can/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Cpu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Cpu/Trap
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Flash/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/I2c/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Pms/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Scu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Smu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Smm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Src/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Stm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Stm/Timer
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Vmt/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Wtu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Platform
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Sfr/TC4Dx
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC4xx/Tricore
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Service/CpuGeneric
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Platform/Compilers)

target_include_directories(arch PRIVATE ${SDK_INCDIR})
target_include_directories(nuttx PRIVATE ${SDK_INCDIR})

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_INCLUDE_DIRECTORIES ${SDK_INCDIR})

set(SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${CONFIG_DIR}/Ifx_Cfg_Ssw.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_Impl/IfxAp_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_Impl/IfxDma_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_Impl/IfxPort_cfg_TC4Dx.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap/TC4Dx/IfxPort_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Ap/Std/IfxApApu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Ap/Std/IfxApProt.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Clock/Std/IfxClock.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Dma/Dma/IfxDma_Dma.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Dma/Std/IfxDma.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Port/Std/IfxPort.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Src/Std/IfxSrc.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxCpu_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxStm_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Cpu/Std/IfxCpu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Cpu/Trap/IfxCpu_Trap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Scu/Std/IfxScuEru.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Stm/Std/IfxStm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Smm/Std/IfxSmm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Smm/Std/IfxSmmRst.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Smu/Std/IfxSmu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Vmt/Std/IfxVmt.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Wtu/Std/IfxWtu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Platform/Compilers/CompilerTasking.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC4xx/Tricore/Ifx_Ssw_Infra.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC4xx/Tricore/Ifx_Ssw_Tc${CONFIG_CPU_COREID}.c
)

if(CONFIG_AURIX_TMADC)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Adc/Std/IfxAdc.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Adc/Tmadc/IfxAdc_Tmadc.c
  )
endif()

if(CONFIG_AURIX_EGTM)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_Impl/IfxEgtm_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap/IfxEgtm_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap/TC4Dx/IfxEgtm_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Atom/Pwm/IfxEgtm_Atom_Pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Atom/Timer/IfxEgtm_Atom_Timer.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Tom/Pwm/IfxEgtm_Tom_Pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Tom/Timer/IfxEgtm_Tom_Timer.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm_Atom.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm_Cmu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm_Dtm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm_Tim.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Std/IfxEgtm_Tom.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Egtm/Tim/In/IfxEgtm_Tim_In.c
  )
endif()

if(CONFIG_AURIX_MCMCAN)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxCan_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap/TC4Dx/IfxCan_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Can/Std/IfxCan.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Can/Can/IfxCan_Can.c
  )
endif()

if(CONFIG_AUTOMID_GATEWAY_DRE)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Dre/Std/IfxDre.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Dre/Dre/IfxDre_Dre.c
  )
endif()

if(CONFIG_AURIX_PMS OR CONFIG_AURIX_SCR)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Pms/Std/IfxPmsEvr.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Pms/Std/IfxPmsPm.c
  )
endif()

if(CONFIG_AURIX_LIN
   OR CONFIG_AURIX_SPI
   OR CONFIG_AURIX_UART)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_Impl/IfxAsclin_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/_PinMap/TC4Dx/IfxAsclin_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Asclin/Std/IfxAsclin.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Asclin/Lin/IfxAsclin_Lin.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/CpuGeneric/Asclin/Spi/IfxAsclin_Spi.c
  )
endif()

if(CONFIG_AURIX_QSPI OR CONFIG_AURIX_I2S_SIM)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxQspi_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap/TC4Dx/IfxQspi_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Qspi/SpiMaster/IfxQspi_SpiMaster.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Qspi/SpiSlave/IfxQspi_SpiSlave.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Qspi/Std/IfxQspi.c
  )
endif()

if(CONFIG_AURIX_SENT)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap/IfxSent_PinMap_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Sent/Sent/IfxSent_Sent.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Sent/Std/IfxSent.c
  )
endif()

if(CONFIG_AURIX_I2C)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxI2c_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_PinMap/TC4Dx/IfxI2c_PinMap_TC4Dx_BGA436_COM.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/I2c/I2c/IfxI2c_I2c.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/I2c/Std/IfxI2c.c
  )
endif()

if(CONFIG_AURIX_UCB OR CONFIG_AURIX_MTD_FLASH)
  list(
    APPEND
    SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/_Impl/IfxFlash_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC4xx/Tricore/Flash/Std/IfxFlash.c
  )
endif()

target_sources(nuttx PRIVATE ${SDK_CSRCS})

if(NOT CONFIG_TRICORE_TOOLCHAIN_TASKING)
  target_compile_options(nuttx PRIVATE -Wno-undef -Wno-strict-prototypes
                                       -Wno-shadow)
  target_compile_options(nuttx PRIVATE -fstrict-volatile-bitfields)
  target_compile_options(nuttx PRIVATE -Wno-unused-parameter
                                       -Wno-unused-but-set-parameter)
  target_compile_options(nuttx PRIVATE -Wno-implicit-fallthrough)
else()
  nuttx_add_extra_library(${CMAKE_CURRENT_LIST_DIR}/illd/prebuilts/tc18/cinit.o)
endif()

target_compile_definitions(
  arch PRIVATE -DSRC_CPU_CPU0_SB=SRC_CPU0SB
               -DIfx_CPU_SYSCON=Ifx_CPU_CORECON
               -DIFX_CPU_TR_EVT_ALD_OFF=IFX_CPU_TREVT_ALD_OFF
               -DIFX_CPU_TR_EVT_AST_OFF=IFX_CPU_TREVT_AST_OFF
               -DIFX_CPU_TR_EVT_BBM_OFF=IFX_CPU_TREVT_BBM_OFF
               -DIFX_CPU_TR_EVT_EN_MSK=IFX_CPU_TREVT_EN_MSK
               -DIFX_CPU_TR_EVT_TYP_OFF=IFX_CPU_TREVT_TYP_OFF
               -DCPU_SYSCON=CPU_CORECON)
