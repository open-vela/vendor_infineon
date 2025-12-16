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

set(ILLD_DIR "illd/tc397")

set(SDK_INCDIR
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Configurations
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Configurations/Ssw
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Asclin/Lin
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Can/Can
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Can/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Cpu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Cpu/Trap
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Geth/Eth
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Geth/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Atom
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Evadc/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/I2c/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Port/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Pms/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Stm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Smm/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Src/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Stm/Timer
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Qspi/SpiMaster
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Qspi/Std
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Platform
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Sfr/TC39B/_Reg
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC39B/Tricore
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Phy_Mvlq1110
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Service/CpuGeneric)

target_include_directories(arch PRIVATE ${SDK_INCDIR})
target_include_directories(nuttx PRIVATE ${SDK_INCDIR})

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_INCLUDE_DIRECTORIES ${SDK_INCDIR})

set(SDK_CSRCS
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Configurations/Ifx_Cfg_Ssw.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Service/CpuGeneric/If/SpiIf.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Dma/Dma/IfxDma_Dma.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Asclin/Spi/IfxAsclin_Spi.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxAsclin_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxCan_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxCif_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxCpu_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxGtm_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxGtm_cfg.h
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxQspi_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxEvadc_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxGeth_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxI2c_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxPort_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_Impl/IfxStm_cfg.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxAsclin_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxGeth_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxGtm_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxI2c_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxPort_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxQspi_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxCan_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/_PinMap/IfxPort_PinMap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Asclin/Lin/IfxAsclin_Lin.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Asclin/Std/IfxAsclin.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Can/Can/IfxCan_Can.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Can/Std/IfxCan.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Cpu/Std/IfxCpu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Cpu/Trap/IfxCpu_Trap.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Evadc/Adc/IfxEvadc_Adc.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Evadc/Std/IfxEvadc.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Geth/Eth/IfxGeth_Eth.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Geth/Std/IfxGeth.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Atom/Pwm/IfxGtm_Atom_Pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Atom/Timer/IfxGtm_Atom_Timer.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Dpll.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Tom.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Atom.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Tim.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Tbu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Std/IfxGtm_Cmu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/I2c/I2c/IfxI2c_I2c.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/I2c/Std/IfxI2c.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Pms/Std/IfxPmsEvr.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Pms/Std/IfxPmsPm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Port/Std/IfxPort.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Qspi/Std/IfxQspi.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Qspi/SpiMaster/IfxQspi_SpiMaster.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Qspi/SpiSlave/IfxQspi_SpiSlave.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std/IfxScuCcu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std/IfxScuEru.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std/IfxScuLbist.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std/IfxScuRcu.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Scu/Std/IfxScuWdt.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Src/Std/IfxSrc.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/iLLD/TC39B/Tricore/Stm/Std/IfxStm.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC39B/Tricore/Ifx_Ssw_Infra.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Infra/Ssw/TC39B/Tricore/Ifx_Ssw_Tc${CONFIG_CPU_COREID}.c
    ${CMAKE_CURRENT_LIST_DIR}/${ILLD_DIR}/Libraries/Phy_Mvlq1110/IfxGeth_Phy_Mvlq1110.c
)

target_sources(nuttx PRIVATE ${SDK_CSRCS})

if(NOT CONFIG_TRICORE_TOOLCHAIN_TASKING)
  target_compile_options(nuttx PRIVATE -Wno-undef -Wno-strict-prototypes
                                       -Wno-shadow)
  target_compile_options(nuttx PRIVATE -fstrict-volatile-bitfields)
  target_compile_options(nuttx PRIVATE -Wno-unused-parameter
                                       -Wno-unused-but-set-parameter)
  target_compile_options(nuttx PRIVATE -Wno-implicit-fallthrough)
else()
  nuttx_add_extra_library(
    ${CMAKE_CURRENT_LIST_DIR}/illd/prebuilts/tc162/cinit.o)
endif()

target_compile_definitions(arch PRIVATE -DSRC_GPSR0_SR0=SRC_GPSR00)
