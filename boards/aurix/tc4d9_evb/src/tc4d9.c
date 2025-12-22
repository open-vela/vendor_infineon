/****************************************************************************
 *  Copyright (C) 2025 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/fs/fs.h>
#include <nuttx/board.h>
#include <arch/chip/chip.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/irq.h>
#include <nuttx/note/note_driver.h>
#include <nuttx/note/noteram_driver.h>
#include <nuttx/userspace.h>
#include <nuttx/kthread.h>

#include <sys/types.h>
#include <sys/param.h>
#include <syslog.h>

#include "Can/Std/IfxCan.h"
#include "Vmt/Std/IfxVmt.h"
#include "tc4d9.h"
#include "tricore_internal.h"
#include "IfxInt_reg.h"
#include "aurix_tmadc.h"
#include "aurix_i2c.h"
#include "aurix_uart.h"
#include "aurix_ioexpander.h"
#include "aurix_egtm.h"
#include "aurix_egtm_atom_pwm.h"
#include "aurix_egtm_capture.h"
#include "aurix_egtm_pwm.h"
#include "aurix_spi.h"
#include "aurix_qspi.h"
#include "aurix_qspi_slave.h"

#ifdef CONFIG_AUTOCORE_SYNBARRIER
#include "os_api.h"
#endif

#ifdef CONFIG_AURIX_QBVSCH
#include "aurix_qbv_schedu.h"
#endif

#ifdef CONFIG_AURIX_EGTM_ATOM_TIMER
#include "aurix_egtm_atom_timer.h"
#endif

#ifdef CONFIG_AURIX_EGTM_TOM_TIMER
#include "aurix_egtm_tom_timer.h"
#endif

#ifdef CONFIG_AURIX_UART
#include "aurix_uart_cfg.h"
#endif

#ifdef CONFIG_AURIX_MCMCAN
#include "aurix_mcmcan.h"
#include "can_mram.h"
#endif

#ifdef CONFIG_AURIX_PWM
#include "aurix_pwm_cfg.h"
#endif

#ifdef CONFIG_GPIO_LOWER_HALF
#include "aurix_gpio_cfg.h"
#endif

#ifdef CONFIG_AURIX_TMADC
#include "aurix_tmadc_cfg.h"
#endif

#ifdef CONFIG_AURIX_EGTM_CAPTURE
#include "aurix_capture_cfg.h"
#endif

#include "aurix_lin.h"

#ifdef CONFIG_AURIX_MTD_FLASH
#include "aurix_mtd_partition.h"
#include "aurix_mtd_flash.h"
#endif

#ifdef CONFIG_AMP_TAS6754
#include <nuttx/audio/audio_comp.h>
#include "tas6754.h"
#endif

#include <IfxApApu.h>

#ifdef CONFIG_AURIX_PMS
#  include <IfxPms_reg.h>
#  include <IfxSmm.h>
#endif

#ifdef CONFIG_AURIX_SBC_TLF4D985
  #include "aurix_tlf4d985q.h"
#endif

#ifdef CONFIG_AURIX_SAFETY
  #include "aurix_safety.h"
#endif

#if defined(CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 0)
#include "IfxCpu_reg.h"
#include "IfxInt_reg.h"
#endif

#include "memory_layout.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_IRQ_GET(SRC_ADDR) (((uintptr_t)&SRC_ADDR - (uintptr_t)&SRC_CPU0_SB) / 4)

/* MCMCAN Start */

/* fill device node number and module sram index, map interrupt */

#define MCMCAN_CONF_MAP(DEV_INDEX, M_ID, TX_NUM, RX_NUM,                 \
                        ERR_NUM, CRE_NUM)                                \
  .intf               = DEV_INDEX,                                       \
  .module_sram_index  = IfxVmt_MbistSel_mcan##M_ID,                      \
  .tx_irq             = TRICORE_IRQ_GET(SRC_CAN##M_ID##_INT##TX_NUM),    \
  .rx_irq             = TRICORE_IRQ_GET(SRC_CAN##M_ID##_INT##RX_NUM),    \
  .err_irq            = TRICORE_IRQ_GET(SRC_CAN##M_ID##_INT##ERR_NUM),   \
  .cre_irq            = TRICORE_IRQ_GET(SRC_CAN##M_ID##_INT##CRE_NUM),   \
  .tx_interrupt_line  = IfxCan_InterruptLine_##TX_NUM,                   \
  .rx_interrupt_line  = IfxCan_InterruptLine_##RX_NUM,                   \
  .err_interrupt_line = IfxCan_InterruptLine_##ERR_NUM,                  \
  .cre_interrupt_line = IfxCan_InterruptLine_##CRE_NUM,

/* setup transceiver stb pin */

#define MCMCAN_NODE_STB_SETUP(STBN_PORT, STBN_PORT_INDEX)                \
.can_transv_pin =                                                        \
  {                                                                      \
    .port     = STBN_PORT,                                               \
    .pinIndex = STBN_PORT_INDEX,                                         \
  },

/* setup some config about the specfic CAN Node */

#define DECLARE_MCMCAN_NODE_CONFIG(M_ID, N_ID, DEV_INDEX)                \
.can             = &MODULE_CAN##M_ID,                                    \
.nodeId          = IfxCan_NodeId_##N_ID,                                 \
.baudRate        =                                                       \
  {                                                                      \
    .baudrate    = CONFIG_AURIX_MCMCAN##DEV_INDEX##_BAUDRATE,            \
    .samplePoint = CONFIG_AURIX_MCMCAN##DEV_INDEX##_SAMPLEPOINT,         \
  },                                                                     \
.fastBaudRate    =                                                       \
  {                                                                      \
    .baudrate    = CONFIG_AURIX_MCMCAN##DEV_INDEX##_FAST_BAUDRATE,       \
    .samplePoint = CONFIG_AURIX_MCMCAN##DEV_INDEX##_FAST_SAMPLEPOINT,    \
  },                                                                     \
.txConfig.dedicatedTxBuffersNumber =                                     \
                   CONFIG_CAN##M_ID##_NODE##N_ID##_DED_TX_BUF_NUM,       \
.txConfig.txFifoQueueSize          =                                     \
                   CONFIG_CAN##M_ID##_NODE##N_ID##_TX_FIFO_Q_NUM,        \
.rxConfig.rxFifo0Size              =                                     \
                   CONFIG_CAN##M_ID##_NODE##N_ID##_RF0_NUM,              \
.rxConfig.rxFifo1Size              =                                     \
                   CONFIG_CAN##M_ID##_NODE##N_ID##_RF1_NUM,              \
.filterConfig.standardListSize     =                                     \
                   CONFIG_CAN##M_ID##_NODE##N_ID##_SFL_NUM,              \

/* the specfic MCMCANX(0, 1, 2, 3, 5) RAM region allocate */

#define DECLARE_MCMCAN_MES_RAM(M_ID, N_ID)                               \
.messageRAM =                                                            \
  {                                                                      \
    .baseAddress                    = (uint32)                           \
                                      &MODULE_CAN##M_ID##_RAM,           \
    .standardFilterListStartAddress = CAN##M_ID##_NODE##N_ID##_FLSSA,    \
    .rxFifo0StartAddress            = CAN##M_ID##_NODE##N_ID##_F0SA,     \
    .rxFifo1StartAddress            = CAN##M_ID##_NODE##N_ID##_F1SA,     \
    .rxBuffersStartAddress          = CAN##M_ID##_NODE##N_ID##_RX_BUFSA, \
    .txEventFifoStartAddress        = CAN##M_ID##_NODE##N_ID##_EFSA,     \
    .txBuffersStartAddress          = CAN##M_ID##_NODE##N_ID##_TX_BUFSA, \
  },

#define DECLARE_MCMCAN_CRE_RAM(M_ID, N_ID)                               \
.can_node_cre_config =                                                   \
  {                                                                      \
    .creStartAddress                  = CAN##M_ID##_NODE##N_ID##_CRE_SA, \
    .stdRoutingTableStartAddress      =                                  \
                              CAN##M_ID##_NODE##N_ID##_CRE_STD_RT_SA,    \
    .stdFrameRateTableStartAddress    =                                  \
                              CAN##M_ID##_NODE##N_ID##_CRE_STD_FRT_SA,   \
    .stdTimeStampDatabaseStartAddress =                                  \
                              CAN##M_ID##_NODE##N_ID##_CRE_STD_TSD_SA,   \
    .stdRoutingRuleSize               =                                  \
                              CONFIG_CAN##M_ID##_NODE##N_ID##_STD_RT_NUM,\
  },

#ifdef CONFIG_AURIX_MCMCAN_CRE
#  define DECLARE_MCMCAN_MERAM_CONFIG(M_ID, N_ID)                        \
.rxbuf_filter_id                      =                                  \
                          CAN##M_ID##_NODE##N_ID##_RXBUF_FILT,           \
.rxbuf_filter_cnt                     =                                  \
                          CONFIG_CAN##M_ID##_NODE##N_ID##_RXBUF_NUM,     \
.rxfifo0_filter                       =                                  \
                          CAN##M_ID##_NODE##N_ID##_RF0_FILT,             \
.rxfifo0_filter_cnt                   =                                  \
                          CONFIG_CAN##M_ID##_NODE##N_ID##_RF0_NUM,       \
.rxfifo1_filter                       =                                  \
                          CAN##M_ID##_NODE##N_ID##_RF1_FILT,             \
.rxfifo1_filter_cnt                   =                                  \
                          CONFIG_CAN##M_ID##_NODE##N_ID##_RF1_NUM,       \
.uni_routing                          =                                  \
                          CAN##M_ID##_NODE##N_ID##_UNI_ROUTING,          \
.uni_routing_cnt                      =                                  \
                          CAN##M_ID##_NODE##N_ID##_UNICAST_ROUTER_NUM,   \
.mul_routing                          =                                  \
                          CAN##M_ID##_NODE##N_ID##_MUL_ROUTING,          \
.mul_routing_cnt                      =                                  \
                          CAN##M_ID##_NODE##N_ID##_MULTICAST_ROUTER_NUM,
#else
#  define DECLARE_MCMCAN_MERAM_CONFIG(M_ID, N_ID)                        \
.rxbuf_filter_id                      = NULL,                            \
.rxbuf_filter_cnt                     = 0,                               \
.rxfifo0_filter                       = NULL,                            \
.rxfifo0_filter_cnt                   = 0,                               \
.rxfifo1_filter                       = NULL,                            \
.rxfifo1_filter_cnt                   = 0,
#endif
/* setup TX/RX pins */

#define DECLARE_MCMCAN_PIN_SETUP(TX_PIN_NUMBER,RX_PIN_NUMBER)            \
.pins =                                                                  \
&(IfxCan_Can_Pins)                                                       \
  {                                                                      \
    .txPin     = TX_PIN_NUMBER,                                          \
    .txPinMode = IfxPort_OutputMode_pushPull,                            \
    .rxPin     = RX_PIN_NUMBER,                                          \
    .rxPinMode = IfxPort_InputMode_noPullDevice,                         \
    .padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed2                  \
  },

/* MCMCAN End */

/* LIN Start */

/* LIN End */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_TIMER
static const struct aurix_egtm_atom_timer_config_s
g_aurix_egtm_atom0_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_0,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_1,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_2,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_3,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_4,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_5,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_6,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM0_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Atom_Ch_7,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM0_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer0_7",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_ATOM_ATOM0_TIMER */

#ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_TIMER
static const struct aurix_egtm_atom_timer_config_s
g_aurix_egtm_atom1_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_0,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_1,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_2,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_3,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_4,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_5,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_6,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM1_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Atom_Ch_7,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM1_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer1_7",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_ATOM_ATOM1_TIMER */

#ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_TIMER
static const struct aurix_egtm_atom_timer_config_s
g_aurix_egtm_atom2_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_0,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_1,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_2,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_3,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_4,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_5,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_6,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_ATOM_ATOM2_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Atom_Ch_7,
    .clock                   = IfxEgtm_Atom_Ch_ClkSrc_cmuclk0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_ATOM2_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/timer2_7",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_ATOM_ATOM2_TIMER */

#ifdef CONFIG_AURIX_EGTM_TOM_TOM0_TIMER
static const struct aurix_egtm_tom_timer_config_s
g_aurix_egtm_tom0_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_0,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_1,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_2,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_3,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_4,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_5,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_6,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_7,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_7",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH8_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_8,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_8",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH9_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_9,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_9",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH10_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_10,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_10",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH11_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_11,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_11",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH12_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_12,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_12",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH13_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_13,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_13",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH14_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_14,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_14",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM0_CH15_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_0,
    .channel                 = IfxEgtm_Tom_Ch_15,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM0_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer0_15",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_TOM_TOM0_TIMER */

#ifdef CONFIG_AURIX_EGTM_TOM_TOM1_TIMER
static const struct aurix_egtm_tom_timer_config_s
g_aurix_egtm_tom1_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_0,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_1,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_2,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_3,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_4,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_5,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_6,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_7,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_7",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH8_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_8,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_8",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH9_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_9,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_9",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH10_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_10,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_10",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH11_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_11,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_11",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH12_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_12,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_12",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH13_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_13,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_13",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH14_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_14,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_14",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM1_CH15_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_1,
    .channel                 = IfxEgtm_Tom_Ch_15,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM1_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer1_15",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_TOM_TOM1_TIMER */

#ifdef CONFIG_AURIX_EGTM_TOM_TOM2_TIMER
static const struct aurix_egtm_tom_timer_config_s
g_aurix_egtm_tom2_timer_config[] =
{
  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH0_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_0,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_0",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH1_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_1,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR0),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_1",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH2_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_2,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_2",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH3_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_3,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR1),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_3",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH4_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_4,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_4",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH5_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_5,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR2),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_5",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH6_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_6,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_6",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH7_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_7,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR3),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_7",
    .oneshotmode             = FALSE,
  },
  #endif

#ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH8_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_8,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_8",
    .oneshotmode             = FALSE,
  },
  #endif

#ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH9_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_9,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR4),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_9",
    .oneshotmode             = FALSE,
  },
  #endif

#ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH10_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_10,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_10",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH11_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_11,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR5),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_11",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH12_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_12,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_12",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH13_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_13,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR6),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_13",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH14_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_14,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_14",
    .oneshotmode             = FALSE,
  },
  #endif

  #ifdef CONFIG_AURIX_EGTM_TOM_TOM2_CH15_TIMER
  {
    .cluster                 = IfxEgtm_Cluster_2,
    .channel                 = IfxEgtm_Tom_Ch_15,
    .clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk2,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_EGTM_TOM2_SR7),
    .interrupt.irqMode       = IfxEgtm_IrqMode_pulseNotify,
    .frequency               = 100.0f,
    .devpath                 = "/dev/tom_timer2_15",
    .oneshotmode             = FALSE,
  },
  #endif
};
#endif/* CONFIG_AURIX_EGTM_TOM_TOM2_TIMER */

/* Global MCMCAN configuration and control structure */

#ifdef CONFIG_AURIX_MCMCAN
static struct aurix_mcmcan_config_s g_aurix_mcmcan_config[] =
{
#ifdef CONFIG_AURIX_MCMCAN0
  {
    MCMCAN_CONF_MAP(0, 0, 0, 1, 2, 3)

    MCMCAN_NODE_STB_SETUP(&MODULE_P00, 3)

    .can_node_config =
      {
        DECLARE_MCMCAN_NODE_CONFIG(0, 0, 0)

        DECLARE_MCMCAN_MES_RAM(0, 0)

        DECLARE_MCMCAN_PIN_SETUP(&IfxCan_TXD00_P33_8_OUT,
                                 &IfxCan_RXD00E_P33_7_IN)
      },

#ifdef CONFIG_AURIX_MCMCAN_CRE
    DECLARE_MCMCAN_CRE_RAM(0, 0)
#endif
#ifdef CONFIG_AURIX_MCMCAN_MERAM_CONFIG
    DECLARE_MCMCAN_MERAM_CONFIG(0, 0)
#endif
  },
#endif

#ifdef CONFIG_AURIX_MCMCAN1
  {
    MCMCAN_CONF_MAP(1, 0, 4, 5, 6, 7)

    MCMCAN_NODE_STB_SETUP(&MODULE_P00, 2)
    .can_node_config =
      {
        DECLARE_MCMCAN_NODE_CONFIG(0, 1, 1)

        DECLARE_MCMCAN_MES_RAM(0, 1)

        /* No correct */

        DECLARE_MCMCAN_PIN_SETUP(&IfxCan_TXD02_P14_10_OUT,
                                 &IfxCan_RXD02D_P14_8_IN)
      },

#ifdef CONFIG_AURIX_MCMCAN_CRE
      DECLARE_MCMCAN_CRE_RAM(0, 1)
#endif
  },
#endif

#ifdef CONFIG_AURIX_MCMCAN2
  {
    MCMCAN_CONF_MAP(2, 0, 8, 9, 10, 11)

    MCMCAN_NODE_STB_SETUP(&MODULE_P00, 2)
    .can_node_config =
      {
        DECLARE_MCMCAN_NODE_CONFIG(0, 2, 2)

        DECLARE_MCMCAN_MES_RAM(0, 2)

        DECLARE_MCMCAN_PIN_SETUP(&IfxCan_TXD02_P14_10_OUT,
                                 &IfxCan_RXD02D_P14_8_IN)
      },

#ifdef CONFIG_AURIX_MCMCAN_CRE
      DECLARE_MCMCAN_CRE_RAM(0, 2)
#endif
#ifdef CONFIG_AURIX_MCMCAN_MERAM_CONFIG
      DECLARE_MCMCAN_MERAM_CONFIG(0, 2)
#endif
  },
#endif
};
#endif

/* Global Lin configuration and control structure */

#ifdef CONFIG_AURIX_LIN
static const struct aurix_lin_config_s g_aurix_lin_config[] =
{
#ifdef CONFIG_AURIX_LIN4
  {
    .asclin = &MODULE_ASCLIN4,
    .port   = 4,
    .tx_irq = 184,      /* IRQ_GET(SRC_ASCLIN4_TX) LIN4 transmit interrupt */
    .rx_irq = 185,      /* IRQ_GET(SRC_ASCLIN4_RX) LIN4 receive interrupt */
    .ex_irq = 186,      /* IRQ_GET(SRC_ASCLIN4_EX) LIN4 error interrupt */
#ifdef CONFIG_AURIX_LIN4_MASTER
    .master = true,
#endif
    .transceiver =
    {
      .port     = &MODULE_P00,
      .pinIndex = 7,
    },
    .pins =
      {
        /* RX port pin */

        &IfxAsclin4_RXA_P00_12_IN, IfxPort_InputMode_pullUp,

        /* TX port pin */

        &IfxAsclin4_TX_P00_9_OUT, IfxPort_OutputMode_pushPull,

        IfxPort_PadDriver_cmosAutomotiveSpeed1
      },
  },
#endif
#ifdef CONFIG_AURIX_LIN2
  {
    .asclin = &MODULE_ASCLIN2,
    .port   = 2,
    .tx_irq = 178,      /* IRQ_GET(SRC_ASCLIN2_TX) LIN2 transmit interrupt */
    .rx_irq = 179,      /* IRQ_GET(SRC_ASCLIN2_RX) LIN2 receive interrupt */
    .ex_irq = 180,      /* IRQ_GET(SRC_ASCLIN2_EX) LIN2 error interrupt */
#ifdef CONFIG_AURIX_LIN2_MASTER
    .master = true,
#endif
    .transceiver =
    {
      .port     = &MODULE_P14,
      .pinIndex = 6,
    },
    .pins =
      {
        /* RX port pin */

        &IfxAsclin2_RXC_P02_10_IN, IfxPort_InputMode_pullUp,

        /* TX port pin */

        &IfxAsclin2_TX_P32_5_OUT, IfxPort_OutputMode_pushPull,

        IfxPort_PadDriver_cmosAutomotiveSpeed1
      },
  },
#endif
};
#endif

#if defined(CONFIG_AURIX_SPI)
static struct aurix_spi_config_s g_aurix_spi_config[] =
{
#ifdef CONFIG_AURIX_SPI0
  [0] =
    {
    .asclin            = &MODULE_ASCLIN1,
    .slso_0            = &IfxAsclin1_SLSO_P14_3_OUT,
    .slso_1            = &IfxAsclin1_SLSO_P20_8_OUT,
    .slso_2            = &IfxAsclin1_SLSO_P33_10_OUT,
    .slso_mode         = IfxPort_OutputMode_pushPull,
    .pinDriver         = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    .tx_irq            = 175,
    .rx_irq            = 176,
    .err_irq           = 177,
    .pins              =
    {
      .sclk            = &IfxAsclin1_SCLK_P20_10_OUT,
      .sclkMode        = IfxPort_OutputMode_pushPull,
      .rx              = &IfxAsclin1_RXB_F_P15_5_IN,
      .rxMode          = IfxPort_InputMode_pullUp,
      .tx              = &IfxAsclin1_TX_F_P15_4_OUT,
      .txMode          = IfxPort_OutputMode_pushPull,
      .pinDriver       = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
    .vm_id             = IfxSrc_VmId_3,
  }
#endif
};
#endif // CONFIG_AURIX_SPI

#ifdef CONFIG_AURIX_MTD_PFLASH
static const struct partition_s g_aurix_pflash_table[] =
{
  {
    .firstblock  = 0,
    .nblocks     = BANKA_BL_PFLASH_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/bl_a",
  },

#ifdef BANKA_CORE0_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE0_PFLASH_USER_START,
                    BANKA_CORE0_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE0_PFLASH_USER_SIZE +
                    BANKA_CORE0_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core0_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE0_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE0_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core0_a",
  },
#endif

#ifdef BANKA_CORE1_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE1_PFLASH_USER_START,
                    BANKA_CORE1_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE1_PFLASH_USER_SIZE +
                    BANKA_CORE1_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core1_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE1_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE1_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core1_a",
  },
#endif

#ifdef BANKA_CORE2_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE2_PFLASH_USER_START,
                    BANKA_CORE2_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE2_PFLASH_USER_SIZE +
                    BANKA_CORE2_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core2_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE2_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE2_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core2_a",
  },
#endif

#ifdef BANKA_CORE3_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE3_PFLASH_USER_START,
                    BANKA_CORE3_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE3_PFLASH_USER_SIZE +
                    BANKA_CORE3_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core3_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE3_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE3_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core3_a",
  },
#endif

#ifdef BANKA_CORE4_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE4_PFLASH_USER_START,
                    BANKA_CORE4_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE4_PFLASH_USER_SIZE +
                    BANKA_CORE4_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core4_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE4_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE4_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core4_a",
  },
#endif

#ifdef BANKA_CORE5_PFLASH_USER_START
  {
    .firstblock  = (MIN(BANKA_CORE5_PFLASH_USER_START,
                    BANKA_CORE5_PFLASH_KERNEL_START) -
                    BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKA_CORE5_PFLASH_USER_SIZE +
                    BANKA_CORE5_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core5_a",
  },

#else
  {
    .firstblock  = (BANKA_CORE5_PFLASH_KERNEL_START - BANKA_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = BANKA_CORE5_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core5_a",
  },
#endif

  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB / PFLASH_PAGE_SIZE),
    .nblocks     = BANKB_BL_PFLASH_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/bl_b",
  },

#ifdef BANKB_CORE0_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE0_PFLASH_USER_START,
                    BANKB_CORE0_PFLASH_KERNEL_START) -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE0_PFLASH_USER_SIZE +
                    BANKB_CORE0_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core0_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE0_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE0_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core0_b",
  },
#endif

#ifdef BANKB_CORE1_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE1_PFLASH_USER_START,
                    BANKB_CORE1_PFLASH_KERNEL_START) -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE1_PFLASH_USER_SIZE +
                    BANKB_CORE1_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core1_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE1_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE1_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core1_b",
  },
#endif

#ifdef BANKB_CORE2_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE2_PFLASH_USER_START,
                    BANKB_CORE2_PFLASH_KERNEL_START) -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE2_PFLASH_USER_SIZE +
                    BANKB_CORE2_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core2_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE2_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE2_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core2_b",
  },
#endif

#ifdef BANKB_CORE3_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE3_PFLASH_USER_START,
                    BANKB_CORE3_PFLASH_KERNEL_START) -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE3_PFLASH_USER_SIZE +
                    BANKB_CORE3_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core3_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE3_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE3_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core3_b",
  },
#endif

#ifdef BANKB_CORE4_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE4_PFLASH_USER_START,
                    BANKB_CORE4_PFLASH_KERNEL_START) - BANKB_BL_PFLASH_START)
                    / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE4_PFLASH_USER_SIZE +
                    BANKB_CORE4_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core4_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE4_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE4_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core4_b",
  },
#endif

#ifdef BANKB_CORE5_PFLASH_USER_START
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    MIN(BANKB_CORE5_PFLASH_USER_START,
                    BANKB_CORE5_PFLASH_KERNEL_START) -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = (BANKB_CORE5_PFLASH_USER_SIZE +
                    BANKB_CORE5_PFLASH_KERNEL_SIZE) / PFLASH_PAGE_SIZE,
    .name        = "/dev/core5_b",
  },

#else
  {
    .firstblock  = (OFFSET_BETWEEN_BANKA_BANKB +
                    BANKB_CORE5_PFLASH_KERNEL_START -
                    BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
    .nblocks     = BANKB_CORE5_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
    .name        = "/dev/core5_b",
  },
#endif

  {
      .firstblock = 0,
      .nblocks = PFLASH_BANK_A_SIZE / PFLASH_PAGE_SIZE,
      .name = "/dev/pflash_a",
  },

  {
      .firstblock = OFFSET_BETWEEN_BANKA_BANKB / PFLASH_PAGE_SIZE,
      .nblocks = PFLASH_BANK_B_SIZE / PFLASH_PAGE_SIZE,
      .name = "/dev/pflash_b",
  },

#ifdef BANKA_VBMETA_START
    {
        .firstblock = (BANKA_VBMETA_START - BANKA_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
        .nblocks = BANKA_VBMETA_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/vbmeta_a",
    },
#endif

#ifdef BANKB_VBMETA_START
    {
        .firstblock = (OFFSET_BETWEEN_BANKA_BANKB + BANKB_VBMETA_START - BANKB_BL_PFLASH_START) / PFLASH_PAGE_SIZE,
        .nblocks = BANKB_VBMETA_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/vbmeta_b",
    },
#endif
};
#endif

#ifdef CONFIG_AURIX_MTD_DFLASH
static const struct partition_s g_aurix_dflash_table[] =
{
  {
    .firstblock = (DFLASH0_NVM_START - DFLASH_START) / DFLASH_PAGE_SIZE,
    .nblocks    = DFLASH0_NVM_SIZE / DFLASH_PAGE_SIZE,
    .name       = CONFIG_AURIX_MTD_CFG_PATH_FOR_NVM,
  },
  {
    .firstblock =
      (DFLASH0_MANUFACTURY_START - DFLASH_START) / DFLASH_PAGE_SIZE,
    .nblocks    = DFLASH0_MANUFACTURY_SIZE / DFLASH_PAGE_SIZE,
    .name       = "/dev/manufactury"
  },
  {
    .firstblock = (DFLASH0_BSWLOG_START - DFLASH_START) / DFLASH_PAGE_SIZE,
    .nblocks    = DFLASH0_BSWLOG_SIZE / DFLASH_PAGE_SIZE,
    .name       = "/dev/bswlog",
  },
  {
    .firstblock = (DFLASH0_TRAPINFO_START - DFLASH_START) / DFLASH_PAGE_SIZE,
    .nblocks    = DFLASH0_TRAPINFO_SIZE / DFLASH_PAGE_SIZE,
    .name       = "/dev/trapinfo",
  },
  {
    .firstblock = (DFLASH0_DFXLOG_START - DFLASH_START) / DFLASH_PAGE_SIZE,
    .nblocks    = DFLASH0_DFXLOG_SIZE / DFLASH_PAGE_SIZE,
    .name       = "/dev/dfxlog",
  },
};
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
static const struct partition_s g_aurix_cspflash_table[] =
{
#ifdef BANKA_CORE6_PFLASH_KERNEL_START
    {
        .firstblock = 0,
        .nblocks = BANKA_CORE6_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/core6_a",
    },
#endif

#ifdef BANKB_CORE6_PFLASH_KERNEL_START
    {
        .firstblock = (BANKB_CORE6_PFLASH_KERNEL_START - BANKA_CORE6_PFLASH_KERNEL_START) / PFLASH_PAGE_SIZE,
        .nblocks = BANKB_CORE6_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/core6_b",
    },
#endif

#ifdef CORE6_RES_PFLASH_KERNEL_START
    {
        .firstblock = (CORE6_RES_PFLASH_KERNEL_START - BANKA_CORE6_PFLASH_KERNEL_START) / PFLASH_PAGE_SIZE,
        .nblocks = CORE6_RES_PFLASH_KERNEL_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/core6_res",
    },
#endif

#ifdef BANKA_CORE6_VBMETA_START
    {
        .firstblock = (BANKA_CORE6_VBMETA_START - BANKA_CORE6_PFLASH_KERNEL_START) / PFLASH_PAGE_SIZE,
        .nblocks = BANKA_CORE6_VBMETA_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/vbmeta_core6_a",
    },
#endif

#ifdef BANKB_CORE6_VBMETA_START
    {
        .firstblock = (BANKB_CORE6_VBMETA_START - BANKA_CORE6_PFLASH_KERNEL_START) / PFLASH_PAGE_SIZE,
        .nblocks = BANKB_CORE6_VBMETA_SIZE / PFLASH_PAGE_SIZE,
        .name = "/dev/vbmeta_core6_b",
    },
#endif
};
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
static const struct partition_s g_aurix_csdflash_table[] =
{
  {
    .firstblock = 0,
    .nblocks    = DFLASH1_SIZE / DFLASH_PAGE_SIZE,
    .name       = CONFIG_AURIX_MTD_CFG_PATH_FOR_NVM,
  },
};
#endif

#ifdef CONFIG_AMP_TAS6754
static struct tas6754_config_s g_aurix_tas6754_0_cfg =
{
  .frequency      = 400000,
  .address        = 0x70,
  .device_id      = 0,
  .def_nchannels  = 2,
  .def_bpsamp     = 32,
  .def_samprate   = 48000,
  .def_volume     = 400,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dfx_noteram_create
 *
 * Description:
 *   Create ntoeram devices for dfxlog daemon
 *
 ****************************************************************************/

#ifdef CONFIG_DFX
static int dfx_noteram_create(void)
{
  struct note_filter_named_tag_s tag_filter;
  struct note_driver_s *note_driver;
  const char *devpath;

  devpath = "/dev/noteram1";
  note_driver = noteram_initialize(devpath, 512, true, false, true);
  if (note_driver == NULL)
    {
      syslog(LOG_ERR, "ERROR: noteram_initialize() failed\n");
      return -ENOMEM;
    }

  strlcpy(tag_filter.name, devpath, NAME_MAX);
  memset(&tag_filter.tag_mask, 0xff, sizeof(tag_filter.tag_mask));
  NOTE_FILTER_TAGMASK_CLR(26, &tag_filter.tag_mask);
  note_filter_tag(NULL, &tag_filter);

  devpath = "/dev/noteram2";
  note_driver = noteram_initialize(devpath, 512, true, false, true);
  if (note_driver == NULL)
    {
      syslog(LOG_ERR, "ERROR: noteram_initialize() failed\n");
      return -ENOMEM;
    }

  strlcpy(tag_filter.name, devpath, NAME_MAX);
  memset(&tag_filter.tag_mask, 0xff, sizeof(tag_filter.tag_mask));
  NOTE_FILTER_TAGMASK_CLR(27, &tag_filter.tag_mask);
  note_filter_tag(NULL, &tag_filter);

  return OK;
}
#endif

/****************************************************************************
 * Name: aurix_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use
 *
 ****************************************************************************/

#ifdef CONFIG_GPIO_LOWER_HALF

#define IOE_MAX_NUM (23)
#define PIN_MAX_NUM (16)

static void aurix_gpio_initialize(void)
{
  int i;
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  int j;
  int ioe_index[IOE_MAX_NUM] = {0};
#endif

  for (i = 0; i < AURIX_GPIO_COUNTS; i++)
    {
      gpio_lower_half(g_ioe[g_pin_config[i].numOfIoe],
                      g_pin_config[i].pin,
                      g_pin_config[i].type,
                      g_pin_config[i].minor);

#ifdef CONFIG_IOEXPANDER_MULTIPIN
        ioe_index[g_pin_config[i].numOfIoe] = 1;
#else
      if (g_pin_config[i].type == GPIO_OUTPUT_PIN_OPENDRAIN ||
          g_pin_config[i].type == GPIO_OUTPUT_PIN)
        {
          IOEXP_WRITEPIN(g_ioe[g_pin_config[i].numOfIoe],
                g_pin_config[i].pin,
                g_pin_config[i].initValue);
        }
#endif
    }

#ifdef CONFIG_IOEXPANDER_MULTIPIN
  for (i = 0; i < IOE_MAX_NUM; i++)
    {
      uint8_t pins[PIN_MAX_NUM];
      bool    values[PIN_MAX_NUM];
      int     count = 0;

      if(ioe_index[i] == 0)
        continue;

      for(j = 0; j < AURIX_GPIO_COUNTS; j++)
        {
          if (g_pin_config[j].numOfIoe != i)
            continue;

          if (g_pin_config[j].type == GPIO_OUTPUT_PIN_OPENDRAIN ||
              g_pin_config[j].type == GPIO_OUTPUT_PIN)
            {
              pins[count] = g_pin_config[j].pin;
              values[count] = g_pin_config[j].initValue;
              count++;
            }
        }

      if (count > 0)
        {
          IOEXP_MULTIWRITEPIN(g_ioe[i], pins, values, count);
        }
    }
#endif
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_earlyinitialize(void)
{
#ifdef CONFIG_AURIX_MTD_PFLASH
  aurix_partition_init(g_aurix_pflash_table,
                       nitems(g_aurix_pflash_table),
                       g_mtd_pflash);
#endif

#ifdef CONFIG_AURIX_MTD_DFLASH
  aurix_partition_init(g_aurix_dflash_table,
                       nitems(g_aurix_dflash_table),
                       g_mtd_dflash);
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
  aurix_partition_init(g_aurix_cspflash_table,
                       nitems(g_aurix_cspflash_table), g_mtd_cspflash);
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
  aurix_partition_init(g_aurix_csdflash_table,
                       nitems(g_aurix_csdflash_table), g_mtd_csdflash);
#endif
}

void board_lateinitialize_phaseA(void)
{
#if defined(CONFIG_AURIX_TMADC) && defined(CONFIG_AURIX_TMADC_MAIN_BOOT_CORE)
    aurix_adc_module_config(g_aurix_tmadc_module_config,
        AURIX_TMADC_MODULE_COUNTS);
#endif
}

void board_lateinitialize_phaseB(void)
{
  int ret;

#ifdef CONFIG_AURIX_PWM
#ifdef CONFIG_AURIX_EGTM_TOM_PWM
    aurix_egtm_tom_pwm_cfg_initialize(g_aurix_pwm_channel_cfgs,
        g_pwm_channels,
        AURIX_PWM_CHANNEL_COUNTS);
#endif /* CONFIG_AURIX_GTM_TOM_PWM */

#ifdef CONFIG_AURIX_EGTM_ATOM_PWM
    aurix_egtm_atom_pwm_cfg_initialize(g_aurix_pwm_channel_cfgs,
        g_pwm_channels,
        AURIX_PWM_CHANNEL_COUNTS);
#endif /* CONFIG_AURIX_GTM_ATOM_PWM */
#endif /* CONFIG_AURIX_PWM */

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM0_TIMER)
  /* Initialize and register ATOM0 Timer Device */

  aurix_egtm_timer_initialize(g_atom0_timer,
                              g_aurix_egtm_atom0_timer_config,
                              nitems(g_aurix_egtm_atom0_timer_config));
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM1_TIMER)
  /* Initialize and register ATOM1 Timer Device */

  aurix_egtm_timer_initialize(g_atom1_timer,
                              g_aurix_egtm_atom1_timer_config,
                                nitems(g_aurix_egtm_atom1_timer_config));
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM2_TIMER)
  /* Initialize and register ATOM2 Timer Device */

  aurix_egtm_timer_initialize(g_atom2_timer,
                              g_aurix_egtm_atom2_timer_config,
                              nitems(g_aurix_egtm_atom2_timer_config));
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM0_TIMER)
  /* Initialize and register TOM0 Timer Device */

  aurix_egtm_tom_timer_initialize(g_tom0_timer,
                              g_aurix_egtm_tom0_timer_config,
                              nitems(g_aurix_egtm_tom0_timer_config));
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM1_TIMER)
  /* Initialize and register TOM1 Timer Device */

  aurix_egtm_tom_timer_initialize(g_tom1_timer,
                              g_aurix_egtm_tom1_timer_config,
                              nitems(g_aurix_egtm_tom1_timer_config));
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM2_TIMER)
  /* Initialize and register TOM2 Timer Device */

  aurix_egtm_tom_timer_initialize(g_tom2_timer,
                              g_aurix_egtm_tom2_timer_config,
                              nitems(g_aurix_egtm_tom2_timer_config));
#endif

#ifdef CONFIG_AURIX_SPI
  aurix_all_spi_initialize(g_spi, g_aurix_spi_config,
                           nitems(g_aurix_spi_config));

#endif

#ifdef CONFIG_AURIX_TMADC
  aurix_adc_initialize(g_adc, g_aurix_tmadc_config,
                       AURIX_TMADC_COUNTS);
#endif

#if defined(CONFIG_AURIX_CAPTURE)
    /* Initialize and register eGTM TIM0 capture devices. */
    aurix_egtm_capture_all_initialize(g_capture_channels,
                                      g_aurix_capture_channel_cfgs,
                                      AURIX_CAPTURE_CHANNEL_COUNTS);
#endif

#if defined(CONFIG_AURIX_MCMCAN)
  #ifdef CONFIG_AURIX_MCMCAN_CHARDRIVER

  /* initialize and register canX chardevice node */

  aurix_chardriver_init_mcmcan(g_mcmcan_devs, g_aurix_mcmcan_config,
                               nitems(g_aurix_mcmcan_config));
  #else

  /* initialize and register canX net device */

  aurix_socket_init_mcmcan(g_mcmcan_devs, g_aurix_mcmcan_config,
                           nitems(g_aurix_mcmcan_config));
  #endif
#endif /* CONFIG_AURIX_MCMCAN */

#ifdef CONFIG_AURIX_LIN
  /* Initialize and register the LIN driver */

  aurix_lin_initialize(g_lin_devs,
                       g_aurix_lin_config,
                       nitems(g_aurix_lin_config));
#endif /* CONFIG_AURIX_LIN */

#ifdef CONFIG_AURIX_QBVSCH
  aurix_qbvgate_timer_init(g_qbv_schedu_timer,
                           aurix_qbvgate_timer_config);
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#if defined(CONFIG_GPIO_LOWER_HALF)

  /* Initialize and register the GPIO driver */

  aurix_gpio_initialize();
#endif /* CONFIG_GPIO_LOWER_HALF */

#ifdef CONFIG_DFX
  ret = dfx_noteram_create();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: dfx_noteram_create() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_AURIX_SBC_TLF4D985
  tlf4d985q_init();
#endif

  UNUSED(ret);
}

void board_lateinitialize(void)
{
    int cpuid = sched_getcpu();

#ifdef CONFIG_AUTOCORE_SYNBARRIER
    autocore_sync_barrier_wait(cpuid);
#endif
    board_lateinitialize_phaseA();
#ifdef CONFIG_AUTOCORE_SYNBARRIER
    autocore_sync_barrier_wait(cpuid);
#endif
    board_lateinitialize_phaseB();

    UNUSED(cpuid);
}

void board_finalinitialize(void)
{
}

#if defined(CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 0)
void Ifx_Ssw_AP_Init(void)
{

#ifndef CONFIG_TRICORE_BL
  IfxApApu_ApuConfig config;
  IfxCan_ApConfig can_config;
  IfxApApu_ApuMemoryConfig memconfig;
  int i;

  Ifx_CAN* can_module_addr[] = {
        &MODULE_CAN0,
        &MODULE_CAN1,
        &MODULE_CAN2,
        &MODULE_CAN3,
        &MODULE_CAN4,
    };

  /* Access for the Core specific SFRRs,
   * INTs and STMs enabled for all
   * masters. This needs to be updated
   * by user as per his needs
   */

  CPU0_PROTSFRSE.U       = 0x0;
  CPU0_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  CPU1_PROTSFRSE.U       = 0x0;
  CPU1_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  CPU2_PROTSFRSE.U       = 0x0;
  CPU2_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  CPU3_PROTSFRSE.U       = 0x0;
  CPU3_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  CPU4_PROTSFRSE.U       = 0x0;
  CPU4_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  CPU5_PROTSFRSE.U       = 0x0;
  CPU5_ACCENSFRCFG_WRA.U = 0xFFFFFFFFU;

  for (i = 0; i < 8; i++)
  {
      CPU0_PROTSTMSE.U       = i << 8U;
      CPU0_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;

      CPU1_PROTSTMSE.U       = i << 8U;
      CPU1_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;

      CPU2_PROTSTMSE.U       = i << 8U;
      CPU2_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;

      CPU3_PROTSTMSE.U       = i << 8U;
      CPU3_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;

      CPU4_PROTSTMSE.U       = i << 8U;
      CPU4_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;

      CPU5_PROTSTMSE.U       = i << 8U;
      CPU5_ACCENSTMCFG_WRA.U = 0xFFFFFFFFU;
  }

  INT_TOS0_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;
  INT_TOS1_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;
  INT_TOS2_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;
  INT_TOS3_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;
  INT_TOS4_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;
  INT_TOS5_ACCENSCFG_WRA.U  = 0xFFFFFFFFU;

  INT_TOS0_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;
  INT_TOS1_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;
  INT_TOS2_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;
  INT_TOS3_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;
  INT_TOS4_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;
  INT_TOS5_ACCENSCTRL_WRA.U = 0xFFFFFFFFU;

  /* Initialize default config */

  IfxApApu_initConfig(&config);
  memconfig.apuConfig = &config;

  /* Initialize apu for dspr */

  IfxApApu_configureAccessToDsprs(&memconfig);

  /* Initialize int srb apu */

  for (i = 0; i < 8; i++)
    {
      IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_INT.ACCENSRB[i], &config);
    }

  /* Initialize apu for uart */

  IfxAsclin_configureAccessToAsclins(&config);

  /* Initialize apu for cpu stm */

  IfxCpu_configureAccessToCpus(&config);

  /* Initialize pms apu */

  IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_PMS.ACCEN, &config);

  /* Initialize port pin apu */

  IfxPort_ApuConfig apuConfig;
  IfxPort_initApuConfig(&apuConfig);
  IfxPort_configureAccessToPorts(&(apuConfig.apuConfig));

  /* Initialize pin MDC: P16.11 apu */

  IfxPort_setApuGroupSelection(&MODULE_P16, 11, 0);

  /* Initialize dlmu/lmu apu */

  IfxApApu_configureAccessToDlmus(&memconfig);
  IfxApApu_configureAccessToLmus(&memconfig);

  /* Initialize egtm apu */

  IfxEgtm_configureAccessToEgtms(&config);
  IfxSrc_configureAccessToSrcs(&config);

  /* Initialize adc apu */

  for (i = 0u; i < IFXADC_NUM_APU; i++)
    {
      IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_ADC.ACCEN[i], &config);
    }

#ifdef CONFIG_AURIX_QSPI
  /* Initialize qspi apu */

  IfxQspi_configureAccessToQspis(&config);
#endif

#ifdef CONFIG_AURIX_PMS

  /* Initialize smm apu */

  IfxSmm_configureAccessToSmm(&config);
#endif

  /* Initialize dma apu */

  IfxDma_configureAccessToDmas(&config);

  /* Initialize vmt apu */

  IfxVmt_configureAccessToVmts(&config);

  /* Initialize clock apu */

  IfxClock_configureAccessToClock(&config);

  /* initialize can apu */

  IfxCan_initApConfig(&can_config);

  for (i = 0; i < nitems(can_module_addr); i++) {
      IfxApApu_init((Ifx_ACCEN_ACCEN*)&(can_module_addr[i]->ACCEN),
          &can_config.apuCanConfig);
      IfxApApu_init((Ifx_ACCEN_ACCEN*)&(can_module_addr[i]->N[0].ACCEN),
          &can_config.apuNode0Config);
      IfxApApu_init((Ifx_ACCEN_ACCEN*)&(can_module_addr[i]->N[1].ACCEN),
          &can_config.apuNode1Config);
      IfxApApu_init((Ifx_ACCEN_ACCEN*)&(can_module_addr[i]->N[2].ACCEN),
          &can_config.apuNode2Config);
      IfxApApu_init((Ifx_ACCEN_ACCEN*)&(can_module_addr[i]->N[3].ACCEN),
          &can_config.apuNode3Config);
  }

#endif
}
#endif

#if defined(CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 6)
void tricore_csrm_apu_init(void)
{
  IfxApApu_ApuConfig config;
  IfxApApu_ApuMemoryConfig memconfig;
  IfxApApu_ApuConfig cssconfig;

  /* Initialize default config */

  IfxApApu_initConfig(&config);
  memconfig.apuConfig = &config;

  /* Initialize int srb apu */

  IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_INT.ACCENSRB[6], &config);

  /* Initialize lmu apu */

  IfxApApu_configureAccessToLmus(&memconfig);

  /* Initialize srcs apu */

  IfxSrc_configureAccessToSrcs(&config);

  /* Initialize css apu */

  IfxApApu_initConfig(&cssconfig);

  /* Provide required Config for APU-PCFGx and APU-PDATx, x=0:20 */

  cssconfig.wraTagId = (uint32)(1 << IfxApProt_TagId_cpu0d |
                                   1 << IfxApProt_TagId_cpucsd);
  cssconfig.rdaTagId = (uint32)(1 << IfxApProt_TagId_cpu0d |
                                   1 << IfxApProt_TagId_cpucsd);
  cssconfig.vmRdId = 0x1;
  cssconfig.vmWrId = 0x1;
  cssconfig.prsRdId = 0x1;
  cssconfig.prsWrId = 0x1;

  /* Allow cpu0 and cpucs to have access to channel 1-20 */

  IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_CSS0.ACCENCS, &cssconfig);

  for (int idx = 1u; idx < IFXCSS_NUM_CHANNELS; idx++)
    {
      IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_CSS0.CH[idx].ACCEN.CFG, &cssconfig);
      IfxApApu_init((Ifx_ACCEN_ACCEN *)&MODULE_CSS0.CH[idx].ACCEN.DATA, &cssconfig);
    }
}
#endif

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the user-space heap.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section.
   */

  uintptr_t ubase       = (uintptr_t)USERSPACE->us_bssend;
  uintptr_t udlmu_start = GENERATE_CORE_DLMU_USER_START(CONFIG_CPU_COREID);
  uintptr_t udlmu_size  = GENERATE_CORE_DLMU_USER_SIZE(CONFIG_CPU_COREID);

  /* Return the user-space heap settings */

  DEBUGASSERT(udlmu_start + udlmu_size > ubase);

  *heap_start = (void *)ubase;
  *heap_size  = udlmu_start + udlmu_size - ubase;
#else

  /* Return the heap settings */

  *heap_start = _sheap;
  *heap_size = (uintptr_t)_eheap - (uintptr_t)_sheap;
#endif
}
