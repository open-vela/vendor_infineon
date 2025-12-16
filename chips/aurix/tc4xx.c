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

#include <arch/chip/chip.h>
#include <sys/param.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/drivers/rpmsgdev.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/irq.h>
#include <nuttx/net/netdev.h>
#include <nuttx/kmalloc.h>

#include <IfxStm.h>

#include "aurix_systimer.h"
#include "tricore_internal.h"
#include "aurix_tmadc.h"
#include "aurix_i2c.h"
#include "aurix_uart.h"
#include "aurix_ioexpander.h"
#include "aurix_egtm.h"
#include "aurix_egtm_capture.h"
#include "aurix_egtm_pwm.h"
#ifdef CONFIG_AUTOMID_GATEWAY_DRE
#include "aurix_dre.h"
#endif
#include "aurix_rpmsg.h"
#include "aurix_spi.h"
#include "aurix_mtd_partition.h"
#include "aurix_wdg.h"
#include "aurix_qspi.h"
#include "aurix_qspi_slave.h"
#include "aurix_lin.h"
#include "aurix_pms.h"

#ifdef CONFIG_AURIX_I2S
#include "aurix_i2s.h"
#endif

#ifdef CONFIG_AURIX_I2S_SIM
#include "aurix_i2s_sim.h"
#endif

#ifdef CONFIG_AURIX_ENET
#include "aurix_enet.h"
#endif

#ifdef CONFIG_AURIX_MTD_FLASH
#include "aurix_mtd_flash.h"
#endif

#ifdef CONFIG_AURIX_EGTM_ATOM_TIMER
#include "aurix_egtm_atom_timer.h"
#endif

#ifdef CONFIG_AURIX_EGTM_TOM_TIMER
#include "aurix_egtm_tom_timer.h"
#endif

#ifdef CONFIG_BUILD_PROTECTED
#include "aurix_userspace.h"
#endif

#ifdef CONFIG_ARCH_USE_MPU
#include "tricore_mpu.h"
#endif

#ifdef CONFIG_AURIX_MCMCAN
#include "aurix_mcmcan.h"
#include "can_mram.h"
#endif

#ifdef CONFIG_AURIX_UCB
#include "aurix_ucb.h"
#endif

#ifdef CONFIG_AURIX_SCR
#include "aurix_scr.h"
#include "scr_core_code.h"
#endif

#ifdef CONFIG_TRICORE_CSRM
#include "aurix_crypto.h"
#endif

#include "memory_layout.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENET_MODULE_NUMBER 2
#define MCMCAN_NODE_NUMBER 20
#define LIN_MODULE_NUMBER  28

#define AURIX_RPMSG_CONFIG(mcpu, scpu, vnum, valign, vbufsize) \
  { \
    .master_cpu     = (mcpu), \
    .slave_cpu      = (scpu), \
    .vring_num      = (vnum), \
    .vring_align    = (valign), \
    .vring_buf_size = (vbufsize), \
  }

#define TRICORE_IRQ_GET(SRC_ADDR) (((uintptr_t)&SRC_ADDR - (uintptr_t)&SRC_CPU0_SB) / 4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

__attribute__((section(".sync_barrier"))) \
shared_data_manual_s shared_data_manual =
{
  .tmadc_sync_barrier = 0,
  .eth_sync_barrier   = 0,
  .eth_mac_ready_flag = false,
  .can_module         =
    {
    {
      .can_module_ref  = 0,
      .can_module_lock = SP_UNLOCKED
    },
    {
      .can_module_ref  = 0,
      .can_module_lock = SP_UNLOCKED
    },
    {
      .can_module_ref  = 0,
      .can_module_lock = SP_UNLOCKED
    },
    {
      .can_module_ref  = 0,
      .can_module_lock = SP_UNLOCKED
    },
    {
      .can_module_ref  = 0,
      .can_module_lock = SP_UNLOCKED
    }
    }
};

#if defined(CONFIG_AURIX_IOEXPANDER)
static const struct aurix_ioexpander_config_s g_aurix_ioe_config[] =
{
#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
    {&MODULE_P00},
    {&MODULE_P01},
    {&MODULE_P02},
    {&MODULE_P03},
    {&MODULE_P04},
    {&MODULE_P10},
    {&MODULE_P13},
    {&MODULE_P14},
    {&MODULE_P15},
    {&MODULE_P16},
    {&MODULE_P20},
    {&MODULE_P21},
    {&MODULE_P22},
    {&MODULE_P23},
    {&MODULE_P25},
    {&MODULE_P30},
    {&MODULE_P31},
    {&MODULE_P32},
    {&MODULE_P33},
    {&MODULE_P34},
    {&MODULE_P35},
    {&MODULE_P40}
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
    {&MODULE_P00},
    {&MODULE_P01},
    {&MODULE_P02},
    {&MODULE_P03},
    {&MODULE_P10},
    {&MODULE_P11},
    {&MODULE_P12},
    {&MODULE_P13},
    {&MODULE_P14},
    {&MODULE_P15},
    {&MODULE_P16},
    {&MODULE_P20},
    {&MODULE_P21},
    {&MODULE_P22},
    {&MODULE_P23},
    {&MODULE_P25},
    {&MODULE_P30},
    {&MODULE_P31},
    {&MODULE_P32},
    {&MODULE_P33},
    {&MODULE_P34},
    {&MODULE_P35},
    {&MODULE_P40}
#endif
};
#endif

#if defined(CONFIG_AURIX_UART0)
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_AURIX_UART
static struct uart_dev_s g_aurix_uart[] =
{
#if defined(CONFIG_AURIX_UART0)
  [0] =
    {
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
    .isconsole = 1,
#  endif
    .recv      =
    {
      .size    = CONFIG_UART0_RXBUFSIZE,
      .buffer  = g_uart0rxbuffer,
    },
    .xmit      =
    {
      .size    = CONFIG_UART0_TXBUFSIZE,
      .buffer  = g_uart0txbuffer,
    },
    .priv = (void *)&g_aurix_uart_config[0],
  },
#endif
};
#endif

/* Watchdog timer configuration definition. */

#ifdef CONFIG_AURIX_WDG
static const struct aurix_wdg_config_s g_aurix_wdg_config[] =
{
  #ifdef CONFIG_AURIX_WDG_WDTCPU0
  {
    .devpath = "/dev/wdt_cpu0",
    .wdt_ptr = &MODULE_WTU.WDTCPU[0],
    .wdt_idx = AURIX_WDG_WDTCPU0,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTCPU1
  {
    .devpath = "/dev/wdt_cpu1",
    .wdt_ptr = &MODULE_WTU.WDTCPU[1],
    .wdt_idx = AURIX_WDG_WDTCPU1,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTCPU2
  {
    .devpath = "/dev/wdt_cpu2",
    .wdt_ptr = &MODULE_WTU.WDTCPU[2],
    .wdt_idx = AURIX_WDG_WDTCPU2,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTCPU3
  {
    .devpath = "/dev/wdt_cpu3",
    .wdt_ptr = &MODULE_WTU.WDTCPU[3],
    .wdt_idx = AURIX_WDG_WDTCPU3,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTCPU4
  {
    .devpath = "/dev/wdt_cpu4",
    .wdt_ptr = &MODULE_WTU.WDTCPU[4],
    .wdt_idx = AURIX_WDG_WDTCPU4,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTCPU5
  {
    .devpath = "/dev/wdt_cpu5",
    .wdt_ptr = &MODULE_WTU.WDTCPU[5],
    .wdt_idx = AURIX_WDG_WDTCPU5,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTSEC
  {
    .devpath = "/dev/wdt_sec",
    .wdt_ptr = &MODULE_WTU.WDTSEC,
    .wdt_idx = AURIX_WDG_WDTSEC,
  },
  #endif

  #ifdef CONFIG_AURIX_WDG_WDTSYS
  {
    .devpath = "/dev/wdt_sys",
    .wdt_ptr = &MODULE_WTU.WDTSYS,
    .wdt_idx = AURIX_WDG_WDTSYS,
  },
  #endif
};
#endif /* CONFIG_AURIX_WDG */

static const struct aurix_systimer_config_s g_aurix_systimer_config =
{
#if CONFIG_CPU_COREID == 0
  .tbase      = &MODULE_CPU0,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU0_SR2),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 1
  .tbase      = &MODULE_CPU1,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU1_SR2),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 2
  .tbase      = &MODULE_CPU2,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU2_SR2),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 3
  .tbase      = &MODULE_CPU3,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU3_SR2),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 4
  .tbase      = &MODULE_CPU4,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU4_SR2),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 5
  .tbase      = &MODULE_CPU5,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPU5_SR2),
  .freq       = SCU_FREQUENCY,
#elif CONFIG_CPU_COREID == 6
  .tbase      = &MODULE_CPUCS,
  .irq        = TRICORE_IRQ_GET(SRC_STMCPUCSSR2),
  .freq       = SCU_FREQUENCY,
#endif
};

#ifdef CONFIG_ARCH_USE_MPU

static unsigned int kstack_mpu_region;

#ifdef CONFIG_BUILD_PROTECTED
static unsigned int ustack_mpu_region;
#endif

static struct mpu_region_s g_mpu_regions[] =
{
  #include "mpuconfig.h"
};

#ifdef CONFIG_ARCH_KSTACK_PROTECT
void os_kstack_protect(void)
{
  mpu_modify_region(g_mpu_kset, kstack_mpu_region,
                    GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);
}
#endif /* CONFIG_ARCH_KSTACK_PROTECT */

#endif /* CONFIG_ARCH_USE_MPU */

#ifdef CONFIG_AURIX_MTD_PFLASH
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
static const flash_map_t g_aurix_pflash_map[PFLASH_BANKS] =
{
  {
    IfxFlash_FlashType_P00,
    IFXFLASH_PFLASH_P00_START,
    OTA_BANK_A,
    PFLASH_START,
    PFLASH_START + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P00_SIZE
  },
  {
    IfxFlash_FlashType_P01,
    IFXFLASH_PFLASH_P01_START,
    OTA_BANK_B,
    PFLASH_START + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START,
    IFXFLASH_PFLASH_P01_SIZE
  },
  {
    IfxFlash_FlashType_P10,
    IFXFLASH_PFLASH_P10_START,
    OTA_BANK_A,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P10_SIZE
  },
  {
    IfxFlash_FlashType_P11,
    IFXFLASH_PFLASH_P11_START,
    OTA_BANK_B,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE,
    IFXFLASH_PFLASH_P11_SIZE
  },
  {
    IfxFlash_FlashType_P20,
    IFXFLASH_PFLASH_P20_START,
    OTA_BANK_A,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P20_SIZE
  },
  {
    IfxFlash_FlashType_P21,
    IFXFLASH_PFLASH_P21_START,
    OTA_BANK_B,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE,
    IFXFLASH_PFLASH_P21_SIZE
  },
  {
    IfxFlash_FlashType_P30,
    IFXFLASH_PFLASH_P30_START,
    OTA_BANK_A,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P30_SIZE
  },
  {
    IfxFlash_FlashType_P31,
    IFXFLASH_PFLASH_P31_START,
    OTA_BANK_B,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE,
    IFXFLASH_PFLASH_P31_SIZE},
  {
    IfxFlash_FlashType_P40,
    IFXFLASH_PFLASH_P40_START,
    OTA_BANK_A,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE
                 + IFXFLASH_PFLASH_P30_SIZE,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE
                 + IFXFLASH_PFLASH_P30_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P40_SIZE
  },
  {
    IfxFlash_FlashType_P41,
    IFXFLASH_PFLASH_P41_START,
    OTA_BANK_B,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE
                 + IFXFLASH_PFLASH_P31_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE
                 + IFXFLASH_PFLASH_P31_SIZE,
    IFXFLASH_PFLASH_P41_SIZE
  },
  {
    IfxFlash_FlashType_P50,
    IFXFLASH_PFLASH_P50_START,
    OTA_BANK_A,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE
                 + IFXFLASH_PFLASH_P30_SIZE
                 + IFXFLASH_PFLASH_P40_SIZE,
    PFLASH_START + IFXFLASH_PFLASH_P00_SIZE
                 + IFXFLASH_PFLASH_P10_SIZE
                 + IFXFLASH_PFLASH_P20_SIZE
                 + IFXFLASH_PFLASH_P30_SIZE
                 + IFXFLASH_PFLASH_P40_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXFLASH_PFLASH_P50_SIZE
  },
  {
    IfxFlash_FlashType_P51,
    IFXFLASH_PFLASH_P51_START,
    OTA_BANK_B,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE
                 + IFXFLASH_PFLASH_P31_SIZE
                 + IFXFLASH_PFLASH_P41_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXFLASH_PFLASH_P01_SIZE
                 + IFXFLASH_PFLASH_P11_SIZE
                 + IFXFLASH_PFLASH_P21_SIZE
                 + IFXFLASH_PFLASH_P31_SIZE
                 + IFXFLASH_PFLASH_P41_SIZE,
    IFXFLASH_PFLASH_P51_SIZE
  },
};
#else // CONFIG_ARCH_CHIP_AURIX_TC48X
static const flash_map_t g_aurix_pflash_map[PFLASH_BANKS] =
{
  {
    IfxNvmr_NvmrType_P00,
    IFXNVMR_PNVM00_START,
    OTA_BANK_A,
    PFLASH_START,
    PFLASH_START + OFFSET_BETWEEN_BANKA_BANKB,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P00,
    IFXNVMR_PNVM01_START,
    OTA_BANK_B,
    PFLASH_START + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P10,
    IFXNVMR_PNVM10_START,
    OTA_BANK_A,
    PFLASH_START + IFXNVMR_PNVM_SIZE,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P11,
    IFXNVMR_PNVM11_START,
    OTA_BANK_B,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXNVMR_PNVM_SIZE,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P20,
    IFXNVMR_PNVM20_START,
    OTA_BANK_A,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P21,
    IFXNVMR_PNVM21_START,
    OTA_BANK_B,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P30,
    IFXNVMR_PNVM30_START,
    OTA_BANK_A,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    IFXNVMR_PNVM_SIZE
  },
  {
    IfxNvmr_NvmrType_P31,
    IFXNVMR_PNVM31_START,
    OTA_BANK_B,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + OFFSET_BETWEEN_BANKA_BANKB,
    PFLASH_START + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE
                 + IFXNVMR_PNVM_SIZE,
    IFXNVMR_PNVM_SIZE
  },
};
#endif

static const struct aurix_mtd_cfg_s g_aurix_pflash_cfg =
{
  .sectorsz = PFLASH_SECTOR_SIZE,
#ifdef CONFIG_LINEAR_ADDRESS_MAP
  .nsectors =
    (PFLASH_BANK_A_SIZE + PFLASH_BANK_B_SIZE) / PFLASH_SECTOR_SIZE,
#else
  .nsectors =
    (PFLASH_BANK_A_SIZE + OFFSET_BETWEEN_BANKA_BANKB) / PFLASH_SECTOR_SIZE,
#endif
  .pagesz   = PFLASH_PAGE_SIZE,
  .baseaddr = PFLASH_START,
};
#endif

#ifdef CONFIG_AURIX_MTD_DFLASH
static const struct aurix_mtd_cfg_s g_aurix_dflash_cfg =
{
  .sectorsz = DFLASH_SECTOR_SIZE,
  .nsectors = DFLASH_SIZE / DFLASH_SECTOR_SIZE,
  .pagesz   = DFLASH_PAGE_SIZE,
  .baseaddr = DFLASH_START
};
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
static const struct aurix_mtd_cfg_s g_aurix_csrm_pflash_cfg =
{
  .sectorsz = PFLASH_SECTOR_SIZE,
  .nsectors = CSRM_PFLASH_SIZE / PFLASH_SECTOR_SIZE,
  .pagesz   = PFLASH_PAGE_SIZE,
  .baseaddr = CSRM_PFLASH_START,
};
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
static const struct aurix_mtd_cfg_s g_aurix_csrm_dflash_cfg =
{
  .sectorsz = DFLASH_SECTOR_SIZE,
  .nsectors = CSRM_DFLASH_SIZE / DFLASH_SECTOR_SIZE,
  .pagesz   = DFLASH_PAGE_SIZE,
  .baseaddr = CSRM_DFLASH_START
};
#endif

#ifdef CONFIG_AURIX_RPMSG
static const struct aurix_rpmsg_config_s g_aurix_rpmsg_cfg[] =
{
  AURIX_RPMSG_CONFIG(0, 1, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(0, 2, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(0, 3, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(0, 4, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(0, 5, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(0, 6, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(1, 2, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(1, 3, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(1, 4, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(1, 5, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(2, 3, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(2, 4, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(2, 5, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(3, 4, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(3, 5, 8, 8, 0x100),
  AURIX_RPMSG_CONFIG(4, 5, 8, 8, 0x100),
};
#endif

#ifdef CONFIG_AURIX_UCB
static UCB_T ucb_map[] =
{
  /* host ucb : swap */

  {
    .ucb_name = "RTC_SWAP",
    .region = UCB_REGION_RTC,
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
    .type = IfxFlash_UcbType_ucbSwap,
#endif
    .ucb_orig_no = UCB_RTC_SWAP_ORIG,
    .ucb_copy_no = UCB_RTC_SWAP_COPY,
    .password =
      {
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000
      },
  },

  /* host ucb : usrcfg */

  {
    .ucb_name = "RTC_USRCFG",
    .region = UCB_REGION_RTC,
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
    .type = IfxFlash_UcbType_ucbUsercfg,
#endif
    .ucb_orig_no = UCB_RTC_USERCFG_ORIG,
    .ucb_copy_no = UCB_RTC_USERCFG_COPY,
    .password =
      {
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000
      },
  },

  /* csrm ucb : swap */

  {
    .ucb_name = "CS_SWAP",
    .region = UCB_REGION_CS,
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
    .type = IfxFlashCsrm_UcbType_csrmucbSwap,
#endif
    .ucb_orig_no = UCB_CS_SWAP_ORIG,
    .ucb_copy_no = UCB_CS_SWAP_COPY,
    .password =
      {
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000
      },
  },

  /* csrm ucb : usrcfg */

  {
    .ucb_name = "CS_USRCFG",
    .region = UCB_REGION_CS,
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
    .type = IfxFlashCsrm_UcbType_csrmucbUsercfg,
#endif
    .ucb_orig_no = UCB_CS_USERCFG_ORIG,
    .ucb_copy_no = UCB_CS_USERCFG_COPY,
    .password =
      {
        0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000
      },
  },
};
#endif

#ifdef CONFIG_AURIX_PMS
static aurix_pms_config_t g_aurix_pms_config =
{
  .pms = &MODULE_PMS,
  .standbyConfig =
  {
    .masterCpu = IfxCpu_ResourceCpu_0,
    .trigger = IfxPmsPm_StandbyTriggerMode_software,
    .standbyCfg.enableVddextdcPowerDown = TRUE,
    .wakeupCfg.wakeupEvent.stdbyModePower =
      IfxPmsPm_WakeupStandbyModeSel_stdbyTorun,
    .voltageCfg.enableStandbyOnVddRampDown = TRUE,
    .minDelayBeforeWakeUp = IfxPmsPm_BlankingFilterDelay_80ms,
    .scrCfg.enableScr = FALSE,
    .scrCfg.scrClockSupply = IfxPmsPm_ScrClocking_startswith70kHz,
    .scrCfg.scrTriggerTransition = IfxPmsPm_ScrTriggerTransition_disable,
    .voltageCfg.vddUnderVoltageThreshold = 930,
    .voltageCfg.vddUnderVoltageMode =
       IfxPmsEvr_UnderVoltageMonitoring_highToLowVoltageTransition,
    .standbyCfg.standbyModeSel = IfxPmsPm_StandbyModeSelection_stdby1,
    .standbyCfg.standbySCRRamSupply = IfxPmsPm_StandbySCRRamSupply_supply,
    .standbyCfg.standbyRamBlock = IfxPmsPm_StandbyRamSupply_cpu0_64Kb,
    .wakeupCfg.enableWakeupOnScr = TRUE,
    .wakeupCfg.enableWakeupOnPorst = TRUE,
    .wakeupCfg.wakeupEvent.stdbyModePinB =
      IfxPmsPm_WakeupStandbyModeSel_stdbyTorun,
    .wakeupCfg.edgeTrigger.pinBTriggerEvent =
      IfxPmsPm_PinEdgeTriggerEvent_fallingEdge,
    .padCfg.allPads = IfxPmsPm_PadStateRequest_tristate,
    .padCfg.standbyPads = IfxPmsPm_PadStateRequest_tristate
  },
};
#endif

#ifdef CONFIG_AURIX_SCR
scr_io_config_t g_scr_io_config[] =
{
  /* {&MODULE_P33, 0}, */

  /* {&MODULE_P34, 1}, */
};

static aurix_scr_config_t g_aurix_scr_config =
{
  .scr_io = g_scr_io_config,
  .scr_io_size = sizeof(g_scr_io_config) / sizeof(scr_io_config_t),
  .rtc_clock_source = RTC_CLOCK_SOURCE_70KHZ,
  .memoryConfig =
  {
    .src = &scr_xram,
    .dest = PMS_XRAM,
    .size = SIZE_scr_xram
  },
  .wake_up_time = 3600
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ioexpander_dev_s *g_ioe[] =
{
#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
  [22] = DEV_END,
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
  [23] = DEV_END,
#endif
};

#if defined(CONFIG_AURIX_UART)
struct uart_dev_s *g_uart[] =
{
#if defined(CONFIG_AURIX_UART0)
  [0] = &g_aurix_uart[0],
#endif
  [1] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_I2C)
struct i2c_master_s *g_i2c[] =
{
  [3] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_CAPTURE)
struct cap_lowerhalf_s *g_capture_channels[] =
{
  [24] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_PWM
struct pwm_lowerhalf_s *g_pwm_channels[] =
{
  [72] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM0_TIMER)
struct timer_lowerhalf_s *g_atom0_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM1_TIMER)
struct timer_lowerhalf_s *g_atom1_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM2_TIMER)
struct timer_lowerhalf_s *g_atom2_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM0_TIMER)
struct timer_lowerhalf_s *g_tom0_timer[] =
{
  [16] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM1_TIMER)
struct timer_lowerhalf_s *g_tom1_timer[] =
{
  [16] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM2_TIMER)
struct timer_lowerhalf_s *g_tom2_timer[] =
{
  [16] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_SPI
struct spi_dev_s *g_spi[] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_TMADC
struct  adc_dev_s *g_adc[] =
{
  [4] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_MTD_PFLASH
struct mtd_dev_s *g_mtd_pflash;
#endif

#ifdef CONFIG_AURIX_MTD_DFLASH
struct mtd_dev_s *g_mtd_dflash;
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
struct mtd_dev_s *g_mtd_cspflash;
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
struct mtd_dev_s *g_mtd_csdflash;
#endif

#if defined(CONFIG_AURIX_I2S) || defined(CONFIG_AURIX_I2S_SIM)
struct i2s_dev_s *g_i2s[] =
{
  [1] = DEV_END,
};

struct audio_lowerhalf_s *g_audio_i2s[] =
{
  [1] = DEV_END,
};
#endif

#if defined CONFIG_AURIX_ENET
struct net_driver_s *g_enet_devs[ENET_MODULE_NUMBER];
#endif

#ifdef CONFIG_AURIX_QBVSCH
struct timer_lowerhalf_s *g_qbv_schedu_timer[] =
{
  [8] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_QSPI
struct spi_dev_s *g_qspi[] =
{
  [8] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_QSPI_SLAVE
struct spi_slave_ctrlr_s *g_qspislave[] =
{
  [8] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_MCMCAN
  #ifdef CONFIG_AURIX_MCMCAN_CHARDRIVER
    struct can_dev_s *g_mcmcan_devs[MCMCAN_NODE_NUMBER];
  #else
    struct net_driver_s *g_mcmcan_devs[MCMCAN_NODE_NUMBER];
  #endif
#endif

#ifdef CONFIG_AURIX_LIN
struct net_driver_s *g_lin_devs[LIN_MODULE_NUMBER];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_nmitrap
 *
 * Description:
 *   It is used to handle Non-Maskable Interrupt (NMI) trap.
 *
 ****************************************************************************/

int tricore_nmitrap(uint32_t tid, void *context, void *arg)
{
#if defined(CONFIG_AURIX_WDG)
  aurix_wdg_interrupt();
#endif

  return OK;
}

/****************************************************************************
 * Name: tricore_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that tricore_earlyserialinit was called previously.
 *
 ****************************************************************************/

void tricore_serialinit(void)
{
#ifdef CONFIG_AURIX_UART
  aurix_uart_allinitialize(g_uart);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_addregion
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void tricore_addregion(void)
{
  extern uintptr_t  __DLMUHEAP[];
  extern uintptr_t  __DLMUHEAP_END[];

  DEBUGASSERT(__DLMUHEAP <= __DLMUHEAP_END);

  if (__DLMUHEAP_END > __DLMUHEAP)
    {
      kmm_addregion((void *)__DLMUHEAP,
                    (size_t)__DLMUHEAP_END - (size_t)__DLMUHEAP);
    }
}
#endif

/****************************************************************************
 * Name: tricore_mpuinit
 *
 * Description:
 *   Initialize the MPU regions.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_USE_MPU
void tricore_mpuinit(void)
{
  mpu_initialize(g_mpu_regions,
                 sizeof(g_mpu_regions) / sizeof(g_mpu_regions[0]));

  kstack_mpu_region =
    mpu_configure_region(g_mpu_kset,
                         GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                         GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                         REGION_TYPE_DATA | REGION_ATTR_RW);

#ifdef CONFIG_BUILD_PROTECTED
  mpu_modify_region(g_mpu_uset, kstack_mpu_region,
                    GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);

  ustack_mpu_region =
    mpu_configure_region(g_mpu_kset,
                         GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                         GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                         REGION_TYPE_DATA | REGION_ATTR_RW);

#ifdef CONFIG_ARCH_STACK_PROTECT
  mpu_modify_region(g_mpu_uset, ustack_mpu_region,
                    GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);
#else
  mpu_modify_region(g_mpu_uset, ustack_mpu_region,
                    GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RW);
#endif
#endif
}
#endif

/****************************************************************************
 * Name: tricore_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segments.
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void tricore_userspace(void)
{
  aurix_userspace();
}
#endif

/****************************************************************************
 * Name: rpmsg_serialinit
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  aurix_rpmsg_serialinit(g_aurix_rpmsg_cfg, nitems(g_aurix_rpmsg_cfg));
}
#endif

/****************************************************************************
 * Name: up_lateinitialize
 *
 * Description:
 *   Initialize all chip driver at board_lateinitialize/board_app_initialize
 *
 ****************************************************************************/

void up_lateinitialize(void)
{
#ifdef CONFIG_AURIX_PMS
  aurix_pms_init(&g_aurix_pms_config);
#endif

#ifdef CONFIG_AURIX_SCR
  aurix_scr_init(&g_aurix_scr_config);
#endif

#ifdef CONFIG_AURIX_EGTM
  /* Initialize eGTM clock source. */

  aurix_egtm_initialize();
#endif

#ifdef CONFIG_AURIX_RPMSG

  /* Initialize rpmsg device. */

  aurix_rpmsg_init(g_aurix_rpmsg_cfg, nitems(g_aurix_rpmsg_cfg),
                   (FAR void *)AURIX_RPMSG_SHMEM_START,
                   (size_t)AURIX_RPMSG_SHMEM_SIZE);
#endif

#if defined(CONFIG_AURIX_IOEXPANDER)

  /* Set the pin mode for p16_0 to prevent the influence of p16_1. */

  IfxPort_setPinModex(&MODULE_P16, 0, IfxPort_Modex_gpioMode);
  aurix_ioexpander_initialize(g_ioe, g_aurix_ioe_config,
                              nitems(g_aurix_ioe_config));
#endif

#if defined(CONFIG_AURIX_WDG)
  /* Initialize and register watchdog devices. */

  aurix_wdg_all_initialize(g_aurix_wdg_config,
                           nitems(g_aurix_wdg_config));
#endif

#if defined(CONFIG_AUTOMID_GATEWAY_DRE)
  /* initialize DRE engine */

  aurix_dre();
#endif /* CONFIG_AUTOMID_GATEWAY_DRE */

#ifdef CONFIG_DEV_RPMSG
  rpmsgdev_register("corecs", "/dev/random", NULL);
#endif

#ifdef CONFIG_TRICORE_CSRM
  aurix_set_hsmstatus(CSRM2HT_HOST_SYNC_STATUS);
#endif

#if defined(CONFIG_AURIX_PMS)
  aurix_voltagerail_init();
  aurix_vmonp_init();
#endif
}

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower;

  DEBUGASSERT(g_aurix_systimer_config.tbase);

  /* According to the manual, we only set the frequency on core0 */

  if (up_cpu_index() == 0)
    {
      IfxClock_setStmFrequency(SCU_FREQUENCY);
    }

  lower = tricore_systimer_initialize(g_aurix_systimer_config.tbase,
                                      g_aurix_systimer_config.irq,
                                      g_aurix_systimer_config.freq);

  DEBUGASSERT(lower != NULL);

  up_alarm_set_lowerhalf(lower);
}

/****************************************************************************
 * Name: up_earlyinitialize
 *
 * Description:
 *   Initialize all chip driver at board_early_initialize
 *
 ****************************************************************************/

void up_earlyinitialize(void)
{
#ifdef CONFIG_AURIX_MTD_PFLASH
  g_mtd_pflash = aurix_mtd_flash("aurix_pflash",
                                 &g_aurix_pflash_cfg, false, false,
                                 g_aurix_pflash_map);
  if (g_mtd_pflash == NULL)
    {
      ferr("ERROR: Failed to get pflash MTD\n");
    }
#endif

#ifdef CONFIG_AURIX_MTD_DFLASH
  g_mtd_dflash = aurix_mtd_flash("aurix_dflash",
                                 &g_aurix_dflash_cfg, true, false, NULL);
  if (g_mtd_dflash == NULL)
    {
      ferr("ERROR: Failed to get Dflash MTD\n");
    }
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
  g_mtd_cspflash = aurix_mtd_flash("aurix_cspflash",
                                   &g_aurix_csrm_pflash_cfg, false, true,
                                   NULL);
  if (g_mtd_cspflash == NULL)
    {
      ferr("ERROR: Failed to get CSRM pflash MTD\n");
    }
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
  g_mtd_csdflash = aurix_mtd_flash("aurix_csdflash",
                                   &g_aurix_csrm_dflash_cfg, true, true,
                                   NULL);
  if (g_mtd_csdflash == NULL)
    {
      ferr("ERROR: Failed to get CSRM Dflash MTD\n");
    }
#endif

#ifdef CONFIG_AURIX_UCB
  aurix_ucb_initialize(ucb_map, nitems(ucb_map));
#endif
}

