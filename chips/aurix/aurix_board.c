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
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/kthread.h>
#include <nuttx/memoryregion.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/coredump.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include <arch/board/board.h>

#include <debug.h>
#include <ctype.h>
#include <sys/mman.h>

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxSrc.h"

#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
#  include "IfxScuRcu.h"
#  include "memory_layout.h"
#endif

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
#  include "IfxSmmRst.h"
#  include "memory_layout.h"
#endif

#include "Ifx_Ssw_Compilers.h"

#if defined(CONFIG_BOARDCTL_SWITCH_BOOT) && defined(CONFIG_AURIX_UCB)
#  include "aurix_ucb.h"
#endif

#ifdef CONFIG_AURIX_PMS
#  include "aurix_pms.h"
#endif

#if defined(CONFIG_BOOT_SHARE_RAM_DOMAIN_BOOT) || defined(CONFIG_BOOT_SHARE_RAM_DOMAIN_APP)
#  include "bootVer.h"
#endif

#ifdef CONFIG_AUTOMID_TRAPINFO
#  include <TrapInfo_Cfg.h>
#  include <TrapInfo.h>
#endif

#if defined(CONFIG_AUTOMID_EMG) || defined(CONFIG_AUTOMID_TSW_BOOT)
#  include "Emg_Server.h"
#endif

#ifdef CONFIG_COREDUMP
#  include "aurix_coredump.h"
#endif

#if  CONFIG_CPU_COREID == 6
 #include "IfxCscu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if defined(CONFIG_NSH_ARCHINIT) || defined(CONFIG_BOOT_MINIBOOT)
#  define BOARD_APP_INITIALIZE 1
#endif

#if !defined(CONFIG_BOARD_LATE_INITIALIZE) && !defined(BOARD_APP_INITIALIZE)
#  error CONFIG_BOARD_LATE_INITIALIZE or BOARD_APP_INITIALIZE is required for late initialization
#endif

#if defined(CONFIG_BOARD_LATE_INITIALIZE) && defined(BOARD_APP_INITIALIZE)
#  error CONFIG_BOARD_LATE_INITIALIZE and BOARD_APP_INITIALIZE can not be defined at the same time
#endif

#if defined(CONFIG_BOARDCTL_RESET_CAUSE) || defined(CONFIG_BOARDCTL_RESET)
#define HAL_SW_BOOTFLAG_MASK 0xFF
#endif
/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_COREDUMP
struct memory_region_s g_coredump_region =
{
  .start = COREDUMP_RAM_START,
  .end   = COREDUMP_RAM_START + COREDUMP_RAM_SIZE,
  .flags = PROT_WRITE | PROT_READ,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_START_AUTOCORE)
extern uint8_t autocore_stack[CONFIG_AUTOCORE_STACKSIZE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BOARD_EARLY_INITIALIZE
void weak_function up_earlyinitialize(void)
{
}
#endif

#if defined(CONFIG_BOARD_LATE_INITIALIZE) || defined(BOARD_APP_INITIALIZE)
void weak_function up_lateinitialize(void)
{
}
#endif

#ifdef CONFIG_BOARD_EARLY_INITIALIZE
#if CONFIG_CPU_COREID == 0

void aurix_syslog_dump(uintptr_t buf, unsigned int size)
{
  char *buffer = (char *)buf;
  bool is_empty = true;
  int head = 0;
  int tail = 0;
  char prev;
  char cur;
  int i;

  prev = buffer[size - 1];

  for (i = 0; i < size; i++)
    {
      cur = buffer[i];
      if (!isprint(cur) && !isspace(cur) && cur != '\0')
        {
          memset(buffer, 0, size);
          is_empty = true;
          break;
        }
      else if (prev && !cur)
        {
          head = i;
          is_empty = false;
        }
      else if (!prev && cur)
        {
          tail = i;
        }

      prev = cur;
    }

  if (is_empty)
    {
      head = tail = 0;
    }

  if (head < tail)
    {
      syslog(LOG_EMERG, "%.*s\n", size - tail, &buffer[tail]);
      syslog(LOG_EMERG, "%s\n", &buffer[0]);
    }
  else
    {
      syslog(LOG_EMERG, "%.*s\n", size - tail, &buffer[tail]);
    }
}

static int aurix_syslog_dump_all(struct notifier_block *block,
                                 unsigned long action, void *data)
{
#ifdef CONFIG_AURIX_RPMSG_CORE0CORE1
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORE1_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORE1_SYSLOG_SIZE);
  syslog_flush();
#endif
#ifdef CONFIG_AURIX_RPMSG_CORE0CORE2
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORE2_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORE2_SYSLOG_SIZE);
  syslog_flush();
#endif
#ifdef CONFIG_AURIX_RPMSG_CORE0CORE3
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORE3_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORE3_SYSLOG_SIZE);
  syslog_flush();
#endif
#ifdef CONFIG_AURIX_RPMSG_CORE0CORE4
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORE4_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORE4_SYSLOG_SIZE);
  syslog_flush();
#endif
#ifdef CONFIG_AURIX_RPMSG_CORE0CORE5
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORE5_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORE5_SYSLOG_SIZE);
  syslog_flush();
#endif
#ifdef CONFIG_AURIX_RPMSG_CORE0CORECS
  aurix_syslog_dump(AURIX_RPMSG_CORE0CORECS_SYSLOG_START,
                    AURIX_RPMSG_CORE0CORECS_SYSLOG_SIZE);
  syslog_flush();
#endif

  memset((FAR void *)AURIX_SYSLOG_RPMSG_START, 0,
         (size_t)AURIX_SYSLOG_RPMSG_SIZE);
  return 0;
}

#endif /* CONFIG_CPU_COREID == 0 */

/****************************************************************************
 * Name: board_early_initialize
 *
 * Description:
 *   Call the board-specific up_initialize() extension to support
 *   early initialization of board-specific drivers and resources
 *   that cannot wait until board_late_initialize.
 ****************************************************************************/

void board_early_initialize()
{
  up_earlyinitialize();

  board_earlyinitialize();

#ifdef CONFIG_BOARDCTL_RESET_CAUSE
  struct boardioc_reset_cause_s rst_cause;

  /* When board_reset_cause is called once, the reset cause
   * will be saved to the internal global variable.
   */

  board_reset_cause(&rst_cause);
#endif

#if !defined(CONFIG_BOARD_CRASHDUMP_NONE) && defined(CONFIG_AUTOMID_TRAPINFO)
  extern TrapInfo_s g_TrapInfo;
  int retval = coredump_add_memory_region(&g_TrapInfo,
                                          sizeof(g_TrapInfo),
                                          0);

  TrapInfo_cfg_PanicNotifierRegist();

  if (retval < 0)
    {
       _err("ERROR: coredump_add_memory_regio failed, retval:%d\n", retval);
    }
#endif

#if CONFIG_CPU_COREID == 0

  /* Register syslog dump to panic notifier lists */

  static struct notifier_block nb;

  nb.notifier_call = aurix_syslog_dump_all;
  nb.priority = -5;
  panic_notifier_chain_register(&nb);
#endif

#ifdef CONFIG_COREDUMP
  /* Register coredump ram disk */

  int ret = devmem_register_region("/dev/coredump", &g_coredump_region);
  if (ret < 0)
    {
      _err("ERROR: Failed to register coredump ramdev: %d\n", ret);
    }
#endif
}

#endif /* CONFIG_BOARD_EARLY_INITIALIZE */

#ifdef CONFIG_BOARD_CRASHDUMP_CUSTOM
void board_crashdump(uintptr_t sp, struct tcb_s *tcb,
                     const char *filename, int lineno,
                     const char *msg, void *regs)
{
#ifdef CONFIG_AUTOMID_TRAPINFO
  TrapInfo_GetResetInfo();
#endif
}
#endif

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE

void board_late_initialize(void)
{
  /* Perform the arch late initialization */

  up_lateinitialize();

  /* Perform the board late initialization */

  board_lateinitialize();

  /* Perform autocore initialization */

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_START_AUTOCORE)
  extern int autocore_main(int argc, char *argv[]);
  kthread_create_with_stack(CONFIG_AUTOCORE_PROGNAME,
                            CONFIG_AUTOCORE_PRIORITY,
                            autocore_stack,
                            CONFIG_AUTOCORE_STACKSIZE,
                            autocore_main, NULL);
#endif

  /* copy boot version and spbl version from Boot to Core0 app */

#if defined(CONFIG_BOOT_SHARE_RAM_DOMAIN_BOOT)
  BootVer_VerShareToApp();
#endif

#if defined(CONFIG_AUTOMID_EMG) || defined(CONFIG_AUTOMID_TSW_BOOT)
  Emg_Tsw_CallFromBoot();
#endif

#if !defined(CONFIG_TRICORE_BL) && defined(CONFIG_COREDUMP)
  tricore_save_coredump();
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef BOARD_APP_INITIALIZE

  /* Perform the arch late initialization */

  up_lateinitialize();

  /* Perform the board late initialization */

  board_lateinitialize();

  /* Perform autocore initialization */

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_START_AUTOCORE)
  extern int autocore_main(int argc, char *argv[]);
  kthread_create_with_stack(CONFIG_AUTOCORE_PROGNAME,
                            CONFIG_AUTOCORE_PRIORITY,
                            autocore_stack,
                            CONFIG_AUTOCORE_STACKSIZE,
                            autocore_main, NULL);
#endif

  /* copy boot version and spbl version from Boot to Core0 app */

#if defined(CONFIG_BOOT_SHARE_RAM_DOMAIN_BOOT)
  BootVer_VerShareToApp();
#endif

#if defined(CONFIG_AUTOMID_EMG) || defined(CONFIG_AUTOMID_TSW_BOOT)
  Emg_Tsw_CallFromBoot();
#endif

#if !defined(CONFIG_TRICORE_BL) && defined(CONFIG_COREDUMP)
  tricore_save_coredump();
#endif

#endif

  return 0;
}

/****************************************************************************
 * Name: board_app_finalinitialize
 *
 * Description:
 *   Perform application specific initialization. This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command
 *   BOARDIOC_FINALINIT.
 *
 * Input Parameters:
 *   arg - The argument has no meaning.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_FINALINIT
void weak_function up_finalinitialize(void)
{
}

int board_app_finalinitialize(uintptr_t arg)
{
  /* Perform the arch final initialization */

  up_finalinitialize();

  /* Perform the board final initialization */

  board_finalinitialize();

  return 0;
}
#endif /* CONFIG_BOARDCTL_FINALINIT */

/****************************************************************************
 * Name: board_reset_cause
 *
 * Description:
 *   Get the cause of last board reset. This should call architecture
 *   specific logic to handle the register read.
 *
 * Input Parameters:
 *   cause - Pointer to boardioc_reset_cause_s structure to which the
 *      reason (and potentially subreason) is saved.
 *
 * Returned Value:
 *   This functions should always return succesfully with 0. We save
 *   BOARDIOC_RESETCAUSE_UNKOWN in cause structure if we are
 *   not able to get last reset cause from HW (which is unlikely).
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET_CAUSE

static struct boardioc_reset_cause_s g_reset_cause;

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
static uint32_t board_sw_bootflag_get(void)
{
  uint32_t sw_bootflag = SMM_USRINFO.U & HAL_SW_BOOTFLAG_MASK;

  return sw_bootflag;
}

int board_reset_cause(struct boardioc_reset_cause_s *cause)
{
  IfxSmmRst_Reset rst_cause;

  if (g_reset_cause.cause != BOARDIOC_RESETCAUSE_NONE)
    {
      *cause = g_reset_cause;
      return 0;
    }

  /* Evaluate the last reset cause/details */

  IfxSmmRst_evaluateReset(&rst_cause);

  /* Get the reset cause from hardware */

  switch (rst_cause.highestResetType)
    {
      case IfxSmmRst_ResetType_lvd:

        /* This reset is triggered in case of power fail
         * of standby supply domains.
         */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
        break;
      case IfxSmmRst_ResetType_coldpoweron:

        /* This reset is triggered in case of power fail
         * of critical IO and core supply rails.
         */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;
        break;
      case IfxSmmRst_ResetType_warmpoweron:

        /* This reset is triggered in case that the external
         * power-on reset pin (PORST) is asserted low.
         */

        cause->cause = BOARDIOC_RESETCAUSE_PIN;
        break;
      case IfxSmmRst_ResetType_system:

        /* The system reset is triggered by software, safety alarms
         * and watchdogs through the SMU unit, ESRx reset pins, system
         * timers, TCU and debug modules.
         */

        cause->cause = BOARDIOC_RESETCAUSE_CPU_SOFT;
        cause->flag = board_sw_bootflag_get();
        break;
      case IfxSmmRst_ResetType_application:

        /* This reset ensures a fast overall microcontroller reset
         * with minimal reset time.
         */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_RWDT;
        break;
      default:

        /* Unknown cause returned from HW */

        cause->cause = BOARDIOC_RESETCAUSE_UNKOWN;
        break;
    }

#ifdef CONFIG_AURIX_AUTO_CLR_RSTCAUSE
  /* Clear all reset registers */

  IfxSmmRst_clearAllResetRegisters();
#endif /* CONFIG_AURIX_AUTO_CLR_RSTCAUSE */

  g_reset_cause = *cause;
  return 0;
}
#endif /* CONFIG_ARCH_CHIP_AURIX_TC4XX */

#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
int board_reset_cause(struct boardioc_reset_cause_s *cause)
{
  IfxScuRcu_ResetCode rst_cause;

  if (g_reset_cause.cause != BOARDIOC_RESETCAUSE_NONE)
    {
      *cause = g_reset_cause;
      return 0;
    }

  /* Evaluate the last reset cause/details */

  rst_cause = IfxScuRcu_evaluateReset();

  /* Get the reset cause from hardware */

  switch (rst_cause.resetType)
    {
      case IfxScuRcu_ResetType_coldpoweron:

        /* Cold Power On Reset */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_CHIPPOR;

        /* Clear Cold Power-On Reset sticky bits */

        IfxScuRcu_clearColdResetStatus();
        break;
      case IfxScuRcu_ResetType_warmpoweron:

        /* Warm Power On Reset */

        cause->cause = BOARDIOC_RESETCAUSE_PIN;
        break;
      case IfxScuRcu_ResetType_system:

        /* System Reset */

        cause->cause = BOARDIOC_RESETCAUSE_CPU_SOFT;
        break;
      case IfxScuRcu_ResetType_application:

        /* Application reset */

        cause->cause = BOARDIOC_RESETCAUSE_SYS_RWDT;
        break;
      default:

        /* Unknown cause returned from HW */

        cause->cause = BOARDIOC_RESETCAUSE_UNKOWN;
        break;
    }

  g_reset_cause = *cause;
  return 0;
}
#endif /* CONFIG_ARCH_CHIP_AURIX_TC3XX */

#endif /* CONFIG_BOARDCTL_RESET_CAUSE */

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset board.  This function may or may not be supported by a
 *   particular board architecture.
 *
 * Input Parameters:
 *   status - Status information provided with the reset event.  This
 *     meaning of this status information is board-specific.  If not used by
 *     a board, the value zero may be provided in calls to board_reset.
 *
 * Returned Value:
 *   If this function returns, then it was not possible to power-off the
 *   board due to some constraints.  The return value int this case is a
 *   board-specific reason for the failure to shutdown.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL_RESET

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
static void board_sw_bootflag_set(int bootflag)
{
  uint32_t sw_bootflag = SMM_USRINFO.U & ~HAL_SW_BOOTFLAG_MASK;

  SMM_USRINFO.U = sw_bootflag | (bootflag & HAL_SW_BOOTFLAG_MASK);
}
#endif

void weak_function up_reset(int status)
{
  /* The watchdog uses Application Reset by default, so we configure
   * the software reset as System Reset. TC3 related registers include
   * SCU RSTCON.SMU and RSTCON.SW. TC4 related registers include SMM
   * RSTTRIGCTRLA.SMUSAFE0AR and RSTTRIGCTRLA.SW.
   * */

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
  board_sw_bootflag_set(status);

#if (IFX_PROT_ENABLED == 1U)
  IfxApProt_setState((Ifx_PROT_PROT *)&MODULE_SMM.PROTE,
                     IfxApProt_State_config);
#endif

  /* Trigger a Software SYSTEM_RESET */

  up_flush_dcache_all();
  SMM_RSTTRIGCTRLA.B.SW   = IfxSmmRst_TriggerRstCfgType_systemReset;
  SMM_SWRSTCON.B.SWRSTREQ = 1u;
#endif

#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
  /* Get the CPU EndInit password */

  uint16 CPUEndinitPw = IfxScuWdt_getCpuWatchdogPassword();

  /* Configure the request trigger in the Reset Configuration Register */

  IfxScuRcu_configureResetRequestTrigger(IfxScuRcu_Trigger_sw,
                                         IfxScuRcu_ResetType_system);

  /* Clear CPU EndInit protection to write in the SWRSTCON register of SCU */

  IfxScuWdt_clearCpuEndinit(CPUEndinitPw);

  /* Trigger a software reset based on the configuration of RSTCON register */

  IfxCpu_triggerSwReset();
#endif

  /* Add some delay for HW to reset */

  while (1);
}

int board_reset(int status)
{
  up_reset(status);
  return 0;
}
#endif /* CONFIG_BOARDCTL_RESET */

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE
int board_boot_image(const char *path, uint32_t hdr_size)
{
  _info("\n boot to app(core0) ...\n");
  IfxCpu_disableInterrupts();

  for (int i = 0; i < NR_IRQS; i++)
    {
      if (g_irqmap[i] != 0)
        {
          up_disable_irq(i);
          IfxSrc_deinit((Ifx_SRC_SRCR *)(&SRC_CPU_CPU0_SB + i));
        }
    }

  Ifx_Ssw_jumpToFunction((void *)NC_START(CORE0_PFLASH_KERNEL_START));
  Ifx_Ssw_infiniteLoop();
  return 0;
}
#endif

#ifdef CONFIG_BOARDCTL_POWEROFF
int board_power_off(int status)
{
#  ifdef CONFIG_AURIX_PMS
  aurix_pms_standby();
#  endif
  return 0;
}
#endif

#ifdef CONFIG_BOARDCTL_START_CPU

/****************************************************************************
 * Name: board_start_cpu
 *
 * Description:
 *   This function is designed for secure boot scenarios, primarily used to
 *   release CPU0 for the CPU Control Module (CPUCs). It handles CPU core
 *   startup operations.
 *
 * Input Parameters:
 *   - cpuid: The ID of the CPU core to be started. Currently, only CPU0 is
 *    supported.
 *
 * Returned Value:
 *   Returns 0 on successful startup of the specified CPU core; returns -1
 *   and logs an error for unsupported CPU core IDs.
 *
 * Assumptions/Limitations:
 *   1. CPU0 can only be started by core6 (security core).
 *   2. Other CPU cores are started in ILLD (Integrated Low-Level Driver) and
 *      do not use this interface for now.
 *   3. CPU0 can only be started when CONFIG_CPU_COREID is configured as 6.
 *
 ****************************************************************************/

int board_start_cpu(int cpuid)
{
  switch (cpuid)
    {
#if  CONFIG_CPU_COREID == 6
      case 0:

        /* only core6(security core) could start core0 */

        uint32_t csrm2ht = IfxCscu_getHostData(&MODULE_CSCU);
        IfxCscu_writeHostData(&MODULE_CSCU, csrm2ht | 0x80000000);
        break;
#endif
      default:
        syslog(LOG_ERR, "Not support start core %d\n", cpuid);
        return -1;
    }

  return 0;
}

#endif

#if defined(CONFIG_BOARDCTL_SWITCH_BOOT) && defined(CONFIG_AURIX_UCB)
/****************************************************************************
 * Name: board_switch_boot
 *
 * Description:
 *   This function handles boot slot operations such as swapping, enabling,
 *   and disabling boot configurations based on the provided command.
 *
 * Input Parameters:
 *   - system: A command string indicating the operation (e.g., "swap",
 *   "enable", "disable", "cs_swap", "cs_enable", "cs_disable").
 *
 * Returned Value:
 *   Returns 0 on success. Possible error returns are indicated by function
 *   calls within the implementation.
 *
 * Assumptions/Limitations:
 *   Assumes that the caller provides valid commands. May not function
 *   as expected if invalid commands are passed.
 *
 ****************************************************************************/

int board_switch_boot(FAR const char *system)
{
  int status;
  int ret;

  /* Log the current command received */

  syslog(LOG_INFO, "curr cmd:%s\n", system);
  status = get_swap_status(UCB_REGION_RTC);

  /* Handle the "swap" command */

  if (!strcmp(system, "swap"))
    {
      syslog(LOG_INFO, "do swap, ota running status is %s\n",
             swap_status_str[status]);

      switch (status)
        {
          case SWAP_A_STATUS:
            syslog(LOG_INFO, "running in A slot, switching to B slot\n");
            ret = tc4_rtc_ab_swap("RTC_SWAP", SLOT_B);
            break;

          case SWAP_B_STATUS:
            syslog(LOG_INFO, "running in B slot, switching to A slot\n");
            ret = tc4_rtc_ab_swap("RTC_SWAP", SLOT_A);
            break;

          default:
            syslog(LOG_ERR,
                  "running in no A/B mode, do not support switch boot\n");
            return -1;
        }

      if (ret != 0)
        {
          syslog(LOG_ERR, "swap failed\n");
        }
      else
        {
          syslog(LOG_INFO, "swap succeeded\n");
        }
    }

  /* Handle the "enable" command */

  else if (!strcmp(system, "enable"))
    {
      if (status == NO_SWAP_STATUS)
        {
          ret = tc4_rtc_ab_swap("RTC_SWAP", SLOT_A);
          if (ret != 0)
            {
              syslog(LOG_ERR, "init swap to SLOT A failed\n");
            }
          else
            {
              syslog(LOG_INFO, "init swap to SLOT A success\n");
              ret = tc4_ab_swap_enable("RTC_USRCFG", true);
              syslog(ret != 0 ? LOG_ERR : LOG_INFO, ret != 0 ?
                     "enable A/B swap failed\n" :
                     "enable A/B swap success\n");
            }
        }
      else
        {
          syslog(LOG_INFO, "A/B swap already enabled\n");
        }
    }

  /* Handle the "disable" command */

  else if (!strcmp(system, "disable"))
    {
      syslog(LOG_INFO, "do disable swap\n");
      ret = tc4_ab_swap_enable("RTC_USRCFG", false);
      syslog(ret != 0 ? LOG_ERR : LOG_INFO, ret != 0 ?
             "disable A/B swap failed\n" : "disable A/B swap success\n");
    }

  /* Handle CS swap commands */

  else if (!strcmp(system, "cs_swap"))
    {
      status = get_swap_status(UCB_REGION_CS);
      if (status < SWAP_MAX)
        {
          syslog(LOG_INFO, "do cs swap, ota running status is %s\n", \
                 swap_status_str[status]);
        }

      switch (status)
        {
          case SWAP_A_STATUS:
            syslog(LOG_INFO, "CS running in A slot, switching to B slot\n");
            ret = tc4_rtc_ab_swap("CS_SWAP", SLOT_B);
            break;

          case SWAP_B_STATUS:
            syslog(LOG_INFO, "CS running in B slot, switching to A slot\n");
            ret = tc4_rtc_ab_swap("CS_SWAP", SLOT_A);
            break;

          default:
            syslog(LOG_INFO,
                  "CS running in no A/B mode, do not support switch boot\n");
            return -1;
        }

      /* Log success or failure of the CS swap operation */

      if (ret != 0)
        {
          syslog(LOG_ERR, "CS swap failed\n");
        }
      else
        {
          syslog(LOG_INFO, "CS swap succeeded\n");
        }
    }
  else if (!strcmp(system, "cs_enable"))
    {
      status = get_swap_status(UCB_REGION_CS);
      if (status == NO_SWAP_STATUS)
        {
          ret = tc4_rtc_ab_swap("CS_SWAP", SLOT_A);
          if (ret != 0)
            {
              syslog(LOG_ERR, "CS init swap to SLOT A failed\n");
            }
          else
            {
              syslog(LOG_INFO, "CS init swap to SLOT A success\n");
              ret = tc4_ab_swap_enable("CS_USRCFG", true);
              syslog(ret != 0 ? LOG_ERR : LOG_INFO, ret != 0 ?
                     "CS enable A/B swap failed\n" :
                     "CS enable A/B swap success\n");
            }
        }
      else
        {
          syslog(LOG_INFO, "CS A/B swap already enabled\n");
        }
    }
  else if (!strcmp(system, "cs_disable"))
    {
      syslog(LOG_INFO, "CS do disable swap\n");
      ret = tc4_ab_swap_enable("CS_USRCFG", false);
      syslog(ret != 0 ? LOG_ERR : LOG_INFO, ret != 0 ?
             "CS disable A/B swap failed\n" :
             "CS disable A/B swap success\n");
    }
  else
    {
      dump_swap_status();
    }

  return 0;
}

#endif