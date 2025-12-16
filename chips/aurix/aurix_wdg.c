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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>

#include "aurix_wdg.h"
#include "Clock/Std/IfxClock.h"
#include "Cpu/Std/IfxCpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum period value in cycles */

#define WDG_TIMER_PERIOD_MAX  (0xFFFF)

/* The watchdog alram which RT monitoring */

#define SMU_RECOVERY_TIMER0   (0x1 << 0)
#define SMU_RECOVERY_TIMER1   (0x1 << 1)
#define SMU_RECOVERY_CS_RT    (0x1 << 2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The "lower-half" driver state structure */

struct aurix_wdg_lowerhalf_s
{
  const struct watchdog_ops_s *ops;       /* Lower half operations */
  IfxWtu_Config                wtucfg;    /* Watchdog configuration */
  uint32_t                     clkfreq;   /* Clock frequency */
  uint32_t                     timeout;   /* The actual selected timeout */
  xcpt_t                       handler;   /* The user handler for timeout */
  uint32_t                     lastreset; /* The last reset time */
  bool                         started;   /* Watchdog is started or not */
  uint8_t                      prescaler; /* Clock prescaler value */
  uint8_t                      rtimer;    /* Recovery timer */
  uint16_t                     reload;    /* Timer reload value */
  const void                  *privinfo;  /* Private info */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* The "Lower half" driver methods */

static int wdg_start(struct watchdog_lowerhalf_s *lower);
static int wdg_stop(struct watchdog_lowerhalf_s *lower);
static int wdg_keepalive(struct watchdog_lowerhalf_s *lower);
static int wdg_getstatus(struct watchdog_lowerhalf_s *lower,
                         struct watchdog_status_s *status);
static int wdg_settimeout(struct watchdog_lowerhalf_s *lower,
                          uint32_t timeout);
static xcpt_t wdg_capture(struct watchdog_lowerhalf_s *lower,
                          xcpt_t handler);
static int wdg_ioctl(struct watchdog_lowerhalf_s *lower,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = wdg_start,
  .stop       = wdg_stop,
  .keepalive  = wdg_keepalive,
  .getstatus  = wdg_getstatus,
  .settimeout = wdg_settimeout,
  .capture    = wdg_capture,
  .ioctl      = wdg_ioctl,
};

/* The initialized instance list of WTU */

static struct aurix_wdg_lowerhalf_s *g_wdglower[AURIX_WDG_NINSTANCES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wdg_start
 *
 * Description:
 *   Start the watchdog timer, register a callback if there is one and
 *   enables interrupt, otherwise, configure it to reset system on
 *   expiration.
 *
 * Input Parameters:
 *   lower         - A pointer the publicly visible representation of the
 *                   "lower-half" driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdg_start(struct watchdog_lowerhalf_s *lower)
{
  struct aurix_wdg_lowerhalf_s *priv;
  const struct aurix_wdg_config_s *cfg;

  priv = (struct aurix_wdg_lowerhalf_s *)lower;
  cfg = (struct aurix_wdg_config_s *)priv->privinfo;

  /* Check the program execution environment */

  if (cfg->wdt_idx < AURIX_WDG_WDTCPU5 &&
      cfg->wdt_idx != (uint32)IfxCpu_getCoreIndex())
    {
      /* Return EPERM to indicate that the access is not permitted */

      return -EPERM;
    }

  if (priv->started)
    {
      /* Return EBUSY to indicate that the timer was already running */

      return -EBUSY;
    }

  /* Mark the watchdog timer has been started */

  priv->started = true;
  priv->lastreset = clock_systime_ticks();

  /* Initialize the Watchdog configuration structure */

  IfxWtu_initConfig(&priv->wtucfg);

  /* Set the clock prescaler and reload value */

  priv->prescaler = IfxWtu_IFSR_divBy16384;
  priv->clkfreq = IfxClock_getSpbFrequency() / 16384;
  priv->reload = (priv->timeout * priv->clkfreq) / 1000;
  priv->reload = WDG_TIMER_PERIOD_MAX - priv->reload;
  priv->wtucfg.reload = priv->reload;
  priv->wtucfg.inputFrequency = priv->prescaler;

  /* Initialize the Watchdog with the given configuration */

  if (cfg->wdt_idx == AURIX_WDG_WDTSYS)
    {
      IfxWtu_initSystemWatchdog(cfg->wdt_ptr, &priv->wtucfg);
    }
  else if (cfg->wdt_idx == AURIX_WDG_WDTSEC)
    {
      IfxWtu_initSecurityWatchdog(cfg->wdt_ptr, &priv->wtucfg);
    }
  else
    {
      IfxWtu_initCpuWatchdog(cfg->wdt_ptr, &priv->wtucfg);
    }

  return OK;
}

/****************************************************************************
 * Name: wdg_stop
 *
 * Description:
 *   Stop the watchdog.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int wdg_stop(struct watchdog_lowerhalf_s *lower)
{
  struct aurix_wdg_lowerhalf_s *priv;
  const struct aurix_wdg_config_s *cfg;

  priv = (struct aurix_wdg_lowerhalf_s *)lower;
  cfg = (struct aurix_wdg_config_s *)priv->privinfo;

  /* Check the program execution environment */

  if (cfg->wdt_idx < AURIX_WDG_WDTCPU5 &&
      cfg->wdt_idx != (uint32)IfxCpu_getCoreIndex())
    {
      /* Return EPERM to indicate that the access is not permitted */

      return -EPERM;
    }

  /* Disable the watchdog instance */

  if (cfg->wdt_idx == AURIX_WDG_WDTSYS)
    {
      IfxWtu_disableSystemWatchdog(IfxWtu_getSystemWatchdogPassword());
    }
  else if (cfg->wdt_idx == AURIX_WDG_WDTSEC)
    {
      IfxWtu_disableSecurityWatchdog(IfxWtu_getSecurityWatchdogPassword());
    }
  else
    {
      IfxWtu_disableCpuWatchdog(IfxWtu_getCpuWatchdogPassword());
    }

  /* Mark the watchdog timer has been stopped */

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: wdg_keepalive
 *
 * Description:
 *   Reset the watchdog to keep it running.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *
 * Returned Value:
 *   Error status. Returns an IO error if the watchdog is not
 *   running, otherwise returns OK.
 *
 ****************************************************************************/

int wdg_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct aurix_wdg_lowerhalf_s *priv;
  const struct aurix_wdg_config_s *cfg;

  priv = (struct aurix_wdg_lowerhalf_s *)lower;
  cfg = (struct aurix_wdg_config_s *)priv->privinfo;

  /* Check the program execution environment */

  if (cfg->wdt_idx < AURIX_WDG_WDTCPU5 &&
      cfg->wdt_idx != (uint32)IfxCpu_getCoreIndex())
    {
      /* Return EPERM to indicate that the access is not permitted */

      return -EPERM;
    }

  /* Service the watchdog immediately */

  if (cfg->wdt_idx == AURIX_WDG_WDTSYS)
    {
      IfxWtu_serviceSystemWatchdog(IfxWtu_getSystemWatchdogPassword());
    }
  else if (cfg->wdt_idx == AURIX_WDG_WDTSEC)
    {
      IfxWtu_serviceSecurityWatchdog(IfxWtu_getSecurityWatchdogPassword());
    }
  else
    {
      IfxWtu_serviceCpuWatchdog(IfxWtu_getCpuWatchdogPassword());
    }

  return OK;
}

/****************************************************************************
 * Name: wdg_getstatus
 *
 * Description:
 *   Get current watchdog status. Returns to status parameter.
 *
 * Input parameters:
 *   lower  - A pointer to the lower-half driver of the watchdog.
 *   status - Return location for the watchdog status.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int wdg_getstatus(struct watchdog_lowerhalf_s *lower,
                  struct watchdog_status_s *status)
{
  uint32_t ticks;
  uint32_t elapsed;
  struct aurix_wdg_lowerhalf_s *priv;
  const struct aurix_wdg_config_s *cfg;

  priv = (struct aurix_wdg_lowerhalf_s *)lower;
  cfg = (struct aurix_wdg_config_s *)priv->privinfo;

  /* Check the program execution environment */

  if (cfg->wdt_idx < AURIX_WDG_WDTCPU5 &&
      cfg->wdt_idx != (uint32)IfxCpu_getCoreIndex())
    {
      /* Return EPERM to indicate that the access is not permitted */

      return -EPERM;
    }

  /* Clear the watchdog status flags */

  status->flags = 0;

  /* If no handler was settled, then RESET on expiration.
   * Otherwise, call the user handler.
   */

  if (priv->handler == NULL)
    {
      status->flags |= WDFLAGS_RESET;
    }
  else
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  /* Return the current timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */

  ticks = clock_systime_ticks() - priv->lastreset;
  elapsed = (uint32_t)TICK2MSEC(ticks);

  if (elapsed < priv->timeout)
    {
      /* Return the approximate time until the watchdog timer expiration */

      status->timeleft = priv->timeout - elapsed;
    }
  else
    {
      status->timeleft = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: wdg_settimeout
 *
 * Description:
 *   Set a new timeout value and reset the watchdog.
 *
 * Input parameters:
 *   lower   - A pointer to the lower-half driver of the watchdog.
 *   timeout - Watchdog timeout, in milliseconds.
 *
 * Returned Value:
 *   Error status. Always returns OK.
 *
 ****************************************************************************/

int wdg_settimeout(struct watchdog_lowerhalf_s *lower,
                   uint32_t timeout)
{
  struct aurix_wdg_lowerhalf_s *priv;
  const struct aurix_wdg_config_s *cfg;

  priv = (struct aurix_wdg_lowerhalf_s *)lower;
  cfg = (struct aurix_wdg_config_s *)priv->privinfo;

  /* Check the program execution environment */

  if (cfg->wdt_idx < AURIX_WDG_WDTCPU5 &&
      cfg->wdt_idx != (uint32)IfxCpu_getCoreIndex())
    {
      /* Return EPERM to indicate that the access is not permitted */

      return -EPERM;
    }

  /* Calculate the new reload value */

  priv->timeout = timeout;
  priv->reload = (priv->timeout * priv->clkfreq) / 1000;
  priv->reload = WDG_TIMER_PERIOD_MAX - priv->reload;
  priv->wtucfg.reload = priv->reload;

  /* Set the new reload value */

  if (cfg->wdt_idx == AURIX_WDG_WDTSYS)
    {
      IfxWtu_changeSystemWatchdogReload(
        IfxWtu_getSystemWatchdogPassword(), priv->reload);
    }
  else if (cfg->wdt_idx == AURIX_WDG_WDTSEC)
    {
      IfxWtu_changeSecurityWatchdogReload(
        IfxWtu_getSecurityWatchdogPassword(), priv->reload);
    }
  else
    {
      IfxWtu_changeCpuWatchdogReload(
        IfxWtu_getCpuWatchdogPassword(), priv->reload);
    }

  return OK;
}

/****************************************************************************
 * Name: wdg_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of
 *                the "lower-half" driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Value:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t wdg_capture(struct watchdog_lowerhalf_s *lower,
                          xcpt_t handler)
{
  struct aurix_wdg_lowerhalf_s *priv =
    (struct aurix_wdg_lowerhalf_s *)lower;
  irqstate_t flags;
  xcpt_t oldhandler;

  wdinfo("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = enter_critical_section();
  oldhandler = priv->handler;
  priv->handler = handler;
  leave_critical_section(flags);

  return oldhandler;
}

/****************************************************************************
 * Name: wdg_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   cmd   - The ioctol command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int wdg_ioctl(struct watchdog_lowerhalf_s *lower,
                     int cmd, unsigned long arg)
{
  wdinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* No ioctls are supported */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_wdg_interrupt
 *
 * Description:
 *   Process MNI exception triggered by the watchdog unit
 *
 ****************************************************************************/

void aurix_wdg_interrupt(void)
{
  struct aurix_wdg_lowerhalf_s *lower = NULL;
  Ifx_WTU *wtu = &MODULE_WTU;
  uint32_t status;
  uint8_t coreid;
  uint8_t handled = 0;

  /* Get the cpu watchdog status */

  coreid = IfxCpu_getCoreIndex();
  if (coreid <= AURIX_WDG_WDTCPU5)
    {
      status = wtu->WDTCPU[coreid].STAT.U;
      if (status & (0x1 << 2))
        {
          lower = g_wdglower[coreid];
        }
    }
  else if (coreid == AURIX_WDG_WDTSEC)
    {
      status = wtu->WDTSEC.STAT.U;
      if (status & (0x1 << 2))
        {
          lower = g_wdglower[AURIX_WDG_WDTSEC];
        }
    }

  if (lower && lower->handler != NULL)
    {
      handled |= lower->rtimer;
      wdg_stop((struct watchdog_lowerhalf_s *)lower);
      lower->handler(0, NULL, lower);
    }

  /* Get the system watchdog status */

  status = wtu->WDTSYS.STAT.U;
  if (status & (0x1 << 2))
    {
      lower = g_wdglower[AURIX_WDG_WDTSYS];
      if (lower && lower->handler != NULL)
        {
          handled |= lower->rtimer;
          wdg_stop((struct watchdog_lowerhalf_s *)lower);
          lower->handler(0, NULL, lower);
        }
    }

  /* Write CMD for Stop Recovery Timer */

  if (handled)
    {
      Ifx_SMU_SAFE_CMD cmdSfr;
      Ifx_SMU_CS_CMD   cmdSfrCs;

      if (handled & SMU_RECOVERY_TIMER0)
        {
          cmdSfr.U                 = MODULE_SMU.SAFE[0].CMD.U;
          cmdSfr.B.CMD             = IfxSmu_Command_stopRT;
          cmdSfr.B.ARG             = 0;
          MODULE_SMU.SAFE[0].CMD.U = cmdSfr.U;
        }

      if (handled & SMU_RECOVERY_TIMER1)
        {
          cmdSfr.U                 = MODULE_SMU.SAFE[1].CMD.U;
          cmdSfr.B.CMD             = IfxSmu_Command_stopRT;
          cmdSfr.B.ARG             = 1;
          MODULE_SMU.SAFE[1].CMD.U = cmdSfr.U;
        }

      if (handled & SMU_RECOVERY_CS_RT)
        {
          cmdSfrCs.U          = MODULE_SMU.CS.CMD.U;
          cmdSfrCs.B.CMD      = IfxSmu_Command_stopRT;
          cmdSfrCs.B.ARG      = 0;
          MODULE_SMU.CS.CMD.U = cmdSfrCs.U;
        }
    }
}

/****************************************************************************
 * Name: aurix_wdg_initialize
 *
 * Description:
 *   Initialize the watchdog timer.
 *
 * Input Parameters:
 *   config - The configuration for the watchdog timer.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int aurix_wdg_initialize(const struct aurix_wdg_config_s *config)
{
  static int smu_rtime0_inited = 0;
  static int smu_rtime1_inited = 0;
  static int smu_csrt0_inited = 0;
  struct aurix_wdg_lowerhalf_s *lower;
  void *upper;

  /* Sanity check */

  DEBUGASSERT(config);
  DEBUGASSERT(config->wdt_idx < AURIX_WDG_NINSTANCES);

  /* Allocate a new lower half instance */

  lower = kmm_zalloc(sizeof(struct aurix_wdg_lowerhalf_s));
  if (!lower)
    {
      wderr("ERROR: Failed to allocate memory for the watchdog\n");
      return -ENOMEM;
    }

  /* Initialize the lower half instance */

  lower->ops      = &g_wdgops;
  lower->started  = false;
  lower->privinfo = config;
  lower->handler  = NULL;
  lower->rtimer   = 0;

  /* Set the SMU recovery time */

  if ((config->wdt_idx == AURIX_WDG_WDTCPU0) ||
      (config->wdt_idx == AURIX_WDG_WDTCPU1) ||
      (config->wdt_idx == AURIX_WDG_WDTCPU2) ||
      (config->wdt_idx == AURIX_WDG_WDTSYS))
    {
      /* The Recovery Timer 0 is configured to service WDT overflow
       * or access error alarms for CPU0 WDT, CPU1 WDT, CPU2 WDT and
       * System WDT.
       */

      lower->rtimer = SMU_RECOVERY_TIMER0;

      /* Set an appropriate timeout for the NMI handler. */

      if (smu_rtime0_inited == 0)
        {
          Ifx_SMU_SAFE_RTC rtcSfr0;

          smu_rtime0_inited = 1;

          rtcSfr0.U     = MODULE_SMU.SAFE[0].RTC.U;
          rtcSfr0.B.RTD = 0xffffff;

          MODULE_SMU.SAFE[0].KEYS.U = (uint32)0x000000bc;
          MODULE_SMU.SAFE[0].RTC.U  = rtcSfr0.U;
          MODULE_SMU.SAFE[0].KEYS.U = 0;
        }
    }
  else if ((config->wdt_idx == AURIX_WDG_WDTCPU3) ||
           (config->wdt_idx == AURIX_WDG_WDTCPU4) ||
           (config->wdt_idx == AURIX_WDG_WDTCPU5))
    {
      /* The Recovery Timer 1 is configured to service WDT overflow
       * or access error alarms for CPU3 WDT, CPU4 WDT and CPU5 WDT.
       */

      lower->rtimer = SMU_RECOVERY_TIMER1;

      /* Set an appropriate timeout for the NMI handler. */

      if (smu_rtime1_inited == 0)
        {
          Ifx_SMU_SAFE_RTC rtcSfr1;

          smu_rtime1_inited = 1;

          rtcSfr1.U     = MODULE_SMU.SAFE[1].RTC.U;
          rtcSfr1.B.RTD = 0xffffff;

          MODULE_SMU.SAFE[1].KEYS.U = (uint32)0x000000bc;
          MODULE_SMU.SAFE[1].RTC.U  = rtcSfr1.U;
          MODULE_SMU.SAFE[1].KEYS.U = 0;
        }
    }
  else if (config->wdt_idx == AURIX_WDG_WDTSEC)
    {
      /* In SMU_CS one instance (RT0) is available. The recovery timer
       * duration is configured in the register CS_RTC. It is possible
       * to enable or disable the RT0, however RT0 is enabled by default
       * as it is required for the operation of the CPUcs watchdog.
       */

      lower->rtimer = SMU_RECOVERY_CS_RT;

      /* Set an appropriate timeout for the NMI handler. */

      if (smu_csrt0_inited == 0)
        {
          Ifx_SMU_CS_RTC   rtcSfrCs;

          smu_csrt0_inited = 1;

          rtcSfrCs.U     = MODULE_SMU.CS.RTC.U;
          rtcSfrCs.B.RTD = 0xffffff;

          MODULE_SMU.CS.KEYS.U = (uint32)0x000000bc;
          MODULE_SMU.CS.RTC.U  = rtcSfrCs.U;
          MODULE_SMU.CS.KEYS.U = 0;
        }
    }

  /* Register the watchdog driver */

  upper = watchdog_register(config->devpath,
                          (struct watchdog_lowerhalf_s *)lower);
  if (upper == NULL)
    {
      wderr("ERROR: Failed to register the watchdog driver\n");
      kmm_free(lower);
      return ERROR;
    }

  g_wdglower[config->wdt_idx] = lower;
  return OK;
}

/****************************************************************************
 * Name: aurix_wdg_all_initialize
 *
 * Description:
 *   Initialize the watchdog timer.
 *
 * Input Parameters:
 *   cfgs  - The list of configuration for the watchdog timer.
 *   count - The number of watchdog timer to initialize.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int aurix_wdg_all_initialize(const struct aurix_wdg_config_s *cfgs,
                             size_t count)
{
  size_t i;

  /* Sanity check */

  DEBUGASSERT(cfgs && count);

  /* Initialize all watchdog timer */

  for (i = 0; i < count; i++)
    {
      if (aurix_wdg_initialize(&cfgs[i]) < 0)
        {
          wderr("ERROR: Failed to initialize the watchdog timer\n");
          return ERROR;
        }
    }

  return OK;
}
