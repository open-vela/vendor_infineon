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

#include <arch/chip/chip.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "tricore_internal.h"
#include "aurix_egtm_atom_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_egtm_atom_timer_lowerhalf_s
{
  struct timer_ops_s                          *ops;
  const struct aurix_egtm_atom_timer_config_s *config;
  IfxEgtm_Atom_Timer                           timer;
  tccb_t                                       callback;
  void                                        *arg;
  bool                                         started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_egtm_atom_start(struct timer_lowerhalf_s *lower);
static int aurix_egtm_atom_stop(struct timer_lowerhalf_s *lower);
static int aurix_egtm_atom_getstatus(struct timer_lowerhalf_s *lower,
                                     struct timer_status_s *status);
static int aurix_egtm_atom_settimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t timeout);
static void aurix_egtm_atom_setcallback(struct timer_lowerhalf_s *lower,
                                        tccb_t callback, void *arg);
static int aurix_egtm_atom_maxtimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t *maxtimeout);

static int aurix_egtm_atom_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer_ops_s g_timer_ops =
{
  .start       = aurix_egtm_atom_start,
  .stop        = aurix_egtm_atom_stop,
  .getstatus   = aurix_egtm_atom_getstatus,
  .settimeout  = aurix_egtm_atom_settimeout,
  .setcallback = aurix_egtm_atom_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = aurix_egtm_atom_maxtimeout,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_atom_handler
 *
 * Description:
 *   one channel timer interrupt handler of egtm-atom module
 *
 ****************************************************************************/

static int aurix_egtm_atom_handler(int irq, void *context, void *lower)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;
  uint32_t next_interval_us = 0;
  bool notify = false;

  notify = IfxEgtm_Atom_Ch_isZeroNotification(priv->timer.atom,
                                              priv->timer.timerChannel);
  if (notify)
    {
      IfxEgtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                            priv->timer.timerChannel);
      if (priv->callback)
        {
          if (priv->callback(&next_interval_us, priv->arg))
            {
              if (next_interval_us > 0)
                {
                  aurix_egtm_atom_settimeout(
                    (struct timer_lowerhalf_s *)lower,
                    next_interval_us);
                }
            }
          else
            {
              aurix_egtm_atom_stop((struct timer_lowerhalf_s *)lower);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_atom_start
 *
 * Description:
 *   Start one channel timer and reset the time
 *
 ****************************************************************************/

static int aurix_egtm_atom_start(struct timer_lowerhalf_s *lower)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;

  if (priv->started)
    {
      return -EBUSY;
    }

  IfxEgtm_Atom_Ch_setCounterValue(priv->timer.atom,
                                  priv->timer.timerChannel, 0);
  IfxEgtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                        priv->timer.timerChannel);

  IfxEgtm_Atom_Timer_run(&priv->timer);

  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_atom_stop
 *
 * Description:
 *    Stop one channel timer
 *
 ****************************************************************************/

static int aurix_egtm_atom_stop(struct timer_lowerhalf_s *lower)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;

  if (!priv->started)
    {
      return -EBUSY;
    }

  IfxEgtm_Atom_Timer_stop(&priv->timer);
  IfxEgtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                        priv->timer.timerChannel);

  priv->callback = NULL;
  priv->arg = NULL;
  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_atom_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 ****************************************************************************/

static void aurix_egtm_atom_setcallback(struct timer_lowerhalf_s *lower,
                                        tccb_t callback, void *arg)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;

  DEBUGASSERT(callback != NULL);

  priv->callback = callback;
  priv->arg      = arg;
}

/****************************************************************************
 * Name: aurix_egtm_atom_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer) in microseconds.
 *
 ****************************************************************************/

static int aurix_egtm_atom_settimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t timeout)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;
  const uint32_t maxtimeout = 0xffffff;
  uint32_t timeouttick;
  float  freq;

  IfxEgtm_Atom_Ch_setCompareZero(priv->timer.atom,
                                 priv->timer.timerChannel,
                                 maxtimeout);
  freq = IfxEgtm_Atom_Ch_getClockFrequency(priv->timer.egtm,
                                           priv->timer.atom,
                                           priv->timer.timerChannel);

  timeouttick = timeout * (freq / USEC_PER_SEC);
  timeouttick = timeouttick > maxtimeout ? maxtimeout : timeouttick;

  IfxEgtm_Atom_Ch_setCompareZero(priv->timer.atom,
                                 priv->timer.timerChannel,
                                 timeouttick);
  IfxEgtm_Atom_Ch_setCounterValue(priv->timer.atom,
                                  priv->timer.timerChannel,
                                  0);

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_atom_maxtimeout
 *
 * Description:
 *   Get the maximum supported timeout value in microseconds.
 *
 ****************************************************************************/

static int aurix_egtm_atom_maxtimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t *maxtimeout)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;
  const uint32_t maxtickout = 0xffffff;
  float  freq;

  freq = IfxEgtm_Atom_Ch_getClockFrequency(priv->timer.egtm,
                                           priv->timer.atom,
                                           priv->timer.timerChannel);
  if (freq == 0.0f)
    {
      return -EIO;
    }

  *maxtimeout = maxtickout / (freq / USEC_PER_SEC);

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_atom_getstatus
 *
 * Description:
 *   get timer status
 *
 * Input Parameters:
 *   lower  - A pointer of the lower-half driver.
 *   status - The location to return the status information.
 *
 ****************************************************************************/

static int aurix_egtm_atom_getstatus(struct timer_lowerhalf_s *lower,
                                     struct timer_status_s *status)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_egtm_atom_timer_lowerhalf_s *)lower;
  uint32_t timeouttick;
  uint32_t currenttick;
  float  freq;
  float  clocktick_usec;

  /* Get the status bit */

  status->flags = 0;

  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  /* Get timeout */

  freq = IfxEgtm_Atom_Ch_getClockFrequency(priv->timer.egtm,
                                           priv->timer.atom,
                                           priv->timer.timerChannel);
  if (freq == 0.0f)
    {
      return -EIO;
    }

  timeouttick = IfxEgtm_Atom_Ch_getCompareZero(priv->timer.atom,
                                               priv->timer.timerChannel);
  clocktick_usec = freq / USEC_PER_SEC;
  status->timeout = timeouttick / clocktick_usec;

  /* Get the time remaining until the timer expires (in microseconds) */

  currenttick = *IfxEgtm_Atom_Ch_getTimerPointer(priv->timer.atom,
                                                 priv->timer.timerChannel);
  DEBUGASSERT(currenttick <= timeouttick);
  status->timeleft = (timeouttick - currenttick) / clocktick_usec;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_timer_initialize
 *
 * Description:
 *   Initialize EGTM ATOM Timer submodule and register the Timer device.
 *
 * Input Parameters:
 *   devs - The list of channel device to be initialized.
 *   cfgs - The list of configuration for the eGTM-ATOM channel.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_egtm_timer_initialize(
  struct timer_lowerhalf_s **dev,
  const struct aurix_egtm_atom_timer_config_s *config,
  size_t num)
{
  struct aurix_egtm_atom_timer_lowerhalf_s *priv;
  IfxEgtm_Atom_Timer_Config timerConfig;
  int i;

  for (i = 0; i < num && dev[i] != DEV_END; i++)
    {
      if (config[i].frequency == 0)
        {
          continue;
        }

      priv = kmm_zalloc(sizeof(struct aurix_egtm_atom_timer_lowerhalf_s));
      if (priv == NULL)
        {
          tmrerr("ERROR: timer kmm_zalloc failed\n");
          return -ENOMEM;
        }

      memset(priv, 0, sizeof(struct aurix_egtm_atom_timer_lowerhalf_s));

      IfxEgtm_Atom_Timer_initConfig(&timerConfig, &MODULE_EGTM);

      timerConfig.cluster               = config[i].cluster;
      timerConfig.timerChannel          = config[i].channel;
      timerConfig.clock                 = config[i].clock;
      timerConfig.frequency             = config[i].frequency;
      timerConfig.interrupt.isrPriority = IRQ_TO_NDX(config[i].isrIrq);
      timerConfig.interrupt.irqMode     = config[i].interrupt.irqMode;

      priv->config = &config[i];
      priv->ops = &g_timer_ops;

      switch (priv->config->clock)
        {
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk0:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK0);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk1:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK1);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk2:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK2);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk3:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK3);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk4:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK4);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk5:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK5);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk6:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK6);
            break;
          case IfxEgtm_Atom_Ch_ClkSrc_cmuclk7:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK7);
            break;
          default:
            IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK0);
        }

      IfxEgtm_Atom_Timer_init(&priv->timer, &timerConfig);

      if (config[i].oneshotmode == TRUE)
        {
          IfxEgtm_Atom_Ch_setOneShotMode(priv->timer.atom,
            priv->timer.timerChannel, TRUE);
        }

      /* irq init */

      irq_attach(priv->config->isrIrq,
                 aurix_egtm_atom_handler, priv);
      up_enable_irq(priv->config->isrIrq);

      dev[priv->config->channel] = (struct timer_lowerhalf_s *)priv;

      if (timer_register(priv->config->devpath,
                         (struct timer_lowerhalf_s *)priv) == NULL)
        {
          irqchain_detach(priv->config->isrIrq,
                          aurix_egtm_atom_handler, priv);
          dev[priv->config->channel] = NULL;
          tmrerr("ERROR: timer register failed:%s\n", priv->config->devpath);
          kmm_free(priv);
          break;
        }
    }

  return OK;
}
