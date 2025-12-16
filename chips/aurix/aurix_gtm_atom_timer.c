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

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "tricore_internal.h"
#include "aurix_gtm_atom_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_gtm_atom_timer_lowerhalf_s
{
  struct timer_ops_s                          *ops;
  const struct aurix_gtm_atom_timer_config_s *config;
  IfxGtm_Atom_Timer                           timer;
  tccb_t                                       callback;
  void                                        *arg;
  bool                                         started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_gtm_atom_start(struct timer_lowerhalf_s *lower);
static int aurix_gtm_atom_stop(struct timer_lowerhalf_s *lower);
static int aurix_gtm_atom_getstatus(struct timer_lowerhalf_s *lower,
                                     struct timer_status_s *status);
static int aurix_gtm_atom_settimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t timeout);
static void aurix_gtm_atom_setcallback(struct timer_lowerhalf_s *lower,
                                        tccb_t callback, void *arg);
static int aurix_gtm_atom_maxtimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t *maxtimeout);

static int aurix_gtm_atom_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer_ops_s g_timer_ops =
{
  .start       = aurix_gtm_atom_start,
  .stop        = aurix_gtm_atom_stop,
  .getstatus   = aurix_gtm_atom_getstatus,
  .settimeout  = aurix_gtm_atom_settimeout,
  .setcallback = aurix_gtm_atom_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = aurix_gtm_atom_maxtimeout,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gtm_atom_handler
 *
 * Description:
 *   one channel timer interrupt handler of gtm-atom module
 *
 ****************************************************************************/
#define DEBUG_COUNT_LIMIT 100
static int aurix_gtm_atom_handler(int irq, void *context, void *lower)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;
  uint32_t next_interval_us = 0;
  bool notify = false;

  notify = IfxGtm_Atom_Ch_isZeroNotification(priv->timer.atom,
                                              priv->timer.timerChannel);
  if (notify)
    {
      IfxGtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                            priv->timer.timerChannel);
      if (priv->callback)
        {
          if (priv->callback(&next_interval_us, priv->arg))
            {
              if (next_interval_us > 0)
                {
                  aurix_gtm_atom_settimeout
                  ((struct timer_lowerhalf_s *)lower, next_interval_us);
                }
            }
          else
            {
              aurix_gtm_atom_stop((struct timer_lowerhalf_s *)lower);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_gtm_atom_start
 *
 * Description:
 *   Start one channel timer and reset the time
 *
 ****************************************************************************/

static int aurix_gtm_atom_start(struct timer_lowerhalf_s *lower)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;

  if (priv->started)
      return -EBUSY;

  IfxGtm_Atom_Ch_setCounterValue(priv->timer.atom,
                                  priv->timer.timerChannel, 0);
  IfxGtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                        priv->timer.timerChannel);

  IfxGtm_Atom_Timer_run(&priv->timer);

  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: aurix_gtm_atom_stop
 *
 * Description:
 *    Stop one channel timer
 *
 ****************************************************************************/

static int aurix_gtm_atom_stop(struct timer_lowerhalf_s *lower)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;

  if (!priv->started)
      return -EBUSY;

  IfxGtm_Atom_Timer_stop(&priv->timer);
  IfxGtm_Atom_Ch_clearZeroNotification(priv->timer.atom,
                                        priv->timer.timerChannel);

  priv->callback = NULL;
  priv->arg = NULL;
  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: aurix_gtm_atom_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 ****************************************************************************/

static void aurix_gtm_atom_setcallback(struct timer_lowerhalf_s *lower,
                                        tccb_t callback, void *arg)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;

  DEBUGASSERT(callback != NULL);

  priv->callback = callback;
  priv->arg      = arg;
}

/****************************************************************************
 * Name: aurix_gtm_atom_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer) in microseconds.
 *
 ****************************************************************************/

static int aurix_gtm_atom_settimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t timeout)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;
  const uint32_t maxtimeout = 0xffffff;
  uint32_t timeouttick;
  float  freq;

  IfxGtm_Atom_Ch_setCompareZero(priv->timer.atom, priv->timer.timerChannel,
                                 maxtimeout);
  freq = IfxGtm_Atom_Ch_getClockFrequency(priv->timer.gtm, priv->timer.atom,
                                           priv->timer.timerChannel);

  timeouttick = timeout * (freq / USEC_PER_SEC);
  timeouttick = timeouttick > maxtimeout ? maxtimeout : timeouttick;

  IfxGtm_Atom_Ch_setCompareZero(priv->timer.atom, priv->timer.timerChannel,
                                 timeouttick);
  IfxGtm_Atom_Ch_setCounterValue(priv->timer.atom,
          priv->timer.timerChannel, 0);

  return OK;
}

/****************************************************************************
 * Name: aurix_gtm_atom_maxtimeout
 *
 * Description:
 *   Get the maximum supported timeout value in microseconds.
 *
 ****************************************************************************/

static int aurix_gtm_atom_maxtimeout(struct timer_lowerhalf_s *lower,
                                      uint32_t *maxtimeout)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;
  const uint32_t maxtickout = 0xffffff;
  float  freq;

  freq = IfxGtm_Atom_Ch_getClockFrequency(priv->timer.gtm, priv->timer.atom,
                                           priv->timer.timerChannel);
  *maxtimeout = maxtickout / (freq / USEC_PER_SEC);

  return OK;
}

/****************************************************************************
 * Name: aurix_gtm_atom_getstatus
 *
 * Description:
 *   get timer status
 *
 * Input Parameters:
 *   lower  - A pointer of the lower-half driver.
 *   status - The location to return the status information.
 *
 ****************************************************************************/

static int aurix_gtm_atom_getstatus(struct timer_lowerhalf_s *lower,
                                     struct timer_status_s *status)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv =
    (struct aurix_gtm_atom_timer_lowerhalf_s *)lower;
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

  freq = IfxGtm_Atom_Ch_getClockFrequency(priv->timer.gtm, priv->timer.atom,
                                           priv->timer.timerChannel);
  if (freq == 0.0f)
    {
      return -EIO;
    }

  timeouttick = IfxGtm_Atom_Ch_getCompareZero(priv->timer.atom,
                                               priv->timer.timerChannel);
  clocktick_usec = freq / USEC_PER_SEC;
  status->timeout = timeouttick / clocktick_usec;

  /* Get the time remaining until the timer expires (in microseconds) */

  currenttick = *IfxGtm_Atom_Ch_getTimerPointer(priv->timer.atom,
          priv->timer.timerChannel);
  DEBUGASSERT(currenttick <= timeouttick);
  status->timeleft = (timeouttick - currenttick) / clocktick_usec;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gtm_timer_initialize
 *
 * Description:
 *   Initialize GTM ATOM Timer submodule and register the Timer device.
 *
 * Input Parameters:
 *   devs - The list of channel device to be initialized.
 *   cfgs - The list of configuration for the GTM-ATOM channel.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_gtm_timer_initialize(struct timer_lowerhalf_s **dev,
        const struct aurix_gtm_atom_timer_config_s *config,
        size_t num)
{
  struct aurix_gtm_atom_timer_lowerhalf_s *priv;
  IfxGtm_Atom_Timer_Config timerConfig;
  int i;

  for (i = 0; i < num && dev[i] != DEV_END; i++)
    {
      if (config[i].base.frequency == 0)
        {
          continue;
        }

      priv = kmm_zalloc(sizeof(struct aurix_gtm_atom_timer_lowerhalf_s));
      if (priv == NULL)
        {
          tmrerr("ERROR: timer kmm_zalloc failed\n");
          return -ENOMEM;
        }

      memset(priv, 0, sizeof(struct aurix_gtm_atom_timer_lowerhalf_s));

      IfxGtm_Atom_Timer_initConfig(&timerConfig, &MODULE_GTM);

      timerConfig.atom                  = config[i].atom;
      timerConfig.timerChannel          = config[i].channel;
      timerConfig.clock                 = config[i].clock;
      timerConfig.base.frequency        = config[i].base.frequency;
      timerConfig.base.isrPriority      = IRQ_TO_NDX(config[i].isrIrq);
      timerConfig.irqModeTimer          = config[i].irqModeTimer;
      timerConfig.irqModeTrigger        = config[i].irqModeTrigger;

      priv->config = &config[i];
      priv->ops = &g_timer_ops;

      switch (priv->config->clock)
        {
          case IfxGtm_Cmu_Clk_0:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
            break;
          case IfxGtm_Cmu_Clk_1:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);
            break;
          case IfxGtm_Cmu_Clk_2:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK2);
            break;
          case IfxGtm_Cmu_Clk_3:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK3);
            break;
          case IfxGtm_Cmu_Clk_4:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK4);
            break;
          case IfxGtm_Cmu_Clk_5:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK5);
            break;
          case IfxGtm_Cmu_Clk_6:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK6);
            break;
          case IfxGtm_Cmu_Clk_7:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK7);
            break;
          default:
            IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
        }

      IfxGtm_Atom_Timer_init(&priv->timer, &timerConfig);

      /* irq init */

      irq_attach(priv->config->isrIrq,
                 aurix_gtm_atom_handler, priv);
      up_enable_irq(priv->config->isrIrq);

      dev[priv->config->channel] = (struct timer_lowerhalf_s *)priv;

      if (timer_register(priv->config->devpath,
                         (struct timer_lowerhalf_s *)priv) == NULL)
        {
          kmm_free(priv);
          irqchain_detach(priv->config->isrIrq,
                          aurix_gtm_atom_handler, priv);
          dev[priv->config->channel] = NULL;
          tmrerr("ERROR: timer register failed:%s\n", priv->config->devpath);
          break;
        }
     }

  return OK;
}
