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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#ifdef CONFIG_AURIX_ENET_PTP
#include <nuttx/timers/ptp_clock.h>
#endif

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <arch/chip/chip.h>
#include <nuttx/timers/timer.h>
#include <nuttx/kmalloc.h>

#include "aurix_enet.h"
#include "Egtm/Tom/Timer/IfxEgtm_Tom_Timer.h"
#include "tricore_internal.h"
#include "aurix_qbv_schedu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define EGTM Tom info */

#define EGTMCLSNUM                       1
#define EGTMTOMTGC0                      0
#define EGTMTOMTGC1                      1
#define QBVTASKTESTPIN                   &MODULE_P13, 0
#define TRICORE_IRQ_GET(SRC_ADDR) (((uintptr_t)&SRC_ADDR - (uintptr_t)&SRC_CPU0_SB) / 4)

/* Define QBVGATE info */

#define QBVGATEOFFSET                    (0)
#define QBVGATECYCLE                     1000000
#define QBVINTERRUPTCPU                  IfxSrc_Tos_cpu1
#define QBVTOMSTOPMASK                   0x40
#define QBVTOMZEROMASK                   0
#define QBVTOMPROTECTTS                  10000
#define QBVDEFFREQ                       1000.0f

struct aurix_ethernet_qbvgate_timer_lowerhalf_s
{
  struct timer_ops_s                                 *ops;
  const struct aurix_qbvsch_timer_cfg_s              *config;
  IfxEgtm_Tom_Timer                                  timer;
  tccb_t                                             callback;
  void                                               *arg;
  bool                                               started;
};

static int aurix_qbv_schedule_start(struct timer_lowerhalf_s *lower);
static int aurix_qbv_schedule_stop(struct timer_lowerhalf_s *lower);
static void aurix_qbv_schedule_setcallback(struct timer_lowerhalf_s *lower,
                                           tccb_t callback, void *arg);
static int aurix_qbv_schedule_getstatus(struct timer_lowerhalf_s *lower,
                                        struct timer_status_s *status);
static int aurix_qbv_schedule_settimeout(struct timer_lowerhalf_s *lower,
                                         uint32_t timeout);
static int aurix_qbv_schedule_maxtimeout(struct timer_lowerhalf_s *lower,
                                         uint32_t *maxtimeout);
extern struct qbv_config g_qbv_config;
unsigned int accp_qbvsch_offset_mac_time;

static struct timer_ops_s g_qbv_schedule_ops =
{
  .start       = aurix_qbv_schedule_start,
  .stop        = aurix_qbv_schedule_stop,
  .getstatus   = aurix_qbv_schedule_getstatus,
  .settimeout  = aurix_qbv_schedule_settimeout,
  .setcallback = aurix_qbv_schedule_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = aurix_qbv_schedule_maxtimeout,
};

/****************************************************************************
 * Name: aurix_qbv_schedule_getstatus
 *
 * Description:
 *   Gets the status of the current TOM timer.
 *
 ****************************************************************************/

static int aurix_qbv_schedule_getstatus(struct timer_lowerhalf_s *lower,
                                        struct timer_status_s *status)
{
  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv =
    (struct aurix_ethernet_qbvgate_timer_lowerhalf_s *)lower;

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

  return OK;
}

/****************************************************************************
 * Name: aurix_qbv_schedule_settimeout
 *
 * Description:
 *   The maximum timeout period is set.
 *
 ****************************************************************************/

static int aurix_qbv_schedule_settimeout(struct timer_lowerhalf_s *lower,
                                         uint32_t timeout)
{
  /* QBV Schedu Need 1000000ns Cycle */

  timeout = QBVGATECYCLE;

  return OK;
}

/****************************************************************************
 * Name: aurix_qbv_schedule_maxtimeout
 *
 * Description:
 *   The maximum timeout period is get.
 *
 ****************************************************************************/

static int aurix_qbv_schedule_maxtimeout(struct timer_lowerhalf_s *lower,
                                         uint32_t *maxtimeout)
{
  /* QBV Schedu Only Support 1000000ns Cycle */

  *maxtimeout = QBVGATECYCLE;

  return OK;
}

/****************************************************************************
 * Name: get_remaining_time
 *
 * Description:
 *   Gets the current MAC time.
 *
 ****************************************************************************/

unsigned int get_remaining_time(void)
{
  int buf_mac_nanosecond;
  if (MODULE_GETH0.CLC.B.DISR)
    {
      return 0;
    }

  buf_mac_nanosecond = aurix_enet_get_mac_time_ns(IfxGeth_PortIndex_1);

  if (buf_mac_nanosecond > 0)
    {
      return (unsigned int)buf_mac_nanosecond;
    }

  return 0;
}

/****************************************************************************
 * Name: rem_settime
 *
 * Description:
 *   Set the timeout period for the next task.
 *
 ****************************************************************************/

void rem_settime(Ifx_EGTM_CLS_TOM *tom, int ns, unsigned char ch)
{
  if (ns < QBVTOMPROTECTTS)
    {
      ns += QBVGATECYCLE;
    }
  else if (ns > (2 * QBVGATECYCLE))
    {
      ns = 2 * QBVGATECYCLE;
    }

  IfxEgtm_Tom_Ch_setCompare(tom, ch,
      (uint16) (ns / (g_qbv_config.egtmcomparenum)),
      (uint16) (ns / (g_qbv_config.egtmcomparenum)));
  IfxEgtm_Tom_Ch_setCompareShadow(tom, ch,
      (uint16) (ns / (g_qbv_config.egtmcomparenum)),
      (uint16)(ns / (g_qbv_config.egtmcomparenum)));
  IfxEgtm_Tom_Ch_setCounterValue(tom, ch, FALSE);
  if (ch < 8)
    {
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[0],
                                     1 << ch, 0, TRUE);
    }
  else
    {
      ch -= 8;
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[1],
                                     1 << ch, 0, TRUE);
    }
}

/****************************************************************************
 * Name: aurix_qbv_schedule_start
 *
 * Description:
 *   Enable real-time task scheduling.
 *
 ****************************************************************************/

static int aurix_qbv_schedule_start(struct timer_lowerhalf_s *lower)
{
  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv =
    (struct aurix_ethernet_qbvgate_timer_lowerhalf_s *)lower;

  if (priv->started)
    return -EBUSY;

  if (aurix_enet_mac_done(IfxGeth_PortIndex_1) == true)
    {
      rem_settime(&priv->timer.egtm->CLS[EGTMCLSNUM].TOM,
              QBVGATECYCLE - ((get_remaining_time() + QBVGATEOFFSET)
              % QBVGATECYCLE), g_qbv_config.qbvtomtimer0);
    }
  else
    {
      rem_settime(&priv->timer.egtm->CLS[EGTMCLSNUM].TOM,
              QBVGATECYCLE - 0
              , g_qbv_config.qbvtomtimer0);
    }

  priv->started = true;

  return OK;
}

/****************************************************************************
 * Name: aurix_qbv_schedule_stop
 *
 * Description:
 *   Disable task scheduling.
 *
 ****************************************************************************/

static int aurix_qbv_schedule_stop(struct timer_lowerhalf_s *lower)
{
  int tom_timer_channel = g_qbv_config.qbvtomtimer0;

  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv =
    (struct aurix_ethernet_qbvgate_timer_lowerhalf_s *)lower;

  if (tom_timer_channel < 8)
    {
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[0],
                                     0, 1 << tom_timer_channel, TRUE);
    }
  else
    {
      tom_timer_channel -= 8;
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[1],
                                     0, 1 << tom_timer_channel, TRUE);
    }

  priv->started = false;

  return OK;
}

/****************************************************************************
 * Name: aurix_qbv_schedule_setcallback
 *
 * Description:
 *   Call this user provided timeout callback.
 *
 ****************************************************************************/

static void aurix_qbv_schedule_setcallback(struct timer_lowerhalf_s *lower,
                                           tccb_t callback, void *arg)
{
  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv =
  (struct aurix_ethernet_qbvgate_timer_lowerhalf_s *)lower;

  DEBUGASSERT(callback != NULL);

  priv->callback = callback;
  priv->arg      = arg;
}

/****************************************************************************
 * Name: interrupt_tom
 *
 * Description:
 *   This is the interrupt function of the Tom timer.
 *
 ****************************************************************************/

static int interrupt_tom(int irq, void *context, void *lower)
{
  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv =
  (struct aurix_ethernet_qbvgate_timer_lowerhalf_s *)lower;

  uint32_t next_interval_us = 0;
  int tom_timer_channel = g_qbv_config.qbvtomtimer0;
  int bvsch_offset = 0;

  /* processing channel IRQ */

  IfxEgtm_Tom_Timer_acknowledgeTimerIrq(&priv->timer);

  if (tom_timer_channel < 8)
    {
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[0],
                                     0, 1 << tom_timer_channel, TRUE);
    }
  else
    {
      tom_timer_channel -= 8;
      IfxEgtm_Tom_Tgc_enableChannels(&MODULE_EGTM.CLS[1].TOM.TGC[1],
                                     0, 1 << tom_timer_channel, TRUE);
    }
#ifdef QBVTESTPIN
  IfxPort_togglePin(QBVTASKTESTPIN);
#endif
  if (priv->started == false)
    {
      return ERROR;
    }

  /* Calling CallBack */

  if (priv->callback)
    {
      priv->callback(&next_interval_us, priv->arg);
    }

  /* Calculate qbvsch offset */

  for (size_t i = 0; i < 5; i++)
  {
    bvsch_offset += QBVOFFSETONE;
    if (g_qbv_config.gate[i].gateon != 0)
      {
        break;
      }
  }

  /* Calculate Next Time */

  accp_qbvsch_offset_mac_time = get_remaining_time();
  if (aurix_enet_mac_done(IfxGeth_PortIndex_1) == true)
    {
      rem_settime(&priv->timer.egtm->CLS[EGTMCLSNUM].TOM,
        QBVGATECYCLE - ((get_remaining_time() + bvsch_offset)
        % QBVGATECYCLE), g_qbv_config.qbvtomtimer0);
    }
  else
    {
      rem_settime(&priv->timer.egtm->CLS[EGTMCLSNUM].TOM,
        QBVGATECYCLE - 0
        , g_qbv_config.qbvtomtimer0);
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_qbvgate_config_init
 *
 * Description:
 *   init qbvgate timer
 *
 * Input Parameters:
 *   qbv_config
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

int aurix_qbvgate_config_init(struct qbv_config *qbv_config)
{
  switch (qbv_config->qbvsrcindex)
    {
      case 0:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR0);
      break;

      case 1:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR1);
      break;

      case 2:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR2);
      break;

      case 3:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR3);
      break;

      case 4:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR4);
      break;

      case 5:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR5);
      break;

      case 6:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR6);
      break;

      case 7:
        qbv_config->qbvsrcnum = TRICORE_IRQ_GET(SRC_EGTMTOM1SR7);
      break;

      default:
      return -ENOMEM;
    }
  return OK;
}

/****************************************************************************
 * Function: aurix_qbvgate_timer_init
 *
 * Description:
 *   init qbvgate timer
 *
 * Input Parameters:
 *   timer_lowerhalf_s
 *   aurix_qbvsch_timer_cfg_s
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

int aurix_qbvgate_timer_init(struct timer_lowerhalf_s **dev,
                             struct aurix_qbvsch_timer_cfg_s *config)
{
  struct aurix_ethernet_qbvgate_timer_lowerhalf_s *priv;
  IfxEgtm_Tom_Timer_Config timer_config;

  priv = kmm_zalloc(sizeof(struct aurix_ethernet_qbvgate_timer_lowerhalf_s));
  if (priv == NULL)
    {
      syslog(LOG_ERR, "ERROR: qbv schedu kmm_zalloc failed\n");
      return -ENOMEM;
    }

  aurix_qbvgate_config_init(&g_qbv_config);
  IfxEgtm_Tom_Timer_initConfig(&timer_config, &MODULE_EGTM);

  timer_config.cluster                 = IfxEgtm_Cluster_1;                    /* Define the timer used            */
  timer_config.timerChannel            = g_qbv_config.qbvtomtimer0;            /* Define the channel used          */
  timer_config.clock                   = IfxEgtm_Tom_Ch_ClkSrc_cmuFxclk1;      /* Define the CMU clock used        */
  timer_config.frequency               = QBVDEFFREQ;                           /* Set timer frequency              */
  timer_config.interrupt.isrPriority   = IRQ_TO_NDX(g_qbv_config.qbvsrcnum);   /* Set interrupt priority           */
  timer_config.interrupt.isrProvider   = QBVINTERRUPTCPU;                      /* Set interrupt provider           */
  IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_FXCLK);             /* Enable the CMU clock             */
  IfxEgtm_Tom_Timer_init(&priv->timer, &timer_config);                         /* Initialize the TOM               */

  irq_attach(g_qbv_config.qbvsrcnum,
    interrupt_tom, priv);
  up_enable_irq(g_qbv_config.qbvsrcnum);

  priv->config = &config[0];
  priv->ops = &g_qbv_schedule_ops;

  dev[0] = (struct timer_lowerhalf_s *)priv;

  IfxEgtm_Tom_Timer_run(&priv->timer);
  priv->started = false;
  if (timer_register("/dev/qbv_schedu",
                    (struct timer_lowerhalf_s *)priv) == NULL)
    {
      up_disable_irq(g_qbv_config.qbvsrcnum);
      irq_detach(g_qbv_config.qbvsrcnum);
      kmm_free(priv);
      dev[0] = NULL;
      syslog(LOG_ERR, "ERROR: timer register failed: qbv_sch\n");
      return EIO;
    }
#ifdef QBVTESTPIN
  IfxPort_setPinModeOutput(QBVTASKTESTPIN, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);   /* Set pin mode */
  IfxPort_setPinState(QBVTASKTESTPIN, IfxPort_State_toggled);
#endif

  return OK;
}
