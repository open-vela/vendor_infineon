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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/spinlock.h>

#include "aurix_tc4dx_pms_rtc.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
struct tc4dx_cbinfo_s
{
  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argument to accompany callback */
  uint8_t id;                       /* Identifies the alarm */
};
#endif /* CONFIG_RTC_ALARM */

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct tc4dx_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;      /* Threads can only exclusively access the RTC */

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  struct tc4dx_cbinfo_s cbinfo;
#endif /* CONFIG_RTC_ALARM */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int tc4dx_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime);
static int tc4dx_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime);
static bool tc4dx_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int tc4dx_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo);
static int
tc4dx_setrelative(struct rtc_lowerhalf_s *lower,
                   const struct lower_setrelative_s *alarminfo);
static int tc4dx_cancelalarm(struct rtc_lowerhalf_s *lower,
                                int alarmid);
static int tc4dx_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_PERIODIC
static int
tc4dx_pms_setperiodic(struct rtc_lowerhalf_s *lower,
                    const struct lower_setperiodic_s *alarminfo);
static int
tc4dx_pms_cancelperiodic(struct rtc_lowerhalf_s *lower, int id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* TC4DX PMS RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = tc4dx_rdtime,
  .settime     = tc4dx_settime,
  .havesettime = tc4dx_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = tc4dx_setalarm,
  .setrelative = tc4dx_setrelative,
  .cancelalarm = tc4dx_cancelalarm,
  .rdalarm     = tc4dx_rdalarm,
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = tc4dx_setperiodic,
  .cancelperiodic = tc4dx_cancelperiodic,
#endif /* CONFIG_RTC_PERIODIC */
};

/* TC4DX PMS RTC device state */

static struct tc4dx_lowerhalf_s g_rtc_lowerhalf =
{
  .ops     = &g_rtc_ops,
  .devlock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4dx_alarm_callback
 *
 * Description:
 *   This is the function that is called from the RTC driver when the alarm
 *   goes off.  It just invokes the upper half drivers callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static void tc4dx_alarm_callback(void *arg, unsigned int alarmid)
{
  struct tc4dx_lowerhalf_s *lower;
  struct tc4dx_cbinfo_s *cbinfo;
  rtc_alarm_callback_t cb;
  void *priv;

  DEBUGASSERT(arg != NULL);
  DEBUGASSERT(alarmid == 0);

  lower        = (struct tc4dx_lowerhalf_s *)arg;
  cbinfo       = &lower->cbinfo;

  /* Sample and clear the callback information to minimize the window in
   * time in which race conditions can occur.
   */

  cb           = (rtc_alarm_callback_t)cbinfo->cb;
  priv         = (void *)cbinfo->priv;

  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Perform the callback */

  if (cb != NULL)
    {
      cb(priv, alarmid);
    }
}
#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: tc4dx_pms_rtc_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int tc4dx_rdtime(struct rtc_lowerhalf_s *lower,
                          struct rtc_time *rtctime)
{
  struct tc4dx_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && rtctime != NULL);

  priv = (struct tc4dx_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = tc4dx_drv_rtc_get_time(rtctime);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: tc4dx_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int tc4dx_settime(struct rtc_lowerhalf_s *lower,
                           const struct rtc_time *rtctime)
{
  struct tc4dx_lowerhalf_s *priv;
  int ret;

  DEBUGASSERT(lower != NULL && rtctime != NULL);

  priv = (struct tc4dx_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* This operation depends on the fact that struct rtc_time is cast
   * compatible with struct tm.
   */

  ret = tc4dx_drv_rtc_set_time(rtctime);

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: tc4dx_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool tc4dx_havesettime(struct rtc_lowerhalf_s *lower)
{
  DEBUGASSERT(lower != NULL);

  return tc4dx_pms_rtc_havesettime();
}

/****************************************************************************
 * Name: tc4dx_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int tc4dx_setalarm(struct rtc_lowerhalf_s *lower,
                            const struct lower_setalarm_s *alarminfo)
{
  struct tc4dx_lowerhalf_s *priv;
  struct tc4dx_cbinfo_s *cbinfo;
  struct alm_setalarm_s lowerinfo;
  int ret;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == 0);
  priv = (struct tc4dx_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Remember the callback information */

  cbinfo            = &priv->cbinfo;
  cbinfo->cb        = alarminfo->cb;
  cbinfo->priv      = alarminfo->priv;
  cbinfo->id        = alarminfo->id;

  /* Set the alarm */

  lowerinfo.as_cb   = tc4dx_alarm_callback;
  lowerinfo.as_arg  = priv;
  memcpy(&lowerinfo.as_time, &alarminfo->time, sizeof(struct rtc_time));

  /* And set the alarm */

  ret = tc4dx_pms_rtc_setalarm(&lowerinfo);
  if (ret < 0)
    {
      cbinfo->cb   = NULL;
      cbinfo->priv = NULL;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4dx_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time.  This function implements
 *   the setrelative() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int
tc4dx_setrelative(struct rtc_lowerhalf_s *lower,
                    const struct lower_setrelative_s *alarminfo)
{
  struct lower_setalarm_s setalarm;
  struct rtc_time time;
  uint64_t rtc_rel_total_ms;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL);
  DEBUGASSERT(alarminfo->id == 0);

  if (alarminfo->reltime > 0)
    {
      /* Disable pre-emption while we do this so that we don't have to worry
       * about being suspended and working on an old time.
       */

      flags = enter_critical_section();

      /* Get the current time in broken out format */

      ret = tc4dx_drv_rtc_get_time(&time);
      if (ret >= 0)
        {
          /* convert time to ms */

          rtc_rel_total_ms =  tc4dx_rtc_to_epoch_ms(&time);

          /* Add the seconds offset. */

          rtc_rel_total_ms += (alarminfo->reltime * 1000);

          /* And convert the time back to rtc_time format */

          tc4dx_epoch_ms_to_rtc(&time, rtc_rel_total_ms);

          /* The set the alarm using this absolute time */

          setalarm.id   = alarminfo->id;
          setalarm.cb   = alarminfo->cb;
          setalarm.priv = alarminfo->priv;
          setalarm.time = time;

          ret = tc4dx_setalarm(lower, &setalarm);
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: tc4dx_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int tc4dx_cancelalarm(struct rtc_lowerhalf_s *lower,
                               int alarmid)
{
  struct tc4dx_lowerhalf_s *priv;
  struct tc4dx_cbinfo_s *cbinfo;
  int ret;

  DEBUGASSERT(lower != NULL);
  DEBUGASSERT(alarmid == 0);
  priv = (struct tc4dx_lowerhalf_s *)lower;

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      return ret;
    }

  /* ID0-> Alarm A; ID1 -> Alarm B */

  /* Nullify callback information to reduce window for race conditions */

  cbinfo       = &priv->cbinfo;
  cbinfo->cb   = NULL;
  cbinfo->priv = NULL;

  /* Then cancel the alarm */

  ret = tc4dx_pms_rtc_cancelalarm();

  nxmutex_unlock(&priv->devlock);
  return ret;
}
#endif

/****************************************************************************
 * Name: tc4dx_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
static int tc4dx_rdalarm(struct rtc_lowerhalf_s *lower,
                           struct lower_rdalarm_s *alarminfo)
{
  struct alm_rdalarm_s lowerinfo;
  int ret = -EINVAL;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);
  DEBUGASSERT(alarminfo->id == 0);

  /* Disable pre-emption while we do this so that we don't have to worry
   * about being suspended and working on an old time.
   */

  flags = enter_critical_section();

  lowerinfo.ar_time = alarminfo->time;

  ret = tc4dx_pms_rtc_rdalarm(&lowerinfo);

  leave_critical_section(flags);

  return ret;
}
#endif

/****************************************************************************
 * Name: tc4dx_pms_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the TC4DX.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "tc4dx_pms_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = tc4dx_pms_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *tc4dx_pms_rtc_lowerhalf(void)
{
  tc4dx_drv_rtc_initialize();
  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the rtc per the selected configuration.
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  FAR struct rtc_lowerhalf_s *lower_rtc = tc4dx_pms_rtc_lowerhalf();
  up_rtc_set_lowerhalf(lower_rtc, false);
  return rtc_initialize(0, lower_rtc);
}

/****************************************************************************
 * Name: tc4dx_check_xtal_32k_cold_process
 *
 * Description:
 *   check is cold start or note when use external 32k osc
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tc4dx_check_xtal_32k_cold_process(void)
{
  if (tc4dx_get_cold_start_flag())
    {
      struct rtc_time rd_rtc_time;
      struct rtc_time rd_alarm_time;
      struct alm_rdalarm_s alminfo;
      memset(&rd_rtc_time, 0, sizeof(struct rtc_time));
      memset(&rd_alarm_time, 0, sizeof(struct rtc_time));
      memset(&alminfo, 0, sizeof(struct alm_rdalarm_s));
      alminfo.ar_time = &rd_alarm_time;
      struct tc4dx_lowerhalf_s *priv = &g_rtc_lowerhalf;

      tc4dx_drv_rtc_get_time(&rd_rtc_time);

      IfxPmsPm_selectRtcClockSource(&MODULE_PMS, IfxPmsPm_RtcClk_32Khz);

      int rdalarm_ret = tc4dx_pms_rtc_rdalarm(&alminfo);

      tc4dx_set_cold_start_flag_false();

      nxmutex_lock(&priv->devlock);

      tc4dx_drv_rtc_set_time(&rd_rtc_time);

      if (TC4DX_RTC_OK == rdalarm_ret)
        {
          tc4dx_sync_alarm(alminfo.ar_time);
        }

      nxmutex_unlock(&priv->devlock);
    }
}

#endif /* CONFIG_RTC_DRIVER */