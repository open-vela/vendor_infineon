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

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <sched.h>
#include <stdio.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "aurix_tc4dx_pms_rtc.h"
#include "tricore_internal.h"

#define TRICORE_PMS_RTC_IRQ_GET(PMS_ADDR) (((uintptr_t)&PMS_ADDR - (uintptr_t)&SRC_CPU0_SB) / 4)
#define SECONDS_PER_MINUTE (60ul)
#define MINUTES_PER_HOUR (60ul)
#ifndef HOURS_PER_DAY
#define HOURS_PER_DAY (24ul)
#endif
#define SECONDS_PER_HOUR (SECONDS_PER_MINUTE * MINUTES_PER_HOUR)
#define SECONDS_PER_DAY (SECONDS_PER_HOUR * HOURS_PER_DAY)

#define LEAP_YEAR_DAYS (366ul)
#define COMMON_YEAR_DAYS (365ul)

/* Start year for RTC time structure (1900) */
#define TM_START_YEAR (1900ul)
/* Start year for Unix timestamp (1970) */
#define EPOCH_START_YEAR (1970ul)

#define RTC_MAX_COUNT (0xFFFFFFFFFFFFull)

#if defined(CONFIG_AURIX_PMS_RTC_XTAL_32K)
#define TC4DX_RTC_REAL_FREQUENCY (32768)
#define PMS_RTC_CALIBRATION_FACTOR_COLD_START (0.959f)
#define TC4DX_RTC_REAL_FREQUENCY_COLD_START (70000 * PMS_RTC_CALIBRATION_FACTOR_COLD_START)
#define TC4DX_RTC_MS_TO_TICKS_COLD_START(ms) \
  (((uint64_t)(ms) * (uint64_t)(TC4DX_RTC_REAL_FREQUENCY_COLD_START) + 500ULL) / 1000ULL)
#else
#define PMS_RTC_CLOCK_DEVIDER (34)
#define PMS_RTC_CALIBRATION_FACTOR (0.959f)
#define TC4DX_RTC_REAL_FREQUENCY (70000 * PMS_RTC_CALIBRATION_FACTOR / (PMS_RTC_CLOCK_DEVIDER + 1))
#endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

#define TC4DX_RTC_MS_TO_TICKS(ms) \
  (((uint64_t)(ms) * (uint64_t)(TC4DX_RTC_REAL_FREQUENCY) + 500ULL) / 1000ULL)

static const uint32_t month_days[12] =
{
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

#if defined(CONFIG_AURIX_PMS_RTC_XTAL_32K)
static uint8_t tc4dx_xtal_32k_cold_start_flag;
#endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* xtal32k_enable with a software timeout set to 1 second */
#define CKGEN_WAIT_XTAL32K_READY_US (1000000u)

#define PMS_BASE_ADDRESS (0xF0240000u)

extern uint8 BswIoc_Read_Safety_standbyram_bsw_rtc_base_unix_time(
                                            void *,
                                            uint32 iocDataSize);
extern uint8 BswIoc_Write_Safety_standbyram_bsw_rtc_base_unix_time(
                                            void *iocData,
                                            uint32 iocDataSize);
extern uint8 BswIoc_Read_Safety_standbyram_bsw_rtc_base_ticks(
                                            void *iocData,
                                            uint32 iocDataSize);
extern uint8 BswIoc_Write_Safety_standbyram_bsw_rtc_base_ticks(
                                            void *iocData,
                                            uint32 iocDataSize);
extern uint8 BswIoc_Read_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                            void *iocData,
                                            uint32 iocDataSize);
extern uint8 BswIoc_Write_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                            void *iocData,
                                            uint32 iocDataSize);

#ifdef CONFIG_RTC_ALARM

struct alm_cbinfo_s
{
    volatile alm_callback_t ac_cb;  /* Client callback function */
    volatile void *ac_arg;          /* Argument to pass with the callback function */
};
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_ALARM
/* Callback to use rtc alarm  */

static struct alm_cbinfo_s g_alarmcb;

#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_PERIODIC
/* Callback to use rtc periodic */

static wakeupcb_t g_periodiccb;

#endif /* CONFIG_RTC_PERIODIC */

/* sec_rtc structures */

static tc4dx_drv_rtc_s g_rtc_config =
{
    .base_ticks = 0,
};

static uint8_t tc4dx_check_rtc_time(const struct rtc_time *tm);
static uint64_t tc4dx_ticks_to_ms(uint64_t ticks, uint64_t freq_hz);
static int tc4dx_pms_rtc_alarm_handler(int irq, void *context,
                                    void *rtc_handler_arg);

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Name: is_leap_year
 *
 * Description:
 *   check year is leap year or not.
 *
 * Input Parameters:
 *   year - year.
 *
 * Returned Value:
 *   true - leap year; false - not leap year.
 *
 ***************************************************************************/

static inline bool is_leap_year(uint32_t year)
{
  return (!(year % 4) && (year % 100)) || !(year % 400);
}

/***************************************************************************
 * Name: tc4dx_rtc_to_epoch_ms
 *
 * Description:
 *   Simple algorithm to convert RTC time to Epoch milliseconds.
 *   Epoch seconds starts from 1970-1-1, 00:00:00.001
 *
 * Input Parameters:
 *   tm - the time to use
 *
 * Returned Value:
 *   milliseconds since the epoch
 *
 ***************************************************************************/

uint64_t tc4dx_rtc_to_epoch_ms(const struct rtc_time *tm)
{
  uint64_t epoch_millisec = 0;
  uint32_t year = 0;

  /* check RTC time and return 0 if valid */

  if ((NULL == tm) || (tc4dx_check_rtc_time(tm) != RTC_DATETIME_OK))
    {
      return 0;
    }

  year = tm->tm_year + TM_START_YEAR;

  /* accumulate milliseconds for each year until the target year */

  for (uint32_t num = EPOCH_START_YEAR; num < year; num++)
    {
      epoch_millisec += (uint64_t)SECONDS_PER_DAY * 1000 * \
        (is_leap_year(num) \
        ? LEAP_YEAR_DAYS : COMMON_YEAR_DAYS);
    }

  /* accumulate milliseconds for each year until the target year */

  for (uint32_t num = 0; num < tm->tm_mon; num++)
    {
      epoch_millisec += (uint64_t)month_days[num] * SECONDS_PER_DAY * 1000;

      if (is_leap_year(year) && (num == 1))
        {
          epoch_millisec += SECONDS_PER_DAY * 1000;
        }
    }

  /* add milliseconds for the day, hour, minute, second, and subsecond */

  epoch_millisec += (uint64_t)(tm->tm_mday - 1) * SECONDS_PER_DAY * 1000;
  epoch_millisec += (uint64_t)tm->tm_hour * SECONDS_PER_HOUR * 1000;
  epoch_millisec += (uint64_t)tm->tm_min * SECONDS_PER_MINUTE * 1000;
  epoch_millisec += (uint64_t)tm->tm_sec * 1000;
  #if defined(CONFIG_AURIX_PMS_HAVE_RTC_SUBSECONDS)
  epoch_millisec += (uint64_t)tm->tm_nsec / 1000000;
  #endif

  return epoch_millisec;
}

/***************************************************************************
 * Name: tc4dx_check_rtc_time
 *
 * Description:
 *   check rtc time is valid or not.
 *
 * Input Parameters:
 *   tm - rtc time structure.
 *
 * Returned Value:
 *   Zero (OK) on success; others failure.
 *
 ***************************************************************************/

static uint8_t tc4dx_check_rtc_time(const struct rtc_time *tm)
{
  /* sub-second: 0 ~ 999ms */

  #if defined(CONFIG_AURIX_PMS_HAVE_RTC_SUBSECONDS)
  if (tm->tm_nsec / 1000000 > 999u)
    {
      return RTC_SUBSEC_ERROR;
    }
  #endif

  /* second: 0 ~ 61s (including leap second) */

  if (tm->tm_sec > 61u)
    {
      return RTC_SEC_ERROR;
    }

  /* minite: 0 ~ 59m */

  if (tm->tm_min > 59u)
    {
      return RTC_MIN_ERROR;
    }

  /* PRQA S 2758 2 */

  /* hour: 0 ~ 23h */

  if (tm->tm_hour > 23u)
    {
      return RTC_HOUR_ERROR;
    }

  /* day of the month: 1 ~ 31 */

  if ((tm->tm_mday < 1u) || (tm->tm_mday > 31u))
    {
      return RTC_DAY_ERROR;
    }

  /* month of the year: 0(January) ~ 11(December) */

  if (tm->tm_mon > 11u)
    {
      return RTC_MONTH_ERROR;
    }

  /* day of the week: 0 ~ 6 */

  if (tm->tm_wday > 6u)
    {
      return RTC_WEEK_ERROR;
    }

  /* max tm_year is 2242 (1900 + 342) */

  if (tm->tm_year > 342u)
    {
      return RTC_YEAR_ERROR;
    }

  return RTC_DATETIME_OK;
}

#ifdef CONFIG_DEBUG_RTC_INFO
/***************************************************************************
 * Name: rtc_dumptime
 ***************************************************************************/

static void rtc_dumptime(const struct rtc_time *tp, const char *msg)
{
    rtcinfo("%s:\n", msg);

    rtcinfo("  tm: %04d-%02d-%02d %02d:%02d:%02d\n",
        tp->tm_year + 1900, tp->tm_mon + 1, tp->tm_mday,
        tp->tm_hour, tp->tm_min, tp->tm_sec);
}
#else
#define rtc_dumptime(tp, msg)
#endif /* CONFIG_DEBUG_RTC_INFO */

/***************************************************************************
 * Name: tc4dx_epoch_ms_to_rtc
 *
 * Description:
 *   convert epoch milliseconds to rtc time.
 *
 * Input Parameters:
 *   tm - rtc time structure.
 *   epoch_millisec - epoch milliseconds.
 *
 * Returned Value:
 *   Zero (OK) on success; others failure.
 *
 ***************************************************************************/

uint8_t tc4dx_epoch_ms_to_rtc(struct rtc_time *tm,
                                    uint64_t epoch_millisec)
{
  uint32_t year = EPOCH_START_YEAR;
  uint32_t month = 0;
  uint32_t days = epoch_millisec / SECONDS_PER_DAY / 1000;
  uint32_t seconds;

  if (NULL == tm)
    {
      return TC4DX_INVALID_PARAM;
    }

  tm->tm_wday = (4 + days) % 7;

  /* calculate the year */

  while (days >= COMMON_YEAR_DAYS)
    {
      if (is_leap_year(year))
        {
          if (days >= LEAP_YEAR_DAYS)
            {
              days -= LEAP_YEAR_DAYS;
            }
          else
            {
              break;
            }
        }
      else
        {
          days -= COMMON_YEAR_DAYS;
        }

      year++;
    }

  tm->tm_year = year - TM_START_YEAR;

  /* calculate the month */

  while (days >= 28)
    {
      if (is_leap_year(year) && (month == 1))
        {
          if (days >= 29)
            {
              days -= 29;
            }
          else
            {
              break;
            }
        }
      else
        {
          if (days >= month_days[month])
            {
              days -= month_days[month];
            }
          else
            {
              break;
            }
        }

      month++;
    }

  tm->tm_mon = month;
  tm->tm_mday = days + 1;

  /* calculate the hour, minute, second, and sub-second */

  seconds = (epoch_millisec % (SECONDS_PER_DAY * 1000)) / 1000;
  tm->tm_hour = seconds / 3600;
  tm->tm_min = (seconds % 3600) / 60;
  tm->tm_sec = (seconds % 3600) % 60;
  #if defined(CONFIG_AURIX_PMS_HAVE_RTC_SUBSECONDS)
  tm->tm_nsec = (epoch_millisec % 1000) * 1000000;
  #endif

  /* check the validity of the rtc time */

  return tc4dx_check_rtc_time(tm);
}

/***************************************************************************
 * Name: tc4dx_ticks_to_ms
 *
 * Description:
 *   convert ticks time to milliseconds.
 *
 * Input Parameters:
 *   tm - rtc time structure.
 *
 * Returned Value:
 *   epoch milliseconds.
 *
 ***************************************************************************/

static uint64_t tc4dx_ticks_to_ms(uint64_t ticks, uint64_t freq_hz)
{
  /* each tick is 1 / freq_hz second,
   * so tick_period_ms is 1,000 / freq_hz ms
   */

  return (ticks * 1000) / freq_hz;
}

#ifdef CONFIG_RTC_ALARM
/***************************************************************************
 * Name: tc4dx_pms_rtc_alarm_handler
 *
 * Description:
 *   rtc wakeup interrupt callback function.
 *
 * Input Parameters:
 *   irq - rtc wakeup interrupt num.
 *   context - Architecture specific register save information.
 *   arg - pointer to interrupt arg(rtc device structure).
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ***************************************************************************/

static int tc4dx_pms_rtc_alarm_handler(int irq, void *context,
                                    void *rtc_handler_arg)
{
  struct alm_cbinfo_s *cbinfo;
  alm_callback_t cb;
  void *arg;
  uint64_t current_ticks;
  uint64_t alarm_ticks;
  int ret = TC4DX_RTC_ERROR;
  IfxPmsPm_clearInterruptStatusFlags(&MODULE_PMS,
                        IfxPmsPm_Interrupt_rtcCmp0);

  /* read current ticks in PMS RTC registers */

  current_ticks = tc4dx_pms_rtc_get_current_ticks();

  /* read the alarm ticks which user set */

  ret = BswIoc_Read_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                                    &alarm_ticks,
                                                    sizeof(alarm_ticks));

  /* check if current ticks larger than alarm ticks which user set */

  if (TC4DX_RTC_OK == ret && current_ticks >= alarm_ticks)
    {
      /* trigger user callback function */

      cbinfo = &g_alarmcb;
      if (cbinfo->ac_cb != NULL)
        {
          /* Alarm A callback */

          cb  = cbinfo->ac_cb;
          arg = (void *)cbinfo->ac_arg;

          cbinfo->ac_cb  = NULL;
          cbinfo->ac_arg = NULL;

          cb(arg, 0);
        }

      /* disable alarm */

      IfxPmsEvr_disableInterrupt(&MODULE_PMS, IfxPmsEvr_Interrupt_rtccmp0);
    }

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: tc4dx_pms_rtc_is_initialized
 *
 * Description:
 *   Check if the RTC has been initialized
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True if the RTC has been initialized
 *
 ***************************************************************************/

bool tc4dx_pms_rtc_is_initialized(void)
{
  Ifx_PMS *pms = &MODULE_PMS;
  #if defined(CONFIG_AURIX_PMS_RTC_XTAL_32K)
  return pms->RTC.CON0.B.RTCCLKSEL == IfxPmsPm_RtcClk_32Khz;
  #endif
  return pms->RTC.CON0.B.RTCEN;
}

/***************************************************************************
 * Name: tc4dx_drv_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; others failure
 *
 ***************************************************************************/

uint8_t tc4dx_drv_rtc_initialize(void)
{
  bool init_stat;
  int ret = TC4DX_RTC_ERROR;

  /* 1970-1-1 00:00:00
   * RTC initial time
   */

  const struct rtc_time tm =
    {
      .tm_sec = 0,
      .tm_min = 0,
      .tm_hour = 0,
      .tm_mday = 1,
      .tm_mon = 1,
      .tm_year = 0,
    };

  /* See if the clock has already been initialized; since it is battery
   * backed, we don't need or want to re-initialize on each reset.
   */

  init_stat = tc4dx_pms_rtc_is_initialized();
  if (!init_stat)
    {
      #if defined(CONFIG_AURIX_PMS_RTC_XTAL_32K)
      IfxPmsPm_enableRtc(&MODULE_PMS, IfxPmsPm_Rtc_enable);
      IfxPmsPm_enableRtcOscillator(&MODULE_PMS, IfxPmsPm_Rtc_enable);
      tc4dx_xtal_32k_cold_start_flag = TRUE;
      #else
      IfxPmsPm_RtcClockConfig pmsRtcClockCfg;
      memset(&pmsRtcClockCfg, 0, sizeof(IfxPmsPm_RtcClockConfig));
      pmsRtcClockCfg.enableRtc = FALSE;
      pmsRtcClockCfg.clockDivider = PMS_RTC_CLOCK_DEVIDER;
      IfxPmsPm_rtcClockConfiguration(&MODULE_PMS, &pmsRtcClockCfg);
      IfxPmsPm_selectRtcClockSource(&MODULE_PMS, IfxPmsPm_RtcClk_70Khz);
      IfxPmsPm_enableRtc(&MODULE_PMS, IfxPmsPm_Rtc_enable);
      #endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

      if (TC4DX_RTC_OK == tc4dx_drv_rtc_set_time(&tm))
        {
          /* Keep the fact that the RTC is initialized */

          ret = TC4DX_RTC_OK;
        }
    }
  else
    {
      /* read the base timestamp from standby ram to g_rtc_config */

      ret = BswIoc_Read_Safety_standbyram_bsw_rtc_base_unix_time(
                                    &g_rtc_config.base_timestamp_ms,
                                    sizeof(g_rtc_config.base_timestamp_ms));
      ret = BswIoc_Read_Safety_standbyram_bsw_rtc_base_ticks(
                                        &g_rtc_config.base_ticks,
                                        sizeof(g_rtc_config.base_ticks));
    }

  return ret;
}

/***************************************************************************
 * Name: tc4dx_drv_rtc_get_time
 *
 * Description:
 *   Get the current date and time from the date/time RTC
 *
 * Input Parameters:
 *   tm - rtc time structure.
 *
 * Returned Value:
 *   Zero (OK) on success; others failure.
 *
 ***************************************************************************/

uint8_t tc4dx_drv_rtc_get_time(struct rtc_time *rtctime)
{
  uint8_t ret = TC4DX_INVALID_PARAM;
  uint64_t delta_ms;
  uint64_t current_ticks = tc4dx_pms_rtc_get_current_ticks();

  if (current_ticks < g_rtc_config.base_ticks)
    {
      /* rtc time is not valid */

      return ret;
    }

  uint64_t delta_ticks = current_ticks - g_rtc_config.base_ticks;
  #ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
  if (tc4dx_xtal_32k_cold_start_flag)
    {
      delta_ms = tc4dx_ticks_to_ms(delta_ticks,
                      TC4DX_RTC_REAL_FREQUENCY_COLD_START);
    }
  else
    {
      delta_ms = tc4dx_ticks_to_ms(delta_ticks,
                                    TC4DX_RTC_REAL_FREQUENCY);
    }
  #else
  delta_ms = tc4dx_ticks_to_ms(delta_ticks,
                                    TC4DX_RTC_REAL_FREQUENCY);
  #endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */
  uint64_t current_timestamp_ms = g_rtc_config.base_timestamp_ms + delta_ms;
  ret = tc4dx_epoch_ms_to_rtc(rtctime, current_timestamp_ms);

  return ret;
}

/***************************************************************************
 * Name: tc4dx_drv_rtc_set_time
 *
 * Description:
 *   Set the RTC to the provided time. write given time to Standby RAM
 *
 * Input Parameters:
 *   tm - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ***************************************************************************/

uint8_t tc4dx_drv_rtc_set_time(const struct rtc_time *rtctime)
{
  /* convert rtc time to milliseconds since the epoch */

  uint64_t new_base_timestamp_ms = tc4dx_rtc_to_epoch_ms(rtctime);
  uint64_t new_base_ticks = tc4dx_pms_rtc_get_current_ticks();
  int ret = TC4DX_RTC_ERROR;

  /* ---write current base_timestamp_ms and base ticks to Standby RAM --- */

  ret = BswIoc_Write_Safety_standbyram_bsw_rtc_base_unix_time(
                                    &new_base_timestamp_ms,
                                    sizeof(new_base_timestamp_ms));
  ret = BswIoc_Write_Safety_standbyram_bsw_rtc_base_ticks(
                                    &new_base_ticks,
                                    sizeof(new_base_ticks));

  /* write new base_timestamp_ms and base ticks to g_rtc_config */

  g_rtc_config.base_timestamp_ms = new_base_timestamp_ms;
  g_rtc_config.base_ticks = new_base_ticks;

  return ret;
}

/***************************************************************************
 * Name: tc4dx_pms_rtc_get_current_ticks
 *
 * Description:
 *   Get the current date and time from the date/time RTC.
 *
 * Returned Value:
 *   ticks - the current ticks.
 *
 ***************************************************************************/

uint64_t tc4dx_pms_rtc_get_current_ticks(void)
{
  IfxPmsPm_RtcTimerCaptureType timerValue;
  timerValue = IfxPmsPm_readRtcTotalTimerValue(&MODULE_PMS);
  return ((uint64_t)timerValue.upperPart << 32) | timerValue.lowerPart;
}

/***************************************************************************
 * Name: tc4dx_pms_rtc_havesettime
 *
 * Description:
 *   Check if RTC time has been set.
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ***************************************************************************/

bool tc4dx_pms_rtc_havesettime(void)
{
  return g_rtc_config.base_ticks != 0;
}

/***************************************************************************
 * Name: tc4dx_pms_rtc_setalarm
 *
 * Description:
 *   Set an alarm to an absolute time using associated hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ***************************************************************************/

#ifdef CONFIG_RTC_ALARM
int tc4dx_pms_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct alm_cbinfo_s *cbinfo;
  int ret = -EINVAL;
  DEBUGASSERT(alminfo != NULL);
  cbinfo = &g_alarmcb;
  cbinfo->ac_cb = alminfo->as_cb;
  cbinfo->ac_arg = alminfo->as_arg;

  /* set rtc alarm args */

  uint64_t alarm_millisecs = tc4dx_rtc_to_epoch_ms(&alminfo->as_time);

  if (alarm_millisecs == 0)
    {
      return TC4DX_RTC_ERROR;
    }

  /* calculate alarm ticks:
   * alarm_ticks - base_ticks + basetimestamp_ms = alarm_millisecs
   */

  uint64_t delta_ms = alarm_millisecs - g_rtc_config.base_timestamp_ms;
  uint64_t delta_ticks = 0;

  #ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
  if (tc4dx_xtal_32k_cold_start_flag)
    {
      delta_ticks = TC4DX_RTC_MS_TO_TICKS_COLD_START(delta_ms);
    }
    else
    {
      delta_ticks = TC4DX_RTC_MS_TO_TICKS(delta_ms);
    }
  #else
      delta_ticks = TC4DX_RTC_MS_TO_TICKS(delta_ms);
  #endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

  uint64_t temp_alarm_ticks = g_rtc_config.base_ticks + delta_ticks;

  /* write alarm ticks to Standby RAM */

  BswIoc_Write_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                                &temp_alarm_ticks,
                                                sizeof(temp_alarm_ticks));

  /* set rtc alarm */

  IfxPmsPm_RtcCmpConfig temp_rtc_cmp_config;
  temp_rtc_cmp_config.mSize0 = IfxPmsPm_RtcMsize_31;
  temp_rtc_cmp_config.mSize1 = IfxPmsPm_RtcMsize_0;
  temp_rtc_cmp_config.mStart0 = IfxPmsPm_RtcMstart_0;
  temp_rtc_cmp_config.mStart1 = IfxPmsPm_RtcMstart_0;
  temp_rtc_cmp_config.cmpVal0 = temp_alarm_ticks & 0xffffffff;
  temp_rtc_cmp_config.cmpVal1 = 0;
  IfxPmsPm_rtcCmpConfiguration(&MODULE_PMS, &temp_rtc_cmp_config);

  /* enable the interrupt */

  int rtc_cmp0_irq = TRICORE_PMS_RTC_IRQ_GET(SRC_PMSSR5);
  ret = irq_attach(rtc_cmp0_irq,
    tc4dx_pms_rtc_alarm_handler,
    NULL);
  if (ret == TC4DX_RTC_OK)
    {
      up_enable_irq(rtc_cmp0_irq);
    }

  /* enable rtc alarm */

  IfxPmsEvr_enableInterrupt(&MODULE_PMS, IfxPmsEvr_Interrupt_rtccmp0);

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Name: tc4dx_pms_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ***************************************************************************/

#ifdef CONFIG_RTC_ALARM
int tc4dx_pms_rtc_cancelalarm(void)
{
  /* Cancel the alarm in hardware and disable interrupts
   * Cancel the global callback function
   */

  g_alarmcb.ac_cb = NULL;
  g_alarmcb.ac_arg = NULL;

  /* disable rtc alarm */

  IfxPmsPm_clearInterruptStatusFlags(&MODULE_PMS,
                                      IfxPmsPm_Interrupt_rtcCmp0);
  IfxPmsEvr_disableInterrupt(&MODULE_PMS, IfxPmsEvr_Interrupt_rtccmp0);

  return TC4DX_RTC_OK;
}
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Name: tc4dx_pms_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ***************************************************************************/

#ifdef CONFIG_RTC_ALARM
int tc4dx_pms_rtc_rdalarm(struct alm_rdalarm_s *alminfo)
{
  uint64_t alarm_ticks = 0;
  uint64_t alarm_millisecs = 0;
  int ret = -EINVAL;
  DEBUGASSERT(alminfo != NULL);
  Ifx_PMS *pms = &MODULE_PMS;

  if (pms->INT.CON2.B.RTCCMP0 != 1) return ret;

  /* Get alarm ms */

  BswIoc_Read_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                        &alarm_ticks,
                                        sizeof(alarm_ticks));
  uint64_t delta_ticks = alarm_ticks - g_rtc_config.base_ticks;

  #ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
  if (tc4dx_xtal_32k_cold_start_flag)
    {
      alarm_millisecs = tc4dx_ticks_to_ms(
                                  delta_ticks,
                                  TC4DX_RTC_REAL_FREQUENCY_COLD_START) \
                                  + g_rtc_config.base_timestamp_ms;
    }
  else
    {
      alarm_millisecs = tc4dx_ticks_to_ms(
                                  delta_ticks,
                                  TC4DX_RTC_REAL_FREQUENCY) \
                                  + g_rtc_config.base_timestamp_ms;
    }
  #else
  alarm_millisecs = tc4dx_ticks_to_ms(delta_ticks,
                                    TC4DX_RTC_REAL_FREQUENCY_COLD_START) \
                                    + g_rtc_config.base_timestamp_ms;
  #endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

  ret = tc4dx_epoch_ms_to_rtc(alminfo->ar_time, alarm_millisecs);

  return ret;
}
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Name: tc4dx_sync_alarm
 *
 * Description:
 *   sync alarm when cold start switch to stable.
 *
 * Returned Value:
 *   None
 *
 ***************************************************************************/

#ifdef CONFIG_RTC_ALARM
void tc4dx_sync_alarm(struct rtc_time *alarm_rtc_time)
{
  uint64_t alarm_millisecs = tc4dx_rtc_to_epoch_ms(alarm_rtc_time);

  /* recalculate alarm ticks:
   * alarm_ticks - base_ticks + basetimestamp_ms = alarm_millisecs
   */

  uint64_t delta_ms = alarm_millisecs - g_rtc_config.base_timestamp_ms;
  uint64_t delta_ticks = TC4DX_RTC_MS_TO_TICKS(delta_ms);
  uint64_t temp_alarm_ticks = g_rtc_config.base_ticks + delta_ticks;

  /* rewrite alarm ticks to Standby RAM */

  BswIoc_Write_Safety_standbyram_bsw_rtc_absolute_alarm_ticks(
                                                &temp_alarm_ticks,
                                                sizeof(temp_alarm_ticks));

  /* reset rtc alarm */

  IfxPmsPm_RtcCmpConfig temp_rtc_cmp_config;
  temp_rtc_cmp_config.mSize0 = IfxPmsPm_RtcMsize_31;
  temp_rtc_cmp_config.mSize1 = IfxPmsPm_RtcMsize_0;
  temp_rtc_cmp_config.mStart0 = IfxPmsPm_RtcMstart_0;
  temp_rtc_cmp_config.mStart1 = IfxPmsPm_RtcMstart_0;
  temp_rtc_cmp_config.cmpVal0 = temp_alarm_ticks & 0xffffffff;
  temp_rtc_cmp_config.cmpVal1 = 0;
  IfxPmsPm_rtcCmpConfiguration(&MODULE_PMS, &temp_rtc_cmp_config);
}
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Name: tc4dx_get_cold_start_flag
 *
 * Description:
 *   get cold start status.
 *
 * Returned Value:
 *   1 means cold start (TRUE), 0 means not (FALSE)
 *
 ***************************************************************************/

#ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
uint8_t tc4dx_get_cold_start_flag()
{
  return tc4dx_xtal_32k_cold_start_flag;
}
#endif

/***************************************************************************
 * Name: tc4dx_set_cold_start_flag_false
 *
 * Description:
 *   set cold start status FALSE.
 *
 * Returned Value:
 *   NONE
 *
 ***************************************************************************/

#ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
void tc4dx_set_cold_start_flag_false()
{
  tc4dx_xtal_32k_cold_start_flag = FALSE;
}
#endif
