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

#ifndef __ARCH_TRTICORE_SRC_TC4DX_PMS_RTC_H
#define __ARCH_TRTICORE_SRC_TC4DX_PMS_RTC_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <time.h>
#include <nuttx/timers/rtc.h>

#include "IfxPmsPm.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/***************************************************************************
 * Public Types
 ***************************************************************************/

#ifndef __ASSEMBLY__

/**
 * @brief RTC device structure.
 */

typedef struct tc4dx_drv_rtc
{
  uint64_t base_timestamp_ms;             /* base timestamp ms */
  uint64_t base_ticks;                    /* base ticks */
} tc4dx_drv_rtc_s;

/**
 * @brief rtc get time
 *
 * Get current rtc time.
 *
 * @param [in] dev tc4dx drv rtc controller
 * @param [out] tm rtc time
 * @return 0 success, otherwise failed.
 */

uint8_t tc4dx_drv_rtc_get_time(struct rtc_time *rtctime);

/**
 * @brief rtc to epoch ms
 *
 * convert rtc time to epoch ms.
 *
 * @param [in] dev tc4dx drv rtc time
 * @return epoch ms.
 */

uint64_t tc4dx_rtc_to_epoch_ms(const struct rtc_time *tm);

/**
 * @brief rtc to epoch ms
 *
 * convert rtc time to epoch ms.
 *
 * @param [in] tc4dx drv rtc time, epoch milliseconds
 * @param [out] tm rtc time
 *@return 0 success, otherwise failed.
 */

uint8_t tc4dx_epoch_ms_to_rtc(struct rtc_time *tm,
                                    uint64_t epoch_millisec);

/**
 * @brief RTC datetime error type.
 */

typedef enum
{
  RTC_DATETIME_OK = 0,          /**< no error */
  RTC_SUBSEC_ERROR,             /**< subseconds error */
  RTC_SEC_ERROR,                /**< seconds error */
  RTC_MIN_ERROR,                /**< minutes error */
  RTC_HOUR_ERROR,               /**< hours error */
  RTC_DAY_ERROR,                /**< days error */
  RTC_MONTH_ERROR,              /**< months error */
  RTC_WEEK_ERROR,               /**< weekdays error */
  RTC_YEAR_ERROR,               /**< years error */
} tc4dx_rtc_datetime_e;

/**
 * @brief RTC common status.
 */

typedef enum
{
  TC4DX_RTC_OK = 0,             /**< no error */
  TC4DX_INVALID_PARAM,          /**< invalid parameter */
  TC4DX_RTC_ERROR,              /**< rtc error */
} tc4dx_rtc_status_e;

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

/**
 * @brief RTC interrupt callback function type.
 */

typedef int (*rtc_cb_t)(uint32_t irq, void *arg);

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  struct rtc_time as_time;            /* Alarm expiration time */
  alm_callback_t as_cb;               /* Callback (if non-NULL) */
  void *as_arg;                       /* Argument for callback */
};

/* Structure used to pass parameters to query an alarm */

struct alm_rdalarm_s
{
  struct rtc_time *ar_time; /* Argument for storing ALARM RTC time */
};

#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Public Data
 ***************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/***************************************************************************
 * Public Functions Prototypes
 ***************************************************************************/

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

uint64_t tc4dx_pms_rtc_get_current_ticks(void);

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
 *   Zero (OK) on success; others failure
 *
 ***************************************************************************/

uint8_t tc4dx_drv_rtc_set_time(const struct rtc_time *rtctime);

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

uint8_t tc4dx_drv_rtc_initialize(void);

/***************************************************************************
 * Name: tc4dx_pms_rtc_is_initialized
 *
 * Description:
 *    Returns 'true' if the RTC has been initialized
 *    Returns 'false' if the RTC has never been initialized since first time
 *    power up, and the counters are stopped until it is first initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if RTC has been initialized.
 *
 ***************************************************************************/

bool tc4dx_pms_rtc_is_initialized(void);

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

bool tc4dx_pms_rtc_havesettime(void);

#ifdef CONFIG_RTC_ALARM
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

int tc4dx_pms_rtc_setalarm(struct alm_setalarm_s *alminfo);

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

int tc4dx_pms_rtc_rdalarm(struct alm_rdalarm_s *alminfo);

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

int tc4dx_pms_rtc_cancelalarm(void);

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

void tc4dx_sync_alarm(struct rtc_time *rtc_time);
#endif /* CONFIG_RTC_ALARM */

/***************************************************************************
 * Name: tc4dx_pms_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the E3650.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "tc4dx_pms_rtc.h"
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
 ***************************************************************************/

#ifdef CONFIG_RTC_DRIVER
struct rtc_lowerhalf_s;
struct rtc_lowerhalf_s *tc4dx_pms_rtc_lowerhalf(void);
#endif /* CONFIG_RTC_DRIVER */

#ifdef CONFIG_AURIX_PMS_RTC_XTAL_32K
void tc4dx_check_xtal_32k_cold_process(void);
uint8_t tc4dx_get_cold_start_flag(void);
void tc4dx_set_cold_start_flag_false(void);
#endif /* CONFIG_AURIX_PMS_RTC_XTAL_32K */

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_TRTICORE_SRC_TC4DX_PMS_RTC_H */