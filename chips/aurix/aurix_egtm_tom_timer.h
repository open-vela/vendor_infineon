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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EGTM_TOM_TIMER__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EGTM_TOM_TIMER__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/timers/timer.h>

#include <Egtm/Tom/Timer/IfxEgtm_Tom_Timer.h>
#include <IfxSrc_reg.h>
#include <IfxEgtm_reg.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_egtm_tom_timer_config_s
{
  IfxEgtm_Cluster               cluster;
  IfxEgtm_Tom_Ch               channel;
  IfxEgtm_Tom_Ch_ClkSrc        clock;
  IfxEgtm_Tom_Timer_Interrupt  interrupt;
  uint32                        isrIrq;
  float32                       frequency;
  const char                   *devpath;
  boolean                       oneshotmode;
};

/****************************************************************************
 * Name: aurix_egtm_tom_timer_initialize
 *
 * Description:
 *   Initialize EGTM-TOM Timer submodule and register the Timer device.
 *
 ****************************************************************************/

int aurix_egtm_tom_timer_initialize(
  struct timer_lowerhalf_s **dev,
  const struct aurix_egtm_tom_timer_config_s *config,
  size_t num);

#endif /* CONFIG_AURIX_EGTM_TOM_TIMER */
