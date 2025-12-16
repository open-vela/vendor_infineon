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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_GTM_ATOM_TIMER__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_GTM_ATOM_TIMER__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/timers/timer.h>

#include <Gtm/Atom/Timer/IfxGtm_Atom_Timer.h>
#include <IfxSrc_reg.h>
#include <IfxGtm_reg.h>
#include <IfxPort.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_gtm_atom_timer_config_s
{
  IfxGtm_Atom                  atom;
  IfxGtm_Atom_Ch               channel;
  IfxGtm_Cmu_Clk               clock;
  IfxStdIf_Timer_Config        base;
  uint32                       isrIrq;
  IfxGtm_IrqMode               irqModeTimer;
  IfxGtm_IrqMode               irqModeTrigger;
  const char                   *devpath;
};

/****************************************************************************
 * Name: aurix_gtm_timer_initialize
 *
 * Description:
 *   Initialize GTM-ATOM Timer submodule and register the Timer device.
 *
 ****************************************************************************/

int aurix_gtm_timer_initialize(struct timer_lowerhalf_s **dev,
      const struct aurix_gtm_atom_timer_config_s *config,
      size_t num);

#endif /* CONFIG_AURIX_GTM_ATOM_TIMER */
