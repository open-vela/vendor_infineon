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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QBVSCH__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QBVSCH__H

#include <nuttx/timers/timer.h>
#include <IfxSrc_reg.h>
#include <IfxEgtm_reg.h>
#include "Egtm/Tom/Timer/IfxEgtm_Tom_Timer.h"

#define QBVSCHEDUNUM 2
#define QBVOFFSETONE -200000

struct aurix_qbvsch_timer_cfg_s
{
    IfxEgtm_Cluster               cluster;
    IfxEgtm_Tom_Ch                channel;
    IfxEgtm_Tom_Ch_ClkSrc         clock;
    IfxEgtm_Tom_Timer_Interrupt   interrupt;
    float32                       frequency;
    const char                    *devpath;
};

int aurix_qbvgate_timer_init(struct timer_lowerhalf_s **dev,
                             struct aurix_qbvsch_timer_cfg_s *config);

#endif /* _VENDOR_INFINEON_CHIPS_H */
