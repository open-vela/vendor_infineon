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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_WDG_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/watchdog.h>

#include "IfxWtu_reg.h"
#include "Wtu/Std/IfxWtu.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The identification of the watchdog instance */

enum aurix_wdg_inst_e
{
  AURIX_WDG_WDTCPU0 = 0, /* The watchdog timer for CPU0 */
  AURIX_WDG_WDTCPU1,     /* The watchdog timer for CPU1 */
  AURIX_WDG_WDTCPU2,     /* The watchdog timer for CPU2 */
  AURIX_WDG_WDTCPU3,     /* The watchdog timer for CPU3 */
  AURIX_WDG_WDTCPU4,     /* The watchdog timer for CPU4 */
  AURIX_WDG_WDTCPU5,     /* The watchdog timer for CPU5 */
  AURIX_WDG_WDTSEC,      /* The watchdog timer for CPUcs */
  AURIX_WDG_WDTSYS,      /* The watchdog timer for system */
  AURIX_WDG_NINSTANCES   /* The number of watchdog instances */
};

/* The configuration of the watchdog instance */

struct aurix_wdg_config_s
{
  const char            *devpath; /* Path to the watchdog */
  volatile void         *wdt_ptr; /* Watchdog instance pointer */
  enum aurix_wdg_inst_e  wdt_idx; /* Watchdog instance index */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_wdg_interrupt
 *
 * Description:
 *   Process MNI exception triggered by the watchdog unit
 *
 ****************************************************************************/

void aurix_wdg_interrupt(void);

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

int aurix_wdg_initialize(const struct aurix_wdg_config_s *config);

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
                             size_t count);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_WDG_H */