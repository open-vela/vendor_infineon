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

#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/arch.h>

#include "Pms/Std/IfxPmsEvr.h"
#include "Stm/Std/IfxStm.h"

#include "aurix_pms.h"
#include "aurix_scr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enable VMONP */

#define ENABLE_VMONP(regcon, regstat, regrst) do { \
    MODULE_PMS.VMONP.regcon.B.OVENABLE = 1; \
    while (MODULE_PMS.VMONP.regstat.B.RESULT <= MODULE_PMS.VMONP.regstat.B.RESETVAL); \
} while (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: aurix_voltagerail_init
 *
 * Description:
 *   Initialize the required power rails
 *
 ****************************************************************************/

void aurix_voltagerail_init(void)
{
  /* switch on VDDPHPHY0, VDDPHY0, VDDPHPHY1,
   * VDDPHY1, VDDPHPHY2, VDDPHY2 and VDDHSIF measurement
   */
#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphphy0);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphy0);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphphy1);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphy1);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphphy2);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphy2);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddhsif);

#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vdd);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddext);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddextdc);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddevrsb);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddm);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphphy1);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddhsif);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddphy1);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddpad);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddp3nvm);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vddexths);

  IfxPmsEvr_enableVoltageRail(&MODULE_PMS,
    IfxPmsEvr_PrimaryMonitorVoltageSource_vdd3pms);

#endif
}

/****************************************************************************
 * Function: aurix_vmonp_init
 *
 * Description:
 *   Initialize the precision vmonp protocol function
 *
 ****************************************************************************/

void aurix_vmonp_init(void)
{
  /* Enable VDDHSIF */

  ENABLE_VMONP(VDDHSIFCON, VDDHSIFSTAT, VDDHSIFRST);

  /* Enable VDDPHY1 */

  ENABLE_VMONP(VDDPHY1CON, VDDPHY1STAT, VDDPHY1RST);

  /* Enable VDDPHPHY1 */

  ENABLE_VMONP(VDDPHPHY1CON, VDDPHPHY1STAT, VDDPHPHY1RST);

#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
  /* Enable VDDPHY0 */

  ENABLE_VMONP(VDDPHY0CON, VDDPHY0STAT, VDDPHY0RST);

  /* Enable VDDPHY2 */

  ENABLE_VMONP(VDDPHY2CON, VDDPHY2STAT, VDDPHY2RST);

  /* Enable VDDPHPHY0 */

  ENABLE_VMONP(VDDPHPHY0CON, VDDPHPHY0STAT, VDDPHPHY0RST);

  /* Enable VDDPHPHY2 */

  ENABLE_VMONP(VDDPHPHY2CON, VDDPHPHY2STAT, VDDPHPHY2RST);
#endif
}

/****************************************************************************
 * Function: aurix_pms_init
 *
 * Description:
 *   Initialize the Power Management System
 *
 ****************************************************************************/

void aurix_pms_init(aurix_pms_config_t *config)
{
  IfxPmsPm_StandbyConfig *standby_config =
    &config->standbyConfig;

  Ifx_PMS *pms = config->pms;

  /* Check wake-up event */

  if (IfxPmsPm_getWakeupEventStatus(pms, IfxPmsPm_WakeupEvent_scr))
    {
      IfxPmsPm_clearWakeupEventStatus(pms, IfxPmsPm_WakeupEvent_scr);
      pwrinfo("Warkup event: SCR");
    }
  else if (IfxPmsPm_getWakeupEventStatus(pms, IfxPmsPm_WakeupEvent_porst))
    {
      IfxPmsPm_clearWakeupEventStatus(pms, IfxPmsPm_WakeupEvent_porst);
      pwrinfo("Warkup event: PORST");
    }
  else
    {
      /* Initialize the PMS */

      IfxPmsPm_startStandbySequenceInFlash(pms, standby_config);
      IfxPmsPm_continueStandbySequenceInRAM(pms, standby_config);
    }

  IfxSmmSysMode_enableGlobalSystemModeEntry();
}

/****************************************************************************
 * Function: aurix_pms_standby
 *
 * Description:
 *   Enter standby mode
 *
 ****************************************************************************/

void aurix_pms_standby(void)
{
#ifdef CONFIG_AURIX_SCR
  aurix_scr_start();
  up_mdelay(60);
#endif
  IfxPmsPm_standbyModeEntry();
}
