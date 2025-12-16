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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_PMS_H
#define __VENDOR_INFINEON_CHIPS_AURIX_PMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <IfxPmsPm.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct aurix_pms_config_s
{
  Ifx_PMS *pms;
  IfxPmsPm_StandbyConfig standbyConfig;
} aurix_pms_config_t;

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Function: aurix_voltagerail_init
 *
 * Description:
 *   Initialize the required power rails
 *
 ****************************************************************************/

void aurix_voltagerail_init(void);

/****************************************************************************
 * Function: aurix_vmonp_init
 *
 * Description:
 *   Initialize the precision vmonp protocol function
 *
 ****************************************************************************/

void aurix_vmonp_init(void);

/****************************************************************************
 * Function: aurix_pms_init
 *
 * Description:
 *   Initialize the Power Management System
 *
 ****************************************************************************/

void aurix_pms_init(aurix_pms_config_t *config);

/****************************************************************************
 * Function: aurix_pms_standby
 *
 * Description:
 *   Enter standby mode
 *
 ****************************************************************************/

void aurix_pms_standby(void);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_PMS_H */
