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

#include <stdbool.h>
#include "Egtm/Std/IfxEgtm_Cmu.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_initialize
 *
 * Description:
 *   Initialize the eGTM CMU clock source.
 *
 * Input Parameters:
 *   NONE
 *
 * Returned Value:
 *   NONE
 *
 ****************************************************************************/

void aurix_egtm_initialize(void)
{
  static boolean initialized = FALSE;
  float32 gclk_frequency = 0.0f;

  /* Have we already initialized? */

  if (initialized == FALSE)
    {
      /* Enable the eGTM module. */

      IfxEgtm_enable(&MODULE_EGTM);

      /* Make IfxEgtm_Cluster_0,1,2 frequency neutral. */

      IfxEgtm_setResetProtection(false);
      IfxEgtm_setClusterClockDiv(IfxEgtm_Cluster_0,
        IfxEgtm_ClusterClockDiv_enable);
      IfxEgtm_setClusterClockDiv(IfxEgtm_Cluster_1,
        IfxEgtm_ClusterClockDiv_enable);
      IfxEgtm_setClusterClockDiv(IfxEgtm_Cluster_2,
        IfxEgtm_ClusterClockDiv_enable);
      IfxEgtm_setResetProtection(true);

      /* Enable the eGTM CMU clock0. */

      IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK0);

      gclk_frequency = IfxEgtm_Cmu_getModuleFrequency(&MODULE_EGTM);
      IfxEgtm_Cmu_setClkFrequency(&MODULE_EGTM,
        IfxEgtm_Cmu_Clk_1, gclk_frequency / 16);
      IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK1);
      IfxEgtm_Cmu_setClkFrequency(&MODULE_EGTM,
        IfxEgtm_Cmu_Clk_2, gclk_frequency / 256);
      IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_CLK2);

      /* Enable the eGTM FXU clock. */

      IfxEgtm_Cmu_enableClocks(&MODULE_EGTM, IFXEGTM_CMU_CLKEN_FXCLK);

      /* Mark that we have initialized */

      initialized = TRUE;
    }
}
