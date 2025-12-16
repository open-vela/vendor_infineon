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
#include "Gtm/Std/IfxGtm_Cmu.h"
#include "Gtm/Std/IfxGtm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gtm_initialize
 *
 * Description:
 *   Initialize the GTM CMU clock source.
 *
 * Input Parameters:
 *   NONE
 *
 * Returned Value:
 *   NONE
 *
 ****************************************************************************/

void aurix_gtm_initialize(void)
{
  static boolean initialized = FALSE;

  /* Have we already initialized? */

  if (initialized == FALSE)
    {
      /* Enable the GTM module. */

      IfxGtm_enable(&MODULE_GTM);

      /* Enable the GTM CMU clock0. */

      IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);

      /* Enable the GTM FXU clock. */

      IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);

      /* Mark that we have initialized */

      initialized = TRUE;
    }
}
