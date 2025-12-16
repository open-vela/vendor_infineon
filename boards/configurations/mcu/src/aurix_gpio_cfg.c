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
#include "aurix_gpio_cfg.h"

#ifdef CONFIG_GPIO_LOWER_HALF
const struct tc4d9_pin_config_s g_pin_config[] =
{
#if (defined (CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 1))
   { 18, 0, GPIO_OUTPUT_PIN, 0, 2 },
#endif
#if (defined (CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 1))
   { 18, 5, GPIO_OUTPUT_PIN, 0, 3 },
#endif
#if (defined (CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 1))
   { 0, 5, GPIO_INPUT_PIN, 0, 0 },
#endif
#if (defined (CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 1))
   { 0, 6, GPIO_INPUT_PIN, 0, 1 },
#endif

};
#endif //CONFIG_GPIO_LOWER_HALF
