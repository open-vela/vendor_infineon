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

#ifndef __VENDOR_INFINEON_CHIP_AURIX_CHIP_H
#define __VENDOR_INFINEON_CHIP_AURIX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "aurix_uart.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct aurix_uart_config_s g_aurix_uart_config[];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lateinitialize
 *
 * Description:
 *   Initialize all chip driver at board_lateinitialize/board_app_initialize
 *
 ****************************************************************************/

void up_lateinitialize(void);

/****************************************************************************
 * Name: up_earlyinitialize
 *
 * Description:
 *   Initialize all chip driver at board_early_initialize
 *
 ****************************************************************************/

void up_earlyinitialize(void);

#endif /* __VENDOR_INFINEON_CHIP_AURIX_CHIP_H */
