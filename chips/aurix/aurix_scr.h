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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_SCR_H
#define __VENDOR_INFINEON_CHIPS_AURIX_SCR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "IfxPmsPm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  RTC_CLOCK_SOURCE_70KHZ  (0)
#define  RTC_CLOCK_SOURCE_32KHZ  (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct scr_io_config_s
{
  Ifx_P *port;
  uint8 pinIndex;
}scr_io_config_t;

typedef struct aurix_scr_config_s
{
  scr_io_config_t *scr_io;
  uint8_t scr_io_size;
  uint8_t rtc_clock_source;
  IfxPmsPm_MemoryConfig memoryConfig;
  uint32_t wake_up_time;
} aurix_scr_config_t;

/****************************************************************************
 * Function: aurix_scr_init
 *
 * Description:
 *   Initialize the scr
 *
 ****************************************************************************/

void aurix_scr_init(aurix_scr_config_t *config);

/****************************************************************************
 * Function: aurix_scr_close
 *
 * Description:
 *   Close the Scr
 *
 ****************************************************************************/

void aurix_scr_close(void);

/****************************************************************************
 * Function: aurix_scr_start
 *
 * Description:
 *   Start the Scr
 *
 ****************************************************************************/

boolean aurix_scr_start(void);

#endif
