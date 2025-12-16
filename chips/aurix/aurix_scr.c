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

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>

#include "Pms/Std/IfxPmsEvr.h"
#include "Stm/Std/IfxStm.h"

#include "aurix_scr.h"
#include "scr/scr_core_code.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper macro to calculate XRAM memory address information for
 * RTC data exchange
 */

 #define XRAM_EXCHANGE_ADDRESS(offset)         (uint32_t *)((uint8_t *)PMS_XRAM + \
 (offset & (PMS_XRAM_SIZE - 1)))
#define XRAM_EXCHANGE_OFFSET (0x7F00)
#define XRAM_EXCHANGE_SIGNAL (0x7F08)

/* Expected RTC period in milliseconds */

#define SCR_RTC_CALIBRATION_FACTOR (0.959f)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: aurix_config_pin_to_scr
 *
 * Description:
 *  allocate  pin to SCR
 *
 ****************************************************************************/

void aurix_config_pin_to_scr(scr_io_config_t *config, uint8_t size)
{
  int i;

  DEBUGASSERT(config != NULL);
  DEBUGASSERT(size <= 32);

  for (i = 0; i < size; i++)
    {
      IfxPort_setPinControllerSelection(config[i].port, config[i].pinIndex);
    }
}

/****************************************************************************
 * Function: set_rtc_period_on_70khz
 *
 * Description:
 *   Set the RTC period on 70kHz
 *
 ****************************************************************************/

static void set_rtc_period_on_70khz(uint32_t const ms_period,
                                    float32 const calib_coeff,
                                    uint32_t *const exchange_address)
{
  uint32_t tick_counter = (uint32_t)(70 * 1000 * ms_period * calib_coeff);

  *exchange_address = tick_counter;
}

/****************************************************************************
 * Function: set_rtc_period_on_32khz
 *
 * Description:
 *   Set the RTC period on 32kHz
 *
 ****************************************************************************/

static void set_rtc_period_on_32khz(uint32_t const s_period,
                                    uint32_t *const exchange_address)
{
  uint32_t tick_counter = (uint32_t)(32768 * s_period);

  *exchange_address = tick_counter;
}

/****************************************************************************
 * Function: scrrtc_cfg_update
 *
 * Description:
 *   Update the RTC period on 70kHz
 *
 ****************************************************************************/

static void scrrtc_cfg_update(uint32 const value,
                              uint32_t *const exchange_address)
{
  uint32_t tick_counter = (uint32_t)(value);

  *exchange_address = tick_counter;
}

/****************************************************************************
 * Function: enable_32khz_oscillator
 *
 * Description:
 *   Enable the 32KHz oscillator
 *
 ****************************************************************************/

static void enable_32khz_oscillator(void)
{
  /* Enable 32KHz oscillator */

  IfxPmsPm_enableRtcOscillator(&MODULE_PMS, IfxPmsPm_Rtc_enable);
  IfxPmsPm_selectRtcClockSource(&MODULE_PMS, IfxPmsPm_RtcClk_32Khz);
}

/****************************************************************************
 * Function: aurix_close_scr
 *
 * Description:
 *   Close the SCR
 *
 ****************************************************************************/

void aurix_scr_init(aurix_scr_config_t *config)
{
  uint8_t index = 0;
  uint32_t wake_up_time = config->wake_up_time;
  IfxPmsPm_MemoryConfig *xram = &config->memoryConfig;

  /* Assign pins to the SCR */

  if (config->scr_io_size != 0)
    {
      aurix_config_pin_to_scr(config->scr_io, config->scr_io_size);
    }

  if (config->rtc_clock_source == RTC_CLOCK_SOURCE_70KHZ)
    {
      set_rtc_period_on_70khz(wake_up_time, SCR_RTC_CALIBRATION_FACTOR,
                            XRAM_EXCHANGE_ADDRESS(XRAM_EXCHANGE_OFFSET));
    }
  else if (config->rtc_clock_source == RTC_CLOCK_SOURCE_32KHZ)
    {
      set_rtc_period_on_32khz(wake_up_time,
                            XRAM_EXCHANGE_ADDRESS(XRAM_EXCHANGE_OFFSET));

      /* enable 32khz oscillator */

      enable_32khz_oscillator();
    }

  /* enable wake-up time cfg */

  scrrtc_cfg_update(0x01, XRAM_EXCHANGE_ADDRESS(XRAM_EXCHANGE_SIGNAL));

  aurix_scr_close();

  if (PMS_SCR_CON0.B.SCREN == 0)
    {
      if (xram != NULL_PTR)
        {
          IfxPmsPm_copyData(xram->src, xram->dest, xram->size);

          /* Copy SCR magic pattern to the end */

          uint16_t *xramptr = \
          (uint16_t *)IFXPMS_SCR_XRAM_MAGIC_PATTERN_ADDRESS;

          for (index = 0; index < 4; index++)
            {
              xramptr[index] = 0xaa55;
            }
        }
    }
}

/****************************************************************************
 * Function: aurix_scr_close
 *
 * Description:
 *   Close the SCR
 *
 ****************************************************************************/

void aurix_scr_close(void)
{
  Ifx_PMS_SCR_CON0 scrcon0;

  scrcon0.U = 0;

  /* SCREN can be updated. */

  scrcon0.B.SCREN_P = 1;

  /* Disable SCR */

  scrcon0.B.SCREN = 0;
  if (PMS_SCR_CON0.B.SCREN == 1)
    {
      PMS_SCR_CON0.U = scrcon0.U;
    }
}

/****************************************************************************
 * Function: aurix_scr_start
 *
 * Description:
 *   Close the SCR
 *
 ****************************************************************************/

boolean aurix_scr_start(void)
{
  bool status = FALSE;
  bool scrstatus;
  uint32_t timeout_val = IFXPMS_SCR_ENABLE_TIMEOUT;
  Ifx_PMS_SCR_CON0 scrcon0;

  if (PMS_SCR_CON0.B.SCREN == 0)
    {
      /* Configure bootmode and enable/disable reset on warm Porst */

      scrcon0.U = PMS_SCR_CON0.U;
      scrcon0.B.PORSTREQ_P = 1u;
      scrcon0.B.PORSTREQ = FALSE;
      scrcon0.B.SCRCFG = IfxPmsPm_ScrBootMode_userMode;
      scrcon0.B.SCREN_P = 1u;
      scrcon0.B.SCREN = 1u;

      PMS_SCR_CON0.U = scrcon0.U;

      /* Wait for SCR to enable */

      do
        {
          scrstatus = PMS_SCR_CON0.B.SCREN;
          timeout_val--;

          if ((timeout_val == 0U) && (scrstatus != 1U))
            {
              /* Timeout happened and scr is not enabled yet */

              status = TRUE;
            }
        }
      while ((scrstatus != 1U) && (timeout_val > 0U));
    }

  return status;
}
