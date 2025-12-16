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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_GTM_PWM_H
#define __VENDOR_INFINEON_CHIPS_AURIX_GTM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include "Gtm/Std/IfxGtm_Tom.h"
#include "Gtm/Tom/Pwm/IfxGtm_Tom_Pwm.h"

#ifdef CONFIG_AURIX_GTM_TOM_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_GTM_TOM_PWM_NCHANNELS (16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  GTM_Module_Type_TOM = 0,
  GTM_Module_Type_ATOM = 1
}Gtm_Pwm_Module_Type;

struct aurix_pwm_cfg_s
{
  const char              *devpath;         /* The full path to the driver to register */
  Gtm_Pwm_Module_Type     module_type;
  uint8                   module_id;
  uint8                   channel_id;
  uint8                   ch_clk_id;        /* Param of enum IfxEgtm_Atom_Ch_ClkSrc */
  const void              *pin_map;         /* Param of struct IfxEgtm_Atom_PinMap */
  IfxPort_PadDriver       driver_strength;  /* Param of driver strength */
  IfxPort_OutputMode      output_mode;      /* Param of output mode */
  IfxSrc_Tos              isr_provider;     /* The param of isr provider. */
};

struct aurix_gtm_tom_pwm_cfg_s
{
  const char *devpath;                  /* The full path to the driver to register */
  int         tom_id;                   /* The index of the GTM-TOM submodule */
  int         tom_ch_id;                /* The channel index of the GTM-TOM */
  int         tom_ch_clk_id;            /* Param of enum IfxGtm_Tom_Ch_ClkSrc */
  const void *pin_map;                  /* Param of struct IfxGtm_Tim_TinMap */
  IfxPort_PadDriver driver_strength;    /* Param of driver strength */
  IfxPort_OutputMode output_mode;       /* Param of output mode */
};

struct aurix_gtm_tom_pwm_dev_s
{
  struct pwm_lowerhalf_s lowerhalf;     /* PWM lower half structure */
  IfxGtm_Tom_Pwm_Config tom_config;     /* Timer configuration structure */
  IfxGtm_Tom_Pwm_Driver tom_driver;     /* Timer Driver structure */
  float32                frequency;     /* Frequency of the PWM signal */
  float32                duty;          /* Duty cycle of the PWM signal */
  boolean                running;       /* Indicates if the PWM is running */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gtm_tom_pwm_cfg_initialize
 *
 * Description:
 *   This function initializes the specified GTM peripheral and the pwm
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   cfgs  - Configuration for the GTM TOM channel.
 *   devs  - The pointer array of channel device to be initialized.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_gtm_tom_pwm_cfg_initialize(
  const struct aurix_pwm_cfg_s *cfgs,
  struct pwm_lowerhalf_s **devs, size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_AURIX_GTM_TOM_PWM */
#endif /* __VENDOR_INFINEON_CHIPS_AURIX_GTM_PWM_H */
