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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_GTM_ATOM_PWM_H
#define __VENDOR_INFINEON_CHIPS_AURIX_GTM_ATOM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include "Gtm/Std/IfxGtm_Atom.h"
#include "Gtm/Atom/Pwm/IfxGtm_Atom_Pwm.h"
#include "aurix_gtm_pwm.h"

#ifdef CONFIG_AURIX_GTM_ATOM_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_gtm_atom_pwm_cfg_s
{
  const char              *devpath;        /* The full path to the driver to register */
  IfxGtm_Cluster           atom_id;
  IfxGtm_Atom_Ch        atom_ch_id;
  IfxGtm_Cmu_Clk    atom_ch_clk_id;        /* Param of enum IfxEgtm_Atom_Ch_ClkSrc */
  const void              *pin_map;        /* Param of struct IfxEgtm_Atom_PinMap */
  IfxPort_PadDriver       driver_strength; /* Param of driver strength */
  IfxPort_OutputMode      output_mode;     /* Param of output mode */
};

struct aurix_gtm_atom_pwm_dev_s
{
  struct pwm_lowerhalf_s lowerhalf;
  IfxGtm_Atom_Pwm_Config atom_config;     /* Timer configuration structure */
  IfxGtm_Atom_Pwm_Driver atom_driver;     /* Timer Driver structure */
  IfxGtm_Cmu_Clk         atom_ch_clk;     /* Param of enum IfxEgtm_Atom_Ch_ClkSrc */
  float32                frequency;       /* Frequency of the PWM signal */
  float32                duty;            /* Duty cycle of the PWM signal */
  boolean                running;         /* Indicates if the PWM is running */
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
 * Name: aurix_gtm_atom_pwm_cfg_initialize
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

int aurix_gtm_atom_pwm_cfg_initialize(
  const struct aurix_pwm_cfg_s * cfgs,
  struct pwm_lowerhalf_s ** devs, size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_AURIX_GTM_ATOM_PWM */
#endif /* __VENDOR_INFINEON_CHIPS_AURIX_GTM_ATOM_PWM_H */
