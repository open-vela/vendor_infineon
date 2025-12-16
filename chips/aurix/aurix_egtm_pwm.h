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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_EGTM_PWM_H
#define __VENDOR_INFINEON_CHIPS_AURIX_EGTM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include "Egtm/Tom/Pwm/IfxEgtm_Tom_Pwm.h"

#ifdef CONFIG_AURIX_EGTM_TOM_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_EGTM_TOM_PWM_NCHANNELS (16)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  EGTM_ADC_TRIG_NONE = 0,
  EGTM_ADC_TRIG_ATOMx_0,
  EGTM_ADC_TRIG_ATOMx_1,
  EGTM_ADC_TRIG_ATOMx_2,
  EGTM_ADC_TRIG_ATOMx_3,
  EGTM_ADC_TRIG_ATOMx_4,
  EGTM_ADC_TRIG_ATOMx_5,
  EGTM_ADC_TRIG_ATOMx_6,
  EGTM_ADC_TRIG_ATOMx_7,
  EGTM_ADC_TRIG_TOMx_0,
  EGTM_ADC_TRIG_TOMx_1,
  EGTM_ADC_TRIG_TOMx_2,
  EGTM_ADC_TRIG_TOMx_3,
  EGTM_ADC_TRIG_TOMx_4,
  EGTM_ADC_TRIG_TOMx_5,
  EGTM_ADC_TRIG_TOMx_6,
  EGTM_ADC_TRIG_TOMx_7,
  EGTM_ADC_TRIG_TOMx_8,
  EGTM_ADC_TRIG_TOMx_9,
  EGTM_ADC_TRIG_TOMx_10,
  EGTM_ADC_TRIG_TOMx_11,
  EGTM_ADC_TRIG_TOMx_12,
  EGTM_ADC_TRIG_TOMx_13,
  EGTM_ADC_TRIG_TOMx_14,
  EGTM_ADC_TRIG_TOMx_15,
} egtm_adc_trig_sel_e;

typedef enum
{
  TOM_ADC_TRIG_NONE = 0,                /* The TOM channel is not an ADC trigger. */
  TOM_ADC_TRIG_SEL0 = 0x10,             /* The TOM channel is an ADC trigger SEL0. */
  TOM_ADC_TRIG_SEL1 = 0x11,             /* The TOM channel is an ADC trigger SEL1. */
  TOM_ADC_TRIG_SEL2 = 0x12,             /* The TOM channel is an ADC trigger SEL2. */
  TOM_ADC_TRIG_SEL3 = 0x13,             /* The TOM channel is an ADC trigger SEL3. */
} tom_adc_trig_e;

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
  tom_adc_trig_e          adc_trig;         /* Param of adc trigger select */
  uint16_t                adc_trig_freq;    /* Param of adc trigger frequency */
};

struct aurix_egtm_tom_pwm_dev_s
{
  struct pwm_lowerhalf_s lowerhalf;  /* PWM lower half structure */
  IfxEgtm_Tom_Pwm_Config tom_config; /* Timer configuration structure */
  IfxEgtm_Tom_Pwm_Driver tom_driver; /* Timer Driver structure */
  float32                frequency;  /* Frequency of the PWM signal */
  float32                duty;       /* Duty cycle of the PWM signal */
  boolean                running;    /* Indicates if the PWM is running */
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
 * Name: aurix_egtm_tom_pwm_cfg_initialize
 *
 * Description:
 *   This function initializes the specified eGTM peripheral and the pwm
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   cfgs  - Configuration for the eGTM TOM channel.
 *   devs  - The pointer array of channel device to be initialized.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_egtm_tom_pwm_cfg_initialize(
  const struct aurix_pwm_cfg_s *cfgs,
  struct pwm_lowerhalf_s **devs, size_t count);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_AURIX_EGTM_TOM_PWM */
#endif /* __VENDOR_INFINEON_CHIPS_AURIX_EGTM_PWM_H */
