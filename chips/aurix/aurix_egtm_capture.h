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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EGTM_CAPTURE_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EGTM_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/capture.h>

#include "_PinMap/IfxEgtm_PinMap.h"
#include "Egtm/Std/IfxEgtm_Cmu.h"
#include "Egtm/Tim/In/IfxEgtm_Tim_In.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*gtm_capture_callback_t)(void);

enum egtm_capture_mode_e
{
  GTM_CAPTURE_MODE_TPWM = 0,
  GTM_CAPTURE_MODE_TIEM,
};

struct egtm_capture_info_s
{
  uint32_t edge1_tstamp;
  uint32_t edge2_tstamp;
  uint32_t edge1_sequence;
  uint32_t edge2_sequence;
  uint32_t history_edges;
};

struct aurix_gtm_capture_config_s
{
  volatile void            *gtm_base;         /* The base address of eGTM. */
  IfxEgtm_Tim               gtm_tim;          /* The index of TIM in eGTM. */
  IfxEgtm_Tim_Ch            gtm_tim_ch;       /* The channel of the TIM. */
  IfxEgtm_Cmu_Clk           gtm_cmu_clk;      /* The clock for the channel. */
  IfxEgtm_Tim_TinMap       *gtm_tim_pin;      /* The input pin mapping. */
  int                       gtm_tim_ch_irq;   /* The IRQ for the channel. */
  const char               *capture_devpath;   /* The full device path. */
  IfxSrc_Tos                isr_provider;      /* The param of isr provider. */
  float32                   filter_time;       /* The filter time in second. */
  float32                   timeout_time;      /* The timeout time in second. */
};

struct aurix_egtm_capture_dev_s
{
  struct cap_lowerhalf_s      lowerhalf;       /* Lowerhalf driver interface. */
  IfxEgtm_Tim_In              egtm_tim_in;     /* The TIM instance. */
  IfxEgtm_Tim_In_Config       egtm_tim_in_cfg; /* The TIM configuration. */
  enum egtm_capture_mode_e    egtm_cap_mode;   /* The capture mode. */
  gtm_capture_callback_t      egtm_callback;   /* The callback function. */
  uint32_t                    clock;           /* The clock frequency. */
  uint32_t                    tbuclk;          /* The TBUCLK frequency. */
  struct egtm_capture_info_s  capinfo;         /* The capture information. */
  uint32_t                    edges;           /* The pwm capture edges. */
  uint32_t                    freq;            /* The pwm capture frequence. */
  uint8_t                     duty;            /* The pwm capture duty value. */
  uint8_t                     isr_count;       /* The ISR count. */
  bool                        ready;           /* The sample data ready flag. */
  bool                        enabled;         /* The capture enabled flag. */
  const void                 *privinfo;        /* The private information. */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_capture_all_initialize
 *
 * Description:
 *   Initialize and register eGTM capture device.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *   cfgs  - The list of configuration for the eGTM TIM channel.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns error code.
 *
 ****************************************************************************/

int aurix_egtm_capture_all_initialize(struct cap_lowerhalf_s **devs,
  const struct aurix_gtm_capture_config_s *cfgs, size_t count);

/****************************************************************************
 * Name: aurix_egtm_capture_set_callback
 *
 * Description:
 *   Register a callback function, and the callback function will be called
 *   in the interrupt handling function.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *   callback  - The callback function registered to the specified channel.
 *
 * Returned Value:
 *   NONE
 *
 ****************************************************************************/

void aurix_gtm_capture_set_callback(struct cap_lowerhalf_s *dev,
  gtm_capture_callback_t callback);

/****************************************************************************
 * Name: aurix_egtm_capture_set_capmode
 *
 * Description:
 *   Set the working mode of the specified channel. The setting can be
 *   successful only when the channel is in the closed state.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *   mode  - The working mode of the specified eGTM TIM channel.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns error code.
 *
 ****************************************************************************/

int aurix_gtm_capture_set_capmode(struct cap_lowerhalf_s *dev,
  enum egtm_capture_mode_e mode);

/****************************************************************************
 * Name: aurix_egtm_capture_get_pinstate
 *
 * Description:
 *   Obtain the status of the PWM input signal, whether it is high level
 *   or low level. This interface is only valid in the EGTM_CAPTURE_MODE_TIEM
 *   working mode.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *
 * Returned Value:
 *   0 represents a low level and 1 represents a high level.
 *
 ****************************************************************************/

bool aurix_gtm_capture_get_pinstate(struct cap_lowerhalf_s *dev);

/****************************************************************************
 * Name: aurix_egtm_capture_clear_data
 *
 * Description:
 *   Clear all the sampled data.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *
 ****************************************************************************/

void aurix_gtm_capture_clear_data(struct cap_lowerhalf_s *dev);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EGTM_CAPTURE_H */
