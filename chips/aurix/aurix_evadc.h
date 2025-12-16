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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EVADC_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_EVADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <arch/chip/chip.h>

#include "Evadc/Adc/IfxEvadc_Adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Max sequence length */

#define ADC_MAX_SAMPLES            (16U)

typedef struct
{
  IfxEvadc_Adc_Channel           priv_channel;
  IfxEvadc_Adc_ChannelConfig    *channel_config;

  IfxEvadc_RequestSource         request_source;
  uint32_t                       refill;
} channel_s;

struct adc_fifo_data_s
{
  uint32_t                      *fifodatabuffer; /* Data transfer buffer */
  uint8_t                       *fifochbuffer;   /* Channel transfer buffer */
  uint16_t                       fifosize;       /* Fifo size */
};

struct adccmn_data_s
{
  uint8_t                        refcount;
  mutex_t                        lock;
};

struct aurix_evadc_config_s
{
  IfxEvadc_Adc_Config           *adc_config;
  IfxEvadc_Adc_GroupConfig       adc_group_config;

  /* EVADC module handle */

  IfxEvadc_Adc                   evadc_handle;

  /* Group handle */

  IfxEvadc_Adc_Group             group_handle;

  IfxEvadc_GroupId               group_id;
  uint8_t                        nchannels;

  channel_s                      adc_channellist[ADC_MAX_SAMPLES];

  const struct adc_callback_s   *cb;      /* callbacks notify upper driver */
  struct adccmn_data_s          *cmn;     /* Common ADC data */
  struct adc_fifo_data_s         fifo;    /* Adc fifo */

  uint8_t                        current;
  uint32_t                       irq;
  xcpt_t                         isr;

  bool                           adc_convert_flag;
};

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: aurix_adc_initialize
 *
 * Description:
 *   Initialize the ADC. See aurix_adc.c for more details.
 *
 ****************************************************************************/

struct adc_dev_s;

int aurix_adc_initialize(struct adc_dev_s **dev,
                         struct aurix_evadc_config_s *config,
                         size_t count);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ADC__H */