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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_TMADC_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_TMADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <arch/chip/chip.h>

#include "Adc/Tmadc/IfxAdc_Tmadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Max sequence length */

#define ADC_MAX_SAMPLES     (16U)
#define ADC_MAX_MONITOR_NUM (2U)
#define TMADC_MAX_NUMS      (4u)
#define ADC_PATH_LEN        (16)

#define ADC_IQR_MIN         (880)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct adc_fifo_data_s
{
  uint32_t *fifodatabuffer;              /* Fifo data transfer buffer */
  uint8_t  *fifochbuffer;                /* Fifo channel transfer buffer */
  uint16_t  fifosize;                    /* Fifo size */
};

struct adccmn_data_s
{
  uint8_t refcount;
  mutex_t lock;
};

typedef struct
{
  IfxAdc_Tmadc_Config          module_config;
  uint32_t                     irq_nums[IFXADC_TMADC_MAX_SERV_REQ_NODE];
}aurix_tmadc_module_config_s;

struct aurix_tmadc_config_s
{
  char                         path[ADC_PATH_LEN];
  IfxAdc_Tmadc_Config          adc_config;

  uint32_t                     irq;         /* Interrupt number */

  IfxAdc_TmadcModule           module_id;
  uint8_t                      nchannels;
  uint8_t                      group_id;
  IfxAdc_Tmadc_ChConfig       *g_adc_channellist[ADC_MAX_SAMPLES];

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL
  uint8_t                      nchannels_monitor;

  IfxAdc_Tmadc_MonitorChannelConfig
                              *adc_monitor_channellist[ADC_MAX_MONITOR_NUM];
#endif
};

struct aurix_adc_priv_s
{
  const struct aurix_tmadc_config_s *config; /* Pointer to config structure */

  IfxAdc_Tmadc                 adc_handle;
  IfxAdc_Tmadc_Group           group_handle;
  Ifx_ADC                     *tmadc_module_handle;

  IfxAdc_TmadcModule           module_id;
  uint8_t                      nchannels;
  uint8_t                      group_id;

  IfxAdc_Tmadc_Ch              g_adc_channel[ADC_MAX_SAMPLES];

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL
  uint8_t                      nchannels_monitor;
  IfxAdc_Tmadc_MonitorCh       g_adc_monitor_channel[ADC_MAX_MONITOR_NUM];
#endif

  uint8_t                      current;    /* Current converted ADC channel */
  uint32_t                     irq;        /* Interrupt number */
  xcpt_t                       isr;        /* Interrupt handler */

  const struct adc_callback_s *cb;         /* callbacks notify upper driver */
  struct adccmn_data_s        *cmn;        /* Common ADC data */
  struct adc_fifo_data_s       fifo;       /* Adc fifo */

  bool                         adc_convert_flag;

#ifdef CONFIG_ADC_USE_DMA
  uint8_t                      *pchlistsbuffer;
  void                         (*call_back)(uint32_t *dma_buf, uint8_t len);

#endif
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
 *   Initialize the ADC. See aurix_tmadc.c for more details.
 *
 ****************************************************************************/

size_t aurix_adc_initialize(struct adc_dev_s **dev,
                            const struct aurix_tmadc_config_s *config,
                            size_t count);

void aurix_adc_module_config(const aurix_tmadc_module_config_s *config,
                             size_t count);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_TMADC__H */