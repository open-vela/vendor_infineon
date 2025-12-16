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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_I2S_SIM__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_I2S_SIM__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_i2s.h>
#include <Audio/Std/IfxAudio.h>
#include <Audio/Audio/IfxAudio_Audio.h>
#include <Audio/Tdm/IfxAudio_Tdm.h>
#include <Dma/Dma/IfxDma_Dma.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_i2s_config_s
{
  Ifx_AUDIO                  *amodule;
  IfxAudio_TdmInterfaceIndex  tdm_index;
  IfxAudio_Txsck_Out         *txsck;
  IfxAudio_Txfsync_Out       *txfsync;
  IfxAudio_Txsd_Out          *txdata;

  /* tdm channels */

  uint8_t                     channels;

  /* tdm tx trigger interrupt */

  uint16_t                    tdm_tx_isr_irq;

  /* tdm err trigger interrupt */

  uint16_t                    tdm_err_isr_irq;

  /* dma configs */

  IfxDma_Index                dma_idx;
  IfxDma_ChannelId            dma_channel;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: aurix_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *aurix_i2sbus_initialize(struct aurix_i2s_config_s *config);

/****************************************************************************
 * Name: aurix_all_i2sbus_initialize
 *
 * Description:
 *   Initialize all i2sbus deivce for aurix.
 *
 ****************************************************************************/

int aurix_all_i2sbus_initialize(struct i2s_dev_s **dev,
                                struct aurix_i2s_config_s *config,
                                size_t count);

#ifdef __cplusplus
}
#endif

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_I2S_SIM__H */
