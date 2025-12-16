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
#include "aurix_uart_cfg.h"

struct aurix_uart_config_s g_aurix_uart_config[] =
{
#if defined(CONFIG_AURIX_UART0)
  [0] =
    {
#if (defined(CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 0))

      .base = &MODULE_ASCLIN0,
      .pins =
      {
        .cts       = NULL,
        .ctsMode   = IfxPort_InputMode_pullUp,
        .rx        = &IfxAsclin0_RXA_F_P14_1_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL,
        .rtsMode   = IfxPort_OutputMode_pushPull,
        .tx        = &IfxAsclin0_TX_F_P14_0_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
      },
      .baud = CONFIG_UART0_BAUD,
      .irq  = 173,
      .bus  = 0,
      .poll = false,
    #ifdef CONFIG_SERIAL_TXDMA
      .txdma_idx = IfxDma_Index_0,
      .txdma_channelid = IfxDma_ChannelId_17,
      .txdma_irq = 477,
      .txdma_tos = IfxSrc_Tos_dma0,
      .txdma = {},
    #endif
    #ifdef CONFIG_SERIAL_RXDMA
      .rxdma_idx = IfxDma_Index_0,
      .rxdma_channelid = IfxDma_ChannelId_16,
      .rxdma_irq = 476,
      .rxdma_tos = IfxSrc_Tos_dma0,
      .rxdma = {},
      .rxdma_recvlen = 1,
      .rxdma_buf = NULL,
    #endif
#endif
#if (defined(CONFIG_CPU_COREID) && (CONFIG_CPU_COREID == 1))

      .base = &MODULE_ASCLIN3,
      .pins =
      {
        .cts       = NULL,
        .ctsMode   = IfxPort_InputMode_pullUp,
        .rx        = &IfxAsclin3_RXE_P00_1_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL,
        .rtsMode   = IfxPort_OutputMode_pushPull,
        .tx        = &IfxAsclin3_TX_P00_0_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
      },
      .baud = CONFIG_UART0_BAUD,
      .irq  = 182,
      .bus  = 0,
      .poll = false,
    #ifdef CONFIG_SERIAL_TXDMA
      .txdma_idx = IfxDma_Index_none,
      .txdma_channelid = IfxDma_ChannelId_19,
      .txdma_irq = 479,
      .txdma_tos = IfxSrc_Tos_dma0,
      .txdma = {},
    #endif
    #ifdef CONFIG_SERIAL_RXDMA
      .rxdma_idx = IfxDma_Index_none,
      .rxdma_channelid = IfxDma_ChannelId_18,
      .rxdma_irq = 478,
      .rxdma_tos = IfxSrc_Tos_dma0,
      .rxdma = {},
      .rxdma_recvlen = 1,
      .rxdma_buf = NULL,
    #endif
#endif

    },
#endif
};
