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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_UART__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_UART__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/serial/serial.h>
#include <Asclin/Asc/IfxAsclin_Asc.h>
#include <Dma/Dma/IfxDma_Dma.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_uart_dma_channel_s
{
  IfxDma_Dma                      dma;
  IfxDma_Dma_Channel              dma_ch;
  IfxDma_Dma_ChannelConfig        dma_ch_cfg;
};

struct aurix_uart_config_s
{
  volatile void      *base; /* Base address of UART registers */
  IfxAsclin_Asc_Pins  pins; /* Pin configuration */
  uint32_t            baud; /* Configured baud */
  uint16_t            irq;  /* IRQ associated with this UART */
  int                 bus;  /* UART bus number */
  bool                poll; /* UART poll mode */

#ifdef CONFIG_SERIAL_TXDMA
  /* tx dma configs */

  IfxDma_Index                    txdma_idx;
  IfxDma_ChannelId                txdma_channelid;
  uint16_t                        txdma_irq;
  IfxSrc_Tos                      txdma_tos;
  struct aurix_uart_dma_channel_s txdma;
#endif

#ifdef CONFIG_SERIAL_RXDMA
  /* rx dma configs */

  IfxDma_Index                    rxdma_idx;
  IfxDma_ChannelId                rxdma_channelid;
  uint16_t                        rxdma_irq;
  IfxSrc_Tos                      rxdma_tos;
  struct aurix_uart_dma_channel_s rxdma;
  size_t                          rxdma_recvlen;
  char                           *rxdma_buf;
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_uart_allinitialize
 *
 * Description:
 *   Initialize all uart deivce for aurix, include console and normal uart
 *   device , and register them to VFS, ex: /dev/console, /dev/ttySx.
 *
 ****************************************************************************/

int aurix_uart_allinitialize(struct uart_dev_s **dev);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_SERIAL__H */
