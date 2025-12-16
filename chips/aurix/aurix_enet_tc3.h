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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ENET_TC3__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ENET_TC3__H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arch/chip/chip.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "IfxGeth.h"
#include "IfxGeth_Eth.h"

#include "IfxGeth_PinMap.h"
#include "IfxPms_reg.h"
#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxSrc_reg.h"
#include "IfxGeth_Phy_Mvlq1110.h"

#ifdef CONFIG_AURIX_ENET_PPS
/* PPS: P14.4 */
#define ETH_PPS_PIN         IfxGeth_PPS_P14_4_OUT
#endif

#define ETH_RMII_CRSDVB     IfxGeth_CRSDVB_P11_14_IN
#define ETH_RMII_REFCLKA    IfxGeth_REFCLKA_P11_12_IN
#define ETH_RMII_RXD0       IfxGeth_RXD0A_P11_10_IN
#define ETH_RMII_RXD1       IfxGeth_RXD1A_P11_9_IN
#define ETH_RMII_MDC        IfxGeth_MDC_P12_0_OUT
#define ETH_RMII_MDIO       IfxGeth_MDIO_P12_1_INOUT
#define ETH_RMII_TXD0       IfxGeth_TXD0_P11_3_OUT
#define ETH_RMII_TXD1       IfxGeth_TXD1_P11_2_OUT
#define ETH_RMII_TXEN       IfxGeth_TXEN_P11_6_OUT

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_enet_config_s
{
  bool                          enable_queues[IFXGETH_NUM_RX_QUEUES];
  uint16_t                      rx_irq[IFXGETH_NUM_RX_CHANNELS];
  uint16_t                      tx_irq[IFXGETH_NUM_TX_CHANNELS];
#ifdef CONFIG_AURIX_ENET_PTP
  uint16_t                      sys_irq;
#endif
#ifdef CONFIG_AURIX_ENET_PPS
  IfxGeth_Pps_Out               *pps_pin;
#endif
  uint8_t                       port;
  uint8_t                       nchans;
  IfxGeth_Eth_RmiiPins          rmiiPins;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_enet_initialize
 *
 * Description:
 *   Initialize the selected lin port as CAN socket interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple lin interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 ****************************************************************************/

int aurix_enet_initialize(struct net_driver_s **dev,
                          const struct aurix_enet_config_s *config,
                          size_t num);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ENET_TC3__H */
