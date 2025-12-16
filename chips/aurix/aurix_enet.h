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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ENET__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_ENET__H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arch/chip/chip.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "_Impl/IfxEgtm_cfg.h"
#include "_PinMap/IfxGeth_PinMap_BGA436_COM.h"
#include "Egtm/Tom/Timer/IfxEgtm_Tom_Timer.h"
#include "IfxEgtm_reg.h"
#include "IfxGeth.h"
#include "IfxGeth_Eth.h"
#ifdef CONFIG_AURIX_ENET_USE_PHY
#include "IfxGeth_Phy_Rtl8221b.h"
#endif
#include "IfxGeth_PinMap.h"
#include "IfxHsphy.h"
#include "IfxHsphy_cfg.h"
#include "IfxHsphy_Hsphy.h"
#include "IfxHsphy_reg.h"
#include "IfxPms_reg.h"
#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxSrc_reg.h"

/* MDC: P16.11 */

#define ETH0_MDC1_PIN                    IfxGeth0_P1_MDC_P16_11_OUT

/* MDIO: P16.14 */

#define ETH0_MDIO1_PIN                   IfxGeth0_PX_MDIO_P16_14_INOUT

#ifdef CONFIG_AURIX_ENET_PPS

/* PPS: P14.14 */

#define ETH0_PPS_PIN                     IfxGeth0_P1_PPS_P14_4_OUT
#endif

#define GETH_TSM_GCL_DEPTH               10

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
  IfxHsphy_Geth_MdioPins        g_mdiopins;
};

/* GCL configuration structure */

struct gcl_list
{
  uint8_t           gateon;
  uint32_t          timeinterval;
};

/* QBV configuration structure */

struct qbv_config
{
  uint8_t           enable;
  uint32_t          basetimes;
  uint32_t          basetimens;
  uint32_t          cycletimes;
  uint32_t          cycletimens;
  uint32_t          egtmcomparenum;
  uint32_t          qbvsrcindex;
  IfxEgtm_Tom_Ch    qbvtomtimer0;
  Ifx_Priority      qbvsrcnum;
  uint16_t          gcllen;
  struct gcl_list   gate[GETH_TSM_GCL_DEPTH];
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
 *
 ****************************************************************************/

int aurix_enet_initialize(struct net_driver_s **dev,
                          const struct aurix_enet_config_s *config,
                          size_t num);

/****************************************************************************
 * Name: aurix_enet_get_mac_time_ns
 *
 * Description:
 *   If the ENET interface is up, return
 *   MODULE_GETH0.PORT[1].CORE.MAC_SYSTEM_TIME_NANOSECONDS.U so other drivers
 *   can read the current MAC nanoseconds value. When the interface is down or
 *   not initialized, return a sentinel value.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   MAC nanoseconds when up; -1 otherwise.
 *
 ****************************************************************************/

int aurix_enet_get_mac_time_ns(int port);

/****************************************************************************
 * Name: aurix_enet_mac_done
 *
 * Description:
 *   Check whether the Ethernet MAC on AURIX has been fully initialized and
 *   is ready for normal operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true  - The MAC has completed initialization and is ready.
 *   false - Either the driver has not been probed yet or initialization
 *           has not finished.
 *
 ****************************************************************************/

bool aurix_enet_mac_done(int port);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_SERIAL_H */
