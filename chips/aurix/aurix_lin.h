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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_LIN_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_LIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/net/netdev.h>

#include "Platform_Types.h"
#include "Port/Std/IfxPort.h"
#include "Asclin/Lin/IfxAsclin_Lin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct
{
  Ifx_P *port;
  uint8  pinIndex;
} aurix_lin_pin_config_t;

struct aurix_lin_config_s
{
  IfxAsclin_Lin_Pins     pins;
  Ifx_ASCLIN            *asclin;
  aurix_lin_pin_config_t transceiver;
  uint16_t               rx_irq;
  uint16_t               tx_irq;
  uint16_t               ex_irq;
  uint16_t               port;
  bool                   master;         /* Master / Slave */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_lin_initialize
 *
 * Description:
 *   Initialize the multi lin devices as CAN socket interface
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int aurix_lin_initialize(struct net_driver_s **dev,
                         const struct aurix_lin_config_s *config,
                         size_t num);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_LIN_H */
