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

#ifndef __AURIX_MCMCAN__H
#define __AURIX_MCMCAN__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/can.h>
#include <nuttx/can.h>
#ifdef CONFIG_AURIX_MCMCAN_CHARDRIVER
#include <nuttx/can/can.h>
#endif

#include "_Impl/IfxVmt_cfg.h"
#include "IfxPort_PinMap_TC4Dx_BGA436_COM.h"
#include "IfxCan_PinMap_TC4Dx_BGA436_COM.h"
#include "Can/Can/IfxCan_Can.h"
#include "Can/Std/IfxCan.h"

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
#include "Compilers/Ifx_Types.h"
#endif

#include "Vmt/Std/IfxVmt.h"
#include "Port/Std/IfxPort.h"
#include "tricore_internal.h"

typedef struct
{
  IfxCan_StdFilterType                type;
  struct can_filter                   filter;
}can_std_filter;

#define RXBUF_MAX_CNT 64
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#if (!defined(CONFIG_AURIX_MCMCAN_CHARDRIVER) && !defined(CONFIG_CAN))
struct can_transv_s;

struct can_transv_ops_s
{
  CODE int (*ct_setstate)(FAR struct can_transv_s *transv, int state);

  CODE int (*ct_getstate)(FAR struct can_transv_s *transv,
                          FAR int *state);
};

struct can_transv_s
{
  FAR const struct can_transv_ops_s *ct_ops;
};
#endif /* !CONFIG_AURIX_MCMCAN_CHARDRIVER and !CONFIG_CAN */

struct aurix_mcmcan_config_s
{
  /* CAN node configuration structure */

  IfxCan_Can_NodeConfig               can_node_config;
  IfxCan_CreConfig                    can_node_cre_config;
  IfxCan_Can_Node                     can_node;

#if (!defined(CONFIG_AURIX_MCMCAN_CHARDRIVER) && !defined(CONFIG_CAN))
  FAR   struct can_transv_s          *can_transv;
#endif

  /* CAN node filter configuration */

  const can_std_filter               *rxfifo0_filter;
  const can_std_filter               *rxfifo1_filter;
  const canid_t                      *rxbuf_filter_id;
  uint8_t                             rxbuf_filter_cnt;
  uint8_t                             rxfifo0_filter_cnt;
  uint8_t                             rxfifo1_filter_cnt;

  /* the flag indicate the CAN node is normal or loopback mode
   * true: loopback mode
   * false:normal mode
   */

  boolean                             loopback_flag;

#ifdef CONFIG_AURIX_MCMCAN_CRE

  /* CAN node unicast and mulcast routing */

  const IfxCan_StdUnicastRouting     *uni_routing;
  uint8                               uni_routing_cnt;
  const IfxCan_StdMulticastRouting   *mul_routing;
  uint8                               mul_routing_cnt;

#endif

  /* structure for CAN transceiver pin */

  IfxPort_Pin                         can_transv_pin;

  /* CAN four interrupt groups */

  uint32                              tx_irq;
  uint32                              rx_irq;
  uint32                              err_irq;
  uint32                              cre_irq;
  uint8                               tx_interrupt_line;
  uint8                               rx_interrupt_line;
  uint8                               err_interrupt_line;
  uint8                               cre_interrupt_line;

  /* registered dev index */

  uint8                               intf;

  /* CAN module sram address index */

  uint8                               module_sram_index;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_chardriver_init_mcmcan
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_MCMCAN_CHARDRIVER

int aurix_chardriver_init_mcmcan(struct can_dev_s **devs,
                                 struct aurix_mcmcan_config_s *config,
                                 size_t num);

#else

/****************************************************************************
 * Name: aurix_socket_init_mcmcan
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

int aurix_socket_init_mcmcan(struct net_driver_s **devs,
                             struct aurix_mcmcan_config_s *config,
                             size_t num);

#endif /* CONFIG_AURIX_MCMCAN_CHARDRIVER */

/****************************************************************************
 * Name: aurix_mcmcan_compare_transv_pinmode
 *
 * Description:
 *   Compare the can transceiver pin mode with the normal mode.
 *
 * Returned Value:
 *   true: the pin mode is not normal mode.
 *   false: the pin mode is normal mode.
 *
 ****************************************************************************/

boolean aurix_mcmcan_compare_transv_pinmode(Ifx_P *port, uint8 pinIndex);

#endif /* __AURIX_MCMCAN__H */
