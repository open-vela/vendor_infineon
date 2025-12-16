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

#include "can_filter.h"

#define DECLARE_FILTER(X, Y, Z)  \
{                                \
  .type        = X,               \
  .filter      =                  \
  {                              \
    .can_id    = Y,               \
    .can_mask  = Z,               \
  },                             \
}


/* CAN_0_NODE_0 RX Buffer Filters */
const canid_t can0_node0_std_rxbuf_filter[] =
{
};


/* CAN_0_NODE_0 RX FIFO0 Filters */
const can_std_filter can0_node0_std_rxfifo0_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0xED, 0x202) /* FIFO0 */,

  DECLARE_FILTER(IfxCan_StdFilterType_classic, 0xB1, 0x7FF) /* FIFO1 */

};


/* CAN_0_NODE_0 RX FIFO1 Filters */
const can_std_filter can0_node0_std_rxfifo1_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x211, 0x350) /* FIFO0 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x351, 0x200) /* FIFO1 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x201, 0x204) /* FIFO2 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x301, 0x369) /* FIFO3 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x151, 0xB3) /* FIFO4 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x32D, 0x33B) /* FIFO5 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x243, 0x2C5) /* FIFO6 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x111, 0x1B6) /* FIFO7 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1DD, 0x1DF) /* FIFO8 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1EA, 0x223) /* FIFO9 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x226, 0x2EC) /* FIFO10 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x314, 0x338) /* FIFO11 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x376, 0x90) /* FIFO12 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x112, 0x190) /* FIFO13 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1B1, 0x1B2) /* FIFO14 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1C4, 0x222) /* FIFO15 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x228, 0x25F) /* FIFO16 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x267, 0x317) /* FIFO17 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x37C, 0x3D0) /* FIFO18 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x585, 0x107) /* FIFO19 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x114, 0x1BA) /* FIFO20 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x251, 0x304) /* FIFO21 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x375, 0x392) /* FIFO22 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x3A1, 0x3A7) /* FIFO23 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x3C9, 0x87) /* FIFO24 */,

  DECLARE_FILTER(IfxCan_StdFilterType_range, 0x500, 0x57F) /* FIFO25 */,

  DECLARE_FILTER(IfxCan_StdFilterType_classic, 0x590, 0x7FF) /* FIFO26 */

};


/* CAN_1_NODE_0 RX Buffer Filters */
const canid_t can1_node0_std_rxbuf_filter[] =
{
};


/* CAN_1_NODE_0 RX FIFO0 Filters */
const can_std_filter can1_node0_std_rxfifo0_filter[] =
{
};


/* CAN_1_NODE_0 RX FIFO1 Filters */
const can_std_filter can1_node0_std_rxfifo1_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_classic, 0x1AD, 0x7FF) /* FIFO0 */,

  DECLARE_FILTER(IfxCan_StdFilterType_range, 0x500, 0x57F) /* FIFO1 */

};


/* CAN_0_NODE_2 RX Buffer Filters */
const canid_t can0_node2_std_rxbuf_filter[] =
{
};


/* CAN_0_NODE_2 RX FIFO0 Filters */
const can_std_filter can0_node2_std_rxfifo0_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x204, 0x148) /* FIFO0 */

};


/* CAN_0_NODE_2 RX FIFO1 Filters */
const can_std_filter can0_node2_std_rxfifo1_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1A9, 0x115) /* FIFO0 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x227, 0x258) /* FIFO1 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x2C3, 0x1B3) /* FIFO2 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1C7, 0x252) /* FIFO3 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x323, 0x355) /* FIFO4 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x209, 0x20A) /* FIFO5 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x248, 0x249) /* FIFO6 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x2F1, 0x2F2) /* FIFO7 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1A3, 0x1BB) /* FIFO8 */,

  DECLARE_FILTER(IfxCan_StdFilterType_range, 0x500, 0x57F) /* FIFO9 */,

  DECLARE_FILTER(IfxCan_StdFilterType_classic, 0x788U, 0x7FF) /* FIFO10 */,

  DECLARE_FILTER(IfxCan_StdFilterType_classic, 0x7DFU, 0x7FF) /* FIFO11 */

};


/* CAN_1_NODE_2 RX Buffer Filters */
const canid_t can1_node2_std_rxbuf_filter[] =
{
};


/* CAN_1_NODE_2 RX FIFO0 Filters */
const can_std_filter can1_node2_std_rxfifo0_filter[] =
{
};


/* CAN_1_NODE_2 RX FIFO1 Filters */
const can_std_filter can1_node2_std_rxfifo1_filter[] =
{
  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1B7, 0x257) /* FIFO0 */,

  DECLARE_FILTER(IfxCan_StdFilterType_dualId, 0x1B8, 0x25C) /* FIFO1 */,

  DECLARE_FILTER(IfxCan_StdFilterType_range, 0x500, 0x57F) /* FIFO2 */

};

