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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>
#include <nuttx/irq.h>
#include <nuttx/lin.h>
#include <nuttx/net/can.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>

#include "tricore_internal.h"
#include "aurix_lin.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define POOL_SIZE            1
#define LIN_SLAVE_CACHE_NUM  (1u << LIN_ID_BITS)
#define PIN_LOW              0
#define SLEEP_PID            (0x3c)

/* LIN error id or event id. The value 0 indicates successful operation, the
 * values 1~7 are consistent with enum lin_state_event_e for state change
 * events, and other values indicate error events defined in lin.h.
 */

#define AURIX_LIN_EV_WKUP_BUS          2u
#define AURIX_LIN_EV_MAXID             6u
#define AURIX_LIN_TX_OK                7u
#define AURIX_LIN_ERR_BREAK_TMO        8u
#define AURIX_LIN_ERR_TXSYNC_TMO       9u
#define AURIX_LIN_ERR_TXPID_TMO        10u
#define AURIX_LIN_ERR_RXNORESP         16u
#define AURIX_LIN_ERR_RXCKSUM          19u
#define AURIX_LIN_ERR_RXSYNC_TMO       20u
#define AURIX_LIN_ERR_RXPID_TMO        22u
#define AURIX_LIN_ERR_PIDPARITY        23u
#define AURIX_LIN_ERR_BUS_PID          24u
#define AURIX_LIN_ERR_BUS_DATA         27u
#define AURIX_LIN_ERR_CTRL_RXOVERFLOW  32u
#define AURIX_LIN_ERR_CTRL_FRAMEERROR  33u
#define AURIX_LIN_ERR_CTRL_NOISEERROR  34u
#define AURIX_LIN_ERR_UNKNOWN          255u

/* LIN event */

#define AURIX_LIN_EVENT_IDLE           0u
#define AURIX_LIN_EVENT_WAKEUP         7u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_lin_priv_s
{
  struct net_driver_s              dev;
  const struct aurix_lin_config_s *config;
  IfxAsclin_Lin                    module;
  struct work_s                    pollwork;
  struct work_s                    txwork;
  struct work_s                    rxwork;
  struct work_s                    exwork;

  /* TX/RX pool */

  struct can_frame                 txdesc[POOL_SIZE];
  struct can_frame                 rxdesc[POOL_SIZE];

  bool                             bifup;  /* true:ifup false:ifdown */
  spinlock_t                       lock;
  uint8_t                          state;
  uint8_t                          event;  /* Idle event/passive wake-up event */
  struct work_s                    delaywork;
  bool                             reported_error;
  struct can_frame                 frame_cache[0];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_lin_ifup(struct net_driver_s *dev);
static int aurix_lin_ifdown(struct net_driver_s *dev);
static int aurix_lin_txavail(struct net_driver_s *dev);
static void aurix_lin_txavail_work(void *arg);
#ifdef CONFIG_NETDEV_IOCTL
static int aurix_lin_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                  unsigned long arg);
static void aurix_lin_wakeup_bus(struct aurix_lin_priv_s *priv);
static int aurix_lin_cmd_wakeup(struct aurix_lin_priv_s *priv,
                                bool internal);
static void aurix_lin_sendsleepframe(struct aurix_lin_priv_s *priv);
static int aurix_lin_cmd_sleep(struct aurix_lin_priv_s *priv, bool internal);
#endif
static bool aurix_lin_is_sleep_signal(struct aurix_lin_priv_s *priv);
static void aurix_lin_setidle(struct aurix_lin_priv_s *priv);
#ifdef CONFIG_AURIX_LIN_IDLE_TO_SLEEP
static void aurix_lin_bus_enter_sleep(void *arg);
#endif
static void aurix_lin_start_sleep_timer(struct aurix_lin_priv_s *priv);
static int aurix_lin_bus_wakeup_handler(struct aurix_lin_priv_s *priv);
static int aurix_lin_txpoll(struct net_driver_s *dev);
static int aurix_lin_interrupt(int irq, void *context, void *arg);
static void aurix_lin_rxhandler(void *arg);
static void aurix_lin_txhandler(void *arg);
static void aurix_lin_exhandler(void *arg);
static bool aurix_lin_txready(struct aurix_lin_priv_s *priv);
static uint8_t aurix_lin_datalength(uint8_t pid, uint8_t can_dlc);
static void aurix_lin_buildframe(struct aurix_lin_priv_s *priv,
                                 struct can_frame *frame, uint8_t event);
static void aurix_lin_frame_report(struct aurix_lin_priv_s *priv,
                                   struct can_frame *frame);
static void aurix_lin_receive_frame(struct aurix_lin_priv_s *priv,
                                    uint8_t pid);
static void lin_pin_initilize(const struct aurix_lin_config_s *config);
static void aurix_lin_set_error_interrupts(Ifx_ASCLIN *asclinSFR,
                                           bool enable);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_lin_frame_report
 *
 * Description:
 *   report a new lin frame to upper-half
 *
 ****************************************************************************/

static void aurix_lin_frame_report(struct aurix_lin_priv_s *priv,
                                   struct can_frame *frame)
{
  net_lock();

  /* Copy the buffer pointer to priv->dev..  Set amount of data
   * in priv->dev.d_len
   */

  priv->dev.d_len = sizeof(struct can_frame);
  priv->dev.d_buf = (uint8_t *)frame;

  /* Send to socket interface */

  NETDEV_RXPACKETS(&priv->dev);

  can_input(&priv->dev);

  /* Point the packet buffer back to the next Tx buffer that will be
   * used during the next write.  If the write queue is full, then
   * this will point at an active buffer, which must not be written
   * to.  This is OK because devif_poll won't be called unless the
   * queue is not full.
   */

  priv->dev.d_buf = (uint8_t *)priv->txdesc;
  net_unlock();
}

/****************************************************************************
 * Name: aurix_lin_receive_frame
 *
 * Description:
 *   Get RX response info from illd and report it to upper-half
 *
 ****************************************************************************/

static void aurix_lin_receive_frame(struct aurix_lin_priv_s *priv,
                                    uint8_t pid)
{
  IfxAsclin_Lin                  *asclin = &priv->module;
  IfxAsclin_Lin_FrameDataControl *linfrm = &asclin->linFrameData;
  struct can_frame *frame;

  if (asclin->acknowledgmentFlags.rxResponseEnd
      && linfrm->rxResponseLength > 0)
    {
      frame          = priv->rxdesc;
      frame->can_id  = pid;
      frame->can_dlc = linfrm->rxResponseLength;

      memcpy(frame->data, linfrm->rxResponseData, linfrm->rxResponseLength);

      aurix_lin_frame_report(priv, frame);
    }
}

/****************************************************************************
 * Name: aurix_lin_slave_send_resp
 *
 * Description:
 *   Slave node send response to the bus
 *
 ****************************************************************************/

static void aurix_lin_slave_send_resp(IfxAsclin_Lin *asclin,
                                      uint8 *data, uint32 length)
{
  Ifx_ASCLIN *asclinSFR = asclin->asclin;

  /* set number of bytes to be transfered */

  IfxAsclin_setDataLength(asclinSFR, (IfxAsclin_DataLength)(length - 1));
  IfxAsclin_clearAllFlags(asclinSFR);                  /* clear all flags */
  IfxAsclin_flushTxFifo(asclinSFR);                    /* flushing Tx FIFO */
  IfxAsclin_enableRxFifoInlet(asclinSFR, FALSE);       /* disable Rx FIFO */
  IfxAsclin_enableTxFifoOutlet(asclinSFR, TRUE);       /* enable Tx FIFO */
  IfxAsclin_write8(asclinSFR, data, length);           /* write the data bytes */
  IfxAsclin_setTransmitResponseRequestFlag(asclinSFR); /* set TRRQS flag */
}

/****************************************************************************
 * Name: aurix_lin_master_tx_done
 *
 * Description:
 *   Master node send response successfully
 *
 ****************************************************************************/

static void aurix_lin_master_tx_done(struct aurix_lin_priv_s *priv,
                                     uint8_t pid)
{
  /* Notify to upper-half if transmit response , tx confirmation */

  if (priv->module.acknowledgmentFlags.txResponseEnd)
    {
      struct can_frame *frame = priv->rxdesc;

      if (aurix_lin_is_sleep_signal(priv))
        {
          return;
        }

      irqstate_t flags = spin_lock_irqsave(&priv->lock);
      aurix_lin_start_sleep_timer(priv);
      spin_unlock_irqrestore(&priv->lock, flags);

      frame->can_id  = pid;
      frame->can_dlc = 0;
      aurix_lin_buildframe(priv, frame, AURIX_LIN_TX_OK);
      aurix_lin_frame_report(priv, frame);

      /* There should be space for a new TX in any event.  Poll the network
       *  for new XMIT data
       */

      net_lock();
      devif_poll(&priv->dev, aurix_lin_txpoll);
      net_unlock();
    }
}

/****************************************************************************
 * Name: aurix_lin_slave_tx_done
 *
 * Description:
 *   Slave node send response successfully
 *
 ****************************************************************************/

static void aurix_lin_slave_tx_done(struct aurix_lin_priv_s *priv,
                                    uint8_t pid)
{
  irqstate_t flags;

  /* Slave send response done */

  flags = spin_lock_irqsave(&priv->lock);
  aurix_lin_start_sleep_timer(priv);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: aurix_lin_master_rx
 *
 * Description:
 *   Master node receive header & response successfully
 *
 ****************************************************************************/

static void aurix_lin_master_rx(struct aurix_lin_priv_s *priv, uint8_t pid)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  aurix_lin_start_sleep_timer(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  aurix_lin_receive_frame(priv, pid);
}

/****************************************************************************
 * Name: aurix_lin_slave_rx
 *
 * Description:
 *   Slave node receive header or response
 *
 ****************************************************************************/

static void aurix_lin_slave_rx(struct aurix_lin_priv_s *priv, uint8_t pid)
{
  IfxAsclin_Lin *asclin   = &priv->module;
  struct can_frame *frame = &priv->frame_cache[pid];
  IfxAsclin_Lin_AcknowledgementFlags *ack_flags =
                                          &asclin->acknowledgmentFlags;
  uint8_t state;
  irqstate_t flags;

  if (!(frame->can_id & LIN_RTR_FLAG  || frame->can_id & LIN_CACHE_RESPONSE))   /* Ignore PID */
    {
      IfxAsclin_Lin_ignoreHeader(asclin);
      return;
    }

  flags = spin_lock_irqsave(&priv->lock);
  state = priv->state;

  if (ack_flags->rxHeaderEnd && state == CAN_STATE_OPERATIONAL)
    {
      priv->state = CAN_STATE_BUSY;
      spin_unlock_irqrestore(&priv->lock, flags);

      if (frame->can_id & LIN_CACHE_RESPONSE)
        {
          aurix_lin_slave_send_resp(asclin, frame->data, frame->can_dlc);

          /* Single response, clear cache */

          if (frame->can_id & LIN_SINGLE_RESPONSE)
            {
              memset(frame, 0, CAN_MTU);
            }
        }
      else if ((frame->can_id & LIN_RTR_FLAG))
        {
          priv->module.linFrameData.rxResponseLength = frame->can_dlc;

          /* prepare the response reception */

          IfxAsclin_Lin_prepareResponseReception(asclin, frame->can_dlc);
        }
    }
  else if (state == CAN_STATE_BUSY && ack_flags->rxResponseEnd)
    {
      /* Slave receive the response */

      /* Slave node check single response mode */

      if (aurix_lin_is_sleep_signal(priv))
        {
          spin_unlock_irqrestore(&priv->lock, flags);
          return;
        }

      aurix_lin_start_sleep_timer(priv);

      spin_unlock_irqrestore(&priv->lock, flags);

      aurix_lin_receive_frame(priv, pid);

      IfxAsclin_Lin_prepareHeaderReception(asclin);
      IfxAsclin_Lin_clearFlagsStatus(asclin);

      return;
    }
  else
    {
      aurix_lin_setidle(priv);
      spin_unlock_irqrestore(&priv->lock, flags);

      /* Ignore unknown rx event */

      nerr("Unknown rx event, state: %d, HeaderEnd: %d, ResponseEnd: %d\n",
           state, ack_flags->rxHeaderEnd, ack_flags->rxResponseEnd);
      IfxAsclin_Lin_prepareHeaderReception(asclin);
      IfxAsclin_Lin_clearFlagsStatus(asclin);
    }
}

/****************************************************************************
 * Name: aurix_lin_rxhandler
 *
 * Description:
 *   RX interrupt process
 *
 ****************************************************************************/

static void aurix_lin_rxhandler(void *arg)
{
  struct aurix_lin_priv_s *priv = arg;
  uint8_t pid;

  /* Call illd API to process */

  IfxAsclin_Lin_isrReceive(&priv->module);

  pid = priv->module.linFrameData.headerID & LIN_ID_MASK;

  if (priv->config->master)
    {
      aurix_lin_master_rx(priv, pid);
    }
  else
    {
      aurix_lin_slave_rx(priv, pid);
    }
}

/****************************************************************************
 * Name: aurix_lin_txhandler
 *
 * Description:
 *   TX interrupt process
 *
 ****************************************************************************/

static void aurix_lin_txhandler(void *arg)
{
  struct aurix_lin_priv_s *priv = arg;
  uint8_t pid;

  /* Call illd API to process */

  IfxAsclin_Lin_isrTransmit(&priv->module);

  pid = priv->module.linFrameData.headerID & LIN_ID_MASK;

  if (priv->config->master)
    {
      aurix_lin_master_tx_done(priv, pid);
    }
  else
    {
      aurix_lin_slave_tx_done(priv, pid);
    }
}

/****************************************************************************
 * Name: aurix_lin_buildframe
 *
 * Description:
 *   This function builds a CAN frame based on the given event type.
 *   It sets the CAN frame ID and data based on whether the event is a
 *   transmission success, a state change, or an error.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *   frame - Pointer to the CAN frame structure to be populated.
 *   event - The event type which determines the content of the CAN frame.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   - The 'frame' pointer is not NULL.
 *   - The 'event' value is within the expected range.
 *
 ****************************************************************************/

static void aurix_lin_buildframe(struct aurix_lin_priv_s *priv,
                                 struct can_frame *frame, uint8_t event)
{
  if (event == AURIX_LIN_TX_OK)
    {
      frame->can_id |= LIN_TCF_FLAG;
    }
  else if (event <= AURIX_LIN_EV_MAXID)
    {
      /* State change event, build a state change event frame */

      frame->can_id = LIN_EVT_FLAG;
      frame->data[0u] = 1u << (event - 1u);
    }
  else
    {
      /* Error event, build an error frame */

      frame->can_id |= LIN_ERR_FLAG;
      frame->data[0] = 1u << (event / 8u - 1u);
      frame->data[event / 8u] = 1u << (event % 8u);
    }
}

/****************************************************************************
 * Name: aurix_lin_exhandler
 *
 * Description:
 *   Error interrupt process
 *
 ****************************************************************************/

static void aurix_lin_exhandler(void *arg)
{
  struct aurix_lin_priv_s *priv = arg;
  IfxAsclin_Lin_FrameControlFlags *pflag = &priv->module.linFrameData.flags;
  struct can_frame *frame = priv->txdesc;
  uint8_t errorflag = 0;
  irqstate_t flags;

  if (priv->state == CAN_STATE_SLEEP &&
      IfxAsclin_getFallingEdgeDetectedFlagStatus(priv->module.asclin))
    {
      aurix_lin_bus_wakeup_handler(priv);
      return;
    }

  /* Call illd API to process */

  IfxAsclin_Lin_isrError(&priv->module);

  /* Notify to upper-half if error happens */

  if (priv->module.errorFlagsStatus.frameError)
    {
      errorflag = priv->config->master ?
      AURIX_LIN_ERR_TXSYNC_TMO : AURIX_LIN_ERR_RXSYNC_TMO;
    }
  else if (priv->module.errorFlagsStatus.linParityError)
    {
      errorflag = AURIX_LIN_ERR_PIDPARITY;
    }
  else if (priv->module.errorFlagsStatus.headerTimeout)
    {
      errorflag = priv->config->master ?
      AURIX_LIN_ERR_TXPID_TMO : AURIX_LIN_ERR_RXPID_TMO;
    }
  else if (priv->module.errorFlagsStatus.responseTimeout)
    {
      errorflag = AURIX_LIN_ERR_RXNORESP; /* Replenish RXFIFOCON.B.FILL == 1 */
    }
  else if (priv->module.errorFlagsStatus.linChecksumError)
    {
      errorflag = AURIX_LIN_ERR_RXCKSUM;
    }
  else if (priv->module.errorFlagsStatus.collisionDetectionError)
    {
      if (pflag->txHeaderErrorOccurred ||
          pflag->rxHeaderErrorOccurred)
        {
          errorflag = AURIX_LIN_ERR_BUS_PID;
        }
      else if (pflag->txResponseErrorOccurred ||
               pflag->rxResponseErrorOccurred)
        {
          errorflag = AURIX_LIN_ERR_BUS_DATA; /* CE Need to be phased, According to TH ... */
        }
    }
  else if (priv->module.errorFlagsStatus.rxFifoOverflow)
    {
      errorflag = AURIX_LIN_ERR_CTRL_RXOVERFLOW;
    }
  else
    {
      errorflag = AURIX_LIN_ERR_UNKNOWN;
    }

  if (errorflag != 0 && !priv->reported_error)
    {
      flags = spin_lock_irqsave(&priv->lock);
      priv->reported_error = true;
      spin_unlock_irqrestore(&priv->lock, flags);
      aurix_lin_buildframe(priv, frame, errorflag);
      aurix_lin_frame_report(priv, frame);
    }

  IfxAsclin_clearAllFlags(priv->module.asclin); /* clear all register flags */

  flags = spin_lock_irqsave(&priv->lock);
  aurix_lin_setidle(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Notify to upper-half if error happens , todo */

  if (priv->config->master)
    {
      /* Master TX send Header or TX/RX Response error */

      if (pflag->txHeaderErrorOccurred || pflag->txResponseErrorOccurred ||
          pflag->rxResponseErrorOccurred)
        {
          IfxAsclin_Lin_clearFlagsStatus(&priv->module);
          pflag->txHeaderErrorOccurred   = 0;
          pflag->txResponseErrorOccurred = 0;
          pflag->rxResponseErrorOccurred = 0;

          /* There should be space for a new TX in any event.
           * Poll the network for new XMIT data
           */

          net_lock();
          devif_poll(&priv->dev, aurix_lin_txpoll);
          net_unlock();
        }
    }
  else
    {
      /* Slave Rx header or Rx/Tx response error */

      if (pflag->rxHeaderErrorOccurred || pflag->rxResponseErrorOccurred ||
          pflag->txResponseErrorOccurred)
        {
          IfxAsclin_Lin_clearFlagsStatus(&priv->module);
          pflag->rxHeaderErrorOccurred   = 0;
          pflag->rxResponseErrorOccurred = 0;
          pflag->txResponseErrorOccurred = 0;

          IfxAsclin_Lin_prepareHeaderReception(&priv->module);
        }
    }

  /* no Noise error */

  /* After reporting an error, setidle and startidlesleeptimer are required */
}

/****************************************************************************
 * Name:  aurix_lin_transceiver_en
 *
 * Description:
 *   transceiver enable/disable
 *
 ****************************************************************************/

static void aurix_lin_transceiver_en(const aurix_lin_pin_config_t *cfg,
                                     bool enable)
{
  IfxPort_setPinModeOutput(cfg->port, cfg->pinIndex,
                           IfxPort_OutputMode_pushPull,
                           IfxPort_OutputIdx_general);

  if (enable)
    {
      IfxPort_setPinHigh(cfg->port, cfg->pinIndex);
    }
  else
    {
      IfxPort_setPinLow(cfg->port, cfg->pinIndex);
    }
}

/****************************************************************************
 * Name: aurix_lin_slave_frame_process
 *
 * Description:
 *   This function processes a CAN frame received by the AURIX LIN slave.
 *   It checks for control flag errors and updates the cache with the
 *   received frame data if necessary.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *   frame - Pointer to the CAN frame structure to be processed.
 *
 * Returned Value:
 *   Returns OK on success, or -EINVAL if there is a control flag error.
 *
 * Assumptions:
 *   - The 'priv' and 'frame' pointers are not NULL.
 *   - The 'frame' structure contains valid data.
 *
 ****************************************************************************/

static int aurix_lin_slave_frame_process(struct aurix_lin_priv_s *priv,
                                         struct can_frame *frame)
{
  struct can_frame *cache = priv->frame_cache
                            + (frame->can_id & LIN_ID_MASK);

  if ((frame->can_id & LIN_RTR_FLAG) && (frame->can_id & LIN_CACHE_RESPONSE))
    {
      /* It is incorrect to set both the LIN_RTR_FLAG
       * and LIN_CACHE_RESP_FLAG flag, return error
       */

      nwarn("%d:ERROR: control flag error\n", __LINE__);
      return -EINVAL;
    }

  cache->can_dlc = frame->can_dlc;
  cache->can_id  = frame->can_id;

  if (frame->can_id & LIN_CACHE_RESPONSE)
    {
      /* Copy the response data to cache buffer */

      memcpy(cache->data, frame->data , CAN_MAX_DLEN);
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_lin_datalength
 *
 * Description:
 *   Get the length of LIN frame based on can_dlc
 *
 * Input Parameters:
 *   pid  - LIN pid
 *   can_dlc  - can_dlc member of struct can_frame
 *
 * Returned Value:
 *   The length of LIN frame.
 *
 * Assumptions:
 *
 ****************************************************************************/

static uint8_t aurix_lin_datalength(uint8_t pid, uint8_t can_dlc)
{
  if (can_dlc == 0u)
    {
      switch ((pid >> 4u) & 0x03u)
        {
          case 3u:
            can_dlc = 8u;
            break;
          case 2u:
            can_dlc = 4u;
            break;
          default:
            can_dlc = 2u;
            break;
        }
    }

  return can_dlc;
}

/****************************************************************************
 * Name: aurix_lin_txavail_work
 ****************************************************************************/

static void aurix_lin_txavail_work(void *arg)
{
  struct aurix_lin_priv_s *priv = arg;

  net_lock();

  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (aurix_lin_txready(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, aurix_lin_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: aurix_lin_send
 ****************************************************************************/

static int aurix_lin_send(struct aurix_lin_priv_s *priv)
{
  struct can_frame *frame = (struct can_frame *)priv->dev.d_buf;
  uint8_t data_len;
  uint8_t pid = frame->can_id & LIN_ID_MASK;

  /* Drop CAN FD frames */

  if (priv->dev.d_len != sizeof(struct can_frame))
    {
      nerr("ERROR: CAN FD frames not supported\n");
      return -ENOTSUP;
    }

  data_len = aurix_lin_datalength(frame->can_id & LIN_ID_MASK,
                                  frame->can_dlc);
  if (data_len > 8u)
    {
      nwarn("%d:ERROR: invalid dlc  %u\n", __LINE__, data_len);
      return -EINVAL;
    }

  if (priv->config->master)
    {
      IfxAsclin_Lin_PduType pdu;
      memset(&pdu, 0, sizeof(pdu));

      irqstate_t flags = spin_lock_irqsave(&priv->lock);
      priv->state      = CAN_STATE_BUSY;
      spin_unlock_irqrestore(&priv->lock, flags);

      pdu.dataPtr      = frame->data;
      pdu.dataLength   = frame->can_dlc;
      pdu.pid          = pid;

      if (frame->can_id & LIN_CHECKSUM_EXTENDED)
        {
          pdu.checksumMode = IfxAsclin_Checksum_enhanced;
        }

      if (frame->can_id & CAN_RTR_FLAG)
        pdu.direction  =
                  IfxAsclin_Lin_Direction_TransmitHeaderAndReceiveResponse;
      else
        pdu.direction  = IfxAsclin_Lin_Direction_TransmitHeaderAndResponse;

      ninfo("master send PID = %d\n", pdu.pid);
      IfxAsclin_Lin_sendFrame(&priv->module, &pdu);
    }
  else
    {
      /* Lin Slave */

      return aurix_lin_slave_frame_process(priv, frame);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_lin_txready
 ****************************************************************************/

static bool aurix_lin_txready(struct aurix_lin_priv_s *priv)
{
  irqstate_t flags;
  bool txready;

  flags = spin_lock_irqsave(&priv->lock);
  txready = (priv->state == CAN_STATE_OPERATIONAL);
  spin_unlock_irqrestore(&priv->lock, flags);

  return txready;
}

/****************************************************************************
 * Name: aurix_lin_bus_wakeup_handler
 *
 * Description:
 *   This function handles the reception of an active signal on the LIN bus.
 *   It checks the current state of the LIN module and performs necessary
 *   actions such as transitioning from sleep to operational state or
 *   reporting a state change event.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL.
 *   - The LIN module is properly initialized.
 *
 ****************************************************************************/

static int aurix_lin_bus_wakeup_handler(struct aurix_lin_priv_s *priv)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->state == CAN_STATE_SLEEP)
    {
      if (priv->event == AURIX_LIN_EVENT_WAKEUP)
        {
          aurix_lin_start_sleep_timer(priv);
          spin_unlock_irqrestore(&priv->lock, flags);
        }
      else
        {
          /* State change event, build a state change event frame */

          struct can_frame *frame = priv->txdesc;
          aurix_lin_buildframe(priv, frame, AURIX_LIN_EV_WKUP_BUS);
          spin_unlock_irqrestore(&priv->lock, flags);
          aurix_lin_frame_report(priv, frame);
        }
    }
  else
    {
      spin_unlock_irqrestore(&priv->lock, flags);
    }

  IfxAsclin_clearFallingEdgeDetectedFlag(priv->module.asclin);

  return OK;
}

/****************************************************************************
 * Name:  aurix_lin_interrupt
 *
 * Description:
 *   TX/RX/EX interrupt handler
 *
 ****************************************************************************/

static int aurix_lin_interrupt(int irq, void *context, void *arg)
{
  struct aurix_lin_priv_s *priv = arg;

  if (priv->config->rx_irq == irq)
    {
      work_queue(HPWORK, &priv->rxwork, aurix_lin_rxhandler, priv, 0);
    }

  if (priv->config->tx_irq == irq)
    {
      work_queue(HPWORK, &priv->txwork, aurix_lin_txhandler, priv, 0);
    }

  if (priv->config->ex_irq == irq)
    {
      work_queue(HPWORK, &priv->exwork, aurix_lin_exhandler, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_lin_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int aurix_lin_txpoll(struct net_driver_s *dev)
{
  struct aurix_lin_priv_s *priv = (struct aurix_lin_priv_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      aurix_lin_send(priv);

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (aurix_lin_txready(priv) == false)
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: aurix_lin_txavail
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 ****************************************************************************/

static int aurix_lin_txavail(struct net_driver_s *dev)
{
  struct aurix_lin_priv_s *priv = (struct aurix_lin_priv_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(HPWORK, &priv->pollwork, aurix_lin_txavail_work, priv, 0);
    }

  return OK;
}

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: aurix_lin_wakeup_bus
 *
 * Description:
 *   This function sends a wake-up signal over
 *   the LIN bus using the ASCLIN module.
 *   It configures the ASCLIN module to transmit the specified data
 *   and sets the transmit wake-up request flag.
 *
 * Input Parameters:
 *   asclin - Pointer to the ASCLIN LIN module handler.
 *   data - Pointer to the data buffer to be transmitted.
 *   length - Number of bytes to be transmitted.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   - The 'asclin' pointer is not NULL.
 *   - The 'data' pointer is not NULL.
 *   - The 'length' is greater than zero.
 *
 ****************************************************************************/

static void aurix_lin_wakeup_bus(struct aurix_lin_priv_s *priv)
{
  /* Generate wakeup signal */

  uint8_t data[2] =
  {
    0x00
  };

  boolean     result    = 0;
  Ifx_ASCLIN *asclinSFR = priv->module.asclin;                            /* getting the pointer to ASCLIN registers from module handler */

  priv->event = AURIX_LIN_EVENT_WAKEUP;

  /* Send wakeup signal to the bus , 16bit * 52.08us  = 833us */

  IfxAsclin_setDataLength(asclinSFR, (IfxAsclin_DataLength)(2 - 1));      /* set number of bytes to be transfered */
  IfxAsclin_clearAllFlags(asclinSFR);                                     /* clear all flags */
  IfxAsclin_flushTxFifo(asclinSFR);                                       /* flushing Tx FIFO */
  IfxAsclin_enableRxFifoInlet(asclinSFR, FALSE);                          /* disable Rx FIFO */
  IfxAsclin_enableTxFifoOutlet(asclinSFR, TRUE);                          /* enable Tx FIFO for transmitting */
  IfxAsclin_write8(asclinSFR, data, 2);                                   /* write the data bytes; */
  IfxAsclin_setTransmitWakeRequestFlag(asclinSFR);                        /* set TWKQS flag */
  IfxAsclin_enableFallingEdgeDetectedFlag(asclinSFR, false);              /* disable Falling Edge interrupts */

  if (result == 1)
    {
      IFX_ASSERT(IFX_VERBOSE_LEVEL_ERROR, FALSE);
    }
}

/****************************************************************************
 * Name: aurix_lin_cmd_wakeup
 *
 * Description:
 *   This function handles the wake-up process for
 *   the AURIX LIN module via an IOCTL call.
 *   It checks the current state of the LIN module
 *   and performs the necessary actions to wake it up,
 *   either internally or by sending a wake-up signal.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *   internal - Boolean flag indicating whether
 *   the wake-up is internal (true) or external (false).
 *
 * Returned Value:
 *   Returns OK on success, or -EBUSY
 *   if the LIN module is not in the sleep state.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL.
 *   - The LIN module is properly initialized.
 *
 ****************************************************************************/

static int aurix_lin_cmd_wakeup(struct aurix_lin_priv_s *priv, bool internal)
{
  irqstate_t flags;
  int ret = OK;
  bool bus_wakeup = false;

  flags = spin_lock_irqsave(&priv->lock);
  switch (priv->state)
    {
      case CAN_STATE_OPERATIONAL:
        break;
      case CAN_STATE_SLEEP:

        /* ioctl wakeup process */

        bus_wakeup = true;
        break;
      default:

        /* Return -EBUSY if LIN is not in sleep */

        ret = -EBUSY;
        break;
    }

  if (bus_wakeup && !internal)
    {
      aurix_lin_wakeup_bus(priv);
    }

  if (bus_wakeup)
    {
      aurix_lin_start_sleep_timer(priv);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: aurix_lin_sendsleepframe
 *
 * Description:
 *   This function sends a sleep signal over the LIN bus
 *   using the ASCLIN module.It configures the ASCLIN module
 *   to transmit a specific PDU (Protocol Data Unit)
 *   with a predefined data pattern to indicate a sleep command.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL.
 *   - The LIN module is properly initialized.
 *
 ****************************************************************************/

static void aurix_lin_sendsleepframe(struct aurix_lin_priv_s *priv)
{
  IfxAsclin_Lin_PduType pdu;
  uint8_t data[8] =
    {
      0X00, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF
    };

  /* Send pid 0x3c, data 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF */

  pdu.dataPtr      = data;
  pdu.dataLength   = 8;
  pdu.pid          = 0x3c;
  pdu.checksumMode = IfxAsclin_Checksum_classic;
  pdu.direction    = IfxAsclin_Lin_Direction_TransmitHeaderAndResponse;

  IfxAsclin_Lin_sendFrame(&priv->module, &pdu);
}

/****************************************************************************
 * Function: aurix_lin_cmd_sleep
 *
 * Description:
 *   Send sleep signal to LIN bus and go to sleep mode
 *
 * Input Parameters:
 *   priv - Reference to the private data of AURIX lin driver
 *   sleepinternal - Sleep internal and do not send sleep command to the bus
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_lin_cmd_sleep(struct aurix_lin_priv_s *priv, bool internal)
{
  irqstate_t flags;
  int ret = OK;
  bool bus_sleep = false;

  flags = spin_lock_irqsave(&priv->lock);

  switch (priv->state)
    {
      case CAN_STATE_SLEEP:

        /* Return OK if LIN is already in sleep mode */

        break;
      case CAN_STATE_OPERATIONAL:

        /* ioctl sleep process */

        if (internal)
          {
            priv->state = CAN_STATE_SLEEP;
            IfxAsclin_disableAllFlags(priv->module.asclin);
            aurix_lin_set_error_interrupts(priv->module.asclin, false);
            IfxAsclin_enableFallingEdgeDetectedFlag(priv->module.asclin,
                                                    true);
          }
        else
          {
            bus_sleep = true;
          }

        break;
      case CAN_STATE_SPENDING:
      case CAN_STATE_BUSY :

        /* Return -EBUSY if LIN is in busy or sleeppending */

        ret = -EBUSY;
        break;
    }

  if (bus_sleep && priv->config->master)
    {
      /* Master node switch to sleep pending state and send sleep
       * signal to the bus. The driver is switched to sleep mode
       * when the sleep signal is completed.
       */

      priv->state = CAN_STATE_SPENDING;
      aurix_lin_sendsleepframe(priv);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return ret;
}

/****************************************************************************
 * Name: aurix_lin_netdev_ioctl
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int aurix_lin_netdev_ioctl(struct net_driver_s *dev, int cmd,
                                  unsigned long arg)
{
  int ret = -ENOTSUP;
#ifdef CONFIG_NETDEV_CAN_STATE_IOCTL
  struct aurix_lin_priv_s *priv = (struct aurix_lin_priv_s *)dev->d_private;
  struct can_ioctl_state_s *st = (struct can_ioctl_state_s *)arg;

  switch (cmd)
    {
      case SIOCGCANSTATE:
        st->state = (enum can_ioctl_state_e)priv->state;
        ret = OK;
        break;
      case SIOCSCANSTATE:
        if (st->state == CAN_STATE_SLEEP)
          {
            ret = aurix_lin_cmd_sleep(priv, (bool)st->priv);
          }
        else if (st->state == CAN_STATE_OPERATIONAL)
          {
            ret = aurix_lin_cmd_wakeup(priv, (bool)st->priv);
          }
        break;
      default:
        break;
    }
#endif /* CONFIG_NETDEV_CAN_STATE_IOCTL */

  return ret;
}
#endif

/****************************************************************************
 * Name: aurix_lin_is_sleep_signal
 *
 * Description:
 *   This function checks if the sleep signal has been successfully
 *   sent to the LIN bus.
 *   If the sleep signal is confirmed, it sets the driver
 *   to sleep mode by disabling and re-enabling interrupts,
 *   and updating the state variables.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   Returns true if the sleep signal is confirmed
 *   and the driver is set to sleep mode, otherwise false.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL.
 *   - The LIN module is properly initialized.
 *
 ****************************************************************************/

static bool aurix_lin_is_sleep_signal(struct aurix_lin_priv_s *priv)
{
  irqstate_t curr_state;

  /* Current state is sleep pending, the sleep signal is send to
   * LIN bus successfully, set driver to sleep mode
   */

  const uint8_t sleep_seq[9u] =
    {
      0x00u, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0x00u
    };

  IfxAsclin_Lin                  *asclin = &priv->module;
  IfxAsclin_Lin_FrameDataControl *linfrm = &asclin->linFrameData;

  if ((linfrm->headerID & LIN_ID_MASK) != SLEEP_PID)
    {
      return false;
    }

  if ((priv->config->master && (priv->state == CAN_STATE_SPENDING))  ||
      (!priv->config->master && memcmp(linfrm->rxResponseData,
       sleep_seq, linfrm->rxResponseLength) == 0))
    {
      curr_state = enter_critical_section();

      IfxAsclin_disableAllFlags(priv->module.asclin);
      aurix_lin_set_error_interrupts(priv->module.asclin, false);
      IfxAsclin_enableFallingEdgeDetectedFlag(priv->module.asclin, true);

      priv->event = AURIX_LIN_EVENT_IDLE;
      priv->state = CAN_STATE_SLEEP;
      leave_critical_section(curr_state);

      return true;
    }

  return false;
}

#ifdef CONFIG_AURIX_LIN_IDLE_TO_SLEEP
/****************************************************************************
 * Name: aurix_lin_bus_enter_sleep
 *
 * Description:
 *   This function handles the transition of
 *   the AURIX LIN module to sleep mode when the bus is idle.
 *   It checks if the current state is operational and, if so,
 *   disables and re-enables interrupts,and sets the state to sleep mode.
 *   This function is typically called after a period of bus inactivity.
 *
 * Input Parameters:
 *   arg - Pointer to the argument, which is cast to a pointer of
 *   the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void aurix_lin_bus_enter_sleep(void *arg)
{
  struct aurix_lin_priv_s *priv = arg;

  if (priv->state == CAN_STATE_OPERATIONAL)
    {
      /* Bus idle 4 - 10 seconds reached, go to sleep */

      irqstate_t flags;
      flags = spin_lock_irqsave(&priv->lock);

      IfxAsclin_disableAllFlags(priv->module.asclin);
      aurix_lin_set_error_interrupts(priv->module.asclin, false);
      IfxAsclin_enableFallingEdgeDetectedFlag(priv->module.asclin, true);

      priv->state = CAN_STATE_SLEEP;
      spin_unlock_irqrestore(&priv->lock, flags);
    }
}
#endif

/****************************************************************************
 * Name: aurix_lin_start_sleep_timer
 *
 * Description:
 *   This function starts the idle sleep timer for the AURIX LIN module.
 *   It schedules a high-priority work item to call
 *   the 'aurix_lin_bus_enter_sleep' function
 *   after a specified idle sleep time.
 *   The timer count is set to 1 to indicate that the
 *   timer has been started.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   Returns OK if the timer is successfully scheduled
 *   or if the feature is not enabled.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL
 *     and points to a valid 'aurix_lin_priv_s' structure.
 *   - The 'delaywork' field in 'priv' is properly initialized
 *     and associated with a valid work item.
 *   - The 'LPWORK' macro is defined
 *     and correctly represents the high-priority work queue.
 *   - The 'CONFIG_AURIX_LIN_IDLE_TO_SLEEP' macro is defined
 *     if the idle-to-sleep feature is enabled.
 *   - The 'CONFIG_AURIX_LIN_IDLE_SLEEP_TIME' macro is defined
 *     and correctly represents the idle sleep time in seconds.
 *
 ****************************************************************************/

static void aurix_lin_start_sleep_timer(struct aurix_lin_priv_s *priv)
{
  aurix_lin_setidle(priv);
#ifdef CONFIG_AURIX_LIN_IDLE_TO_SLEEP
  work_queue(LPWORK, &priv->delaywork, aurix_lin_bus_enter_sleep,
  priv, SEC2TICK(CONFIG_AURIX_LIN_IDLE_SLEEP_TIME));
#endif
}

/****************************************************************************
 * Name: aurix_lin_set_error_interrupts
 *
 * Description:
 *   Enable or disable the LIN module error interrupts
 *   based on the enable parameter.
 *
 * Input Parameters:
 *   asclinSFR - Pointer to the ASCLIN register structure.
 *   enable - Boolean value to enable (true) or
 *   disable (false) interrupts.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void aurix_lin_set_error_interrupts(Ifx_ASCLIN *asclinSFR,
                                           bool enable)
{
  IfxAsclin_enableFrameErrorFlag(asclinSFR, enable);
  IfxAsclin_enableLinParityErrorFlag(asclinSFR, enable);
  IfxAsclin_enableHeaderTimeoutFlag(asclinSFR, enable);
  IfxAsclin_enableResponseTimeoutFlag(asclinSFR, enable);
  IfxAsclin_enableLinChecksumErrorFlag(asclinSFR, enable);
  IfxAsclin_enableCollisionDetectionErrorFlag(asclinSFR, enable);
  IfxAsclin_enableRxFifoOverflowFlag(asclinSFR, enable);
}

/****************************************************************************
 * Name: aurix_lin_setidle
 *
 * Description:
 *   This function sets the AURIX LIN module to an idle state.
 *   It updates the frame state and module state to indicate
 *   that the module is idle and operational.
 *
 * Input Parameters:
 *   priv - Pointer to the private data structure for the AURIX LIN module.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   - The 'priv' pointer is not NULL
 *     and points to a valid 'aurix_lin_priv_s' structure.
 *   - The 'AURIX_LIN_EVENT_IDLE' and 'CAN_STATE_OPERATIONAL' macros
 *     are defined and correctly represent the idle frame state
 *     and operational module state, respectively.
 *
 ****************************************************************************/

static void aurix_lin_setidle(struct aurix_lin_priv_s *priv)
{
  priv->event = AURIX_LIN_EVENT_IDLE;
  priv->state = CAN_STATE_OPERATIONAL;
  priv->reported_error = false;
  aurix_lin_set_error_interrupts(priv->module.asclin, true);
}

/****************************************************************************
 * Name: aurix_lin_ifup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int aurix_lin_ifup(struct net_driver_s *dev)
{
  struct aurix_lin_priv_s *priv = (struct aurix_lin_priv_s *)dev->d_private;
  IfxAsclin_Lin_Config moduleConfig;
  const struct aurix_lin_config_s *config;
  irqstate_t flags;
  Ifx_ASCLIN *asclinSFR;

  DEBUGASSERT(priv);
  config = priv->config;
  DEBUGASSERT(config);

  /* Create module config */

  memset(&moduleConfig, 0, sizeof(moduleConfig));
  IfxAsclin_Lin_initModuleConfig(&moduleConfig, config->asclin);
  moduleConfig.brg.baudrate    = 19200;
  moduleConfig.isInterruptMode = TRUE;
  moduleConfig.pins            = &config->pins;
  moduleConfig.linMode         = config->master ? IfxAsclin_LinMode_master
                                                : IfxAsclin_LinMode_slave;
  moduleConfig.frame.leadDelay = IfxAsclin_LeadDelay_2;

  /* Initialize header timeout threshold */

  moduleConfig.lin.headerTimeout = 48;  /* The maximum length of the LIN header threshold is 48 bits. */

  /* Enable Response timeout Mode and Bit */

  moduleConfig.data.responseTimeoutMode =
                            IfxAsclin_LinResponseTimeoutMode_responseTimeout;
  moduleConfig.data.responseTimeout = 126;  /* The maximum length of the LIN frame threshold is 126 bits. */

  /* Initialize module */

  IfxAsclin_Lin_initModule(&priv->module, &moduleConfig);

  /* Enable the interrupts at the NVIC */

  up_enable_irq(config->rx_irq);
  up_enable_irq(config->tx_irq);
  up_enable_irq(config->ex_irq);

  flags = spin_lock_irqsave(&priv->lock);
  priv->bifup     = true;
  priv->dev.d_buf = (uint8_t *)priv->txdesc;
  priv->state     = CAN_STATE_SLEEP;
  priv->event     = AURIX_LIN_EVENT_IDLE;
  spin_unlock_irqrestore(&priv->lock, flags);

  asclinSFR       = priv->module.asclin;

  if (!config->master)
    {
      IfxAsclin_Lin_prepareHeaderReception(&priv->module);

      /* Enable Rx interrupts */

      IfxAsclin_enableRxHeaderEndFlag(asclinSFR, true);
      IfxAsclin_enableRxResponseEndFlag(asclinSFR, true);

      /* Enable Tx interrupts */

      IfxAsclin_enableTxResponseEndFlag(asclinSFR, true);
    }

  IfxAsclin_enableFallingEdgeDetectedFlag(asclinSFR, true);
  aurix_lin_transceiver_en(&config->transceiver, true);

  ninfo("aurix_lin_ifup:  up ok\n");
  return OK;
}

/****************************************************************************
 * Name: aurix_lin_ifdown
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int aurix_lin_ifdown(struct net_driver_s *dev)
{
  struct aurix_lin_priv_s *priv = (struct aurix_lin_priv_s *)dev->d_private;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  if (!priv->bifup || priv->state == CAN_STATE_BUSY)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return -EBUSY;
    }

  priv->bifup = false;
  priv->state = 0;

  up_disable_irq(priv->config->ex_irq);
  up_disable_irq(priv->config->rx_irq);
  up_disable_irq(priv->config->tx_irq);
  spin_unlock_irqrestore(&priv->lock, flags);

  IfxAsclin_Lin_disableModule(&priv->module);
  aurix_lin_transceiver_en(&priv->config->transceiver, false);

  return OK;
}

void lin_pin_initilize(const struct aurix_lin_config_s *config)
{
  /* Pin mapping */

  const IfxAsclin_Lin_Pins *pins = &(config->pins);

  if (pins != NULL_PTR)
    {
      IfxAsclin_Tx_Out *tx = pins->tx;

      if (tx != NULL_PTR)
        {
          IfxPort_setPinModeOutput(tx->pin.port, tx->pin.pinIndex,
                                   pins->txMode, tx->select);
          IfxPort_setPinPadDriver(tx->pin.port, tx->pin.pinIndex,
                                  pins->pinDriver);
          IfxPort_setPinHigh(tx->pin.port, tx->pin.pinIndex);
        }
    }
}

/****************************************************************************
 * Public Functions
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
                         const struct aurix_lin_config_s *cfg,
                         size_t num)
{
  struct aurix_lin_priv_s *priv;
  int ret = -EINVAL;
  int i;

  for (i = 0; i < num; i++)
    {
      const struct aurix_lin_config_s *config = cfg + i;
      uint32_t size = config->master ? sizeof(struct aurix_lin_priv_s) :
                            (sizeof(struct aurix_lin_priv_s) +
                             CAN_MTU * LIN_SLAVE_CACHE_NUM);
      priv = kmm_zalloc(size);
      if (priv == NULL)
        {
          nerr("aurix lin kmm_zalloc failed\n");
          return -ENOMEM;
        }

      /* Initialize the spin lock */

      spin_lock_init(&priv->lock);

      /* Initialize the driver structure */

      priv->dev.d_ifup    = aurix_lin_ifup;
      priv->dev.d_ifdown  = aurix_lin_ifdown;
      priv->dev.d_txavail = aurix_lin_txavail;
#ifdef CONFIG_NETDEV_IOCTL
      priv->dev.d_ioctl   = aurix_lin_netdev_ioctl;
#endif
      priv->dev.d_private = priv;
      priv->config        = config;

      snprintf(priv->dev.d_ifname, IFNAMSIZ, "lin%d", config->port);

      lin_pin_initilize(config);

      /* Put the interface in the down state.This usually amounts
       * to resetting the device and/or calling fdcan_ifdown().
       */

      aurix_lin_ifdown(&priv->dev);

      /* Register the device with the OS
       * so that socket IOCTLs can be performed
       */

      ret = netdev_register(&priv->dev, NET_LL_CAN);
      if (ret < 0)
        {
          nerr("register lin interface %d failed: %d\n", config->port, ret);
          kmm_free(priv);
          continue;
        }

      ninfo("register lin interface %d ok\n", i);

#ifdef CONFIG_AURIX_LIN_ISR_WQUEUE
      irq_attach_wqueue(config->tx_irq, NULL, aurix_lin_interrupt, priv,
                        CONFIG_AURIX_LIN_ISR_WQUEUE_PRIORITY);
      irq_attach_wqueue(config->rx_irq, NULL, aurix_lin_interrupt, priv,
                        CONFIG_AURIX_LIN_ISR_WQUEUE_PRIORITY);
      irq_attach_wqueue(config->ex_irq, NULL, aurix_lin_interrupt, priv,
                        CONFIG_AURIX_LIN_ISR_WQUEUE_PRIORITY);
#else
      irq_attach(config->tx_irq, aurix_lin_interrupt, priv);
      irq_attach(config->rx_irq, aurix_lin_interrupt, priv);
      irq_attach(config->ex_irq, aurix_lin_interrupt, priv);
#endif

      /* using port as index to get net_driver_s */

      dev[config->port] = &priv->dev;
    }

  return ret;
}
