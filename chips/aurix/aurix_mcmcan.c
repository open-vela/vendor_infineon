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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include "IfxGeth_reg.h"
#include "aurix_mcmcan.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct node_group_s
{
  Ifx_CAN *can;  /* module address */
  uint8    refs; /* module referance numbers */
};

struct aurix_mcmcan_priv_s
{
  struct can_dev_s              dev;        /* CAN device struct object */
  struct aurix_mcmcan_config_s *config;     /* mcmcan config object */
  struct node_group_s          *node_group; /* CAN node group */
                                            /* any bit = 0, txmb available,
                                             * any bit = 1, txmb transmition
                                             * is doing
                                             */
  uint32                        txmb_sflags;
  uint8                         state;      /* CAN node controller state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#ifdef CONFIG_CAN_TIMESTAMP
static void mcmcan_set_timestamp(struct can_hdr_s *msg_hdr);
#endif
static int mcmcan_flush_txbuff(struct aurix_mcmcan_priv_s *priv);
static void mcmcan_disable_mcan_module(Ifx_CAN *can);
static int mcmcan_fill_txmsg(struct aurix_mcmcan_priv_s *priv,
                             struct can_msg_s *msg, IfxCan_Message *message);
static void mcmcan_config_setup(struct aurix_mcmcan_config_s *config);

#ifdef CONFIG_CAN_TXCONFIRM
static void aurix_mcmcan_tx_confirm(struct can_dev_s *dev,
                                    uint8 txbuffer_id);
#endif
static int aurix_mcmcan_tx_interrupt(int irq, void *context, void *arg);
static int aurix_mcmcan_rx_interrupt(int irq, void *context, void *arg);
#if defined(CONFIG_CAN_ERRORS)
static int aurix_mcmcan_err_interrupt(int irq, void *context, void *arg);
static int aurix_mcmcan_busoff_recovery(struct aurix_mcmcan_priv_s *priv);
#endif

/* CAN driver methods */

static inline int aurix_mcmcan_getmode(struct aurix_mcmcan_priv_s *priv,
                                       uint8 *state);
static int aurix_mcmcan_setmode(struct aurix_mcmcan_priv_s *priv,
                                unsigned long arg);
static void aurix_mcmcan_co_reset(struct can_dev_s *dev);
static void aurix_mcmcan_set_loopback_mode(struct aurix_mcmcan_priv_s *priv);
static int aurix_mcmcan_co_setup(struct can_dev_s *dev);
static void aurix_mcmcan_co_shutdown(struct can_dev_s *dev);
static void aurix_mcmcan_co_rxint(struct can_dev_s *dev, bool enable);
static void aurix_mcmcan_co_txint(struct can_dev_s *dev, bool enable);
static int aurix_mcmcan_co_ioctl(struct can_dev_s *dev, int cmd,
                                 unsigned long arg);
static int aurix_mcmcan_co_send(struct can_dev_s *dev,
                                struct can_msg_s *msg);
static bool aurix_mcmcan_co_txready(struct can_dev_s *dev);
static bool aurix_mcmcan_co_txempty(struct can_dev_s *dev);
static void aurix_mcmcan_filter(struct can_dev_s *dev);
#if defined(CONFIG_AURIX_MCMCAN_CRE)
static void aurix_mcmcan_uni_routing(struct can_dev_s *dev);
static void aurix_mcmcan_mul_routing(struct can_dev_s *dev);
#endif
#ifdef CONFIG_CAN_TXCANCEL
static bool aurix_mcmcan_co_cancel(struct can_dev_s *dev,
                                   struct can_msg_s *msg);
#endif
#ifdef CONFIG_CAN_ERROR_POLLING
static void enable_destruct_readmode(IfxCan_Can_Node *can_node);
static void aurix_mcmcan_co_errhandle(struct can_dev_s *dev);
#endif

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static struct node_group_s g_node_group[] =
{
  {
    .can = &MODULE_CAN0,
  },
  {
    .can = &MODULE_CAN1,
  },
  {
    .can = &MODULE_CAN2,
  },
  {
    .can = &MODULE_CAN3,
  },
  {
    .can = &MODULE_CAN4,
  },
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

static const struct can_ops_s g_aurix_mcmcanops =
{
  .co_reset     = aurix_mcmcan_co_reset,
  .co_setup     = aurix_mcmcan_co_setup,
  .co_shutdown  = aurix_mcmcan_co_shutdown,
  .co_rxint     = aurix_mcmcan_co_rxint,
  .co_txint     = aurix_mcmcan_co_txint,
  .co_ioctl     = aurix_mcmcan_co_ioctl,
  .co_send      = aurix_mcmcan_co_send,
  .co_txready   = aurix_mcmcan_co_txready,
  .co_txempty   = aurix_mcmcan_co_txempty,
#ifdef CONFIG_CAN_TXCANCEL
  .co_cancel    = aurix_mcmcan_co_cancel,
#endif
#ifdef CONFIG_CAN_ERROR_POLLING
  .co_errhandle = aurix_mcmcan_co_errhandle,
#endif
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

boolean aurix_mcmcan_compare_transv_pinmode(Ifx_P *port, uint8 pinIndex)
{
  /* "0x1" indicate the DRVCFG register is set */

  return port->PADCFG[pinIndex].DRVCFG.U != 0x1;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcmcan_set_timestamp
 *
 * Description:
 *  Set the gptp timestamp to The CAN message structure object.
 *
 * Inpit Parameter:
 *  msg - The CAN message structure object.
 *
 * Return Value:
 *  None
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_TIMESTAMP
static void mcmcan_set_timestamp(struct can_hdr_s *msg_hdr)
{
  Ifx_GETH_PORT_CORE *core    = &MODULE_GETH0.PORT[1].CORE;
  uint16_t hs; /* high 16bit of 48bit second */
  uint32_t ls; /* lower 32bit of 48bit second */

#ifdef CONFIG_SYSTEM_TIME64
  hs = core->MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS.B.TSHWR;
  ls = core->MAC_SYSTEM_TIME_SECONDS.U;
  msg_hdr->ch_ts.tv_sec = (int64_t)hs << 32 | ls;
#else
  msg_hdr->ch_ts.tv_sec = core->MAC_SYSTEM_TIME_SECONDS.U;
#endif
  msg_hdr->ch_ts.tv_usec = core->MAC_SYSTEM_TIME_NANOSECONDS.U / 1000u;
}
#endif

/****************************************************************************
 * Name: process_rxbuf_msg
 *
 * Description:
 *   Process a received CAN message by reading it from the hardware
 *   and passing it to the upper layer driver.
 *
 * Input Parameters:
 *   priv      - An instance of the "lower half" can driver structure.
 *   rxmessage - The received CAN message structure variable.
 *   hdr       - Header structure to the received CAN message.
 *   data      - Buffer to store the received CAN message data.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void process_rxbuf_msg(struct aurix_mcmcan_priv_s *priv,
                                     IfxCan_Message *rxmessage,
                                     struct can_hdr_s *hdr, uint8 *data)
{
  IfxCan_Can_readMessage(&priv->config->can_node, rxmessage, (uint32 *)data);

  hdr->ch_id  = rxmessage->messageId;
  hdr->ch_dlc = rxmessage->dataLengthCode;
  hdr->ch_edl = (rxmessage->frameMode ==
                IfxCan_FrameMode_standard) ? 0 : 1;
  hdr->ch_brs = (rxmessage->frameMode ==
                IfxCan_FrameMode_fdLongAndFast) ? 1 : 0;
  hdr->ch_rtr = 0;
#ifdef CONFIG_CAN_TIMESTAMP

  /* set gptp timestamp */

  mcmcan_set_timestamp(hdr);
#endif

  /* Provide the data to the upper half driver */

  can_receive(&priv->dev, hdr, data);
}

/****************************************************************************
 * Name: mcmcan_interrupt_setup
 *
 * Description:
 *   enable/disable specfic interrupt.
 *
 * Input Parameters:
 *   enable - enable or disable interrupt.
 *   node - CAN node.
 *   inte_number - the specfic interrupt enum number.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static inline void mcmcan_interrupt_setup(bool enable, Ifx_CAN_N *node,
                                          uint8 inte_number)
{
  irqstate_t irqflags = enter_critical_section();

  if (enable)
    {
      IfxCan_Node_enableInterrupt(node, inte_number);
    }
  else
    {
      IfxCan_Node_disableInterrupt(node, inte_number);
    }

  leave_critical_section(irqflags);
}

/****************************************************************************
 * Name: aurix_mcmcan_busoff_recovery
 *
 * Description:
 *   try to recover from busoff
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *
 * Returned Value:
 *   Zero on success
 *
 ****************************************************************************/

static int aurix_mcmcan_busoff_recovery(struct aurix_mcmcan_priv_s *priv)
{
  IfxCan_Can_Node       *can_node    = &priv->config->can_node;

  /* now we need to clear all pending tx buffers */

  mcmcan_flush_txbuff(priv);

  /* clear transmissionCompleted interrupt flag in case that
   * can_txdone called results undefine behavior after busoff recovery.
   *
   * the reason of IfxCan_Node_clearInterruptFlag called
   * behind cancellation operation why is that cancellation will wait
   * the transmission is over. note: a transmission has already
   * been started when a cancellation is requested.
   */

  IfxCan_Node_clearInterruptFlag(can_node->node,
                                 IfxCan_Interrupt_transmissionCompleted);

  /* try to entry normal mode.
   * do not polling some register value because this time is unknown.
   */

  IfxCan_Node_setInitialisation(can_node->node, false);
  canwarn("bus off recovery finished");
  return OK;
}

/****************************************************************************
 * Name: aurix_mcmcan_getmode
 *
 * Description:
 *   Set aurix mcmcan state machine mode
 *
 * Input Parameters:
 *   dev   - An instance of the "upper half" can driver structure.
 *   state - Mode value pointer.
 *
 * Returned Value:
 *   Zero on success
 *
 ****************************************************************************/

static inline int aurix_mcmcan_getmode(struct aurix_mcmcan_priv_s *priv,
                                       uint8 *state)
{
  *state = priv->state;
  return 0;
}

/****************************************************************************
 * Name: aurix_mcmcan_setmode
 *
 * Description:
 *   Set mcmcan state machine mode.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *   arg  - Mode.
 *
 * Returned Value:
 *   Zero on success; a negative errno on failure
 *
 ****************************************************************************/

static int aurix_mcmcan_setmode(struct aurix_mcmcan_priv_s *priv,
                                unsigned long arg)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;

  /* the detail of operation steps refer to the MCMCAN operating Mode
   * section of TC4DX and TC3XX user manual.
   *
   * confirm below matter:
   * enable CAN module that is correspond with CAN node
   * before operating that CAN node register.
   */

  switch (arg)
    {
      case CAN_STATE_START:

        /* make controller come in normal operation */

        if (priv->node_group->refs == 1)
          {
             if (IfxCan_isModuleEnabled(priv->node_group->can) != TRUE)
              {
                /* Enable module, disregard Sleep Mode request */

                IfxCan_enableModule(priv->node_group->can);
              }
          }

        if (priv->state == CAN_STATE_STOP)
          {
            IfxCan_Node_setInitialisation(can_node->node, false);
            while (can_node->node->CCCR.B.INIT != 0);
          }
        else if (priv->state == CAN_STATE_DOZE)
          {
            can_node->node->CCCR.B.CSR          = 0;
            while (can_node->node->CCCR.B.CSA  != 0);

            IfxCan_Node_setInitialisation(can_node->node, false);
            while (can_node->node->CCCR.B.INIT != 0);
          }

        priv->state = CAN_STATE_START;
        return OK;
      case CAN_STATE_STOP:

        /* make controller come in stop operation */

        if (priv->state == CAN_STATE_START)
          {
            IfxCan_Node_setInitialisation(can_node->node, true);
            while (can_node->node->CCCR.B.INIT != 1);
          }
        else if (priv->state == CAN_STATE_DOZE)
          {
            can_node->node->CCCR.B.CSR          = 0;
            while (can_node->node->CCCR.B.CSA  != 0);

            IfxCan_Node_setInitialisation(can_node->node, true);
            while (can_node->node->CCCR.B.INIT != 1);
          }

        priv->state = CAN_STATE_STOP;
        return OK;
      case CAN_STATE_DOZE:

        /* make controller come in sleep operation */

        can_node->node->CCCR.B.CSR          = 1;
        while (can_node->node->CCCR.B.INIT != 1 &&
               can_node->node->CCCR.B.CSA  != 1);

        if (priv->node_group->refs == 1)
          {
            mcmcan_disable_mcan_module(priv->node_group->can);
          }

        priv->state = CAN_STATE_DOZE;
        return OK;
      default:
        return -ENOTTY;
    }
}

#ifdef CONFIG_CAN_ERROR_POLLING

/****************************************************************************
 * Name: enable_destruct_readmode
 *
 * Description:
 *   Enable destructive read mode for CAN node.
 *
 * Input Parameters:
 *   can_node - CAN node.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void enable_destruct_readmode(IfxCan_Can_Node *can_node)
{
  IfxCan_Node_enableConfigurationChange(can_node->node);

  /* Enable destructive read mode for CAN node */

  can_node->node->PORTCTRL.B.DELE = 1;

  IfxCan_Node_disableConfigurationChange(can_node->node);
}

/****************************************************************************
 * Name: aurix_mcmcan_co_errhandle
 *
 * Description:
 *   error information handle by upper half driver.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void aurix_mcmcan_co_errhandle(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;
  struct can_hdr_s hdr =
  {
    0
  };

  IfxCan_CanNodeErrorWarningLimitStatus warn_state;
  IfxCan_LastErrorCodeType              last_errcode;
  uint16_t                              errbits;
  uint8_t                               data[CAN_ERROR_DLC];

  last_errcode = IfxCan_Node_getLastErroCodeStatus(can_node->node);

  if (last_errcode == IfxCan_LastErrorCodeType_noError ||
      last_errcode == IfxCan_LastErrorCodeType_noCANBusEvent)
    {
      return;
    }

  errbits = 0;
  memset(data, 0, sizeof(data));

  if (last_errcode == IfxCan_LastErrorCodeType_stuffError)
    {
      /* Stuff Error */

      data[2] |= CAN_ERROR2_STUFF;
      errbits |= CAN_ERROR_PROTOCOL;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_formError)
    {
      /* Format Error */

      data[2] |= CAN_ERROR2_FORM;
      errbits |= CAN_ERROR_PROTOCOL;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_ackError)
    {
      /* Acknowledge Error */

      errbits |= CAN_ERROR_NOACK;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_bit1Error)
    {
      /* Bit recessive Error */

      data[2] |= CAN_ERROR2_BIT;
      data[2] |= CAN_ERROR2_BIT1;
      errbits |= CAN_ERROR_PROTOCOL;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_bit0Error)
    {
      /* Bit domainant Error */

      data[2] |= CAN_ERROR2_BIT;
      data[2] |= CAN_ERROR2_BIT0;
      errbits |= CAN_ERROR_PROTOCOL;
    }

  if (last_errcode ==  IfxCan_LastErrorCodeType_crcError)
    {
      /* Receive CRC Error */

      data[3] |= CAN_ERROR3_CRCSEQ;
      errbits |= CAN_ERROR_PROTOCOL;
    }

  warn_state = IfxCan_Node_getWarningStatus(can_node->node);

  if (warn_state == IfxCan_CanNodeErrorWarningLimitStatus_notReached)
    {
      /* Warnning is not reached */
    }
  else
    {
      /* warnning is reached */

      if (can_node->node->ECR.B.TEC >= 96)
        {
          /* TX error warning flag */

          data[1] |= CAN_ERROR1_TXWARNING;
          errbits |= CAN_ERROR_CONTROLLER;
        }

      if (can_node->node->ECR.B.REC >= 96)
        {
          /* RX error warning flag */

          data[1] |= CAN_ERROR1_RXWARNING;
          errbits |= CAN_ERROR_CONTROLLER;
        }
    }

  if (IfxCan_Node_isErrorPassive(can_node->node))
    {
      /* Error Passive */

      if (can_node->node->ECR.B.RP == 1)
        {
          /* RX passive flag */

          data[1] |= CAN_ERROR1_RXPASSIVE;
        }

      if (can_node->node->ECR.B.TEC >= 127)
        {
          /* TX passive flag */

          data[1] |= CAN_ERROR1_TXPASSIVE;
        }

      errbits |= CAN_ERROR_CONTROLLER;
    }

  /* Report a CAN error */

  if (errbits != 0)
    {
      canerr("ERROR: errbits = 0x%04x\n", errbits);

      /* Format the CAN header for the error report */

      hdr.ch_id     = errbits;
      hdr.ch_dlc    = CAN_ERROR_DLC;
      hdr.ch_rtr    = 0;
      hdr.ch_error  = 1;
      hdr.ch_tcf    = 0;

      can_receive(dev, &hdr, data);
    }
}
#endif

/****************************************************************************
 * Name: aurix_mcmcan_co_reset
 *
 * Description:
 *   Reset the CAN device. Called early to initialize the hardware.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void aurix_mcmcan_co_reset(struct can_dev_s *dev)
{
  return;
}

/****************************************************************************
 * Name: aurix_mcmcan_set_loopback_mode
 *
 * Description:
 *   Configure external loop back mode.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void aurix_mcmcan_set_loopback_mode(struct aurix_mcmcan_priv_s *priv)
{
  if (priv->config->loopback_flag == TRUE)
    {
      /* Configure external loop back mode */

      IfxCan_Can_Node *can_node = &priv->config->can_node;
      IfxCan_Node_enableConfigurationChange(can_node->node);

      /* Enable write access to register TEST */

      can_node->node->CCCR.B.TEST = 1;
      can_node->node->TEST.B.LBCK = 1;
      IfxCan_Node_disableConfigurationChange(can_node->node);
    }
}

/****************************************************************************
 * Name: aurix_mcmcan_co_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int aurix_mcmcan_co_setup(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;
  bool                        ret;

  /* enable the CAN module and add reference count */

  if (priv->node_group->refs == 0)
    {
      if (IfxCan_isModuleEnabled(priv->node_group->can) != TRUE)
        {
          /* Enable module, disregard Sleep Mode request */

          IfxCan_enableModule(priv->node_group->can);
        }

      /* clear used mcanX RAM */

      IfxVmt_clearSram(priv->config->module_sram_index);
    }

  /* setup common config */

  mcmcan_config_setup(priv->config);

  /* Initialises the CAN Node and sets CAN controller
   * mode to normal mode(also called START mode)
   */

  ret = IfxCan_Can_initNode(&priv->config->can_node,
                            &priv->config->can_node_config);
  if (priv->config->loopback_flag != true && ret != true)
    {
      return -EFAULT;
    }

  aurix_mcmcan_set_loopback_mode(priv);

#ifdef CONFIG_CAN_ERROR_POLLING
  enable_destruct_readmode(&priv->config->can_node);
#endif

  priv->node_group->refs++;

  /* setup node filter cre config */

  aurix_mcmcan_filter(dev);

#if defined(CONFIG_AURIX_MCMCAN_CRE)
  IfxCan_Can_initCre(&priv->config->can_node,
                     &priv->config->can_node_cre_config);
  aurix_mcmcan_uni_routing(dev);
  aurix_mcmcan_mul_routing(dev);
#endif

  /* attach service function to specfic irq
   * this about interrupt group and irq
   * need to map interrupt source to interrupt line.
   * tx_irq <--> TRACO <--> CAN0 Ni_G1INTR 28 ~ 31
   * rx_irq <--> REINT <--> CAN0 Ni_G1INTR 0 ~ 3
   * ...
   * see user manual 22.4.3.2.1 Mapping of interrupts
   */

  /* map rx/tx interrupt group to interrupt line */

  IfxCan_Node_setInterruptLine(can_node->node,
                               IfxCan_Interrupt_transmissionCompleted,
                               priv->config->tx_interrupt_line);
  IfxCan_Node_setInterruptLine(can_node->node,
                    IfxCan_Interrupt_messageStoredToDedicatedRxBuffer,
                               priv->config->rx_interrupt_line);
#if defined(CONFIG_CAN_ERRORS)
  IfxCan_Node_setInterruptLine(can_node->node,
                               IfxCan_Interrupt_busOffStatus,
                               priv->config->err_interrupt_line);
#endif

#if defined(CONFIG_AURIX_MCMCAN_CRE)

  /* mapping "creRxBuffer1 interrupt" to rx_interrupt_line,
   * because we need to acquire rxmsg form cre Rx Host Buffer1.
   */

  IfxCan_Node_setInterruptLine(can_node->node,
                               IfxCan_Interrupt_creRxBuffer1,
                               priv->config->rx_interrupt_line);
#endif

  /* enable tx/rx interrupt line */

  up_enable_irq(priv->config->tx_irq);
  up_enable_irq(priv->config->rx_irq);
#if defined(CONFIG_CAN_ERRORS)
  up_enable_irq(priv->config->err_irq);
#endif

  /* enable can node interrupt register corresponding bit */

  IfxCan_Node_enableInterrupt(can_node->node,
                              IfxCan_Interrupt_transmissionCompleted);
  IfxCan_Node_enableInterrupt(can_node->node,
                   IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
#if defined(CONFIG_CAN_ERRORS)
  IfxCan_Node_enableInterrupt(can_node->node,
                              IfxCan_Interrupt_busOffStatus);
#endif
  priv->state       = CAN_STATE_START;
  priv->txmb_sflags = 0;
  return OK;
}

/****************************************************************************
 * Name: aurix_mcmcan_co_shutdown
 *
 * Description:
 *   Shutdown the CAN device. This method is called when the CAN device
 *   is closed. This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void aurix_mcmcan_co_shutdown(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  /* disable specific CAN interrupt */

#if defined(CONFIG_CAN_ERRORS)
  IfxCan_Node_disableInterrupt(can_node->node,
                               IfxCan_Interrupt_busOffStatus);
#endif
  IfxCan_Node_disableInterrupt(can_node->node,
                    IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
  IfxCan_Node_disableInterrupt(can_node->node,
                               IfxCan_Interrupt_transmissionCompleted);

  /* disable specific interrupt line X */

  up_disable_irq(priv->config->tx_irq);
  up_disable_irq(priv->config->rx_irq);
#if defined(CONFIG_CAN_ERRORS)
  up_disable_irq(priv->config->err_irq);
#endif

  /* set CAN controller mode is stop mode */

  aurix_mcmcan_setmode(priv, CAN_STATE_STOP);

  /* check whether disable module by countting module reference counter */

  priv->node_group->refs--;
  if (priv->node_group->refs == 0)
    {
      mcmcan_disable_mcan_module(priv->node_group->can);
    }
}

/****************************************************************************
 * Name: aurix_mcmcan_co_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void aurix_mcmcan_co_rxint(struct can_dev_s *dev, bool enable)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  if (priv->state == CAN_STATE_DOZE)
    {
      canerr("CAN controller is in sleep mode\n");
      return;
    }

  mcmcan_interrupt_setup(enable, can_node->node,
                         IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
}

/****************************************************************************
 * Name: aurix_mcmcan_co_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void aurix_mcmcan_co_txint(struct can_dev_s *dev, bool enable)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  if (priv->state == CAN_STATE_DOZE)
    {
      canerr("CAN controller is in sleep mode\n");
      return;
    }

  mcmcan_interrupt_setup(enable, can_node->node,
                         IfxCan_Interrupt_transmissionCompleted);
}

/****************************************************************************
 * Name: aurix_mcmcan_co_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int aurix_mcmcan_co_ioctl(struct can_dev_s *dev, int cmd,
                                 unsigned long arg)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;
  int                         ret  = -ENOTTY;
  uint8                      *state;

  switch (cmd)
    {
      case CANIOC_BUSOFF_RECOVERY:
        ret = aurix_mcmcan_busoff_recovery(priv);
        break;
      case CANIOC_SET_STATE:
        ret = aurix_mcmcan_setmode(priv, arg);
        break;
      case CANIOC_GET_STATE:
        state = (uint8 *)arg;
        ret = aurix_mcmcan_getmode(priv, state);
        break;
      case CANIOC_OFLUSH:

        /* "aurix_mcmcan_busoff_recovery" API has already process hard tx
         * buffers, now "mcmcan_flush_txbuff" offers clearing all
         * pending tx buffers alone.
         */

        ret = mcmcan_flush_txbuff(priv);
        break;
      default:
        canerr("Unrecognized ioctl command: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: aurix_mcmcan_co_send
 *
 * Description:
 *   Send one can message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *   msg - The CAN message to be sent.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int aurix_mcmcan_co_send(struct can_dev_s *dev,
                                struct can_msg_s *msg)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;
  IfxCan_Message              message;

  if (priv->state == CAN_STATE_DOZE)
    {
      return -EIO;
    }

  /* fill tx message and find one available buffer */

  if (mcmcan_fill_txmsg(priv, msg, &message) == -EBUSY)
    {
      canerr("No available tx buffer\n");
      return -EBUSY;  /* No available TxBuffer */
    }

  /* send message to bus */

  caninfo("message.bufferNumber is %d\n", message.bufferNumber);
  if (IfxCan_Status_notSentBusy ==
      IfxCan_Can_sendMessage(&priv->config->can_node, &message,
                             (uint32 *)&msg->cm_data))
    {
      /* previous message was not transferred,
       * e.g. due to busy bus, BUS-OFF or others
       */

      canerr("send failed, SentBusy.\n");
      return -EBUSY;
    }

  caninfo("TX send success, canID is %ld\n", message.messageId);

  /* indicate one transition is doing in corresponding buffer */

  priv->txmb_sflags |= (1 << message.bufferNumber);
  return OK;
}

/****************************************************************************
 * Name: aurix_mcmcan_co_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool aurix_mcmcan_co_txready(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;
  uint32                      txbuffer_mask;
  uint8                       tx_buffs;

  /* Tranverse all txbuffer in this node for checking txmb_sflags */

  tx_buffs      = priv->config->can_node_config.
                  txConfig.dedicatedTxBuffersNumber;
  txbuffer_mask = (1 << tx_buffs) - 1;

  /* find available tx buffer */

  if ((priv->txmb_sflags & txbuffer_mask) != txbuffer_mask)
    {
      return true;  /* already find a available TxBuffer */
    }

  canwarn("WARN: No available TX buffer.\n");
  return false;  /* No available TxBuffer */
}

/****************************************************************************
 * Name: aurix_mcmcan_co_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool aurix_mcmcan_co_txempty(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;

  /* dircetly checking txmb_sflags */

  return (priv->txmb_sflags == 0);
}

/****************************************************************************
 * Name: aurix_mcmcan_rxfifo_filter
 *
 * Description:
 *   Obtain a rx fifo filter configuration
 *
 * Input Parameters:
 *   acf_filter - An instance of can filter structure.
 *   config - An instance of the "upper half" can driver config structure.
 *   rxfifo_filter_index - An index is used to traverse the rx fifo
 *                         filter array.
 *
 ****************************************************************************/

static void aurix_mcmcan_rxfifo_filter(IfxCan_Filter *acf_filter,
                                       struct aurix_mcmcan_config_s *config,
                                       uint8_t rxfifo_filter_index)
{
  const can_std_filter *rxfifo_filter;
  uint8_t         rxfifo1_filter_index;

  if (rxfifo_filter_index < config->rxfifo0_filter_cnt)
    {
      acf_filter->elementConfiguration =
                          IfxCan_FilterElementConfiguration_storeInRxFifo0;
      rxfifo_filter                    =
                          config->rxfifo0_filter + rxfifo_filter_index;
    }
  else
    {
      acf_filter->elementConfiguration =
                          IfxCan_FilterElementConfiguration_storeInRxFifo1;
      rxfifo1_filter_index             =
                          rxfifo_filter_index - config->rxfifo0_filter_cnt;
      rxfifo_filter                    =
                          config->rxfifo1_filter + rxfifo1_filter_index;
    }

  acf_filter->stdType = rxfifo_filter->type;
  acf_filter->id1     = rxfifo_filter->filter.can_id;
  acf_filter->id2     = rxfifo_filter->filter.can_mask;
}

/****************************************************************************
 * Name: aurix_mcmcan_rxbuf_filter
 *
 * Description:
 *   Obtain a rx buffer filter configuration
 *
 * Input Parameters:
 *   acf_filter - An instance of the can filter structure.
 *   config - An instance of the "upper half" can driver config structure.
 *   rxbuf_filter_index - An index is used to traverse the rx buffer
 *                        filter array.
 *
 ****************************************************************************/

static void aurix_mcmcan_rxbuf_filter(IfxCan_Filter *acf_filter,
                                      struct aurix_mcmcan_config_s *config,
                                      uint8_t rxbuf_filter_index)
{
  acf_filter->stdType              = IfxCan_StdFilterType_classic;
  acf_filter->rxBufferOffset       = rxbuf_filter_index % RXBUF_MAX_CNT;
  acf_filter->elementConfiguration =
                        IfxCan_FilterElementConfiguration_storeInRxBuffer;
  acf_filter->id1                  =
                        config->rxbuf_filter_id[rxbuf_filter_index];
}

/****************************************************************************
 * Name: aurix_mcmcan_filter
 *
 * Description:
 *
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 ****************************************************************************/

static void aurix_mcmcan_filter(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv               = dev->cd_priv;
  uint8_t                     rxfifo0_filter_cnt =
                                      priv->config->rxfifo0_filter_cnt;
  uint8_t                     rxfifo1_filter_cnt =
                                      priv->config->rxfifo1_filter_cnt;
  uint8_t                     rxbuf_filter_cnt   =
                                      priv->config->rxbuf_filter_cnt;
  uint8_t                     filter_cnt;
  uint8_t                     rxfifo_filter_cnt;

  rxfifo_filter_cnt = rxfifo0_filter_cnt + rxfifo1_filter_cnt;
  filter_cnt        = rxfifo_filter_cnt + rxbuf_filter_cnt;

  for (int filer_index = 0; filer_index < filter_cnt; filer_index++)
    {
      IfxCan_Filter acf_filter =
        {
          0
        };

      acf_filter.number = filer_index;

      if (filer_index < rxfifo_filter_cnt)
        {
          aurix_mcmcan_rxfifo_filter(&acf_filter,
                                     priv->config, filer_index);
        }
      else
        {
          aurix_mcmcan_rxbuf_filter(&acf_filter, priv->config,
                                     filer_index - rxfifo_filter_cnt);
        }

      IfxCan_Can_setStandardFilter(&priv->config->can_node, &acf_filter);
    }
}

/****************************************************************************
 * Name: aurix_mcmcan_uni_routing
 *
 * Description:
 *
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 ****************************************************************************/

#if defined(CONFIG_AURIX_MCMCAN_CRE)
static void aurix_mcmcan_uni_routing(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;

  uint8 uni_routing_num = priv->config->uni_routing_cnt;

  for (uint8 i = 0; i < uni_routing_num; i++)
    {
      IfxCan_Can_setStandardUnicastRouting(&priv->config->can_node,
                                           &priv->config->uni_routing[i]);
    }
}
#endif

/****************************************************************************
 * Name: aurix_mcmcan_mul_routing
 *
 * Description:
 *
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver structure.
 *
 ****************************************************************************/

#if defined(CONFIG_AURIX_MCMCAN_CRE)
static void aurix_mcmcan_mul_routing(struct can_dev_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;
  uint8                       num  = priv->config->mul_routing_cnt;

  for (uint8 i = 0; i < num; i++)
    {
      IfxCan_Can_setStandardMulticastRouting(&priv->config->can_node,
                                             &priv->config->mul_routing[i]);
    }
}
#endif

/****************************************************************************
 * Name: aurix_mcmcan_co_cancel
 *
 * Description:
 *   Cancel one can message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *   msg - One can message.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_TXCANCEL
static bool aurix_mcmcan_co_cancel(struct can_dev_s *dev,
                                   struct can_msg_s *msg)
{
  struct aurix_mcmcan_priv_s *priv     = dev->cd_priv;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;
  uint8 tx_buffs                       = priv->config->can_node_config.
                                         txConfig.dedicatedTxBuffersNumber;
  uint8 txbuffer_id;

  /* Tranverse all txbuffer in this node for finding specfic txbuffer
   * that includes identical canID with msg->cm_hdr.can_id.
   */

  for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
    {
      /* Get the Tx Bufer ELement address */

      Ifx_CAN_TXMSG *txbuf_element = IfxCan_Node_getTxBufferElementAddress(
                                  can_node->node,
                                  can_node->messageRAM.baseAddress,
                                  can_node->messageRAM.txBuffersStartAddress,
                                  txbuffer_id);

      /* Acquire the msgID of the specfic txbuffer */

      uint32 txbuf_msgid = txbuf_element->T0.B.ID >> 18;

      if (txbuf_msgid == msg->cm_hdr.ch_id)
        {
          /* Cancel the specfic txbuffer */

          if ((priv->txmb_sflags & (1 << txbuffer_id)) != 0
              && !IfxCan_Node_isTxBufferRequestPending(can_node->node,
                                                       txbuffer_id)
              && IfxCan_Node_isTxBufferTransmissionOccured(can_node->node,
                                                           txbuffer_id))
            {
              /* Do not cancel if txmsg have sent to bus */
            }
          else
            {
              /* Cancel sending and pending tx buffers */

              IfxCan_Node_setTxBufferCancellationRequest(can_node->node,
                                                         txbuffer_id);

              /* If trasmission is started, cancellation will wait that this
               * transmission is over, because of cancellation operation
               * do not block long time.
               */

              while (!IfxCan_Node_isTxBufferCancellationFinished(
                     can_node->node, txbuffer_id));

              /* Check whether successful transmission or not in spite
               * of cancellation. please refer the of 22.5.3.5.7 chapter
               * the AURIX TC4DX user manual.
               */

              if (!IfxCan_Node_isTxBufferTransmissionOccured(
                  can_node->node, txbuffer_id))
                {
                  /* If transmission is failure we must clear
                   * the specfic txmb_sflags
                   */

                  priv->txmb_sflags &= ~(1 << txbuffer_id);
                  return true;
                }
            }

          return false;
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Name:  aurix_mcmcan_tx_interrupt
 * Description:
 *   this function is "tx interrupt service function" that about tx matters
 *   handle.
 *
 * Input Parameters:
 *    irq     - number of the IRQ that generated the interrupt.
 *    context - the interrupt register state save
 *              info (architecture-specific).
 *    arg     - the argument passed when the interrupt was attached.
 *
 * Returned Value:
 *    OK on success.
 ****************************************************************************/

static int aurix_mcmcan_tx_interrupt(int irq, void *context, void *arg)
{
  struct aurix_mcmcan_priv_s *priv     = arg;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
      IfxCan_Interrupt_transmissionCompleted))
    {
      uint8 tx_buffs = priv->config->can_node_config.
                       txConfig.dedicatedTxBuffersNumber;
      uint8 txbuffer_id;

      IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                                     IfxCan_Interrupt_transmissionCompleted);

      /* Tranverse all txbuffer in this node for checking txmb_sflags */

      for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
        {
          /* TX.BRP corresponding bit reset and TX.BTO corresponding
           *  bit reset when transmition completed
           */

          if ((priv->txmb_sflags & (1 << txbuffer_id)) != 0
              && !IfxCan_Node_isTxBufferRequestPending(can_node->node,
                                                       txbuffer_id)
              && IfxCan_Node_isTxBufferTransmissionOccured(can_node->node,
                                                           txbuffer_id))
            {
#ifdef CONFIG_CAN_TXCONFIRM
              aurix_mcmcan_tx_confirm((struct can_dev_s *)priv,
                                      txbuffer_id);
#endif
              priv->txmb_sflags &= ~(1 << txbuffer_id);
              can_txdone((struct can_dev_s *)priv);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_mcmcan_rx_interrupt
 * Description:
 *   this function is "rx interrupt service function" that about rx matters
 *   handle.
 *
 * Input Parameters:
 *    irq     - number of the IRQ that generated the interrupt.
 *    context - the interrupt register state save
 *              info (architecture-specific).
 *    arg     - the argument passed when the interrupt was attached.
 *
 * Returned Value:
 *    OK on success.
 ****************************************************************************/

static int aurix_mcmcan_rx_interrupt(int irq, void *context, void *arg)
{
  struct aurix_mcmcan_priv_s *priv     = arg;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  /* check whether there is new rxmsg in CRE RX Buffer 1,
   * first we must enable CRE ability and let "COGFIG_CAN'X'_NODE'Y'_RF1_NUM"
   * equal to non-zero value if we want to use "CRE RX Buffer 1" to acquire
   * rxmsg for the application.
   */

#if defined(CONFIG_AURIX_MCMCAN_CRE)
  Ifx_CAN_N_CRE *cre = &(can_node->node->CRE);
  uint8_t       *hbuf_data;

  if (cre->HBUF.RX[IfxCan_CreRxHostBufferIndex_1].STAT.B.RHREQ == 1
     && (cre->IR.U & (1 << IfxCan_CreInterrupt_RxBuffer1)) != 0)
    {
      struct can_hdr_s hdr =
      {
        0
      };

      /* Take data address to specfic location
       * for resolving data address not alignment
       */

      IfxCan_CreRxBuffer buffer aligned_data(4) =
        {
          0
        };

      /* CRE RX Buffer 1 interrupt is occurred, clear it. */

      IfxCan_Node_clearCreInterrupt(can_node->node,
                                    IfxCan_CreInterrupt_RxBuffer1);

      /* Use buffer to acquire rxmsg info form CRE RX Buffer 1. */

      IfxCan_Can_Cre_readNewMessageBlock(can_node,
                                         IfxCan_CreRxHostBufferIndex_1,
                                         &buffer);

      if (buffer.isMessageAvailable)
        {
          hdr.ch_id  = buffer.rxHostBuffer.R0.B.ID >> 18;
          hdr.ch_dlc = (IfxCan_DataLengthCode)buffer.rxHostBuffer.R1.B.DLC;
          hdr.ch_edl = buffer.rxHostBuffer.R1.B.FDF ? 1 : 0;
          hdr.ch_brs = buffer.rxHostBuffer.R1.B.BRS ? 1 : 0;
          hdr.ch_rtr = 0;
          hbuf_data  = (uint8_t *)&buffer.rxHostBuffer.RHBUF_DB[0];
#ifdef CONFIG_CAN_TIMESTAMP

          /* set gptp timestamp */

          mcmcan_set_timestamp(&hdr);
#endif
          /* Provide the CAN message to the upper half driver */

          can_receive((struct can_dev_s *)priv, &hdr, hbuf_data);
        }
    }
#endif

  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
      IfxCan_Interrupt_messageStoredToDedicatedRxBuffer))
    {
      uint32_t       new_data_flag1 = can_node->node->NDAT1.U;
      uint32_t       new_data_flag2 = can_node->node->NDAT2.U;
      IfxCan_Message rxmessage;

      struct can_hdr_s hdr =
      {
        0
      };

      /* Take data address to specfic location
       * for resolving data address not alignment
       */

      uint8 data[CAN_MAXDATALEN] aligned_data(4) =
        {
          0
        };

      IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
      IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);

      /* Read the received CAN message from specfic RX buffer type determined
       * by priv->config->can_node_config.rxConfig.rxMode. in this code block
       * we acquire messages from dedicated RX Buffers.
       */

      IfxCan_Can_initMessage(&rxmessage);

      /* Process NDAT1 register (RX buffers 0-31) */

      while (new_data_flag1 != 0)
        {
          /* Find lowest set bit position */

          rxmessage.bufferNumber = ffs(new_data_flag1) - 1;
          new_data_flag1         &= ~(1 << rxmessage.bufferNumber);
          process_rxbuf_msg(priv, &rxmessage, &hdr, data);
        }

      /* Process NDAT2 register (RX buffers 32 ~ 63) */

      while (new_data_flag2 != 0)
        {
          /* Find lowest set bit position */

          uint8_t bit_pos = ffs(new_data_flag2) - 1;

          rxmessage.bufferNumber = bit_pos + 32;
          new_data_flag2         &= ~(1 << bit_pos);
          process_rxbuf_msg(priv, &rxmessage, &hdr, data);
        }
    }

  return OK;
}

#if defined(CONFIG_CAN_ERRORS)

/****************************************************************************
 * Name: aurix_mcmcan_err_interrupt
 * Description:
 *   this function is "error interrupt service function" that about
 *   error matters handle.
 *
 * Input Parameters:
 *    irq     - number of the IRQ that generated the interrupt.
 *    context - the interrupt register state save
 *              info (architecture-specific).
 *    arg     - the argument passed when the interrupt was attached.
 *
 * Returned Value:
 *    OK on success.
 ****************************************************************************/

static int aurix_mcmcan_err_interrupt(int irq, void *context, void *arg)
{
  struct aurix_mcmcan_priv_s *priv     = arg;
  IfxCan_Can_Node            *can_node = &priv->config->can_node;

  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
                                         IfxCan_Interrupt_busOffStatus))
    {
      IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                                     IfxCan_Interrupt_busOffStatus);

      /* InterruptFlag maybe accumulate but PSR(protocol state register)
       * have already been recovery.
       */

      if (IfxCan_Node_getBusOffStatus(can_node->node))
        {
          uint8_t data[8];
          struct can_hdr_s hdr =
          {
            0
          };

          hdr.ch_id            = CAN_ERROR_BUSOFF;
          hdr.ch_dlc           = CAN_ERROR_DLC;
          hdr.ch_error         = 1;
          memset(data, 0, sizeof(data));
          canwarn("bus off interrupt");
          return can_receive((struct can_dev_s *)priv, &hdr, data);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: mcmcan_flush_txbuff
 *
 * Description:
 *   cancel all pending tx buffers. in mcmcan no register is used to clear
 *   txbuffers, so we use "cancel register" to clear txbuffers.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static int mcmcan_flush_txbuff(struct aurix_mcmcan_priv_s *priv)
{
  IfxCan_Can_Node       *can_node    = &priv->config->can_node;
  IfxCan_Can_NodeConfig *node_config = &priv->config->can_node_config;
  uint8                  tx_buffs    = node_config->txConfig.
                                       dedicatedTxBuffersNumber;
  uint8                  txbuffer_id;

  /* cancel all pending tx buffers */

  for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
    {
      if ((priv->txmb_sflags & (1 << txbuffer_id)) != 0
          && !IfxCan_Node_isTxBufferRequestPending(can_node->node,
                                                   txbuffer_id)
          && IfxCan_Node_isTxBufferTransmissionOccured(can_node->node,
                                                       txbuffer_id))
        {
          continue; /* do not cancel if txmsg have sent to bus */
        }
      else
        {
          /* cancel sending and pending tx buffers */

          IfxCan_Node_setTxBufferCancellationRequest(can_node->node,
                                                     txbuffer_id);

          /* if trasmission is started, cancellation will wait that this
           * transmission is over, because of cancellation operation
           * do not block long time.
           */

          while (!IfxCan_Node_isTxBufferCancellationFinished(can_node->node,
                                                             txbuffer_id));
        }
    }

  priv->txmb_sflags = 0;
  return OK;
}

/****************************************************************************
 * Name: mcmcan_fill_txmsg
 * Description:
 *   fill msg to message and find one available tx buffer.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *   msg - the message from upper half driver.
 *   message - the message to be filled.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure.
 *
 ****************************************************************************/

static int mcmcan_fill_txmsg(struct aurix_mcmcan_priv_s *priv,
                          struct can_msg_s *msg, IfxCan_Message *message)
{
  IfxCan_Can_Node  *can_node   = &priv->config->can_node;
  uint8             tx_buffs   = priv->config->can_node_config.
                                 txConfig.dedicatedTxBuffersNumber;
  bool              avail_flag = false;
  IfxCan_TxBufferId txbuffer_id;
  uint8             txbuffer_index;

  for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
    {
      /* Finding one available tx buffer */

      if (avail_flag == false &&
          ((1 << txbuffer_id) & priv->txmb_sflags) == 0)
        {
          caninfo("txbuffer_id is %d.\n", txbuffer_id);

          /* Already find a available tx buffer */

          txbuffer_index = txbuffer_id;
          avail_flag     = true;
          continue;
        }

      /* Check whether there is a msg with the same canid as
       * msg->cm_hdr.ch_id in the tx buffer where is transmitting tx msg.
       *
       * If sure there is a msg with the same canid in the tx buffer,
       * return -EBUSY.
       */

      if (((1 << txbuffer_id) & priv->txmb_sflags) == 1)
        {
          Ifx_CAN_TXMSG *txbuf_element =
                         IfxCan_Node_getTxBufferElementAddress(
                         can_node->node,
                         can_node->messageRAM.baseAddress,
                         can_node->messageRAM.txBuffersStartAddress,
                         txbuffer_id);

          /* Acquire the canid of the specfic txbuffer */

          uint32 txbuf_msgid = txbuf_element->T0.B.ID >> 18;

          if (txbuf_msgid == msg->cm_hdr.ch_id)
            {
              canwarn("WARN: Found the same CAN ID in txbuffer %d.\n",
                      txbuffer_id);
              return -EBUSY;  /* Found the same canid */
            }
        }
    }

  if (avail_flag == true)
    {
      /* Initialize the message frame with default values */

      IfxCan_Can_initMessage(message);

      /* Fill message */

      message->bufferNumber    = txbuffer_index;
      message->messageId       = msg->cm_hdr.ch_id;
      message->messageIdLength = IfxCan_MessageIdLength_standard;
      message->dataLengthCode  = msg->cm_hdr.ch_dlc;

      /* Confirm message mode */

      if (msg->cm_hdr.ch_edl == 1 && msg->cm_hdr.ch_brs == 1)
        {
          message->frameMode = IfxCan_FrameMode_fdLongAndFast;
        }
      else if (msg->cm_hdr.ch_edl == 1)
        {
          message->frameMode = IfxCan_FrameMode_fdLong;
        }
      else
        {
          message->frameMode = IfxCan_FrameMode_standard;
        }
    }

  caninfo("avail_flag = %d, if avail_flag is 1, it found available "
          "tx buffer; else no available tx buffer\n", avail_flag);
  return avail_flag ? OK : -EBUSY;
}

/****************************************************************************
 * Name: mcmcan_config_setup
 * Description:
 *  setup common mcmcan config.
 *
 * Input Parameters:
 *   can_node_config - CAN node configuration structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static void mcmcan_config_setup(struct aurix_mcmcan_config_s *g_config)
{
  IfxCan_Can_NodeConfig *config = &g_config->can_node_config;
#if defined(CONFIG_AURIX_MCMCAN_CRE)
  IfxCan_CreConfig      *cre_config = &g_config->can_node_cre_config;
#endif

  config->baudRate.syncJumpWidth             = 3;
  config->baudRate.prescaler                 = 0;
  config->baudRate.timeSegment1              = 3;
  config->baudRate.timeSegment2              = 10;

  config->fastBaudRate.syncJumpWidth         = 3;
  config->fastBaudRate.prescaler             = 1;
  config->fastBaudRate.timeSegment1          = 3;
  config->fastBaudRate.timeSegment2          = 10;
  config->fastBaudRate.tranceiverDelayOffset = 0;
  config->clockSource                        = IfxCan_ClockSource_both;
  config->frame.type                         =
                                   IfxCan_FrameType_transmitAndReceive;
  config->frame.mode                         =
                                        IfxCan_FrameMode_fdLongAndFast;
  config->txConfig.txMode                    = IfxCan_TxMode_sharedQueue;
  config->txConfig.txBufferDataFieldSize     = IfxCan_DataFieldSize_64;
  config->rxConfig.rxMode                    = IfxCan_RxMode_sharedAll;
  config->rxConfig.rxBufferDataFieldSize     = IfxCan_DataFieldSize_64;
  config->rxConfig.rxFifo0DataFieldSize      = IfxCan_DataFieldSize_64;
  config->rxConfig.rxFifo1DataFieldSize      = IfxCan_DataFieldSize_64;
  config->calculateBitTimingValues           = TRUE;

  /* Must to startup accept filtet ability
   * before using dedicated Rx Buffer
   */

  config->filterConfig.rejectRemoteFramesWithStandardId   = TRUE;
  config->filterConfig.rejectRemoteFramesWithExtendedId   = TRUE;
  config->filterConfig.standardFilterForNonMatchingFrames =
                                          IfxCan_NonMatchingFrame_reject;
  config->filterConfig.extendedFilterForNonMatchingFrames =
                                          IfxCan_NonMatchingFrame_reject;

#if defined(CONFIG_CAN_ERRORS)
  config->interruptConfig.busOffStatusEnabled                     = TRUE;
#endif
  config->interruptConfig.messageStoredToDedicatedRxBufferEnabled = TRUE;
  config->interruptConfig.transmissionCompletedEnabled            = TRUE;

#if defined(CONFIG_AURIX_MCMCAN_CRE)
  cre_config->enableCre                           = TRUE;
  cre_config->enableCreRouting                    = TRUE;
  cre_config->enableDestinationRouting            = TRUE;
  cre_config->rxBuf0DreTriggerEnable              = TRUE;
  cre_config->rxBuf1DreTriggerEnable              = FALSE;

  /* Cre Rx Host Buffer1 interrupt enable */

  cre_config->interrupt.rxBuffer1InterruptEnable  = TRUE;
#endif
}

/****************************************************************************
 * Name: mcmcan_disable_mcan_module
 * Description:
 *  disable the specfic mcmcan module.
 *
 * Input Parameters:
 *   can - CAN module address.
 *
 * Returned Value:
 *
 ****************************************************************************/

static void mcmcan_disable_mcan_module(Ifx_CAN *can)
{
  if (IfxCan_isModuleEnabled(can) == true)
    {
      /* Reset can module for deinitialization */

      IfxCan_resetModule(can);

      /* disable this module */

      IfxCan_disableModule(can);
    }
}

/****************************************************************************
 * Name: aurix_mcmcan_tx_confirm
 * Description:
 *  acquire the msg id that had been sent when tx interrupt is triggered.
 *
 * Input Parameters:
 *   dev - the struct can_dev_s object.
 *   txbuffer_id - the tx buffer id.
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_TXCONFIRM
static void aurix_mcmcan_tx_confirm(struct can_dev_s *dev,
                                    uint8 txbuffer_id)
{
  struct aurix_mcmcan_priv_s *priv = dev->cd_priv;
  IfxCan_Can_Node            *node = &priv->config->can_node;
  Ifx_CAN_TXMSG              *txBufferElement;
  struct can_hdr_s            hdr  =
  {
    0
  };

  /* fill the "tcf" member variable of tx confirmation msg */

  hdr.ch_dlc      = 0;
  hdr.ch_tcf      = 1;

  /* get the Tx message id from the specific Tx Bufer and
   * transmit this msg to software buffers
   */

  txBufferElement = IfxCan_Node_getTxBufferElementAddress(
                    node->node, node->messageRAM.baseAddress,
                    node->messageRAM.txBuffersStartAddress, txbuffer_id);
  hdr.ch_id       = txBufferElement->T0.B.ID >> 18;
  can_receive(dev, &hdr, NULL);
}
#endif

/****************************************************************************
 * Public Functions
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

int aurix_chardriver_init_mcmcan(struct can_dev_s **dev,
                                 struct aurix_mcmcan_config_s *config,
                                 size_t num)
{
  struct aurix_mcmcan_priv_s *priv;
  char                        name[16];
  uint8                       i;
  int                         ret;

  /* Ensure correct clock frequencies for Mcan */

  IfxClock_setMcanFrequency(40000000);
  IfxClock_setMcanhFrequency(250000000);

  for (i = 0; i < num; i++)
    {
      priv = kmm_zalloc(sizeof(struct aurix_mcmcan_priv_s));
      if (priv == NULL)
        {
          nerr("aurix can kmm_zalloc failed\n");
          return -ENOMEM;
        }

      /* fill ops and lower driver data struct,
       * set dev[i] must be front to can_register,
       * because dev_reset will be called in can_register
       */

      priv->config     = &config[i];
      priv->node_group = &g_node_group[config[i].intf / 4];
      dev[i]           = &priv->dev;
      dev[i]->cd_ops   = &g_aurix_mcmcanops;
      dev[i]->cd_priv  = priv;

      /* CANX STB pin of transceiver must set 0 for normal operation */

      if (priv->config->can_transv_pin.port != NULL)
        {
          IfxPort_Pin *transv_pin;

          transv_pin = &priv->config->can_transv_pin;
          IfxPort_setPinLow(transv_pin->port, transv_pin->pinIndex);
          IfxPort_setPinModeOutput(transv_pin->port, transv_pin->pinIndex,
                                   IfxPort_OutputMode_pushPull,
                                   IfxPort_OutputIdx_general);
        }

      /* register /dev/canX */

      snprintf(name , sizeof(name), "/dev/can%d", priv->config->intf);
      ret = can_register(name, dev[i]);
      if (ret < 0)
        {
          kmm_free(priv);
          dev[i] = NULL;
          canerr("ERROR: Register CAN interfacefailed: %d\n", ret);
          return -ENOMEM;
        }

      /* attach irq */

#ifdef CONFIG_AURIX_MCMCAN_ISR_WQUEUE
      ret = irq_attach_wqueue(priv->config->tx_irq, NULL,
                              aurix_mcmcan_tx_interrupt, priv,
                              CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
      ret = irq_attach_wqueue(priv->config->rx_irq, NULL,
                              aurix_mcmcan_rx_interrupt, priv,
                              CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
#if defined(CONFIG_CAN_ERRORS)
      ret = irq_attach_wqueue(priv->config->err_irq, NULL,
                              aurix_mcmcan_err_interrupt, priv,
                              CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
#endif
#else
      irq_attach(priv->config->tx_irq, aurix_mcmcan_tx_interrupt, priv);
      irq_attach(priv->config->rx_irq, aurix_mcmcan_rx_interrupt, priv);
#if defined(CONFIG_CAN_ERRORS)
      irq_attach(priv->config->err_irq, aurix_mcmcan_err_interrupt, priv);
#endif
#endif

      priv->state = CAN_STATE_STOP;
    }

  return OK;
}
