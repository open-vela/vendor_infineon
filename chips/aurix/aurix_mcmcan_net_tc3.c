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

#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include "aurix_mcmcan_tc3.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct node_group_s
{
  Ifx_CAN *can;      /* module address */
  uint8   ref_index; /* index of the can_module in shared_data_manual */
                     /* service for multi core,
                      * en_flag == false, disable can moudle.
                      * en_flag == true, enable can module.
                      */
  bool    en_flag;
};

struct aurix_mcmcan_priv_s
{
  struct netdev_lowerhalf_s     dev;              /* CAN device struct object */
  struct aurix_mcmcan_config_s *config;           /* mcmcan config object */
  struct node_group_s          *node_group;       /* CAN node group */

  /* this member is used to trace
    * those net pkts which are stored
    * into hardware tx buffer but do
    * not finish to send out.
    */

  netpkt_t                     **tx_pkt_pending;
  /* any bit = 0, txmb available
   * any bit = 1, txmb transmition
   * is doing
   */

  uint32                        txmb_sflags;
  uint32                        rxmb0_sflags;
  uint32                        rxmb1_sflags;
#ifdef CONFIG_AURIX_MCMCAN_CRE
  bool                          cre_rxfifo0_pending;
  bool                          cre_rxfifo1_pending;
#else
  uint8                         rxfifo0_pending;
  uint8                         rxfifo1_pending;
#endif
  bool                          err_pending;

  uint8                         state;            /* CAN node controller state */
#ifdef CONFIG_AURIX_CAN_ERROR_POLLING
  struct work_s                 errwork;          /* Report error info to upper
                            * user by using LP workqueue
                            */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mcmcan_fill_txmsg(struct aurix_mcmcan_priv_s *priv,
                             FAR struct canfd_frame * frame,
                             IfxCan_Message *message);
static void mcmcan_config_setup(struct aurix_mcmcan_config_s *config);

static void mcmcan_disable_mcan_module(Ifx_CAN *can);

static int aurix_mcmcan_rx_interrupt(int irq, void *context, void *arg);
static int aurix_mcmcan_tx_interrupt(int irq, void *context, void *arg);
#ifdef CONFIG_AURIX_MCMCAN_TXCONFIRM
static uint8 aurix_mcmcan_tx_confirm(struct aurix_mcmcan_priv_s *priv,
                                     uint8 *buf);
#else
static void aurix_mcmcan_tx_done(struct aurix_mcmcan_priv_s *priv);
#endif

static int mcmcan_flush_txbuff(struct aurix_mcmcan_priv_s *priv);

#ifdef CONFIG_NETDEV_IOCTL
static int mcmcan_transv_setmode(struct aurix_mcmcan_config_s *config,
    unsigned long arg);
static int mcmcan_transv_getmode(struct aurix_mcmcan_config_s *config,
    unsigned long arg);
static inline int aurix_mcmcan_getmode(struct aurix_mcmcan_priv_s *priv,
                                       enum can_ioctl_state_e *state);
#endif

static int aurix_mcmcan_setmode(struct aurix_mcmcan_priv_s *priv,
    enum can_ioctl_state_e state);

static void aurix_mcmcan_set_loopback_mode(struct aurix_mcmcan_priv_s *priv);
static int aurix_mcmcan_co_setup(struct aurix_mcmcan_priv_s *dev);
static void aurix_mcmcan_co_shutdown(struct aurix_mcmcan_priv_s *dev);
static void aurix_mcmcan_co_rxint(struct aurix_mcmcan_priv_s *dev,
    bool enable);
static void aurix_mcmcan_co_txint(struct aurix_mcmcan_priv_s *dev,
    bool enable);
static void aurix_mcmcan_filter(struct aurix_mcmcan_priv_s *dev);

#if defined(CONFIG_AURIX_MCMCAN_CRE)
static void aurix_mcmcan_mul_routing(struct aurix_mcmcan_priv_s *priv);
static void aurix_mcmcan_uni_routing(struct aurix_mcmcan_priv_s *priv);
#endif

static int aurix_mcmcan_err_interrupt(int irq, void *context, void *arg);
static int aurix_mcmcan_busoff_recovery(struct aurix_mcmcan_priv_s *priv);
#ifdef CONFIG_NET_CAN_ERRORS
static uint8 aurix_mcmcan_errhandle(struct aurix_mcmcan_priv_s *priv,
                                    uint8 *buf);
#endif

#if defined(CONFIG_AURIX_CAN_ERROR_POLLING)
static void aurix_mcmcan_errpolling(FAR void *arg);
static void enable_destruct_readmode(IfxCan_Can_Node *can_node);
#endif

static uint8 aurix_mcmcan_read_message(IfxCan_Can_Node *node, uint8 *buf,
    uint8 buffer_type, uint8 buffer_id);
#ifdef CONFIG_AURIX_MCMCAN_CRE
static uint8 aurix_mcmcan_read_cre_message(IfxCan_Can_Node *node,
                                           uint8 *buf, uint8 buffer_type);
#endif
static int net_mcmcan_ifup(FAR struct netdev_lowerhalf_s *dev);
static int net_mcmcan_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int net_mcmcan_transmit(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt);
static netpkt_t *net_mcmcan_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int net_mcmcan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
    unsigned long arg);
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
    }
};

static const struct netdev_ops_s g_aurix_mcmcan_ops =
{
  .ifup     = net_mcmcan_ifup,      /* ifup */
  .ifdown   = net_mcmcan_ifdown,    /* ifdown */
  .transmit = net_mcmcan_transmit,  /* transmit */
  .receive  = net_mcmcan_receive,   /* receive */
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = net_mcmcan_ioctl,     /* ioctl */
#endif
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

static_assert(CONFIG_IOB_BUFSIZE >= sizeof(struct canfd_frame) +
              CONFIG_NET_LL_GUARDSIZE, "iob size too small");

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  IfxCan_Can_Node *can_node = &priv->config->can_node;

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
 * Name: mcmcan_transv_setmode
 *
 * Description:
 *   Set mcmcan transceiver state machine mode.
 *
 * Input Parameters:
 *   config - An instance of the "lower half" can driver structure.
 *   arg    - Mode.
 *
 * Returned Value:
 *   Zero on success; a negative errno on failure
 *
 ****************************************************************************/

static int mcmcan_transv_setmode(struct aurix_mcmcan_config_s *config,
    unsigned long arg)
{
  FAR struct can_transv_s             *transv       = config->can_transv;
  FAR struct can_ioctl_transv_state_s *transv_state =
                                      (struct can_ioctl_transv_state_s *)arg;
  int                                  ret;

  if (transv && transv->ct_ops && transv->ct_ops->ct_setstate)
    {
      FAR const struct can_transv_ops_s *ct_ops = transv->ct_ops;

      ret = ct_ops->ct_setstate(transv, transv_state->state);
    }
  else
    {
      canerr("transceiver set mode is failed!");
      ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Name: mcmcan_transv_getmode
 *
 * Description:
 *   Get mcmcan transceiver state machine mode.
 *
 * Input Parameters:
 *   config - An instance of the "lower half" can driver structure.
 *   arg    - Mode.
 *
 * Returned Value:
 *   Zero on success; a negative errno on failure
 *
 ****************************************************************************/

static int mcmcan_transv_getmode(struct aurix_mcmcan_config_s *config,
    unsigned long arg)
{
  FAR struct can_transv_s             *transv       = config->can_transv;
  FAR struct can_ioctl_transv_state_s *transv_state =
                                      (struct can_ioctl_transv_state_s *)arg;
  int                                  ret;

  if (transv && transv->ct_ops && transv->ct_ops->ct_getstate)
    {
      int *state  = (int *)&transv_state->state;
      FAR const struct can_transv_ops_s *ct_ops = transv->ct_ops;

        ret = ct_ops->ct_getstate(transv, state);
    }
  else
    {
        caninfo("transceiver set mode is failed!");
        ret = -ENOTTY;
    }

  return ret;
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
                                       enum can_ioctl_state_e *state)
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
    enum can_ioctl_state_e state)
{
  IfxCan_Can_Node     *can_node = &priv->config->can_node;
  uint8               ref_index = priv->node_group->ref_index;
  volatile spinlock_t *lock     =
           &shared_data_manual.can_module[ref_index].can_module_lock;
  irqstate_t          irq_mask;

  if (priv->state == state)
    {
      return OK;
    }

  /* the detail of operation steps refer to the MCMCAN operating Mode
   * section of TC4DX and TC3XX user manual.
   *
   * confirm below matter:
   * enable CAN module that is correspond with CAN node
   * before operating that CAN node register.
   */

  switch (state)
    {
    case CAN_STATE_OPERATIONAL:

        /* add spin lock to protect shared_data_manual */

        irq_mask = spin_lock_irqsave_notrace(lock);

        /* make controller come in normal operation */

        if (shared_data_manual.can_module[ref_index].can_module_ref == 1)
          {
            priv->node_group->en_flag = true;
            shared_data_manual.can_module[ref_index].can_module_ref++;
          }

        /* unlock spin lock to protect shared_data_manual */

        spin_unlock_irqrestore_notrace(lock, irq_mask);

        if (priv->node_group->en_flag == true &&
            IfxCan_isModuleEnabled(priv->node_group->can) != TRUE)
          {
            /* Enable module, disregard Sleep Mode request */

            IfxCan_enableModule(priv->node_group->can);
        }

        if (priv->state == CAN_STATE_STOPPED)
          {
            IfxCan_Node_setInitialisation(can_node->node, false);
            while (can_node->node->CCCR.B.INIT != 0);
          }
        else if (priv->state == CAN_STATE_SLEEP)
          {
            can_node->node->CCCR.B.CSR          = 0;
            while (can_node->node->CCCR.B.CSA  != 0);

            IfxCan_Node_setInitialisation(can_node->node, false);
            while (can_node->node->CCCR.B.INIT != 0);
        }

        priv->state = CAN_STATE_OPERATIONAL;
        return OK;
    case CAN_STATE_STOPPED:

        /* make controller come in stop operation */

        if (priv->state == CAN_STATE_OPERATIONAL)
          {
            IfxCan_Node_setInitialisation(can_node->node, true);
            while (can_node->node->CCCR.B.INIT != 1);
          }
        else if (priv->state == CAN_STATE_SLEEP)
          {
            can_node->node->CCCR.B.CSR          = 0;
            while (can_node->node->CCCR.B.CSA  != 0);

            IfxCan_Node_setInitialisation(can_node->node, true);
            while (can_node->node->CCCR.B.INIT != 1);
        }

        priv->state = CAN_STATE_STOPPED;
        return OK;
    case CAN_STATE_SLEEP:

        /* make controller come in sleep operation */

        can_node->node->CCCR.B.CSR          = 1;
        while (can_node->node->CCCR.B.INIT != 1 &&
               can_node->node->CCCR.B.CSA  != 1);

        /* add spin lock to protect shared_data_manual */

        irq_mask = spin_lock_irqsave_notrace(lock);

        shared_data_manual.can_module[ref_index].can_module_ref--;
        if (shared_data_manual.can_module[ref_index].can_module_ref == 1)
          {
            priv->node_group->en_flag = false;
        }

        /* unlock spin lock to protect shared_data_manual */

        spin_unlock_irqrestore_notrace(lock, irq_mask);

        if (priv->node_group->en_flag == false &&
            IfxCan_isModuleEnabled(priv->node_group->can) == TRUE)
          {
            mcmcan_disable_mcan_module(priv->node_group->can);
        }

        priv->state = CAN_STATE_SLEEP;
        return OK;
    default:
        return -ENOTTY;
    }
}

#if defined(CONFIG_AURIX_CAN_ERROR_POLLING)

/****************************************************************************
 * Name: aurix_mcmcan_errpolling
 *
 * Description:
 *   This function performs in specfic cycle that is specified by
 *   CONFIG_CAN_ERROR_POLLING_CYCLE.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void aurix_mcmcan_errpolling(FAR void *arg)
{
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)arg;

  priv->err_pending = true;
  netdev_lower_rxready((struct netdev_lowerhalf_s *)priv);

  work_queue(LPWORK, &priv->errwork, aurix_mcmcan_errpolling,
        priv, MSEC2TICK(CONFIG_CAN_ERROR_POLLING_CYCLE));
}

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
#endif

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: aurix_mcmcan_errhandle
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

static uint8 aurix_mcmcan_errhandle(struct aurix_mcmcan_priv_s *priv,
                                    uint8 *buf)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;
  struct can_frame *frame = (struct can_frame *)buf;

  IfxCan_CanNodeErrorWarningLimitStatus warn_state;
  IfxCan_LastErrorCodeType              last_errcode;
  uint16_t                              errbits;
  uint8_t                               data[CAN_ERR_DLC];

  if (IfxCan_Node_getBusOffStatus(can_node->node))
    {
      errbits = CAN_ERR_BUSOFF;
      goto err_out;
    }

  last_errcode = IfxCan_Node_getLastErroCodeStatus(can_node->node);

  if (last_errcode == IfxCan_LastErrorCodeType_noError ||
      last_errcode == IfxCan_LastErrorCodeType_noCANBusEvent)
    {
      return 0;
    }

  errbits = 0;
  memset(data, 0, sizeof(data));

  if (last_errcode == IfxCan_LastErrorCodeType_stuffError)
    {
      /* Stuff Error */

      data[2] |= CAN_ERR_PROT_STUFF;
      errbits |= CAN_ERR_PROT;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_formError)
    {
      /* Format Error */

      data[2] |= CAN_ERR_PROT_FORM;
      errbits |= CAN_ERR_PROT;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_ackError)
    {
      /* Acknowledge Error */

      errbits |= CAN_ERR_ACK;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_bit1Error)
    {
      /* Bit recessive Error */

      data[2] |= CAN_ERR_PROT_BIT;
      data[2] |= CAN_ERR_PROT_BIT1;
      errbits |= CAN_ERR_PROT;
    }

  if (last_errcode == IfxCan_LastErrorCodeType_bit0Error)
    {
      /* Bit domainant Error */

      data[2] |= CAN_ERR_PROT_BIT;
      data[2] |= CAN_ERR_PROT_BIT0;
      errbits |= CAN_ERR_PROT;
    }

  if (last_errcode ==  IfxCan_LastErrorCodeType_crcError)
    {
      /* Receive CRC Error */

      data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
      errbits |= CAN_ERR_PROT;
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

          data[1] |= CAN_ERR_CRTL_TX_WARNING;
          errbits |= CAN_ERR_CRTL;
        }

      if (can_node->node->ECR.B.REC >= 96)
        {
          /* RX error warning flag */

          data[1] |= CAN_ERR_CRTL_RX_WARNING;
          errbits |= CAN_ERR_CRTL;
        }
    }

  if (IfxCan_Node_isErrorPassive(can_node->node))
    {
      /* Error Passive */

      if (can_node->node->ECR.B.RP == 1)
        {
          /* RX passive flag */

          data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
        }

      if (can_node->node->ECR.B.TEC >= 127)
        {
          /* TX passive flag */

          data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
        }

      errbits |= CAN_ERR_CRTL;
    }

  /* Report a CAN error */

err_out:
  if (errbits != 0)
    {
      canerr("ERROR: errbits = 0x%04x\n", errbits);

      /* Format the CAN header for the error report */

      frame->can_id = errbits | CAN_ERR_FLAG;
      frame->can_dlc = CAN_ERR_DLC;
      frame->flags = 0;

      /* Send to socket interface */

      NETDEV_ERRORS(&priv->dev);
      return frame->can_dlc;
    }

  return 0;
}

#endif /* CONFIG_NET_CAN_ERRORS */

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

static int aurix_mcmcan_co_setup(struct aurix_mcmcan_priv_s *priv)
{
  IfxCan_Can_Node     *can_node = &priv->config->can_node;
  uint8               ref_index = priv->node_group->ref_index;
  volatile spinlock_t *lock     =
           &shared_data_manual.can_module[ref_index].can_module_lock;
  irqstate_t          irq_mask;

  /* add spin lock to protect shared_data_manual */

  irq_mask = spin_lock_irqsave_notrace(lock);

  /* add reference count */

  if (shared_data_manual.can_module[ref_index].can_module_ref == 0)
    {
      priv->node_group->en_flag = true;
      shared_data_manual.can_module[ref_index].can_module_ref++;
    }

  /* unlock spin lock to protect shared_data_manual */

  spin_unlock_irqrestore_notrace(lock, irq_mask);

  /* enable the CAN module */

  if (priv->node_group->en_flag == true &&
      IfxCan_isModuleEnabled(priv->node_group->can) != TRUE)
    {
      Ifx_CAN *can_module = priv->node_group->can;

      /* Enable module, disregard Sleep Mode request */

      IfxCan_enableModule(can_module);

      /* clear used mcanX RAM */

      /* IfxVmt_clearSram(priv->config->module_sram_index); */
    }

  /* setup common config */

  mcmcan_config_setup(priv->config);

  /* Initialises the CAN Node and sets CAN controller
   * mode to normal mode(also called START mode)
   */

  if (!IfxCan_Can_initNode(&priv->config->can_node,
                           &priv->config->can_node_config))
    {
      return -EFAULT;
    }

  aurix_mcmcan_set_loopback_mode(priv);

#if defined(CONFIG_AURIX_CAN_ERROR_POLLING)
  enable_destruct_readmode(&priv->config->can_node);
  work_queue(LPWORK, &priv->errwork, aurix_mcmcan_errpolling, priv, 0);
#endif

  /* setup node filter cre config */

  aurix_mcmcan_filter(priv);

#if defined(CONFIG_AURIX_MCMCAN_CRE)
  IfxCan_Can_initCre(&priv->config->can_node,
      &priv->config->can_node_cre_config);
  aurix_mcmcan_uni_routing(priv);
  aurix_mcmcan_mul_routing(priv);
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
  IfxCan_Node_setInterruptLine(can_node->node,
      IfxCan_Interrupt_busOffStatus,
      priv->config->err_interrupt_line);

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
  up_enable_irq(priv->config->err_irq);

  /* enable can node interrupt register corresponding bit */

  IfxCan_Node_enableInterrupt(can_node->node,
      IfxCan_Interrupt_transmissionCompleted);
  IfxCan_Node_enableInterrupt(can_node->node,
      IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
  IfxCan_Node_enableInterrupt(can_node->node,
      IfxCan_Interrupt_busOffStatus);

  priv->state       = CAN_STATE_OPERATIONAL;
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

static void aurix_mcmcan_co_shutdown(struct aurix_mcmcan_priv_s *priv)
{
  IfxCan_Can_Node     *can_node = &priv->config->can_node;
  uint8               ref_index = priv->node_group->ref_index;
  volatile spinlock_t *lock     =
           &shared_data_manual.can_module[ref_index].can_module_lock;
  irqstate_t          irq_mask;

  /* disable specific CAN interrupt */

  IfxCan_Node_disableInterrupt(can_node->node,
      IfxCan_Interrupt_busOffStatus);
  IfxCan_Node_disableInterrupt(can_node->node,
      IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
  IfxCan_Node_disableInterrupt(can_node->node,
      IfxCan_Interrupt_transmissionCompleted);

  /* disable specific interrupt line X */

  up_disable_irq(priv->config->tx_irq);
  up_disable_irq(priv->config->rx_irq);
  up_disable_irq(priv->config->err_irq);

  /* set CAN controller mode is stop mode */

  aurix_mcmcan_setmode(priv, CAN_STATE_STOPPED);

  /* add spin lock to protect shared_data_manual */

  irq_mask = spin_lock_irqsave_notrace(lock);

  shared_data_manual.can_module[ref_index].can_module_ref--;
  if (shared_data_manual.can_module[ref_index].can_module_ref == 0)
    {
        priv->node_group->en_flag = false;
    }

  /* unlock spin lock to protect shared_data_manual */

  spin_unlock_irqrestore_notrace(lock, irq_mask);

  if (priv->node_group->en_flag == false &&
      IfxCan_isModuleEnabled(priv->node_group->can) == TRUE)
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

static void aurix_mcmcan_co_rxint(struct aurix_mcmcan_priv_s *priv,
    bool enable)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;

  if (priv->state == CAN_STATE_SLEEP)
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

static void aurix_mcmcan_co_txint(struct aurix_mcmcan_priv_s *priv,
    bool enable)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;

  if (priv->state == CAN_STATE_SLEEP)
    {
        canerr("CAN controller is in sleep mode\n");
        return;
    }

    mcmcan_interrupt_setup(enable, can_node->node,
        IfxCan_Interrupt_transmissionCompleted);
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
 *   rxfifo_filter_index - An index is used to traverse the rx fifo filter
 *   array.
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

  acf_filter->type    = rxfifo_filter->type;
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
 *   rxbuf_filter_index - An index is used to traverse the rx buffer filter
 *   array.
 *
 ****************************************************************************/

static void aurix_mcmcan_rxbuf_filter(IfxCan_Filter *acf_filter,
                                      struct aurix_mcmcan_config_s *config,
    uint8_t rxbuf_filter_index)
{
  acf_filter->type                 = IfxCan_FilterType_classic;
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

static void aurix_mcmcan_filter(struct aurix_mcmcan_priv_s *priv)
{
    uint8_t rxfifo0_filter_cnt = priv->config->rxfifo0_filter_cnt;
    uint8_t rxfifo1_filter_cnt = priv->config->rxfifo1_filter_cnt;
  uint8_t rxbuf_filter_cnt   = priv->config->rxbuf_filter_cnt;
    uint8_t filter_cnt;
    uint8_t rxfifo_filter_cnt;

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
static void aurix_mcmcan_uni_routing(struct aurix_mcmcan_priv_s *priv)
{
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
static void aurix_mcmcan_mul_routing(struct aurix_mcmcan_priv_s *priv)
{
    uint8 mul_routing_num = priv->config->mul_routing_cnt;

  for (uint8 i = 0; i < mul_routing_num; i++)
    {
        IfxCan_Can_setStandardMulticastRouting(&priv->config->can_node,
            &priv->config->mul_routing[i]);
    }
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
#ifdef CONFIG_AURIX_MCMCAN_TXCONFIRM
      /* Notify upper half driver to read the tx confirm frame */

      netdev_lower_rxready((struct netdev_lowerhalf_s *)priv);

      netdev_lower_txdone((struct netdev_lowerhalf_s *)priv);
      IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                    IfxCan_Interrupt_transmissionCompleted);
#else
        aurix_mcmcan_tx_done(priv);
#endif
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

#ifdef CONFIG_AURIX_MCMCAN_CRE
  Ifx_CAN_N_CRE *cre = &(can_node->node->CRE);

  if (cre->HBUF.RX[IfxCan_CreRxHostBufferIndex_0].STAT.B.RHREQ == 1)
    {
      /* Notify rx ready to the upper half driver */

      priv->cre_rxfifo0_pending = true;
    }

  if (cre->HBUF.RX[IfxCan_CreRxHostBufferIndex_1].STAT.B.RHREQ == 1)
    {
      /* Notify rx ready to the upper half driver */

      priv->cre_rxfifo1_pending = true;
    }
#else
  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
      IfxCan_Interrupt_rxFifo0NewMessage))
    {
      priv->rxfifo0_pending =
            IfxCan_Node_getRxFifo0FillLevel(can_node->node);
    }

  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
      IfxCan_Interrupt_rxFifo1NewMessage))
    {
      priv->rxfifo1_pending =
            IfxCan_Node_getRxFifo1FillLevel(can_node->node);
    }
#endif

  if (IfxCan_Node_getInterruptFlagStatus(can_node->node,
      IfxCan_Interrupt_messageStoredToDedicatedRxBuffer))
    {
      /* Notify rx ready to the upper half driver */

      priv->rxmb0_sflags = can_node->node->NDAT1.U;
      priv->rxmb1_sflags = can_node->node->NDAT2.U;
    }

  netdev_lower_rxready((struct netdev_lowerhalf_s *)priv);

  return OK;
}

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
      /* InterruptFlag maybe accumulate but PSR(protocol state register)
       * have already been recovery.
       */

      if (IfxCan_Node_getBusOffStatus(can_node->node))
        {
#ifdef CONFIG_NET_CAN_ERRORS
            priv->err_pending = true;
          netdev_lower_rxready((struct netdev_lowerhalf_s *)priv);
#else
            aurix_mcmcan_busoff_recovery(priv);
#endif
        }

        IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
            IfxCan_Interrupt_busOffStatus);
    }

  return OK;
}

#ifdef CONFIG_AURIX_MCMCAN_CRE

/****************************************************************************
 * Name: aurix_mcmcan_read_cre_message
 *
 * Description:
 *   Read message from cre rx host buffer.
 *
 * Input Parameters:
 *   node - An instance of the hardware can node.
 *   buf  - Target buffer of rx message to save.
 *   buffer_type - cre host buffer0 or buffer1.
 *
 * Returned Value:
 *   Return message len with read from cre rx host buffer.
 *
 ****************************************************************************/

static uint8 aurix_mcmcan_read_cre_message(IfxCan_Can_Node *node,
                                           uint8 *buf, uint8 buffer_type)
{
  Ifx_CAN_N_CRE *cre = &(node->node->CRE);
  Ifx_CAN_RHBUF *rxBufPtr      = NULL_PTR;
  uint32         rxHBufAddress = 0;
  IfxCan_DataLengthCode dataLengthCode;
  Ifx_CAN_RHBUF_R0 rx_R0;
  Ifx_CAN_RHBUF_R1 rx_R1;

#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *frame = (struct canfd_frame *)buf;
#else
  struct can_frame *frame = (struct can_frame *)buf;
#endif

  switch (buffer_type)
    {
      case IfxCan_RxMode_fifo0:
        rxHBufAddress = (node->messageRAM.baseAddress
                         & IFXCAN_MODULE_ADDRESS_MASK)
                         + cre->CONFIGADR.U + (IFXCAN_CRE_TABLE_SIZE * 4);
        rxBufPtr = (Ifx_CAN_RHBUF *)(rxHBufAddress);
      break;

      case IfxCan_RxMode_fifo1:
        rxHBufAddress = (node->messageRAM.baseAddress
                         & IFXCAN_MODULE_ADDRESS_MASK)
                         + cre->CONFIGADR.U
                         + ((IFXCAN_CRE_TABLE_SIZE
                             + IFXCAN_CRE_RXFIFO_SPACING) * 4);
        rxBufPtr = (Ifx_CAN_RHBUF *)(rxHBufAddress);
      break;

      default:
      break;
  }

  rx_R0.U = rxBufPtr->R0.U;
  rx_R1.U = rxBufPtr->R1.U;

  frame->flags = 0;

  /* get message ID */

  if (rx_R0.B.XTD == IfxCan_MessageIdLength_extended)
    {
      frame->can_id = rx_R0.B.ID;
      frame->can_id |= CAN_EFF_FLAG;
    }
  else
    {
      frame->can_id = (rx_R0.B.ID >> 18);
    }

  frame->flags |= rx_R1.B.FDF ? CANFD_FDF : 0;
  frame->flags |= rx_R1.B.BRS ? CANFD_BRS : 0;

  /* get data length code */

  dataLengthCode = (IfxCan_DataLengthCode)rx_R1.B.DLC;
  frame->len = can_dlc2bytes(dataLengthCode);

  /* read word data */

  uint32 *data_ptr = (uint32 *)frame->data;
  uint32 *rx_data_ptr = (uint32 *)&rxBufPtr->RHBUF_DB[0];

  for (int i = 0;
       i < IfxCan_Can_xtdFrameLengthToNumOfWords[dataLengthCode]; i++)
    {
      data_ptr[i] = rx_data_ptr[i];
    }

  return frame->len;
}
#endif

/****************************************************************************
 * Name: aurix_mcmcan_read_message
 *
 * Description:
 *   Read message from normal fifo or dedicatedBuffers.
 *
 * Input Parameters:
 *   node - An instance of the hardware can node.
 *   buf  - Target buffer of rx message to save.
 *   buffer_type - Fifo0/Fifo1/DedicatedBuffers.
 *   buffer_id   - Dedicated buffer id.
 *
 * Returned Value:
 *   Return message len with read from cre rx host buffer.
 *
 ****************************************************************************/

static uint8 aurix_mcmcan_read_message(IfxCan_Can_Node *node, uint8 *buf,
    uint8 buffer_type, uint8 buffer_id)
{
  Ifx_CAN_RXMSG    *rxbuffer_element;
    IfxCan_RxBufferId read_buffer_id = IfxCan_RxBufferId_0;
    IfxCan_DataLengthCode dataLengthCode;
#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *frame = (struct canfd_frame *)buf;
#else
  struct can_frame *frame = (struct can_frame *)buf;
#endif

  switch (buffer_type)
    {
    case IfxCan_RxMode_fifo0:

        /* get the Tx FIFO 0 ELement address */

        read_buffer_id        = IfxCan_Node_getRxFifo0GetIndex(node->node);
        rxbuffer_element = IfxCan_Node_getRxFifo0ElementAddress(node->node,
            node->messageRAM.baseAddress,
            node->messageRAM.rxFifo0StartAddress,
            read_buffer_id);
        break;

    case IfxCan_RxMode_fifo1:

        /* get the Tx FIFO 1 ELement address */

        read_buffer_id        = IfxCan_Node_getRxFifo1GetIndex(node->node);
        rxbuffer_element = IfxCan_Node_getRxFifo1ElementAddress(node->node,
            node->messageRAM.baseAddress,
            node->messageRAM.rxFifo1StartAddress,
            read_buffer_id);
        break;

    case IfxCan_RxMode_dedicatedBuffers:

        /* get the Rx Bufer ELement address */

        read_buffer_id        = (IfxCan_RxBufferId)buffer_id;
        rxbuffer_element = IfxCan_Node_getRxBufferElementAddress(node->node,
            node->messageRAM.baseAddress,
            node->messageRAM.rxBuffersStartAddress,
            read_buffer_id);
        break;

    default:
        break;
    }

    frame->flags = 0;

  frame->flags |= rxbuffer_element->R1.B.FDF ? CANFD_FDF : 0;
  frame->flags |= rxbuffer_element->R1.B.BRS ? CANFD_BRS : 0;

    /* get message ID */

    frame->can_id = IfxCan_Node_getMesssageId(rxbuffer_element);

    /* get message ID length */

  if (rxbuffer_element->R0.B.XTD == IfxCan_MessageIdLength_extended)
    {
        frame->can_id |= CAN_EFF_FLAG;
    }

    /* get data length code */

    dataLengthCode = (IfxCan_DataLengthCode)IfxCan_Node_getDataLengthCode(
        rxbuffer_element);
    frame->len = can_dlc2bytes(dataLengthCode);

    /* read data */

    IfxCan_Node_readData(rxbuffer_element,
                       dataLengthCode, (uint32 *)frame->data);

    /* write acknowledgement index incase of FIFO */

  switch (buffer_type)
    {
    case IfxCan_RxMode_fifo0:
        IfxCan_Node_setRxFifo0AcknowledgeIndex(node->node, read_buffer_id);
        break;

    case IfxCan_RxMode_fifo1:
        IfxCan_Node_setRxFifo1AcknowledgeIndex(node->node, read_buffer_id);
        break;

    case IfxCan_RxMode_dedicatedBuffers:
    default:
        break;
    }

    /* clear new data flag for hw buffer */

    IfxCan_Node_clearRxBufferNewDataFlag(node->node, read_buffer_id);

    return frame->len;
}

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

  do
    {
      for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
        {
          if ((priv->txmb_sflags & (1 << txbuffer_id)) != 0)
            {
              if (!IfxCan_Node_isTxBufferRequestPending(can_node->node,
                        txbuffer_id) &&
                  IfxCan_Node_isTxBufferTransmissionOccured(
                                            can_node->node,
                                            txbuffer_id))
                {
                  continue; /* do not cancel if txmsg have sent to bus */
                }
              else
                {
                  /* cancel sending and pending tx buffers */

                  IfxCan_Node_setTxBufferCancellationRequest(can_node->node,
                        txbuffer_id);

                  /* if trasmission is started, cancellation will
                   * wait that this transmission is over,
                   * because of cancellation operation
                   * do not block long time.
                   */

                  while (!IfxCan_Node_isTxBufferCancellationFinished(
                                      can_node->node, txbuffer_id));

                  /* clear tx buffer */

                  netpkt_free((struct netdev_lowerhalf_s *)priv,
                        priv->tx_pkt_pending[txbuffer_id], NETPKT_TX);
                    priv->txmb_sflags &= (~(1 << txbuffer_id));
                  netdev_lower_txdone((struct netdev_lowerhalf_s *)priv);
                }
            }
        }
    }
  while (priv->txmb_sflags != 0);

  return OK;
}

/****************************************************************************
 * Name: mcmcan_fill_txmsg
 * Description:
 *   fill msg to message and find one available tx buffer.
 *
 * Input Parameters:
 *   priv - An instance of the "lower half" can driver structure.
 *   frame - the message from upper half driver.
 *   message - the message to be filled.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CAN_CANFD
static int mcmcan_fill_txmsg(struct aurix_mcmcan_priv_s *priv,
                             FAR struct canfd_frame * frame,
                             IfxCan_Message *message)
#else
static int mcmcan_fill_txmsg(struct aurix_mcmcan_priv_s *priv,
                             FAR struct can_frame * frame,
                             IfxCan_Message *message)
#endif
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

          uint32 txbuf_msgid;
          if (txbuf_element->T0.B.XTD != IfxCan_MessageIdLength_extended)
            {
              txbuf_msgid = txbuf_element->T0.B.ID;
            }
          else
            {
              txbuf_msgid = txbuf_element->T0.B.ID >> 18;
            }

          if (txbuf_msgid == (frame->can_id & CAN_EFF_MASK))
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

        message->bufferNumber = txbuffer_index;
      if (frame->can_id & CAN_EFF_FLAG)
        {
          /* Extended frame */

          message->messageId = frame->can_id & CAN_EFF_MASK;
          message->messageIdLength = IfxCan_MessageIdLength_extended;
        }
      else
        {
          /* Standard frame */

          message->messageId = frame->can_id & CAN_SFF_MASK;
          message->messageIdLength = IfxCan_MessageIdLength_standard;
        }

#ifdef CONFIG_NET_CAN_CANFD
        message->dataLengthCode = can_bytes2dlc(frame->len);
#else
        message->dataLengthCodE = frame->can_dlc;
#endif

      /* Confirm message mode */

#ifdef CONFIG_NET_CAN_CANFD
      if (((frame->flags & CANFD_FDF) != 0)
          && ((frame->flags & CANFD_BRS) != 0))
        {
          message->frameMode = IfxCan_FrameMode_fdLongAndFast;
        }
      else if ((frame->flags & CANFD_FDF) != 0)
        {
          message->frameMode = IfxCan_FrameMode_fdLong;
        }
      else
#endif
        {
          message->frameMode = IfxCan_FrameMode_standard;
        }
    }

  caninfo("avail_flag = %d, if avail_flag is 1, it found available "
          "tx buffer; else no available tx buffer\n", avail_flag);
  return avail_flag ? OK : -EBUSY;
}

#ifdef CONFIG_AURIX_MCMCAN_TXCONFIRM

/****************************************************************************
 * Name: aurix_mcmcan_tx_confirm
 * Description:
 *  acquire the msg id that had been sent when tx interrupt is triggered.
 *
 * Input Parameters:
 *   dev - the struct can_dev_s object.
 *   buf - tx confirm frame buffer.
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static uint8 aurix_mcmcan_tx_confirm(struct aurix_mcmcan_priv_s *priv,
                                     uint8 *buf)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;
  uint8 tx_buffs = priv->config->can_node_config.
                      txConfig.dedicatedTxBuffersNumber;
  uint8 txbuffer_id;
  Ifx_CAN_TXMSG *txBufferElement;
#ifdef CONFIG_NET_CAN_CANFD
  struct canfd_frame *frame = (struct canfd_frame *)buf;
#else
  struct can_frame *frame = (struct can_frame *)buf;
#endif

  /* Tranverse all txbuffer in this node for checking txmb_sflags */

  for (txbuffer_id = 0; txbuffer_id < tx_buffs; txbuffer_id++)
    {
      /* TX.BRP corresponding bit reset and TX.BTO corresponding
       * bit reset when transmition completed
       */

      if ((priv->txmb_sflags & (1 << txbuffer_id)) != 0
            && !IfxCan_Node_isTxBufferRequestPending(can_node->node,
                txbuffer_id)
            && IfxCan_Node_isTxBufferTransmissionOccured(can_node->node,
                                                        txbuffer_id))
        {
          frame->flags = 0;

          /* fill the "tcf" member variable of tx confirmation msg */

          frame->flags |= CAN_TCF;
          frame->len    = 0;

          /* get the Tx message id from the specific Tx Bufer and
           * transmit this msg to software buffers
           */

            txBufferElement = IfxCan_Node_getTxBufferElementAddress(
                can_node->node,
                can_node->messageRAM.baseAddress,
                can_node->messageRAM.txBuffersStartAddress,
                txbuffer_id);
          if (txBufferElement->T0.B.XTD == IfxCan_MessageIdLength_extended)
            {
              frame->can_id = txBufferElement->T0.B.ID;
              frame->can_id |= CAN_EFF_FLAG;
            }
          else
            {
              frame->can_id = txBufferElement->T0.B.ID >> 18;
            }

          /* clear the tx buffer corresponding bit */

          priv->txmb_sflags &= ~(1 << txbuffer_id);
          netpkt_free((struct netdev_lowerhalf_s *)priv,
                priv->tx_pkt_pending[txbuffer_id], NETPKT_TX);
          return 1;
        }
    }

  return 0;
}

#else /* CONFIG_AURIX_MCMCAN_TXCONFIRM */

/****************************************************************************
 * Name: aurix_mcmcan_tx_done
 * Description:
 *  acquire the msg id that had been sent when tx interrupt is triggered.
 *
 * Input Parameters:
 *   dev - the struct can_dev_s object.
 *   buf - tx confirm frame buffer.
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static void aurix_mcmcan_tx_done(struct aurix_mcmcan_priv_s *priv)
{
  IfxCan_Can_Node *can_node = &priv->config->can_node;
  uint8 tx_buffs = priv->config->
      can_node_config.txConfig.dedicatedTxBuffersNumber;
  uint8 txbuffer_id;

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
          priv->txmb_sflags &= ~(1 << txbuffer_id);
          netpkt_free((struct netdev_lowerhalf_s *)priv,
                priv->tx_pkt_pending[txbuffer_id], NETPKT_TX);
        }
    }

  netdev_lower_txdone((struct netdev_lowerhalf_s *)priv);
  IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
        IfxCan_Interrupt_transmissionCompleted);
}

#endif /* CONFIG_AURIX_MCMCAN_TXCONFIRM */

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
  config->fastBaudRate.syncJumpWidth         = 3;
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
   * before using dedicated Rx Buffer.
   *
   * setup messageIdLength to both, so that the configs of
   * rejectRemoteFramesWithExtendedId and extendedFilterForNonMatchingFrames
   * can be enabled.
   */

  config->filterConfig.messageIdLength                    =
                                          IfxCan_MessageIdLength_both;
  config->filterConfig.rejectRemoteFramesWithStandardId   = TRUE;
  config->filterConfig.rejectRemoteFramesWithExtendedId   = TRUE;
  config->filterConfig.standardFilterForNonMatchingFrames =
                                          IfxCan_NonMatchingFrame_reject;
  config->filterConfig.extendedFilterForNonMatchingFrames =
                                          IfxCan_NonMatchingFrame_reject;

  config->interruptConfig.busOffStatusEnabled                     = TRUE;
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
      /* disable this module */

      IfxCan_disableModule(can);
    }
}

/****************************************************************************
 * Name: net_mcmcan_ifup
 * Description:
 *  set up the mcmcan module.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static int net_mcmcan_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)dev;

  if (priv->state == CAN_STATE_OPERATIONAL)
    {
      return OK;
    }

  aurix_mcmcan_co_setup(priv);
  aurix_mcmcan_co_rxint(priv, true);
  aurix_mcmcan_co_txint(priv, true);

  return OK;
}

/****************************************************************************
 * Name: net_mcmcan_ifdown
 * Description:
 *  shut down the specfic mcmcan module.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *
 ****************************************************************************/

static int net_mcmcan_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)dev;

  aurix_mcmcan_co_shutdown(priv);
  return OK;
}

/****************************************************************************
 * Function: net_mcmcan_transmit
 * Description:
 *   Start hardware transmission.  Called by netdev upperhalf driver
 *
 * Input Parameters:
 *   dev  - Reference to the netdev lowerhalf structure
 *   pkt  - the packet to be transmitted
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int net_mcmcan_transmit(struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt)
{
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)dev;
    IfxCan_Message message;
#ifdef CONFIG_NET_CAN_CANFD
  FAR struct canfd_frame *frame =
      (FAR struct canfd_frame *)netpkt_getdata(dev, pkt);
#else
  FAR struct can_frame *frame =
      (FAR struct can_frame *)netpkt_getdata(dev, pkt);
#endif

  if (priv->state == CAN_STATE_SLEEP)
    {
      return -EIO;
    }

  /* fill tx message and find one available buffer */

  if (mcmcan_fill_txmsg(priv, frame, &message) == -EBUSY)
    {
      canerr("No available tx buffer\n");
      return -EBUSY;  /* No available TxBuffer */
    }

  /* send message to bus */

  caninfo("message.bufferNumber is %d\n", message.bufferNumber);
  if (IfxCan_Status_notSentBusy ==
          IfxCan_Can_sendMessage(&priv->config->can_node, &message,
                                  (uint32 *)&frame->data))
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
  priv->tx_pkt_pending[message.bufferNumber] = pkt;
  return OK;
}

/****************************************************************************
 * Name:  net_mcmcan_receive
 * Description:
 *   Receive a CAN frame from the CAN driver and pass it to the network
 *
 * Input Parameters:
 *   dev - An instance of the "lower half" can driver structure.
 *
 * Returned Value:
 *   A pointer to the received packet on success; NULL on failure.
 *
 ****************************************************************************/

static netpkt_t *net_mcmcan_receive(struct netdev_lowerhalf_s *dev)
{
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)dev;
  netpkt_t *pkt = NULL;
  uint8 buffer_number;
  uint8 pkt_len = 0;
  uint8 msg_len;

  /* Allocate a network packet for RX message. */

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      nwarn("Allocate RX MB buffer failed\n");
      return NULL;
    }

  /* Set the data length of the packet. */

#ifdef CONFIG_NET_CAN_CANFD
  msg_len = sizeof(struct canfd_frame);
  pkt_len = netpkt_setdatalen(&priv->dev, pkt, msg_len);
  if (pkt_len < msg_len)
#else
  msg_len = sizeof(struct can_frame);
  pkt_len = netpkt_setdatalen(&priv->dev, pkt, msg_len);
  if (pkt_len < msg_len)
#endif
    {
      nwarn("RX iob buffer out of memory\n");
      netpkt_free(dev, pkt, NETPKT_RX);
      return NULL;
    }

  msg_len = 0;

  /* Check if err message pending to be read */

#ifdef CONFIG_NET_CAN_ERRORS
  if (priv->err_pending != 0)
    {
      msg_len = aurix_mcmcan_errhandle(priv, netpkt_getdata(dev, pkt));
      priv->err_pending = false;
      goto receive_out;
    }
#endif

  /* Check if there is a new message in dedicated Rx Buffer. */

  if (priv->config->can_node_config.
        interruptConfig.messageStoredToDedicatedRxBufferEnabled)
    {
      /* If there are new messages in dedicated Rx Buffer 0 or 1,
       * clear the interrupt flag after all messages are read.
       */

      if (priv->rxmb0_sflags != 0)
        {
          buffer_number = ffs(priv->rxmb0_sflags) - 1;
          msg_len = aurix_mcmcan_read_message(&priv->config->can_node,
                                              netpkt_getdata(dev, pkt),
                                              IfxCan_RxMode_dedicatedBuffers,
                                              buffer_number);
          priv->rxmb0_sflags &= ~(1 << buffer_number);
          if (priv->rxmb0_sflags == 0 && priv->rxmb1_sflags == 0)
            {
              /* If no more message in Rx Buffer, clear the interrupt flag */

              IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                  IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
            }

          goto receive_out;
        }

      if (priv->rxmb1_sflags != 0)
        {
          buffer_number = ffs(priv->rxmb1_sflags) - 1 + 32;
          msg_len = aurix_mcmcan_read_message(&priv->config->can_node,
              netpkt_getdata(dev, pkt),
              IfxCan_RxMode_dedicatedBuffers,
              buffer_number);
          priv->rxmb1_sflags &= ~(1 << (buffer_number - 32));
          if (priv->rxmb0_sflags == 0 && priv->rxmb1_sflags == 0)
            {
              /* If no more message in Rx Buffer, clear the interrupt flag */

              IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                  IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);
            }

          goto receive_out;
        }
    }

#ifdef CONFIG_AURIX_MCMCAN_CRE

  /* Check if there is a new message in cre Rx host buffer 0 or 1 */

  if (priv->config->can_node_cre_config.interrupt.rxBuffer0InterruptEnable)
    {
      if (priv->cre_rxfifo0_pending != 0)
        {
          IfxCan_Node_clearCreInterrupt(priv->config->can_node.node,
                                          IfxCan_CreInterrupt_RxBuffer0);

          msg_len = aurix_mcmcan_read_cre_message(&priv->config->can_node,
                                          netpkt_getdata(dev, pkt),
                                          IfxCan_RxMode_fifo0);

          priv->cre_rxfifo0_pending  = false;
          goto receive_out;
        }
    }

  if (priv->config->can_node_cre_config.interrupt.rxBuffer1InterruptEnable)
    {
      if (priv->cre_rxfifo1_pending != 0)
        {
          IfxCan_Node_clearCreInterrupt(priv->config->can_node.node,
                                      IfxCan_CreInterrupt_RxBuffer1);

          msg_len = aurix_mcmcan_read_cre_message(&priv->config->can_node,
                                        netpkt_getdata(dev, pkt),
                                        IfxCan_RxMode_fifo1);

          priv->cre_rxfifo1_pending  = false;
          goto receive_out;
        }
    }
#else

  /* Check if there is a new message in Rx FIFO 0 or 1 */

  if (priv->config->can_node_config.interruptConfig.rxFifo0NewMessageEnabled)
    {
      if (priv->rxfifo0_pending > 0)
        {
          msg_len = aurix_mcmcan_read_message(&priv->config->can_node,
              netpkt_getdata(dev, pkt),
              IfxCan_RxMode_fifo0,
              0);
          priv->rxfifo0_pending--;
          if (priv->rxfifo0_pending == 0)
            {
              /* If no more message in Rx FIFO 0, clear the interrupt flag */

              IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                  IfxCan_Interrupt_rxFifo0NewMessage);
            }

          goto receive_out;
        }
    }

  if (priv->config->can_node_config.interruptConfig.rxFifo1NewMessageEnabled)
    {
      if (priv->rxfifo1_pending > 0)
        {
          msg_len = aurix_mcmcan_read_message(&priv->config->can_node,
              netpkt_getdata(dev, pkt),
              IfxCan_RxMode_fifo1,
              0);
          priv->rxfifo1_pending--;
          if (priv->rxfifo1_pending == 0)
            {
              /* If no more message in Rx FIFO 1, clear the interrupt flag */

              IfxCan_Node_clearInterruptFlag(priv->config->can_node.node,
                  IfxCan_Interrupt_rxFifo1NewMessage);
            }

          goto receive_out;
        }
    }
#endif

  /* Check if tx confirm message pending to be read */

#ifdef CONFIG_AURIX_MCMCAN_TXCONFIRM
  if (priv->txmb_sflags != 0)
    {
      /* If there is a tx confirm message, read the message and clear the
       * interrupt flag.
       */

      msg_len = aurix_mcmcan_tx_confirm(priv, netpkt_getdata(dev, pkt));
      goto receive_out;
    }
#endif

  /* If no valid message is received, free the packet and return NULL.
   * Otherwise, return to pointer of the received packet.
   */

receive_out:
  if (msg_len <= 0)
    {
      netpkt_free(dev, pkt, NETPKT_RX);
      return NULL;
    }

  return pkt;
}

/****************************************************************************
 * Name: net_mcmcan_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int net_mcmcan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
    unsigned long arg)
{
    int ret = -ENOTTY;
  struct aurix_mcmcan_priv_s *priv = (struct aurix_mcmcan_priv_s *)dev;
  FAR struct can_ioctl_state_s *state = (struct can_ioctl_state_s *)arg;

  switch (cmd)
    {
    case SIOCCANRECOVERY:
        ret = aurix_mcmcan_busoff_recovery(priv);
        break;
    case SIOCSCANSTATE:
        ret = aurix_mcmcan_setmode(priv, state->state);
        break;
    case SIOCGCANSTATE:
        ret = aurix_mcmcan_getmode(priv, &state->state);
        break;
    case SIOCCANOFLUSH:

        /* "aurix_mcmcan_busoff_recovery" API has already process hard tx
         * buffers, now "mcmcan_flush_txbuff" offers clearing all
         * pending tx buffers alone.
         */

        ret = mcmcan_flush_txbuff(priv);
        break;
    case SIOCGCANTRSVSTATE:
        ret = mcmcan_transv_getmode(priv->config, arg);
        break;
    case SIOCSCANTRSVSTATE:
        ret = mcmcan_transv_setmode(priv->config, arg);
        break;
    default:
        canerr("Unrecognized ioctl command: %d\n", cmd);
        break;
    }

    return ret;
}

#endif

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

  return (port->PDISC.U >> pinIndex) & 0x01;
}

/****************************************************************************
 * Name: aurix_mcmcan_get_transv_pin_level
 *
 * Description:
 *   Compare the can transceiver pin level with the high.
 *
 * Returned Value:
 *   true: the pin level is high.
 *   false: the pin level is low.
 *
 ****************************************************************************/

boolean aurix_mcmcan_get_transv_pin_level(Ifx_P *port, uint8 pinIndex)
{
  return  (port->OUT.U >> pinIndex) & 1U;
}

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

int aurix_socket_init_mcmcan(struct net_driver_s **dev,
                             struct aurix_mcmcan_config_s *config,
    size_t num)
{
  struct aurix_mcmcan_priv_s *priv;
  uint8                       i;
  int                         ret;

  /* Ensure correct clock frequencies for Mcan,
   * set the clock frequency of Mcan to CONFIG_AURIX_MCMCAN_CLOCK Hz
   * and Mcanh to CONFIG_AURIX_MCMCAN_ACCESS_RAM_CLOCK Hz when
   * the clock frequency is not correct.
   */

  if (IfxScuCcu_getMcanFrequency() != CONFIG_AURIX_MCMCAN_CLOCK)
    {
      IfxScuCcu_setMcanFrequency(CONFIG_AURIX_MCMCAN_CLOCK);
    }

  if (IfxScuCcu_getMcanhFrequency() != CONFIG_AURIX_MCMCAN_ACCESS_RAM_CLOCK)
    {
      IfxScuCcu_setMcanhFrequency(CONFIG_AURIX_MCMCAN_ACCESS_RAM_CLOCK);
    }

  for (i = 0; i < num; i++)
    {
      if (config[i].intf / 4 >= CAN_MODULE_NUM)
        {
          nerr("Invalid CAN interface index: %d\n", config[i].intf);
          return -EINVAL;
        }

        priv = kmm_zalloc(sizeof(struct aurix_mcmcan_priv_s));
      if (priv == NULL)
        {
          nerr("aurix can kmm_zalloc failed\n");
          return -ENOMEM;
        }

      /* Fill ops and lower driver data struct. */

      priv->tx_pkt_pending = kmm_zalloc(sizeof(netpkt_t *) *
                                        config[i].can_node_config.txConfig.
                                        dedicatedTxBuffersNumber);
      if (priv->tx_pkt_pending == NULL)
        {
          kmm_free(priv);
          nerr("aurix can kmm_zalloc tx_pkt_pending failed\n");
          return -ENOMEM;
        }

      priv->config = &config[i];
      priv->node_group = &g_node_group[config[i].intf / 4];
      priv->node_group->ref_index = config[i].intf / 4;
      priv->dev.ops = &g_aurix_mcmcan_ops;
#ifdef CONFIG_AURIX_MCMCAN_ISR_WQUEUE
      priv->dev.rxtype = NETDEV_RX_DIRECT;
#else
      priv->dev.rxtype = NETDEV_RX_THREAD;
      priv->dev.priority = CONFIG_AURIX_MCMCAN_RXTHREAD_PRIORITY;
#endif
      priv->dev.quota[NETPKT_TX] =
          config[i].can_node_config.txConfig.dedicatedTxBuffersNumber;
      priv->dev.quota[NETPKT_RX] = 1;

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

      snprintf(priv->dev.netdev.d_ifname ,
            IFNAMSIZ, "can%d", priv->config->intf);
      ret = netdev_lower_register(&priv->dev, NET_LL_CAN);
      if (ret < 0)
        {
          kmm_free(priv);
          canerr("ERROR: Register CAN interface failed: %d\n", ret);
          return -ret;
        }

      up_disable_irq(priv->config->tx_irq);
      up_disable_irq(priv->config->rx_irq);
      up_disable_irq(priv->config->err_irq);

      /* attach irq */

#ifdef CONFIG_AURIX_MCMCAN_ISR_WQUEUE
      ret = irq_attach_wqueue(priv->config->tx_irq, NULL,
          aurix_mcmcan_tx_interrupt, priv,
          CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
      ret = irq_attach_wqueue(priv->config->rx_irq, NULL,
          aurix_mcmcan_rx_interrupt, priv,
          CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
      ret = irq_attach_wqueue(priv->config->err_irq, NULL,
          aurix_mcmcan_err_interrupt, priv,
          CONFIG_AURIX_MCMCAN_ISR_WQUEUE_PRIORITY);
#else
      irq_attach(priv->config->tx_irq, aurix_mcmcan_tx_interrupt, priv);
      irq_attach(priv->config->rx_irq, aurix_mcmcan_rx_interrupt, priv);
      irq_attach(priv->config->err_irq, aurix_mcmcan_err_interrupt, priv);
#endif

      priv->state = CAN_STATE_STOPPED;

      /* using port as index to get net_driver_s */

      dev[config[i].intf] = &priv->dev.netdev;
    }

  return OK;
}
