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
#include "aurix_enet_tc3.h"
#include <nuttx/arch.h>
#include <arch/barriers.h>
#include <nuttx/irq.h>

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Marvell Phy device address */

#define ETH_PHY_ADDRESS                  CONFIG_AURIX_ENET1_PHY_ADDR
#define AURIX_ENET_MAC_ADDR              CONFIG_AURIX_ENET1_MAC_ADDR

/* Ethernet Parameters */

#define GETH_DMA_TX_BUFFER_SIZE          (512)
#define GETH_DMA_RX_BUFFER_SIZE          (512)

/* Define specific channel */

#define UNTAGGED_CHANNEL                 1

/* Define the number of channels */

#ifdef CONFIG_AURIX_ENET_QUEUE_NUM
#define IFXGETH_NUM_QUEUES_CHANNELS CONFIG_AURIX_ENET_QUEUE_NUM
#else
#define IFXGETH_NUM_QUEUES_CHANNELS 1
#endif

#define MAX_TIMEOUT  (15000)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ENET configuration dynamic structure */

struct aurix_enet_module_config_s
{
  IfxGeth_Eth                         geth;
  Ifx_P                              *port_p;
  IfxGeth_Eth_Config                  gethConfig;
  IfxGeth_Eth_FrameConfig             frameConfig;
};

/* ENET configuration structure */

struct aurix_enet_dev_s
{
  struct netdev_lowerhalf_s           dev;
  struct aurix_enet_module_config_s  *config_d;
  bool                                ifup;
  const struct aurix_enet_config_s   *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_enet_ifup(struct netdev_lowerhalf_s *dev);
static int aurix_enet_ifdown(struct netdev_lowerhalf_s *dev);
static int aurix_enet_transmit(struct netdev_lowerhalf_s *dev,
                               netpkt_t *pkt);
static netpkt_t *aurix_enet_receive(struct netdev_lowerhalf_s *dev);
static int aurix_enet_rxhandler(int irq, void *context, void *arg);
static int aurix_enet_txhandler(int irq, void *context, void *arg);
#ifdef CONFIG_NETDEV_IOCTL
static int aurix_enet_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                            unsigned long arg);
#endif

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
static int aurix_mvlethphy_init(struct aurix_enet_dev_s *priv);
#ifdef CONFIG_AURIX_ENET_USE_PHY
static int aurix_ethlink_up(void);
#endif
#endif
static void aurix_geth_init(struct aurix_enet_dev_s *priv);
static void aurix_mac_rece_trans_en(struct aurix_enet_dev_s *priv);
static int aurix_get_rxframe_size(struct netdev_lowerhalf_s *dev,
                                  uint8_t rx_channel);
static void aurix_enet_setmacaddress(uint8_t *mac);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ethernet transmit buffer */

static aligned_data(32) uint8 g_gethDmaTxBuffer_u8
                              [IFXGETH_NUM_QUEUES_CHANNELS]
                              [IFXGETH_MAX_TX_DESCRIPTORS]
                              [GETH_DMA_TX_BUFFER_SIZE];

/* Ethernet receive buffer */

static aligned_data(32) uint8 g_gethDmaRxBuffer_u8
                              [IFXGETH_NUM_QUEUES_CHANNELS]
                              [IFXGETH_MAX_RX_DESCRIPTORS]
                              [GETH_DMA_RX_BUFFER_SIZE];

/* Sync barrier for ethernet init */

static const struct netdev_ops_s g_aurix_enet_ops =
{
  .ifup = aurix_enet_ifup,            /* ifup */
  .ifdown = aurix_enet_ifdown,        /* ifdown */
  .transmit = aurix_enet_transmit,    /* transmit */
  .receive = aurix_enet_receive,      /* receive */
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl = aurix_enet_ioctl,          /* ioctl */
#endif
};

#ifdef CONFIG_AURIX_ENET

/* Constant configuration */

static struct aurix_enet_module_config_s g_enet_config_d[] =
{
  {
    .geth.gethSFR         = &MODULE_GETH,
    .port_p               = &MODULE_P11,
  },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 #ifdef CONFIG_NETDEV_IOCTL

/****************************************************************************
 * Function: aurix_enet_ioctl
 *
 * Description:
 *   NuttX Callback: ENET ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_enet_ioctl(struct netdev_lowerhalf_s *dev, int cmd,
                            unsigned long arg)
{
  int ret = -ENOTTY;
  return ret;
}
#endif

/****************************************************************************
 * Function: aurix_enet_transmit
 *
 * Description:
 *   NuttX Callback: Start hardware transmission
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   pkt  - the packet to be transmitted
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 ****************************************************************************/

static int aurix_enet_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct aurix_enet_dev_s *priv   = (struct aurix_enet_dev_s *)dev;
  struct eth_8021qhdr_s *vlan_hdr = NULL;
  uint8 *transmit_buffer          = NULL;
  uint8_t channel                 = 0;
  uint32_t pktlen;
  uint32_t index;
  uint32_t upper_len;
  volatile IfxGeth_TxDescr *base_descr;
  volatile IfxGeth_TxDescr *actual_descr;
  void *base_addr;

  if (priv->config_d->geth.gethSFR->CLC.B.DISR)
    {
      return -ENETDOWN;
    }

  vlan_hdr = (FAR struct eth_8021qhdr_s *)netpkt_getdata(dev, pkt);
  if (vlan_hdr->tpid == HTONS(TPID_8021QVLAN))
    {
      /* VLAN packet */

      channel =
        (ntohs(vlan_hdr->tci) >> VLAN_PRIO_SHIFT) % IFXGETH_NUM_TX_CHANNELS;
    }
  else
    {
      channel = UNTAGGED_CHANNEL;
    }

  if (priv->config->enable_queues[channel] == false)
    {
      nerr("ERROR: Transmit channel %d is disabled\n", channel);
      return -ECHRNG;
    }

  /* configure transmit DMA channel */

  base_descr =
      IfxGeth_Eth_getBaseTxDescriptor(&priv->config_d->geth, channel);
  actual_descr =
      IfxGeth_Eth_getActualTxDescriptor(&priv->config_d->geth, channel);
  index = actual_descr - base_descr;
  upper_len = (IFXGETH_MAX_TX_DESCRIPTORS - index) * GETH_DMA_TX_BUFFER_SIZE;
  transmit_buffer = (uint8_t *)IfxGeth_Eth_getTransmitBuffer(
                                        &priv->config_d->geth, channel);
  priv->config_d->frameConfig.channelId = channel;
  if (transmit_buffer != NULL)
    {
      pktlen = netpkt_getdatalen(&priv->dev, pkt);
      priv->config_d->frameConfig.packetLength = pktlen;
      if (pktlen > upper_len)
        {
          base_addr =
          ((void *)(uint32)(
                    priv->config_d->geth.txChannel[channel].buffer1Address));
          netpkt_copyout(&priv->dev, transmit_buffer, pkt, upper_len, 0);
          netpkt_copyout(&priv->dev, base_addr, pkt,
                         pktlen - upper_len, upper_len);
        }
      else
        {
          netpkt_copyout(&priv->dev, transmit_buffer, pkt, pktlen, 0);
        }

      IfxGeth_Eth_sendFrame(&priv->config_d->geth, &priv->config_d->frameConfig);
      netpkt_free(&priv->dev, pkt, NETPKT_TX);
      return OK;
    }

    nerr("ERROR: Transmit buffer not available."
         "Driver sends packet: Channel=%d, Packet Count=%ld\n",
         channel, priv->config_d->geth.txChannel[channel].txCount);
  return -EBUSY;
}

/****************************************************************************
 * Function: aurix_get_rxframe_size
 *
 * Description:
 *   Get RX frame size
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   pkt  - the packet to be transmitted
 *
 * Returned Value:
 *   pkt_size.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_get_rxframe_size(struct netdev_lowerhalf_s *dev,
                                  uint8_t rx_channel)
{
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;
  volatile IfxGeth_RxDescr3 *rdes3;

  rdes3 = &priv->config_d->geth.rxChannel[rx_channel].rxDescrPtr->RDES3;
  if (rdes3->W.OWN == 1u || rdes3->W.ES == 1u ||
      rdes3->W.FD == 0u || rdes3->W.LD == 0u)
    {
      /* Error, this block is invalid */

      return -1;
    }
  else
    {
      /* Subtract CRC */

      return (rdes3->W.PL - 4u);
    }
}

/****************************************************************************
 * Function: aurix_enet_receive
 *
 * Description:
 *   NuttX Callback: Receive the ENET message
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   netpkt_t - Reference to the received packet
 *
 * Assumptions:
 *
 ****************************************************************************/

static netpkt_t *aurix_enet_receive(struct netdev_lowerhalf_s *dev)
{
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;
  netpkt_t *pkt;
  int pkt_size;
  void *data;

  if (priv->config_d->geth.gethSFR->CLC.B.DISR)
    {
      return NULL;
    }

  for (int channel = 0; channel < IFXGETH_NUM_RX_CHANNELS; channel++)
    {
      if (priv->config->enable_queues[channel])
        {
          data = (void *)IfxGeth_Eth_getReceiveBuffer(&priv->config_d->geth,
                                                      channel);
          if (data != NULL)
            {
              pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
              if (pkt)
                {
                  pkt_size = aurix_get_rxframe_size(&priv->dev, channel);
                  if (pkt_size >= 0)
                    {
                      netpkt_setdatalen(&priv->dev, pkt, pkt_size);
                      netpkt_copyin(&priv->dev, pkt, data, pkt_size, 0);
                    }

                  IfxGeth_Eth_freeReceiveBuffer(
                                        &priv->config_d->geth, channel);
                  return pkt;
                }
            }

          nerr("ERROR: Aurix recieve packet error, receive buffer is NULL!"
               "channel:%d, rxcount:%ld\n",
               channel, priv->config_d->geth.rxChannel[channel].rxCount);
        }
    }

  return NULL;
}

/****************************************************************************
 * Function: aurix_geth_init
 *
 * Description:
 *   Initialize the geth module
 *
 * Input Parameters:
 *   priv - Reference to the private data of aurix enet driver
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void aurix_geth_init(struct aurix_enet_dev_s *priv)
{
  IfxGeth_Eth_Config *gethConfig = &priv->config_d->gethConfig;
  priv->config_d->geth.gethSFR = &MODULE_GETH;
  IfxGeth_Eth_RxChannelConfig *rxchannel;
  IfxGeth_Eth_DmaInterruptConfig *rx_dma_interrupt;
  IfxGeth_Eth_RxQueueConfig *rx_mtl_queue;
  IfxGeth_Eth_TxChannelConfig *txchannel;
  IfxGeth_Eth_DmaInterruptConfig *tx_dma_interrupt;
  IfxGeth_Eth_TxQueueConfig *tx_mtl_queue;
  int provider = IfxCpu_getCoreId();
  int tx_nchannel = 0;
  int rx_nchannel = 0;

  /* get the default configuration for the GETH module */

  IfxGeth_Eth_initModuleConfig(gethConfig, &MODULE_GETH);

  /* configure rmii pins */

  if (gethConfig->phyInterfaceMode == IfxGeth_PhyInterfaceMode_rmii)
    {
      gethConfig->pins.rmiiPins = &(priv->config->rmiiPins);
    }

  /* configure the number of dma channel and mtl queues */

  gethConfig->dma.numOfTxChannels = IFXGETH_NUM_TX_CHANNELS;
  gethConfig->dma.numOfRxChannels = IFXGETH_NUM_RX_CHANNELS;
  gethConfig->mtl.numOfTxQueues = IFXGETH_NUM_TX_CHANNELS;
  gethConfig->mtl.numOfRxQueues = IFXGETH_NUM_RX_CHANNELS;

  /* configure the schedule algorithm. */

  gethConfig->mtl.txSchedulingAlgorithm = IfxGeth_TxSchedulingAlgorithm_sp;
  gethConfig->mtl.rxArbitrationAlgorithm = IfxGeth_RxArbitrationAlgorithm_wsp;

  /* configure the "own" MAC address into slot0 */

  aurix_enet_setmacaddress(gethConfig->mac.macAddress);

  /* configure Ethernet DMA */

  for (int i = 0; i < IFXGETH_NUM_RX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          /* set up Rx channel */

          rxchannel = &gethConfig->dma.rxChannel[i];

          /* set receive buffer size in the Rx.Dscr */

          rxchannel->rxBuffer1Size = GETH_DMA_RX_BUFFER_SIZE;

          /* assign the receive buffer */

          rxchannel->rxBuffer1StartAddress =
                    (uint32 *)g_gethDmaRxBuffer_u8[rx_nchannel];
          rx_nchannel++;

          /* set up the DMA Rx interrupt */

          rx_dma_interrupt = &gethConfig->dma.rxInterrupt[i];

          /* set up the DMA Rx priority */

          rx_dma_interrupt->priority = ISR_PRIORITY_GETH_DMA0_RX;

          /* set up the DMA Rx interrupt receiver, CPU0 */

          rx_dma_interrupt->provider = provider;

          /* enable Rx Queue */

          rx_mtl_queue = &gethConfig->mtl.rxQueue[i];

          /* enable Rx queue store and forward function */

          rx_mtl_queue->storeAndForward = true;

          rx_mtl_queue->forwardUndersizedGoodPacket = true;

          /* set up the Rx queue size */

          rx_mtl_queue->rxQueueSize = IfxGeth_QueueSize_2048Bytes;

          /* set up the Rx queue to DMA channel mapping */

          rx_mtl_queue->rxDmaChannelMap = i;
        }
    }

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          /* set up Tx channel */

          txchannel = &gethConfig->dma.txChannel[i];

          /* set transmit buffer size in the Tx.Dscr */

          txchannel->txBuffer1Size = GETH_DMA_TX_BUFFER_SIZE;

          /* assign the transmit buffer */

          txchannel->txBuffer1StartAddress =
                      (uint32 *)g_gethDmaTxBuffer_u8[tx_nchannel];
          tx_nchannel++;

          /* set up the DMA Tx interrupt */

          tx_dma_interrupt = &gethConfig->dma.txInterrupt[i];

          /* set up the DMA Tx priority */

          tx_dma_interrupt->priority = ISR_PRIORITY_GETH_DMA0_TX;

          /* set up the DMA Tx interrupt receiver, CPU0 */

          tx_dma_interrupt->provider = provider;

          /* enable Tx Queue */

          tx_mtl_queue = &gethConfig->mtl.txQueue[i];

          /* enable Tx queue store and forward function */

          tx_mtl_queue->storeAndForward = true;

          /* set up Tx queue size */

          tx_mtl_queue->txQueueSize = IfxGeth_QueueSize_2048Bytes;
        }
    }

  /* perform the GETH configuration */

  IfxGeth_Eth_initModule(&priv->config_d->geth, gethConfig);
}

/****************************************************************************
 * Function: aurix_enet_txhandler
 *
 * Description:
 *   The interrupt handler of transmit queue
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - driver private parameter
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_enet_txhandler(int irq, void *context, void *arg)
{
  struct aurix_enet_dev_s *priv = arg;

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config_d->geth.gethSFR->DMA_CH[i].STATUS.B.TI)
        {
          priv->config_d->geth.gethSFR->DMA_CH[i].STATUS.B.TI = 1;

          if (priv->config_d->geth.gethSFR->DMA_CH[i].INTERRUPT_ENABLE.B.TIE)
            {
              netdev_lower_txdone(&priv->dev);
              break;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_enet_rxhandler
 *
 * Description:
 *   The interrupt handler of receive queue
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - driver private parameter
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_enet_rxhandler(int irq, void *context, void *arg)
{
  struct aurix_enet_dev_s *priv = arg;
  IfxGeth_Eth *geth = &priv->config_d->geth;
  int channel = irq - priv->config->rx_irq[0];

  if (channel < 0 || channel >= IFXGETH_NUM_RX_CHANNELS)
    {
      return ERROR;
    }

  if (geth->gethSFR->DMA_CH[channel].STATUS.B.RI)
    {
      geth->gethSFR->DMA_CH[channel].STATUS.B.RI = 1;

      if (geth->gethSFR->DMA_CH[channel].INTERRUPT_ENABLE.B.RIE)
        {
          netdev_lower_rxready(&priv->dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_mac_rece_trans_en
 *
 * Description:
 *   Enable MAC Receiver and Transmitter
 *
 * Input Parameters:
 *   priv - Reference to the private data of aurix enet driver
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void aurix_mac_rece_trans_en(struct aurix_enet_dev_s *priv)
{
  IfxGeth_Eth *geth = &priv->config_d->geth;
  int i;

  for (i = 0; i < IFXGETH_NUM_RX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          IfxGeth_Eth_startReceiver(geth, i);
        }
    }

  for (i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          IfxGeth_Eth_startTransmitter(geth, i);
        }
    }
}

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

static int aurix_mvlethphy_init(struct aurix_enet_dev_s *priv)
{
  /* Release phy reset */

  IfxPort_setPinModeOutput(priv->config_d->port_p, 1,
                           IfxPort_OutputMode_pushPull,
                           IfxPort_OutputIdx_general);
  IfxPort_setPinHigh(priv->config_d->port_p, 1);

  IfxGeth_Phy_Mvlq1110_init(ETH_PHY_ADDRESS);
  return 0;
}

#ifdef CONFIG_AURIX_ENET_USE_PHY

static int aurix_ethlink_up(void)
{
  bool              linkup_b;
  uint32            timeout_u32;
  IfxGeth_phyStatus phyStatus;

  timeout_u32 = MAX_TIMEOUT;
  do
    {
      /* read the link state from the phy */

      linkup_b = IfxGeth_Phy_Mvlq1110_link_status(ETH_PHY_ADDRESS, &phyStatus);
      timeout_u32--;
    }
  while ((!linkup_b) && (timeout_u32 > 0));

  return linkup_b ? OK : ERROR;
}

#endif
#endif

/****************************************************************************
 * Function: aurix_enet_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the enet network interface
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_enet_ifup(struct netdev_lowerhalf_s *dev)
{
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;
  const struct aurix_enet_config_s *config = priv->config;

  if (priv->ifup)
    {
      return OK;
    }

  /* Configure GETH */

  aurix_geth_init(priv);

  /* Enable the interrupts at the NVIC */

  for (int i = 0; i < IFXGETH_NUM_RX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          up_enable_irq(config->rx_irq[i]);
          ninfo("Enet attach rx_irq = %d\n", config->rx_irq[i]);
        }
    }

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          up_enable_irq(config->tx_irq[i]);
          ninfo("enet attach tx_irq = %d\n", config->tx_irq[i]);
        }
    }

  /* Enable MAC Receiver and Transmitter */

  aurix_mac_rece_trans_en(priv);

  priv->ifup = true;
  return OK;
}

/****************************************************************************
 * Function: aurix_enet_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the ENET network interface
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int aurix_enet_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;
  struct aurix_enet_module_config_s *config_d = priv->config_d;

  if (priv->ifup)
    {
      IfxGeth_disableModule(config_d->geth.gethSFR);
      priv->ifup = false;
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_enet_setmacaddress
 *
 * Description:
 *   Set MAC address
 *
 * Input Parameters:
 *   mac  - The output MAC address
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static void aurix_enet_setmacaddress(uint8_t *mac)
{
  uint64_t mac_addr = AURIX_ENET_MAC_ADDR;

  mac[0] = (mac_addr & 0xff0000000000u) >> 40u;
  mac[1] = (mac_addr & 0x00ff00000000u) >> 32u ;
  mac[2] = (mac_addr & 0x0000ff000000u) >> 24u;
  mac[3] = (mac_addr & 0x000000ff0000u) >> 16u;
  mac[4] = (mac_addr & 0x00000000ff00u) >> 8u;
  mac[5] = (mac_addr & 0x0000000000ffu);
}

/****************************************************************************
 * Public Functions
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
                          size_t num)
{
  struct aurix_enet_dev_s  *priv = NULL;
  int                       ret  = -EINVAL;
  int                       i;
  int                       channel;

  for (i = 0; i < num; i++)
    {
      const struct aurix_enet_config_s *cfg = config + i;
      priv = kmm_zalloc(sizeof(struct aurix_enet_dev_s));
      if (priv == NULL)
        {
          nerr("ERROR: aurix enet kmm_zalloc failed\n");
          return -ENOMEM;
        }

      priv->dev.netdev.d_pktsize = 0u;
      priv->dev.ops              = &g_aurix_enet_ops;
      priv->dev.quota[NETPKT_TX] = CONFIG_AURIX_ENET1_NTX0DESC;
      priv->dev.quota[NETPKT_RX] = CONFIG_AURIX_ENET1_NRX0DESC;
      priv->config               = cfg;
      priv->config_d             = &g_enet_config_d[i];
      aurix_enet_setmacaddress(
        priv->dev.netdev.d_mac.ether.ether_addr_octet);
      snprintf(priv->dev.netdev.d_ifname, IFNAMSIZ, "eth%d", i);
      aurix_enet_ifdown(&priv->dev);

      /* Register the device with the OS
       * so that socket IOCTLs can be performed
       */

      ret = netdev_lower_register(&priv->dev, NET_LL_ETHERNET);
      if (ret < 0)
        {
          nerr("ERROR: register enet interface failed: %d\n", ret);
          kmm_free(priv);
          break;
        }

      /* using port as index to get net_driver_s */

      dev[cfg->port] = &priv->dev.netdev;

      for (channel = 0; channel < IFXGETH_NUM_RX_CHANNELS; channel++)
        {
          if (priv->config->enable_queues[channel])
            {
#ifdef CONFIG_AURIX_ENET_ISR_WQUEUE
              irq_attach_wqueue(config->rx_irq[channel], NULL,
                                aurix_enet_rxhandler, priv,
                                CONFIG_AURIX_ENET_ISR_WQUEUE_PRIORITY);
#else
              irq_attach(config->rx_irq[channel], aurix_enet_rxhandler,
                         (void *)priv);
#endif
            }
        }

      for (channel = 0; channel < IFXGETH_NUM_TX_CHANNELS; channel++)
        {
          if (priv->config->enable_queues[channel])
            {
#ifdef CONFIG_AURIX_ENET_ISR_WQUEUE
              irq_attach_wqueue(config->tx_irq[channel], NULL,
                                aurix_enet_txhandler, priv,
                                CONFIG_AURIX_ENET_ISR_WQUEUE_PRIORITY);
#else
              irq_attach(config->tx_irq[channel],
                         aurix_enet_txhandler, (void *)priv);
#endif
            }
        }

      /* Configure GETH */

      aurix_geth_init(priv);

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
      /* Configure Phy */

      ret = aurix_mvlethphy_init(priv);
      if (ret < 0)
        {
          nerr("ERROR: marvell enet phy initlalize failed: %d\n", ret);
          break;
        }

#ifdef CONFIG_AURIX_ENET_USE_PHY
      ret = aurix_ethlink_up();
      if (ret < 0)
        {
          nerr("ERROR: Link up failed: %d\n", ret);
          break;
        }
#endif
#endif

      /* Enable the interrupts at the NVIC */

      for (int j = 0; j < IFXGETH_NUM_RX_CHANNELS; j++)
        {
          if (priv->config->enable_queues[j])
            {
              up_enable_irq(cfg->rx_irq[j]);
              ninfo("Enet attach rx_irq = %d\n", cfg->rx_irq[j]);
            }
        }

      for (int j = 0; j < IFXGETH_NUM_TX_CHANNELS; j++)
        {
          if (priv->config->enable_queues[j])
            {
              up_enable_irq(cfg->tx_irq[j]);
              ninfo("enet attach tx_irq = %d\n", cfg->tx_irq[j]);
            }
        }

#ifdef CONFIG_AURIX_ENET_PTP
      up_enable_irq(cfg->sys_irq);
      ninfo("enet attach sys_irq = %d\n", cfg->sys_irq);
#endif

      /* Enable MAC Receiver and Transmitter */

      aurix_mac_rece_trans_en(priv);
      priv->ifup = true;
    }

  return ret;
}
