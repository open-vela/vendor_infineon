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

#include <nuttx/arch.h>
#include <arch/barriers.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#ifdef CONFIG_AURIX_ENET_PTP
#include <nuttx/timers/ptp_clock.h>
#endif

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "aurix_enet.h"
#ifdef CONFIG_AURIX_ENET_DRE
#include "aurix_dre.h"
#endif

#ifdef CONFIG_AUTOMID_CFG_ETH_CONFIG
#include "enet_cfg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* set the values for System Timer0 (STM0) */

/* generate STM0 IRQ at 1Hz rate, if every thing is fine */

#define IFX_CFG_STM0_TICKS_PER_MS        (400000)

/* Realtek Phy1 device address */

#define ETH_PHY1_ADDRESS                 CONFIG_AURIX_ENET1_PHY_ADDR
#define AURIX_ENET_MAC_ADDR              CONFIG_AURIX_ENET1_MAC_ADDR

/* Realtek Switch register address */

#define SWITCH_OPCR1_REG_ADDR                            (0x4A001200)
#define SWITCH_INTERNAL_CPU_VARIABLE_39_REG_ADDR         (0x4A1B004C)

/* Target Speed */

#ifdef CONFIG_AURIX_ENET_SPEED_100M
#define TARGET_DEVICE_SPEED              IfxHsphy_TrgtDeviceSpeed_0P1G_1P25gbps
#else
#define TARGET_DEVICE_SPEED              IfxHsphy_TrgtDeviceSpeed_1G
#endif

/* Ethernet Parameters */

#define GETH_DMA_TX_BUFFER_SIZE          (512)
#define GETH_DMA_RX_BUFFER_SIZE          (1528)

/* maximal timeout definition for link-up detection */

#define MAX_TIMEOUT                      (150000)

/* Define gPTP type */

#ifndef ETHERTYPE_PTP
#define ETHERTYPE_PTP                    0x88f7
#endif

/* Define GCL */

#define GETH_TSM_GCL_DEPTH               10
#define GETH_MAC_GCL_BASE_CONFIG         1
#define GETH_MAC_GCL_GATE_CONFIG         0

#define GETH_MAC_GCL_BTR_LOW_ADDR       (0u)
#define GETH_MAC_GCL_BTR_HIGH_ADDR      (1u)
#define GETH_MAC_GCL_CTR_LOW_ADDR       (2u)
#define GETH_MAC_GCL_CTR_HIGH_ADDR      (3u)
#define GETH_MAC_GCL_TER_ADDR           (4u)
#define GETH_MAC_GCL_LLR_ADDR           (5u)

/* Define specific channel */

#define UNTAGGED_CHANNEL                 1
#define PTP_CHANNEL                      7
#define MIRT_CHANNEL                     7
#ifndef GETH_DMA_CHANNEL_DRE
#define DRE_CHANNEL                      6
#else
#define DRE_CHANNEL                      GETH_DMA_CHANNEL_DRE
#endif

/* Define main boot core */

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
#define IS_BOOT_CORE 1
#else
#define IS_BOOT_CORE 0
#endif

/* Define the number of channels */

#ifdef CONFIG_AURIX_ENET_QUEUE_NUM
#define IFXGETH_NUM_QUEUES_CHANNELS CONFIG_AURIX_ENET_QUEUE_NUM
#else
#define IFXGETH_NUM_QUEUES_CHANNELS 1
#endif

#define FPTP_MHZ                        CONFIG_AURIX_ENET_PTP_FREQ

#define QBV_CYCLE_TIME 1000000
#define QBV_START_CYCLE_TIME 10

/* Multi-core synchronization */

#ifndef CONFIG_AUTOCORE_SYNBARRIER_LOC
#define CONFIG_AUTOCORE_SYNBARRIER_LOC ".sync_barrier"
#endif

#define AURIX_ENET_NUM_INSTANCES        CONFIG_AURIX_ENET_PORT_NUM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ENET configuration dynamic structure */

struct aurix_enet_module_config_s
{
  IfxGeth_Eth                         geth;
  Ifx_P                              *port_p;
  IfxHsphy_Hsphy                      hsphy;
  IfxGeth_Eth_MdioSingleConfig        mdioSingleConfig;
  IfxGeth_Eth_Config                  gethConfig;
  IfxGeth_Eth_FrameConfig             frameConfig;
};

/* ENET configuration structure */

struct aurix_enet_dev_s
{
  struct netdev_lowerhalf_s           dev;
#ifdef CONFIG_AURIX_ENET_PTP
  struct ptp_lowerhalf_s              ptp_dev;
#endif
  struct aurix_enet_module_config_s  *config_d;
  bool                                ifup;
  bool                                mac_ready;
  const struct aurix_enet_config_s   *config;
  struct work_s                       init_work;
#ifdef CONFIG_AURIX_ENET_USE_PHY
  unsigned int                        link_retry_count;
  unsigned int                        link_max_retries;
#endif
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
#ifdef CONFIG_AURIX_ENET_PTP
static int aurix_enet_adjfine(FAR struct ptp_lowerhalf_s *lower,
                              long ppb);
static int aurix_enet_adjtime (FAR struct ptp_lowerhalf_s *lower,
                               int64_t delta);
static int aurix_enet_adjphase(FAR struct ptp_lowerhalf_s *lower,
                               int32_t phase);
static int aurix_enet_gettime(FAR struct ptp_lowerhalf_s *lower,
                              FAR struct timespec *ts,
                              FAR struct ptp_system_timestamp *sts);
static int aurix_enet_settime (FAR struct ptp_lowerhalf_s *lower,
                               FAR const struct timespec *ts);
static void aurix_timestamp_init(struct aurix_enet_dev_s *priv);
static int aurix_enet_syshandler(int irq, void *context, void *arg);
static void aurix_ts_contextdesr_init(IfxGeth_Eth *geth,
                                      IfxGeth_TxDmaChannel channelId);
#endif
#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
static int aurix_phy_init(void);
static void aurix_mdio_init(struct aurix_enet_dev_s *priv);
static int aurix_realtekethphy_init(struct aurix_enet_dev_s *priv);
#ifdef CONFIG_AURIX_ENET_USE_PHY
static int aurix_ethlink_up(struct aurix_enet_dev_s *priv);
static int aurix_serdeslink(struct aurix_enet_dev_s *priv);
#endif
#endif
static void aurix_enet_init_worker(FAR void *arg);
static void aurix_geth_init(struct aurix_enet_dev_s *priv);
static void aurix_mac_rece_trans_en(struct aurix_enet_dev_s *priv);
static int aurix_get_rxframe_size(struct netdev_lowerhalf_s *dev,
                                  uint8_t rx_channel);
static void aurix_enet_setmacaddress(uint8_t *mac);
#ifdef CONFIG_AURIX_ENET_DRE
static void aurix_geth_dre_dma_des_config(Ifx_GETH_DMA_CH *dre_ch,
                                          uint8_t eth_id);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* override default iLLD configuration to use 25MHz reference clock */

const IfxHsphy_GethXpcsParams g_ifxHsphyMyConfig[IFXHSPHY_NUM_OF_PHY] =
{
    {IfxHsphy_XpcsIndex_0, IfxHsphy_XpcsRefClk_25Mhz,
     IfxHsphy_EthIndex_0},
    {IfxHsphy_XpcsIndex_1, IfxHsphy_XpcsRefClk_25Mhz,
     IfxHsphy_EthIndex_1},
    {IfxHsphy_XpcsIndex_max, IfxHsphy_XpcsRefClk_25Mhz,
     IfxHsphy_EthIndex_none}
};

/* Ethernet receive buffer */

static aligned_data(32) uint8 g_gethDmaRxBuffer_u8
                              [IFXGETH_NUM_QUEUES_CHANNELS]
                              [IFXGETH_MAX_RX_DESCRIPTORS]
                              [GETH_DMA_RX_BUFFER_SIZE]
                              locate_data(CONFIG_AURIX_ENET_DMA_LAYOUT);

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
#ifdef CONFIG_AURIX_ENET_PTP
static const struct ptp_ops_s g_aurix_enet_ptp_ops =
{
  .adjfine = aurix_enet_adjfine,
  .adjphase = aurix_enet_adjphase,
  .adjtime = aurix_enet_adjtime,
  .gettime = aurix_enet_gettime,
  .settime = aurix_enet_settime
};
#endif

/* Constant configuration */

static struct aurix_enet_module_config_s g_enet_config_d[] =
{
#ifdef CONFIG_AURIX_ENET0
  {
    .geth.gethSFR         = &MODULE_GETH0,
    .port_p               = &MODULE_P00,
  },
#endif

#ifdef CONFIG_AURIX_ENET1
  {
    .geth.gethSFR         = &MODULE_GETH0,
    .port_p               = &MODULE_P00,
  },
#endif
};

#endif

/* Global pointers per configured instance for cross-driver access */

static struct aurix_enet_dev_s *g_aurix_enet_priv[AURIX_ENET_NUM_INSTANCES] =
{
  NULL
};

/* Ethernet0 Tx queue to Traffic Class mapping */

static int g_txqueue2tc[] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7
};

#ifdef CONFIG_AURIX_ENET_PTP

/* Define a simeple gptp packet, for tx timestamp */

static uint8_t g_gptp_pkt_ts[] =
{
  0x01, 0x80, 0xc2, 0x00, 0x00, 0x0e,   /* DMAC infor */
  0x02, 0x04, 0x00, 0x00, 0x00, 0x02,   /* SMAC infor */
  0x88, 0xf7,                           /* gPTP ether type */
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* simeple data */
};
#endif

/* Gatewayte list config */

#ifdef CONFIG_AUTOMID_CFG_ETH_CONFIG
extern struct qbv_config g_qbv_config;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: aurix_ts_contextdesr_init
 *
 * Description:
 *   Initialize DMA timestamp functionality for Ethernet transmission
 *
 * Input Parameters:
 *   geth      - Pointer to Ethernet peripheral instance
 *   channelId - DMA channel ID for transmission
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during Ethernet initialization
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_ENET_PTP
static void aurix_ts_contextdesr_init(IfxGeth_Eth *geth,
                                      IfxGeth_TxDmaChannel channelId)
{
  volatile IfxGeth_TxDescr *ls_desc_nx;
  volatile IfxGeth_TxDescr *ls_desc =
                          IfxGeth_Eth_getActualTxDescriptor(geth, channelId);

  /* Configure transmit descriptor */

  ls_desc->TDES0.C.TTSL = 0x1;    /* Enable timestamp low */
  ls_desc->TDES1.C.TTSH = 0x0;    /* Disable timestamp high */
  ls_desc->TDES2.C.IVT = 0x0;     /* Disable internal VLAN tag */

  /* Configure context descriptor */

  ls_desc->TDES3.U = 0;           /* Clear all flags */
  ls_desc->TDES3.C.CTXT = 1;      /* Set as context descriptor */
  ls_desc->TDES3.C.PIDV = 1;      /* Enable packet ID valid */
  ls_desc->TDES3.C.OWN = 1;       /* DMA owns descriptor */

  /* Update descriptor pointer */

  IfxGeth_Eth_shuffleTxDescriptor(geth, channelId);
  ls_desc_nx = IfxGeth_Eth_getActualTxDescriptor(geth, channelId);
  geth->gethSFR->DMA.CH[channelId].TXDESC_TAIL_LPOINTER.U =
                                                    (uint32_t)ls_desc_nx;
}

/****************************************************************************
 * Function: aurix_enet_adjfine
 *
 * Description:
 *   Adjust the PTP clock frequency using fine update method
 *
 * Input Parameters:
 *   lower - Reference to the driver state structure
 *   ppb   - Desired frequency offset from nominal frequency in parts
 *           per billion.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during PTP clock synchronization
 *
 ****************************************************************************/

static int aurix_enet_adjfine(FAR struct ptp_lowerhalf_s *lower,
                              long ppb)
{
  uint32 value;
  int tarfreq;
  struct aurix_enet_dev_s *priv =
                container_of(lower, struct aurix_enet_dev_s, ptp_dev);
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  /* Wait till H/w resets the bit */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSADDREG)
    {
    }

  /* Calculate target frequency based on ppb adjustment */

  tarfreq = ((double)ppb * (double)(IfxClock_getXGeth0Frequency())) /
            (double)1000000000 + (double)(IfxClock_getXGeth0Frequency());

  /* Calculate addend value for fine update */

  value = (uint32)((double)(1ull << 32u) /
          ((double)tarfreq /
          (float32)(FPTP_MHZ)));

  /* Update timestamp addend register */

  core->MAC_TIMESTAMP_ADDEND.B.TSAR = value;

  /* Trigger addend register update */

  core->MAC_TIMESTAMP_CONTROL.B.TSADDREG = 1;

  /* Wait till H/w resets the bit */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSADDREG)
    {
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_enet_adjphase
 *
 * Description:
 *   Adjust the PTP clock phase using coarse update method
 *
 * Input Parameters:
 *   lower - Reference to the driver state structure
 *   phase - Phase adjustment value
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called during PTP clock synchronization
 *
 ****************************************************************************/

static int aurix_enet_adjphase(FAR struct ptp_lowerhalf_s *lower,
                               int32_t phase)
{
  int sub = 1;
  unsigned int tar;
  struct aurix_enet_dev_s *priv =
                container_of(lower, struct aurix_enet_dev_s, ptp_dev);
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  /* Wait till H/w resets the bit */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSUPDT)
    {
    }

  if (phase < 0)
    {
      sub = 0;
    }
  else if(phase == 0)
    {
      return OK;
    }

  if (sub == 1)
    {
      tar = (unsigned int)(0x3b9ac9ff - abs(phase));
    }
  else
    {
      tar = (unsigned int)abs(phase);
    }

  core->MAC_SYSTEM_TIME_SECONDS_UPDATE.U            = 0;
  core->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE.B.TSSS   = tar;
  core->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE.B.ADDSUB = sub;

  core->MAC_TIMESTAMP_CONTROL.B.TSUPDT              = 1;

  /* Wait till H/w resets the bit and coarse update is complete */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSUPDT)
    {
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_enet_adjtime
 *
 * Description:
 *   Adjust the PTP hardware clock time by a time offset. This function takes
 *   a delta value in nanoseconds and adjusts the current PTP hardware clock
 *   time by that amount. The adjustment is done by splitting the delta into
 *   seconds and nanoseconds components and updating the respective hardware
 *   registers.
 *
 * Input Parameters:
 *   lower - Reference to the driver state structure
 *   delta - Time offset in nanoseconds to adjust the clock by
 *
 * Returned Value:
 *   OK on success; Negated errno on failure
 *
 ****************************************************************************/

static int aurix_enet_adjtime(FAR struct ptp_lowerhalf_s *lower,
                              int64_t delta)
{
  uint32_t ls = (uint32_t)(delta / NSEC_PER_SEC);
  uint32_t ns = (uint32_t)(delta % NSEC_PER_SEC);
  struct aurix_enet_dev_s *priv =
                container_of(lower, struct aurix_enet_dev_s, ptp_dev);
  Ifx_GETH_PORT_CORE *core =
                &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  if (ns >= 999999999)
    {
      ns -= 999999999;
      ls += 1;
    }

  /* accuracy of 1 ns */

  core->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE.B.TSSS = ns;
  core->MAC_SYSTEM_TIME_SECONDS_UPDATE.B.TSS = ls;
  core->MAC_TIMESTAMP_CONTROL.B.TSINIT = 1;

  while (core->MAC_TIMESTAMP_CONTROL.B.TSINIT)
    {
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_qbv_init
 *
 * Description:
 *   qbv init
 *
 * Input Parameters:
 *   priv - Reference to the private data of aurix enet driver
 *   qbv_config - qbv config
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_ENET1_QBV
static int aurix_qbv_init(struct aurix_enet_dev_s *priv,
                          struct qbv_config *qbv_config)
{
  IfxGeth_Eth *geth = &priv->config_d->geth;
  uint32_t gcldata;

  if (qbv_config->enable != 1)
    {
      return OK;
    }

  IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1, GETH_MAC_GCL_BASE_CONFIG,
                      GETH_MAC_GCL_BTR_LOW_ADDR, qbv_config->basetimens);

  IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1, GETH_MAC_GCL_BASE_CONFIG,
                      GETH_MAC_GCL_BTR_HIGH_ADDR, qbv_config->basetimes);

  IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1, GETH_MAC_GCL_BASE_CONFIG,
                      GETH_MAC_GCL_CTR_LOW_ADDR, qbv_config->cycletimens);

  IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1, GETH_MAC_GCL_BASE_CONFIG,
                      GETH_MAC_GCL_CTR_HIGH_ADDR, qbv_config->cycletimes);

  IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1, GETH_MAC_GCL_BASE_CONFIG,
                      GETH_MAC_GCL_LLR_ADDR, qbv_config->gcllen);

  for (uint8_t i = 0; i < qbv_config->gcllen; i++)
    {
      gcldata = ((uint32_t)(qbv_config->gate[i].gateon) << 24)
                | qbv_config->gate[i].timeinterval;

      IfxGeth_Mac_writeGcl(geth, IfxGeth_PortIndex_1,
                          GETH_MAC_GCL_GATE_CONFIG, i, gcldata);
    }

  geth->gethSFR->PORT[1].MTL.EST_CONTROL.B.SSWL = qbv_config->enable;
  geth->gethSFR->PORT[1].MTL.EST_CONTROL.B.EEST = qbv_config->enable;

  return OK;
}
#endif

/****************************************************************************
 * Name: aurix_enet_settime
 *
 * Description:
 *   Set the hardware PTP timestamp registers with the provided time value.
 *   This function updates both the seconds and nanoseconds portions of the
 *   timestamp in the Ethernet MAC's PTP hardware clock.
 *
 * Input Parameters:
 *   lower - A pointer to the lower-half PTP driver instance
 *   ts    - The timespec structure containing the time value to set
 *
 * Returned Value:
 *   OK (0) on success; Negated errno on failure
 *
 ****************************************************************************/

static int aurix_enet_settime(FAR struct ptp_lowerhalf_s *lower,
                              FAR const struct timespec *ts)
{
  uint32_t ls = ts->tv_sec;
  uint32_t ns = ts->tv_nsec;
  struct aurix_enet_dev_s *priv =
                container_of(lower, struct aurix_enet_dev_s, ptp_dev);
  Ifx_GETH_PORT_CORE *core =
              &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  if (ns >= 999999999)
    {
      ns -= 999999999;
      ls += 1;
    }

  /* accuracy of 1 ns */

  core->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE.B.TSSS = ns;
  core->MAC_SYSTEM_TIME_SECONDS_UPDATE.B.TSS = ls;
  core->MAC_TIMESTAMP_CONTROL.B.TSINIT = 1;

  while (core->MAC_TIMESTAMP_CONTROL.B.TSINIT)
    {
    }

  /* Gateway Control init */

  #ifdef CONFIG_AURIX_ENET1_QBV
  g_qbv_config.basetimes = ts->tv_sec;
  g_qbv_config.basetimens = ts->tv_nsec  / QBV_CYCLE_TIME *
                    QBV_CYCLE_TIME + QBV_START_CYCLE_TIME * QBV_CYCLE_TIME;
  aurix_qbv_init(priv, &g_qbv_config);
  #endif

  return OK;
}

/****************************************************************************
 * Name: aurix_enet_gettime
 *
 * Description:
 *   This function retrieves the current time from the Ethernet hardware's
 *   Precision Time Protocol (PTP) timestamp registers. It reads the high
 *   and low word seconds, as well as the nanoseconds, and converts them
 *   into a `ptp_timestamp` structure.
 *
 *   The function also ensures the consistency of the timestamp by verifying
 *   the values read from the hardware registers, handling cases where
 *   the timestamp might be updated during the read process.
 *
 * Input Parameters:
 *   lower - A pointer to the lower-half PTP driver instance.
 *           This typically points to a structure implementing the
 *           `ptp_lowerhalf_s` interface.
 *   ts    - Holds the PHC timestamp
 *   sts   - A pointer to a `ptp_timestamp` structure where the retrieved
 *           system time will be stored. The structure includes high word
 *           seconds (hs), low word seconds (ls), and nanoseconds (ns).
 *
 * Returned Value:
 *   Returns OK (0) on success. No error codes are currently defined, as
 *   this operation is expected to always succeed if the hardware is
 *   operational and properly configured.
 *
 ****************************************************************************/

static int aurix_enet_gettime(FAR struct ptp_lowerhalf_s *lower,
                              FAR struct timespec *ts,
                              FAR struct ptp_system_timestamp *sts)
{
  uint32_t ns;
  uint32_t ls;
  struct aurix_enet_dev_s *priv =
                container_of(lower, struct aurix_enet_dev_s, ptp_dev);
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  if (sts != NULL)
    {
      clock_gettime(CLOCK_REALTIME, &sts->pre_ts);
    }

  ts->tv_sec = core->MAC_SYSTEM_TIME_SECONDS.B.TSS;
  ns = core->MAC_SYSTEM_TIME_NANOSECONDS.B.TSSS;
  ls = core->MAC_SYSTEM_TIME_SECONDS.B.TSS;

  if (ls != ts->tv_sec)
    {
      ts->tv_sec = ls;
    }

  ts->tv_nsec = ns;

  if (sts != NULL)
    {
      clock_gettime(CLOCK_REALTIME, &sts->post_ts);
    }

  return OK;
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
 *
 ****************************************************************************/

static int aurix_enet_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct aurix_enet_dev_s *priv   = (struct aurix_enet_dev_s *)dev;
  struct eth_hdr_s *eth_hdr       = NULL;
  struct eth_8021qhdr_s *vlan_hdr = NULL;
#ifdef CONFIG_AURIX_ENET_PTP
  bool ptp_packet                 = false;
#endif
  uint8_t channel                 = 0;
  uint32_t pktlen;
  uint32_t descr_num;
  uint32 * pkt_buffer             = NULL_PTR;
  IfxGeth_Eth *geth      = &priv->config_d->geth;
  volatile IfxGeth_TxDescr *actual_descr;
  volatile IfxGeth_TxDescr *next_descr;
  volatile IfxGeth_TxDescr *lastDescr;
  volatile IfxGeth_TxDescr *descr;
  netpkt_t *frag;
  int frag_idx;

  if (geth->gethSFR->CLC.B.DISR)
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
      eth_hdr = (struct eth_hdr_s *)vlan_hdr;
      if (eth_hdr->type == HTONS(ETHERTYPE_PTP))
        {
          /* PTP packet */

          channel = PTP_CHANNEL;
#ifdef CONFIG_AURIX_ENET_PTP
          aurix_ts_contextdesr_init(geth, channel);
          ptp_packet = true;
#endif
        }
      else
        {
          /* Untagged packet */

          channel = UNTAGGED_CHANNEL;
        }
    }

  if (priv->config->enable_queues[channel] == false)
    {
      nerr("ERROR: Transmit channel %d is disabled\n", channel);
      return -ECHRNG;
    }

  actual_descr = IfxGeth_Eth_getActualTxDescriptor(geth, channel);
  geth->txChannel[channel].txBuf1Size = GETH_DMA_TX_BUFFER_SIZE;
  priv->config_d->frameConfig.channelId = channel;
  pkt_buffer = (uint32 *)netpkt_getdata(&priv->dev, pkt);
  frag = pkt;

  lastDescr =
    &geth->txChannel[channel].txDescrList->descr[IFXGETH_MAX_TX_DESCRIPTORS - 1];

  if (pkt_buffer != NULL)
    {
      if (actual_descr->TDES3.R.OWN == 0)
        {
          actual_descr->TDES0.U = (uint32)pkt_buffer;
          actual_descr->TDES2.R.B1L = frag->io_len + NET_LL_HDRLEN(&priv->dev.netdev);
          next_descr = actual_descr;
#ifdef CONFIG_AURIX_ENET_PTP
          if (ptp_packet)
            {
              ptp_packet = false;
              geth->txChannel[channel].timeStampEnable = true;
            }
          else
            {
              geth->txChannel[channel].timeStampEnable = false;
            }
#endif

          pktlen = netpkt_getdatalen(&priv->dev, pkt);
          descr_num = iob_count(pkt);

          for (frag_idx = 0; frag_idx < (int)(descr_num - 1); frag_idx++)
            {
              frag = frag->io_flink;
              if (next_descr == lastDescr)
                {
                  descr = IfxGeth_Eth_getBaseTxDescriptor(geth, channel);

                  /* wrap around the descriptors */

                  next_descr = descr;
                }
              else
                {
                  /* point to the next descriptor */

                  next_descr = &next_descr[1];
                }

              next_descr->TDES0.U = (uint32)&frag->io_data[0];
              next_descr->TDES2.R.B1L = frag->io_len + frag->io_offset;
            }

          pktlen = netpkt_getdatalen(&priv->dev, pkt);
          priv->config_d->frameConfig.packetLength = pktlen;
          IfxGeth_Eth_sendFrame(geth, priv->config->port,
                                &priv->config_d->frameConfig, descr_num);
          netpkt_free(&priv->dev, pkt, NETPKT_TX);
          return OK;
        }
    }

  nerr("ERROR: Transmit buffer not available."
       "Driver sends packet: Channel=%d, Packet Count=%ld\n",
       channel, geth->txChannel[channel].txCount);
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
#ifdef CONFIG_AURIX_ENET_PTP
  int tstsc;
#endif

  if (priv->config_d->geth.gethSFR->CLC.B.DISR)
    {
      return NULL;
    }

#ifdef CONFIG_AURIX_ENET_PTP
  volatile IfxGeth_RxDescr *descr_ptp;
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  if (core->MAC_INTERRUPT_STATUS.B.TSIS)
    {
      /* Read timestamp information from MAC registers */

      tstsc = core->MAC_TIMESTAMP_STATUS.B.TXTSC;

      priv->dev.netdev.d_rxtime.tv_nsec =
                          core->MAC_TX_TIMESTAMP_STATUS_NANOSECONDS.U;
      priv->dev.netdev.d_rxtime.tv_sec =
                          core->MAC_TX_TIMESTAMP_STATUS_SECONDS.U;
      core->MAC_TX_TIMESTAMP_STATUS_SECONDS.U = tstsc;
      pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
      if (pkt)
        {
          netpkt_copyin(&priv->dev, pkt,
                        g_gptp_pkt_ts, sizeof(g_gptp_pkt_ts), 0);
        }
      else
        {
          nerr("ERROR: Aurix recieve packet error, can not allocate packet!\n");
        }

      return pkt;
    }
#endif

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

#ifdef CONFIG_AURIX_ENET_PTP
                  descr_ptp = IfxGeth_Eth_getActualRxDescriptor(
                                          &priv->config_d->geth, channel);
                  if (descr_ptp->RDES3.W.CTXT && !descr_ptp->RDES3.W.OWN)
                    {
                      priv->dev.netdev.d_rxtime.tv_nsec =
                                                descr_ptp->RDES0.C.RTSL;

                      priv->dev.netdev.d_rxtime.tv_sec =
                                                descr_ptp->RDES1.C.RTSH;
                      IfxGeth_Eth_freeReceiveBuffer(&priv->config_d->geth,
                                                    channel);
                    }
#endif

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

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

/****************************************************************************
 * Function: aurix_phy_init
 *
 * Description:
 *   Initialize the phy module
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

static int aurix_phy_init(void)
{
  IfxHsphy_Hsphy hsphy =
    {
      0
    };

  IfxHsphy_Hsphy_Cfg phyConfig =
    {
      0
    };

#ifdef CONFIG_AURIX_ENET_USE_PHY
  IfxHsphy_Hsphy_phyData hsphyData0 =
    {
      0
    };

  IfxHsphy_Hsphy_phyData hsphyData2 =
    {
      0
    };

#endif
  IfxHsphy_Hsphy_phyData hsphyData1 =
    {
      0
    };

  IfxHsphy_status hsphyStatus =
    {
      0
    };

  int ret = OK;

  /* enable HSPHY module */

  IfxHsphy_enableModule(&MODULE_HSPHY);

  /* preload phyConfig with default data */

  IfxHsphy_Hsphy_initModuleConfig(&MODULE_HSPHY, &phyConfig);

#ifdef CONFIG_AURIX_ENET_USE_PHY

  /* assign XpcsParams to HSPHY.Port0 configuration */

  phyConfig.phyConfig[IfxHsphy_PhyIndex_0].cfgData =
                                    &g_ifxHsphyMyConfig[IfxHsphy_PhyIndex_0];
  phyConfig.phyConfig[IfxHsphy_PhyIndex_0].deviceSpeed = TARGET_DEVICE_SPEED;

  /* link HSPHY data to configuration structure */

  hsphy.hsphyData[IfxHsphy_PhyIndex_0] = &hsphyData0;
#endif

  /* assign XpcsParams to HSPHY.Port1 configuration */

  phyConfig.phyConfig[IfxHsphy_PhyIndex_1].cfgData =
                                    &g_ifxHsphyMyConfig[IfxHsphy_PhyIndex_1];
  phyConfig.phyConfig[IfxHsphy_PhyIndex_1].deviceSpeed = TARGET_DEVICE_SPEED;

  /* link HSPHY data to configuration structure */

  hsphy.hsphyData[IfxHsphy_PhyIndex_1] = &hsphyData1;

#ifdef CONFIG_AURIX_ENET_USE_PHY

  /* assign XpcsParams to HSPHY.Port2 configuration */

  phyConfig.phyConfig[IfxHsphy_PhyIndex_2].cfgData =
                                    &g_ifxHsphyMyConfig[IfxHsphy_PhyIndex_2];

  /* link HSPHY data to configuration structure */

  hsphy.hsphyData[IfxHsphy_PhyIndex_2] = &hsphyData2;
#endif

  /* initialize HSPHY module */

  hsphyStatus = IfxHsphy_Hsphy_initModule(&hsphy, &phyConfig);

  if (hsphyStatus != IfxHsphy_status_success)
    {
      /* HSPHY module init failed */

      ret = ERROR;
      nerr("ERROR: HSPHY module init failed\n");
    }

  return ret;
}

/****************************************************************************
 * Function: aurix_mdio_init
 *
 * Description:
 *   Initialize the mdio module
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

static void aurix_mdio_init(struct aurix_enet_dev_s *priv)
{
  IfxGeth_Eth_MdioSingleConfig mdioSingleConfig;

  IfxHsphy_Geth_setMdioOnlyPins(&MODULE_HSPHY,
                      &priv->config->g_mdiopins);

  /* define clock range selection to set MDC to f < 2.5MHz */

  mdioSingleConfig.mdioAppClockRangeSel =
                              IfxGeth_MdioAppClockRangeSel_Crs0Divider102;
  mdioSingleConfig.mdioClkRangeEnable = FALSE;

  /* use Clause45 or Clause22 for configuring Realtek Phy */

#ifdef CONFIG_AURIX_ENET_USE_PHY
  mdioSingleConfig.mdioClause22.mdioClause22All = 0;
#else
  mdioSingleConfig.mdioClause22.mdioClause22All = 0x1000000;
#endif

  /* set single command operation */

  mdioSingleConfig.mdioControlCommandType =
                                  IfxGeth_MdioControlCommandType_singleWrite;

  /* select port range 0..3 */

#ifdef CONFIG_AURIX_ENET_USE_PHY
  mdioSingleConfig.mdioPortRangeSel = IfxGeth_MdioPortRangeSel_0to3;
#else
  mdioSingleConfig.mdioPortRangeSel = IfxGeth_MdioPortRangeSel_24to27;
#endif

  /* transmit a 32bit bit preamble */

  mdioSingleConfig.mdioPreambleSuppressionEnable = FALSE;

  /* send addressing frame before write/read */

  mdioSingleConfig.mdioSkipAddrFrameEnable = FALSE;

  /* initialize MDC/MDIO interface */

  IfxGeth_Eth_initMdio(&priv->config_d->geth, priv->config->port,
                       IfxGeth_MdioModeSelectionType_singleTransfer,
                       &mdioSingleConfig);
}

#ifdef CONFIG_AURIX_ENET_USE_PHY

/****************************************************************************
 * Function: aurix_serdeslink
 *
 * Description:
 *   Configure SerDes link
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

static int aurix_serdeslink(struct aurix_enet_dev_s *priv)
{
  boolean           serdeslinkactive_b;
  PHY_SERDES_MODE   serdesMode_t;
  int               ret = OK;

  /* Realtek: set SerDes link options */

  if (IfxGeth_Eth_Phy_Rtl8221b_serdes_option_set(
                                    &priv->config_d->geth,
                                    priv->config->port,
                                    ETH_PHY1_ADDRESS,
                                    PHY_SERDES_OPTION_2500BASEX_SGMII)
                                    == FALSE)
    {
      /* configuration of Realtek Phy failed,
       * e.g. due to failure in MDIO communication
       */

      ret = ERROR;
      nerr("ERROR: set SerDes link failed\n");
      return ret;
    }

  /* Realtek: disable auto negotiation for SerDes link */

  if (IfxGeth_Eth_Phy_Rtl8221b_serdes_autoNego_set(&priv->config_d->geth,
                                                   priv->config->port,
                                                   ETH_PHY1_ADDRESS, FALSE)
                                                   == FALSE)
    {
      /* configuration of Realtek Phy failed,
       * e.g. due to failure in MDIO communication
       */

      ret = ERROR;
      nerr("ERROR: disable auto negotiation for SerDes link failed\n");
      return ret;
    }

  /* Realtek: check for SerDes link to the HSPHY */

  serdeslinkactive_b =
        IfxGeth_Eth_Phy_Rtl8221b_serdes_link_get(&priv->config_d->geth,
                                                  priv->config->port,
                                                  ETH_PHY1_ADDRESS,
                                                  &serdesMode_t);

  if (serdeslinkactive_b == FALSE)
    {
      /* No Link was established between HSPHY and Realtek Phy */

      ret = ERROR;
      nerr("ERROR: No Link was established between HSPHY and Realtek Phy\n");
    }

  return ret;
}
#endif
#endif

/****************************************************************************
 * Function: aurix_timestamp_init
 *
 * Description:
 *   Initialize the timestamp functionality of the AURIX Ethernet peripheral.
 *   This function enables timestamping for all packets, configures the
 *   sub-second increment and addend values, sets the system time to zero,
 *   and prepares the hardware for precise timekeeping operations.
 *
 * Input Parameters:
 *   priv - Reference to the private data of the AURIX Ethernet driver.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The Ethernet peripheral must be properly initialized before calling
 *   this function. The function assumes the hardware is ready for
 *   timestamp configuration and initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_ENET_PTP
static void aurix_timestamp_init(struct aurix_enet_dev_s *priv)
{
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;
  IfxGeth_TxContextDescriptorConfig *ctxdesc;

  /* Enable timestamp functionality */

  /* Enable two-step timestamp for all active TX channels */

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          /* Configure timestamp settings for this channel */

          ctxdesc =
              &priv->config_d->geth.txChannel[i].contextDescriptorConfig;
          ctxdesc->twoStepTimeStampEnable = 1;
          ctxdesc->twoStepTimeStampPacketID = 15;
        }
    }

  core->MAC_TIMESTAMP_CONTROL.B.TSENA = 1;

  /* Enable timestamp for all packets */

  core->MAC_TIMESTAMP_CONTROL.B.TSEVNTENA = 0;
  core->MAC_TIMESTAMP_CONTROL.B.SNAPTYPSEL = 0x01;
  core->MAC_TIMESTAMP_CONTROL.B.TSMSTRENA = 1;
  core->MAC_TIMESTAMP_CONTROL.B.TSVER2ENA = 1;

  /* Set rollover control */

  core->MAC_TIMESTAMP_CONTROL.B.TSCTRLSSR = 1;
  core->MAC_CSR_SW_CTRL.B.RCWE = 1;
  core->MAC_TIMESTAMP_CONTROL.B.TXTSSTSM = 1;
  core->MAC_TIMESTAMP_CONTROL.B.TSIPENA = 1;
  core->MAC_TIMESTAMP_CONTROL.B.AV8021ASMEN = 1;
  core->MAC_TIMESTAMP_CONTROL.B.TSENMACADDR = 1;

  /* Set sub-second increment value */

  core->MAC_SUB_SECOND_INCREMENT.B.SSINC = (uint32)1E9 / FPTP_MHZ;

  /* Set sub-nanosecond increment value */

  core->MAC_SUB_SECOND_INCREMENT.B.SNSINC =
                              (((double)(1E9) / (double)FPTP_MHZ) -
                              (uint32)(1E9 / FPTP_MHZ)) * 256;

  /* Set addend value */

  core->MAC_TIMESTAMP_ADDEND.B.TSAR =
                        (uint32)((double)(1ull << 32u) /
                        (double)(IfxClock_getXGeth0Frequency() /
                        (float)FPTP_MHZ));

  /* Set update method */

  core->MAC_TIMESTAMP_CONTROL.B.TSADDREG = 1;

  /* Wait for update to complete */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSADDREG == 1u)
    {
    }

  /* Set fine or coarse update mode */

  core->MAC_TIMESTAMP_CONTROL.B.TSCFUPDT = 1;

  /* Set system time seconds */

  core->MAC_SYSTEM_TIME_SECONDS_UPDATE.U = 0;

  /* Set system time sub-seconds */

  core->MAC_SYSTEM_TIME_NANOSECONDS_UPDATE.U = 0;

  /* Initialize timestamp */

  core->MAC_TIMESTAMP_CONTROL.B.TSINIT = 1u;

  /* Wait till H/w resets the bit and initialization is complete */

  while (core->MAC_TIMESTAMP_CONTROL.B.TSINIT)
    {
    }

  core->MAC_INTERRUPT_ENABLE.B.TSIE = 1;
}
#endif

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
  IfxGeth_Eth_Config  *gethConfig = &priv->config_d->gethConfig;
  priv->config_d->geth.gethSFR = &MODULE_GETH0;
  IfxGeth_Eth_RxChannelConfig *rxchannel;
  IfxGeth_Eth_DmaInterruptConfig *rx_dma_interrupt;
  IfxGeth_Eth_RxQueueConfig *rx_mtl_queue;
  IfxGeth_Eth_TxChannelConfig *txchannel;
  IfxGeth_Eth_DmaInterruptConfig *tx_dma_interrupt;
  IfxGeth_Eth_TxQueueConfig *tx_mtl_queue;
  IfxGeth_Eth_Bridge_PortConfig *bridge_port_config =
                  &gethConfig->bridge.portConfig[priv->config->port];
  int provider = IfxCpu_getCoreId();
  int tx_nchannel = 0;
  int rx_nchannel = 0;

  IfxGeth_Eth_RxRoutingConfig rx_rt_config =
    {
      0,
    };

  IfxGeth_Eth_VlanRxRoutingConfig vlan_rt_config =
    {
      0
    };

  /* get the default configuration for the GETH module */

  IfxGeth_Eth_initModuleConfig(gethConfig, &MODULE_GETH0);

  /* set RGMII as interface to the HSPHY */

#ifdef CONFIG_AURIX_ENET_SPEED_100M
  gethConfig->port[priv->config->port].phyInterfaceMode =
                                    IfxGeth_PhyInterfaceMode_mii_100;
#else
  gethConfig->port[priv->config->port].phyInterfaceMode =
                                    IfxGeth_PhyInterfaceMode_rgmii_1000;
#endif

  /* configure the "own" MAC address into slot0 */

  aurix_enet_setmacaddress(
                gethConfig->port[priv->config->port].mac.macAddress);

  /* configure Ethernet Bridge */

  /* configure the bridge to operate in single port mode, MAC0 only is used */

  gethConfig->bridge.mode = IfxGeth_BridgePortMode_singlePort1;

  /* enable TxQueue0 on bridge port 0 */

  bridge_port_config->configTxQueuesAndRxChannels.enable.txQueue0 = 1;

  /* enable RxChannel0 on bridge port 0 */

  bridge_port_config->configTxQueuesAndRxChannels.enable.rxChannel0 = 1;

  for (int i = 0; i < IFXGETH_NUM_RX_CHANNELS; i++)
    {
      /* set up Rx queue map according to vlan priority */

      vlan_rt_config.rxQueuePriorityMap[i].F = 1 << i;
      rxchannel = &gethConfig->dma.rxChannel[i];
      rxchannel->channelEnable = false;
      txchannel = &gethConfig->dma.txChannel[i];
      txchannel->channelEnable = false;
    }

  /* configure Ethernet DMA */

  for (int i = 0; i < IFXGETH_NUM_RX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          /* set up Rx channel */

          rxchannel = &gethConfig->dma.rxChannel[i];

          /* enable Rx DMA channel */

          rxchannel->channelEnable = true;

          /* set receive buffer size in the Rx.Dscr */

          rxchannel->rxBuffer1Size = GETH_DMA_RX_BUFFER_SIZE;

          /* assign the receive buffer */

          rxchannel->rxBuffer1StartAddress =
                    (uint32 *)g_gethDmaRxBuffer_u8[rx_nchannel];
          rx_nchannel++;

          /* set up the DMA Rx interrupt */

          rx_dma_interrupt = &gethConfig->dma.rxInterrupt[i];

          /* set up the DMA Rx priority */

          rx_dma_interrupt->priority = IRQ_TO_NDX(priv->config->rx_irq[i]);

          /* set up the DMA Rx interrupt receiver, CPU0 */

          rx_dma_interrupt->provider = provider;

          /* set up virtual machine id */

          rx_dma_interrupt->vmId = IfxSrc_VmId_none;

          /* enable Rx Queue */

          rx_mtl_queue =
                    &gethConfig->port[IfxGeth_PortIndex_1].mtl.rxQueue[i];

          rx_mtl_queue->enable = true;

          /* enable Rx queue store and forward function */

          rx_mtl_queue->storeAndForward = true;

          /* set up the Rx queue size */

          rx_mtl_queue->rxQueueSize = 15;

          /* set up the Rx queue to DMA channel mapping */

          rx_mtl_queue->rxDmaChannelMap = i;

          if (DRE_CHANNEL == i)
            {
              /* Enable the current queue to receive AVTP packets */

              rx_mtl_queue->enableAudioVideoBridge = true;
            }
        }
    }

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          /* set up Tx channel */

          txchannel = &gethConfig->dma.txChannel[i];

          /* enable Tx DMA channel */

          txchannel->channelEnable = true;

          /* set transmit buffer size in the Tx.Dscr */

          txchannel->txBuffer1Size = 0;

          /* assign the transmit buffer */

          txchannel->txBuffer1StartAddress = NULL_PTR;
          tx_nchannel++;

          /* set up the DMA Tx interrupt */

          tx_dma_interrupt = &gethConfig->dma.txInterrupt[i];

          /* set up the DMA Tx priority */

          tx_dma_interrupt->priority = IRQ_TO_NDX(priv->config->tx_irq[i]);

          /* set up the DMA Tx interrupt receiver, CPU0 */

          tx_dma_interrupt->provider = provider;

          /* set up virtual machine id */

          tx_dma_interrupt->vmId = IfxSrc_VmId_none;

          /* enable Tx Queue */

          tx_mtl_queue =
                &gethConfig->port[IfxGeth_PortIndex_1].mtl.txQueue[i];

          tx_mtl_queue->enable = true;

          /* set up Tx queue size */

          tx_mtl_queue->txQueueSize = 15;

          /* set up Tx queue to Traffic Class map */

          tx_mtl_queue->queueToTrafficClassMap = g_txqueue2tc[i];
        }
    }

  /* perform the GETH configuration */

#ifdef CONFIG_AURIX_ENET_PTP
  IfxGeth_enableInterrupt(IfxGeth_ServiceRequest_INTR,
                          provider,
                          IRQ_TO_NDX(priv->config->sys_irq),
                          IfxSrc_VmId_none);
#endif

#ifndef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
  while (!shared_data_manual.eth_sync_barrier)
    {
      up_udelay(CONFIG_USEC_PER_TICK);
    }

  UP_DMB();
#endif

  IfxGeth_Eth_initModule(&priv->config_d->geth, gethConfig, IS_BOOT_CORE);

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
  UP_DMB();
  shared_data_manual.eth_sync_barrier = 1;
#endif

#ifdef CONFIG_AURIX_ENET_DRE
  /* Config TX and RX DMA CH X descriptors handled by DRE */

  aurix_geth_dre_dma_des_config(
                        &priv->config_d->geth.gethSFR->DMA.CH[DRE_CHANNEL],
                        DRE_BUF0);
#endif

  /* map packet to Rx queue according VLAN priority */

  IfxGeth_Eth_rxVlanPacket2QueueMap(&priv->config_d->geth,
                                    IfxGeth_PortIndex_1, &vlan_rt_config);

  /* map untaged packet and ptp packet to specific channel */

  rx_rt_config.untaggedPacketQueue = UNTAGGED_CHANNEL;
  rx_rt_config.ptpPacketQueue = PTP_CHANNEL;
  rx_rt_config.overrideMulticastOrBroadcastRouting = 1;

  IfxGeth_Eth_rxPacket2QueueMap(&priv->config_d->geth,
                                IfxGeth_PortIndex_1, &rx_rt_config);

#ifdef CONFIG_AURIX_ENET_PPS
  IfxPort_setPinModeOutput(priv->config->pps_pin->pin.port,
                           priv->config->pps_pin->pin.pinIndex,
                           IfxPort_OutputMode_pushPull,
                           priv->config->pps_pin->select);
  priv->config_d->geth.gethSFR->PORT[IfxGeth_PortIndex_1]
                      .CORE.MAC_PPS_CONTROL.B.PPSEN0 = 0;
  priv->config_d->geth.gethSFR->PORT[IfxGeth_PortIndex_1]
                      .CORE.MAC_PPS_CONTROL.B.PPSCTRL0_PPSCMD0 = 1;
#endif

#ifdef CONFIG_AURIX_ENET_PTP
  aurix_timestamp_init(priv);
  aurix_enet_adjtime(&priv->ptp_dev, 0);
#endif
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
      if (priv->config_d->geth.gethSFR->DMA.CH[i].STATUS.B.TI)
        {
          priv->config_d->geth.gethSFR->DMA.CH[i].STATUS.B.TI = 1;

          if (priv->config_d->geth.gethSFR->DMA.CH[i].INTERRUPT_ENABLE.B.TIE)
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

  if (geth->gethSFR->DMA.CH[channel].STATUS.B.RI)
    {
      geth->gethSFR->DMA.CH[channel].STATUS.B.RI = 1;

      if (geth->gethSFR->DMA.CH[channel].INTERRUPT_ENABLE.B.RIE)
        {
          netdev_lower_rxready(&priv->dev);
        }
    }

  return OK;
}

#ifdef CONFIG_NETDEV_IOCTL

/****************************************************************************
 * Function: aurix_switch_mdio_read
 *
 * Description:
 *   Read a 32-bit value from the specified switch register address
 *   using the MDIO protocol (Clause 22).
 *
 * Input Parameters:
 *   priv - Reference to the private data of the aurix Ethernet driver.
 *   switch_regaddr - The address of the switch register to read from.
 *   reg_val - The address of the variable to store the read value.
 *
 * Returned Value:
 *   The 32-bit value read from the switch register on success;
 *   Negated errno (-1) on failure if the MDIO bus is busy after 10 retries.
 *
 * Assumptions:
 *   The MDIO bus is correctly initialized and the switch is responsive.
 *
 ****************************************************************************/

static uint8_t aurix_switch_mdio_read(struct aurix_enet_dev_s *priv,
                                      uint16_t switch_regaddr,
                                      uint16_t *reg_val)
{
  int ret = 0;

  ret = IfxGeth_Eth_mdio_read_clause22(&priv->config_d->geth,
                                       priv->config->port,
                                       ETH_PHY1_ADDRESS, switch_regaddr, reg_val);
  if (ret)
    {
      nerr("ERROR: Read reg error\n");
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Function: aurix_switch_mdio_write
 *
 * Description:
 *   Write a 32-bit value to the specified switch register address
 *   using the MDIO protocol (Clause 22).
 *
 * Input Parameters:
 *   priv - Reference to the private data of the aurix Ethernet driver.
 *   switch_regaddr - The address of the switch register to write to.
 *   value - The 32-bit value to be written to the switch register.
 *
 * Returned Value:
 *   1 on success;
 *   Negated errno (-1) on failure, which occurs if the MDIO bus
 *   is busy after 20 retries or if there is a write error.
 *
 * Assumptions:
 *   The MDIO bus is correctly initialized and the switch is responsive.
 *
 ****************************************************************************/

static uint8_t aurix_switch_mdio_write(struct aurix_enet_dev_s *priv,
                                       uint16_t switch_regaddr,
                                       uint16_t value)
{
  int ret = 0;

  ret = IfxGeth_Eth_mdio_write_clause22(&priv->config_d->geth,
                                        priv->config->port,
                                        ETH_PHY1_ADDRESS, switch_regaddr,
                                        value);
  if (ret)
    {
      nerr("ERROR: Write reg error\n");
      return -1;
    }

  return OK;
}

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
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)dev;
  struct mii_ioctl_data_s *req =
          (struct mii_ioctl_data_s *)((uintptr_t)arg);
  int ret = -ENOTTY;

  DEBUGASSERT(req);
  switch (cmd)
    {
      case SIOCGMIIREG:
        {
          ret = aurix_switch_mdio_read(priv, req->reg_num, &req->val_out);
        }
        break;

      case SIOCSMIIREG:
        {
          ret = aurix_switch_mdio_write(priv, req->reg_num, req->val_in);
        }
        break;

      default:
          ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Function: aurix_enet_syshandler
 *
 * Description:
 *   System interrupt handler for Ethernet timestamp events
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *   arg     - Driver private parameter
 *
 * Returned Value:
 *   OK(0) on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called in interrupt context
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_ENET_PTP
static int aurix_enet_syshandler(int irq, void *context, void *arg)
{
  struct aurix_enet_dev_s *priv = arg;
  IfxGeth_Eth_Config  *gethConfig = &priv->config_d->gethConfig;
  Ifx_GETH_PORT_CORE *core =
        &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  /* Check if timestamp interrupt is pending and enabled */

  for (int i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if ((!core->MAC_INTERRUPT_STATUS.B.TSIS) &&
          gethConfig->dma.txChannel[i].timeStampEnable)
        {
          if (core->MAC_INTERRUPT_ENABLE.B.TSIE)
            {
              netdev_lower_rxready(&priv->dev);
            }
        }
    }

  return OK;
}
#endif

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
          IfxGeth_Eth_startReceiver(geth, priv->config->port, i);
        }
    }

  for (i = 0; i < IFXGETH_NUM_TX_CHANNELS; i++)
    {
      if (priv->config->enable_queues[i])
        {
          IfxGeth_Eth_startTransmitter(geth, priv->config->port, i);
        }
    }
}

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

/****************************************************************************
 * Function: aurix_realtekethphy_init
 *
 * Description:
 *   Release Realtek Phy from Reset and initialize it
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

static int aurix_realtekethphy_init(struct aurix_enet_dev_s *priv)
{
  int ret = OK;

#ifdef CONFIG_AURIX_ENET_USE_PHY

  /* release phy reset */

  IfxPort_setPinModeOutput(priv->config_d->port_p, 11,
                           IfxPort_OutputMode_pushPull,
                           IfxPort_OutputIdx_general);
  IfxPort_setPinHigh(priv->config_d->port_p, 11);

  /* initialize the phy */

  if (IfxGeth_Eth_Phy_Rtl8221b_init(&priv->config_d->geth,
                                    priv->config->port,
                                    ETH_PHY1_ADDRESS) == 0)
    {
      /* initialization of Realtek Phy failed,
       * e.g. due to failure in MDIO communication
       */

      ret = ERROR;
      nerr("ERROR: set SerDes link failed\n");
    }
#endif

  return ret;
}

#ifdef CONFIG_AURIX_ENET_USE_PHY

/****************************************************************************
 * Function: aurix_ethlink_up
 *
 * Description:
 *   Check if Link between Laptop and Realtek PHY is up
 *   and determine Link Speed
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

static int aurix_ethlink_up(struct aurix_enet_dev_s *priv)
{
  bool              linkup_b;
  IfxGeth_phyStatus phyStatus;

  /* give the Realtek phy some time to establish the link */

  /* read the link state from the phy */

  linkup_b = IfxGeth_Eth_Phy_Rtl8221b_link_status(&priv->config_d->geth,
                                                  priv->config->port,
                                                  ETH_PHY1_ADDRESS,
                                                  &phyStatus);

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
  int ret;

  if (priv->ifup)
    {
      return OK;
    }

  /* Schedule the initialization work to be done in workqueue */

  ret = work_queue(HPWORK, &priv->init_work, aurix_enet_init_worker, priv, 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to schedule enet init work: %d\n", ret);
      return ret;
    }

  ninfo("Scheduled enet initialization work in workqueue\n");

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
      shared_data_manual.eth_sync_barrier = 0;
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
 * Name: aurix_geth_dre_dma_des_config
 *
 * Description:
 *   Initialize the selected DMA for DRE
 *
 * Input Parameters:
 *   dre_ch - DMA channel to be used for DRE
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_AURIX_ENET_DRE
static void aurix_geth_dre_dma_des_config(Ifx_GETH_DMA_CH *dre_ch,
                                          uint8_t eth_id)
{
  /* TX and RX DMA CH X descriptors handled by DRE */

  dre_ch->TX_CONTROL2.B.TDRL = GETH_DES_RING_LENGTH - GETH_DES_RING_INDEX;
  dre_ch->TXDESC_LIST_LADDRESS.U = (uint32) &
                      ((Ifx_DRE_RAM *)DRE_RAM)->eth[eth_id].tx_desc[0];
  dre_ch->RX_CONTROL2.B.RDRL = GETH_DES_RING_LENGTH - GETH_DES_RING_INDEX;
  dre_ch->RXDESC_LIST_LADDRESS.U = (uint32) &
                      ((Ifx_DRE_RAM *)DRE_RAM)->eth[eth_id].rx_desc[0];
}
#endif

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

      /* Initialize link detection state */

#ifdef CONFIG_AURIX_ENET_USE_PHY
      priv->link_retry_count = 0;
      priv->link_max_retries = 50;
#endif

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

      /* Save priv per configured instance only once */

      if (g_aurix_enet_priv[cfg->port] == NULL)
        {
          g_aurix_enet_priv[cfg->port] = priv;
        }

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

#ifdef CONFIG_AURIX_ENET_PTP
#ifdef CONFIG_AURIX_ENET_ISR_WQUEUE
      irq_attach_wqueue(config->sys_irq, NULL, aurix_enet_syshandler,
                        priv, CONFIG_AURIX_ENET_ISR_WQUEUE_PRIORITY);
#else
      irq_attach(config->sys_irq, aurix_enet_syshandler, (void *)priv);
#endif
      priv->ptp_dev.ops = &g_aurix_enet_ptp_ops;
      ptp_clock_register(&priv->ptp_dev, INT32_MAX, i);
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: aurix_enet_get_mac_time_ns
 *
 * Description:
 *   Return the current MAC system time nanoseconds from
 *   MODULE_GETH0.PORT[1].CORE.MAC_SYSTEM_TIME_NANOSECONDS.U when the ENET
 *   interface is up. If the interface is not up or not initialized, a
 *   sentinel value is returned.
 *
 * Input Parameters:
 *  Port number (for hardware that has multiple enet interfaces)
 *
 * Returned Value:
 *   The current MAC nanoseconds value (int) when the interface is up;
 *   otherwise -1.
 *
 ****************************************************************************/

int aurix_enet_get_mac_time_ns(int port)
{
  struct aurix_enet_dev_s *priv = g_aurix_enet_priv[port];
  Ifx_GETH_PORT_CORE *core =
                &priv->config_d->geth.gethSFR->PORT[priv->config->port].CORE;

  if (priv != NULL)
    {
      return core->MAC_SYSTEM_TIME_NANOSECONDS.U;
    }

  return -1;
}

/****************************************************************************
 * Name: aurix_enet_mac_done
 *
 * Description:
 *   Check whether the Ethernet MAC on AURIX has been fully initialized and
 *   is ready for normal operation.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple enet interfaces)
 *
 * Returned Value:
 *   true  - The MAC has completed initialization and is ready.
 *   false - Either the driver has not been probed yet or initialization
 *           has not finished.
 *
 ****************************************************************************/

bool aurix_enet_mac_done(int port)
{
  struct aurix_enet_dev_s *priv = g_aurix_enet_priv[port];

  if (priv != NULL)
    {
      return priv->mac_ready;
    }

  return false;
}

/****************************************************************************
 * Function: aurix_enet_init_worker
 *
 * Description:
 *   Worker function to perform complete Ethernet initialization in workqueue context.
 *   This function handles all initialization tasks including PHY configuration,
 *   MDIO setup, link detection, interrupt enabling, and MAC configuration
 *   to avoid blocking the main initialization thread. All operations are
 *   performed sequentially in a single workqueue to avoid nested workqueues.
 *
 * Input Parameters:
 *   arg - Reference to the driver state structure (struct aurix_enet_dev_s *)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the low priority work queue context
 *   The priv structure must be valid
 *
 ****************************************************************************/

static void aurix_enet_init_worker(FAR void *arg)
{
  struct aurix_enet_dev_s *priv = (struct aurix_enet_dev_s *)arg;
  const struct aurix_enet_config_s *cfg = priv->config;

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
  int ret = OK;
#endif

  DEBUGASSERT(priv != NULL);

  ninfo("Starting enet initialization in workqueue\n");

  if (!priv->ifup)
    {
#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

      /* Configure HSPHY */

      ret = aurix_phy_init();

      if (ret < 0)
        {
          nerr("ERROR: enet phy initlalize failed: %d\n", ret);
          return;
        }
#endif

      /* Configure GETH */

      aurix_geth_init(priv);

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

      priv->mac_ready = true;
      aurix_mdio_init(priv);

      /* Release Realtek Phy from Reset and initialize it */

      ret = aurix_realtekethphy_init(priv);

      if (ret < 0)
        {
          nerr("ERROR: Realtek Phy reset and initialize failed: %d\n", ret);
          return;
        }
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

#ifndef CONFIG_AURIX_ENET_MAIN_BOOT_CORE

      priv->mac_ready = true;
      shared_data_manual.eth_mac_ready_flag = priv->mac_ready;

#endif

#ifdef CONFIG_AURIX_ENET_PTP
      up_enable_irq(cfg->sys_irq);
      ninfo("enet attach sys_irq = %d\n", cfg->sys_irq);
#endif

      /* Enable MAC Receiver and Transmitter */

      aurix_mac_rece_trans_en(priv);

      priv->ifup = true;

      ninfo("Enet initialization completed successfully in workqueue\n");
    }

#ifdef CONFIG_AURIX_ENET_MAIN_BOOT_CORE
#ifdef CONFIG_AURIX_ENET_USE_PHY

  ret = aurix_ethlink_up(priv);
  if (ret < 0)
    {
      priv->link_retry_count++;
      nerr("ERROR: Link is up failed: %d (retry %u/%u)\n",
           ret, priv->link_retry_count, priv->link_max_retries);

      if (priv->link_retry_count < priv->link_max_retries)
        {
          /* Schedule next check in 100ms */

          work_queue(HPWORK, &priv->init_work, aurix_enet_init_worker, priv,
                     MSEC2TICK(100));
          return;
        }
      else
        {
          nerr("ERROR: Link detection timeout after %u retries\n",
               priv->link_max_retries);
          priv->link_retry_count = 0;
          return;
        }
    }
  else
    {
      ninfo("Carrier notification sent to network stack\n");
      netdev_lower_carrier_on(&priv->dev);
    }

  /* Set Realtek Phy SerDes link options,
   * disable auto negotiation for SerDes link
   * and check if SerDes link to HSPHY is up
   */

  ret = aurix_serdeslink(priv);

  if (ret < 0)
    {
      priv->link_retry_count++;
      nerr("ERROR: serdeslink failed: %d (retry %u/%u)\n",
           ret, priv->link_retry_count, priv->link_max_retries);

      if (priv->link_retry_count < priv->link_max_retries)
        {
          /* Schedule next check in 100ms */

          work_queue(HPWORK, &priv->init_work, aurix_enet_init_worker, priv,
                     MSEC2TICK(100));
          return;
        }
      else
        {
          nerr("ERROR: SerDes link detection timeout after %u retries\n",
               priv->link_max_retries);
          priv->link_retry_count = 0;
          return;
        }
    }

#endif
#endif
  netdev_lower_carrier_on(&priv->dev);
  return;
}
