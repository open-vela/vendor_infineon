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

#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/slave.h>
#include <nuttx/spinlock.h>

#include "IfxPort.h"
#include "aurix_qspi.h"
#include "tricore_internal.h"
#include "If/SpiIf.h"
#include "aurix_qspi_slave.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_WORDS2BYTES(priv, w)   ((w) * ((priv)->datawidth / 8))
#define AURIX_BYTES2WORDS(priv, b)   ((b) / ((priv)->datawidth / 8))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_qspi_slave_priv_s
{
  struct spi_slave_ctrlr_s ctrlr;
  struct spi_slave_dev_s *dev;
  const struct aurix_qspi_slave_config_s *config;
  IfxQspi_SpiSlave slave;
  uint8_t datawidth;
  uint8_t *txbuf;
  uint8_t *rxbuf;
  uint32_t txpos;
  uint32_t txsend;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void aurix_qspi_slave_bind(struct spi_slave_ctrlr_s *ctrlr,
                                 struct spi_slave_dev_s *dev,
                                 enum spi_slave_mode_e mode, int nbits);
static void aurix_qspi_slave_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int aurix_qspi_slave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                   const void *data, size_t nwords);
static bool aurix_qspi_slave_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void aurix_qspi_slave_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t aurix_qspi_slave_qpoll(struct spi_slave_ctrlr_s *ctrlr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_slave_ctrlrops_s aurix_qspi_slave_ops =
{
  .bind     = aurix_qspi_slave_bind,
  .unbind   = aurix_qspi_slave_unbind,
  .enqueue  = aurix_qspi_slave_enqueue,
  .qfull    = aurix_qspi_slave_qfull,
  .qflush   = aurix_qspi_slave_qflush,
  .qpoll    = aurix_qspi_slave_qpoll,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_qspi_slave_transmit_done
 ****************************************************************************/

static void
aurix_qspi_slave_transmit_done(struct aurix_qspi_slave_priv_s *priv,
                               uint32_t len)
{
  spiinfo("set %" PRIu32 "  to send, %" PRIu32 " has been transferred\n",
          priv->txsend, len);

  if (priv->txsend && priv->txpos >= len)
    {
      /* New data has been filled to tx buffer during the transfer process,
       * move the data to the beginning of the buffer.
       */

       priv->txpos -= len;
       memmove(priv->txbuf, priv->txbuf + len, priv->txpos);
    }

  SPIS_DEV_NOTIFY(priv->dev, SPISLAVE_TX_COMPLETE);
}

/****************************************************************************
 * Name: aurix_qspi_slave_transmit
 ****************************************************************************/

static int aurix_qspi_slave_transmit(int irq, void *context, void *arg)
{
  struct aurix_qspi_slave_priv_s *priv = arg;

  if (priv->config->usedma)
    {
      IfxQspi_SpiSlave_isrDmaTransmit(&priv->slave);
    }
  else
    {
      IfxQspi_SpiSlave_isrTransmit(&priv->slave);
      if (priv->slave.txJob.remaining > 0)
        {
          return 0;
        }
    }

  aurix_qspi_slave_transmit_done(priv, priv->txsend);
  return 0;
}

/****************************************************************************
 * Name: aurix_qspi_slave_receive_done
 ****************************************************************************/

static void
aurix_qspi_slave_receive_done(struct aurix_qspi_slave_priv_s *priv,
                              uint32_t len)
{
  int nwords;

  spiinfo("%" PRIu32 " sent and %" PRIu32 " left\n", len, priv->txpos);

  nwords = SPIS_DEV_RECEIVE(priv->dev, priv->rxbuf,
                            AURIX_BYTES2WORDS(priv, len));
  if (nwords != AURIX_BYTES2WORDS(priv, len))
    {
      spierr("data is not fully received by user\n");
    }

  SPIS_DEV_NOTIFY(priv->dev, SPISLAVE_RX_COMPLETE);
#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
  while (IfxQspi_SpiSlave_getStatus(&priv->slave) == SpiIf_Status_busy);
#else
  while (IfxQspi_SpiSlave_getStatus(&priv->slave) == IfxQspi_Status_busy);
#endif
  if (priv->txpos != 0)
    {
      IfxQspi_SpiSlave_exchange(&priv->slave, priv->txbuf,
                                priv->rxbuf,
                                AURIX_BYTES2WORDS(priv, priv->txpos));
      priv->txsend = priv->txpos;
    }
  else
    {
      IfxQspi_SpiSlave_exchange(&priv->slave, NULL, priv->rxbuf,
                                AURIX_BYTES2WORDS(priv,
                                priv->config->bufsize));
      priv->txsend = 0;
    }
}

/****************************************************************************
 * Name: aurix_qspi_slave_receive
 ****************************************************************************/

static int aurix_qspi_slave_receive(int irq, void *context, void *arg)
{
  struct aurix_qspi_slave_priv_s *priv = arg;
  int len;

  if (priv->config->usedma)
    {
      IfxQspi_SpiSlave_isrDmaReceive(&priv->slave);
    }
  else
    {
      IfxQspi_SpiSlave_isrReceive(&priv->slave);
      if (priv->slave.onTransfer)
        {
          return 0;
        }
    }

  len = priv->txsend ? priv->txsend : priv->config->bufsize;
  aurix_qspi_slave_receive_done(priv, len);
  return 0;
}

/****************************************************************************
 * Name: aurix_qspi_slave_error
 ****************************************************************************/

static int aurix_qspi_slave_error(int irq, void *context, void *arg)
{
  struct aurix_qspi_slave_priv_s *priv = arg;

  IfxQspi_SpiSlave_isrError(&priv->slave);
  spierr("qspi slave error: parity %d, configure %d, baudrate %d "
         "txoverflow %d, txunderflow %d, rxoverflow %d, rxunderflow %d "
         "timeout %d, slsiinactive %d\n",
         priv->slave.errorFlags.parityError,
         priv->slave.errorFlags.configurationError,
         priv->slave.errorFlags.baudrateError,
         priv->slave.errorFlags.txFifoOverflowError,
         priv->slave.errorFlags.txFifoUnderflowError,
         priv->slave.errorFlags.rxFifoOverflowError,
         priv->slave.errorFlags.rxFifoUnderflowError,
         priv->slave.errorFlags.expectTimeoutError,
         priv->slave.errorFlags.slsiMisplacedInactivation);

  return 0;
}

/****************************************************************************
 * Name: aurix_qspi_slave_reset
 ****************************************************************************/

static void aurix_qspi_slave_reset(struct aurix_qspi_slave_priv_s *priv)
{
  volatile Ifx_SRC_SRCR *src;
  Ifx_QSPI_BACONENTRY bacon;

  IfxQspi_pause(priv->slave.qspi);
  bacon.U = priv->slave.qspi->BACON.U;

  IfxQspi_requestReset(priv->slave.qspi, IfxQspi_Reset_stateMachineAndFifo);

  priv->slave.qspi->BACONENTRY.U = bacon.U;

  IfxQspi_clearAllEventFlags(priv->slave.qspi);
  src = IfxQspi_getTransmitSrc(priv->slave.qspi);
  IfxSrc_clearRequest(src);
  src = IfxQspi_getReceiveSrc(priv->slave.qspi);
  IfxSrc_clearRequest(src);
  src = IfxQspi_getErrorSrc(priv->slave.qspi);
  IfxSrc_clearRequest(src);

  IfxQspi_run(priv->slave.qspi);
}

/****************************************************************************
 * Name: aurix_qspi_cs_interrupt
 ****************************************************************************/

static int aurix_qspi_cs_interrupt(int irq, void *context, void *arg)
{
  struct aurix_qspi_slave_priv_s *priv = arg;
  Ifx_DMA *dmaSFR;
  int len;

  spierr("detect a CS interrupt\n");

#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
  dmaSFR = &MODULE_DMA;
#else
  dmaSFR = (Ifx_DMA *)IfxDma_cfg_indexMap[priv->slave.dma.dmaIndex].module;
#endif

  /* Get actual data length remain still need to be received */

  len = IfxDma_getChannelTransferCount(dmaSFR,
                                       priv->slave.dma.rxDmaChannelId);

  /* Cancel in-process DMA transferring */

  IfxDma_disableChannelTransaction(dmaSFR, priv->slave.dma.txDmaChannelId);
  IfxDma_disableChannelTransaction(dmaSFR, priv->slave.dma.rxDmaChannelId);
  IfxDma_resetChannel(dmaSFR, priv->slave.dma.txDmaChannelId);
  IfxDma_resetChannel(dmaSFR, priv->slave.dma.rxDmaChannelId);
  priv->slave.onTransfer = FALSE;

  aurix_qspi_slave_reset(priv);

  len = (priv->txsend ? priv->txsend : priv->config->bufsize) - len;
  aurix_qspi_slave_transmit_done(priv, len);
  aurix_qspi_slave_receive_done(priv, len);

  return 0;
}

/****************************************************************************
 * Name: aurix_qspi_slave_bind
 ****************************************************************************/

static void aurix_qspi_slave_bind(struct spi_slave_ctrlr_s *ctrlr,
                                  struct spi_slave_dev_s *dev,
                                  enum spi_slave_mode_e mode, int nbits)
{
  struct aurix_qspi_slave_priv_s *priv =
    (struct aurix_qspi_slave_priv_s *)ctrlr;
  IfxQspi_SpiSlave_Config spiSlaveConfig;

  DEBUGASSERT(ctrlr != NULL && dev != NULL && priv->dev == NULL);
  spiinfo("ctrlr=%p dev=%p mode=%d nbits=%d\n", ctrlr, dev, mode, nbits);

  if (mode != SPISLAVE_MODE1)
    {
      spierr("only mode 1 is supported\n");
      return;
    }

  memset(&spiSlaveConfig, 0, sizeof(IfxQspi_SpiSlave_Config));
  IfxQspi_SpiSlave_initModuleConfig(&spiSlaveConfig, priv->config->qspi);

  /* Set mode and datawidth */

  priv->datawidth = nbits;
  spiSlaveConfig.protocol.dataWidth = nbits;
  spiSlaveConfig.protocol.clockPolarity = SpiIf_ClockPolarity_idleLow;
#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
  spiSlaveConfig.protocol.shiftClock =
    SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;

  /* Set the maximum baudrate */

  spiSlaveConfig.base.maximumBaudrate = priv->config->maxbaudrate;

  /* ISR priorities and interrupt target */

  spiSlaveConfig.base.txPriority = IRQ_TO_NDX(priv->config->tx_irq);
  spiSlaveConfig.base.rxPriority = IRQ_TO_NDX(priv->config->rx_irq);
  spiSlaveConfig.base.erPriority = IRQ_TO_NDX(priv->config->err_irq);
  spiSlaveConfig.base.isrProvider = (IfxSrc_Tos)IfxCpu_getCoreId();

#else
  spiSlaveConfig.protocol.shiftClock =
    SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;

  spiSlaveConfig.maximumBaudrate = priv->config->maxbaudrate;

  spiSlaveConfig.txPriority = IRQ_TO_NDX(priv->config->tx_irq);
  spiSlaveConfig.rxPriority = IRQ_TO_NDX(priv->config->rx_irq);
  spiSlaveConfig.erPriority = IRQ_TO_NDX(priv->config->err_irq);
  spiSlaveConfig.isrProvider = (IfxSrc_Tos)IfxCpu_getCoreId();
  spiSlaveConfig.vmId = IfxSrc_VmId_0;
#endif

  /* Set DMA configuration */

  if (priv->config->usedma)
    {
      spiSlaveConfig.dma.txDmaChannelId = priv->config->txdmachannel;
      spiSlaveConfig.dma.rxDmaChannelId = priv->config->rxdmachannel;
      spiSlaveConfig.dma.useDma = 1;
#ifndef CONFIG_ARCH_CHIP_AURIX_TC3XX
      spiSlaveConfig.dma.dmaIndex = priv->config->dmaindex;
#endif
    }

  /* Set SPI slave pins */

  spiSlaveConfig.pins = &priv->config->pins;

  /* Configure SPI slave module according to the configuartion */

  IfxQspi_SpiSlave_initModule(&priv->slave, &spiSlaveConfig);

  /* Enable PT2 interrrup */

  if (priv->config->pt_enable)
    {
      priv->config->qspi->GLOBALCON1.B.PT2EN = 1;
      priv->config->qspi->GLOBALCON1.B.PT2 = 5;
    }

  priv->dev = dev;
  up_enable_irq(priv->config->err_irq);
  if (priv->config->pt_enable)
    {
      up_enable_irq(priv->config->pt_irq);
    }
  else
    {
      up_enable_irq(priv->config->tx_irq);
      up_enable_irq(priv->config->rx_irq);
    }

  /* Prepare to receive data */

#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
  while (IfxQspi_SpiSlave_getStatus(&priv->slave) == SpiIf_Status_busy);
#else
  while (IfxQspi_SpiSlave_getStatus(&priv->slave) == IfxQspi_Status_busy);
#endif
  IfxQspi_SpiSlave_exchange(&priv->slave, priv->txbuf, priv->rxbuf,
                            AURIX_BYTES2WORDS(priv, priv->config->bufsize));

  spiinfo("SPI slave bound to %p\n", dev);

  return;
}

/****************************************************************************
 * Name: aurix_qspi_slave_unbind
 ****************************************************************************/

static void aurix_qspi_slave_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct aurix_qspi_slave_priv_s *priv =
    (struct aurix_qspi_slave_priv_s *)ctrlr;

  up_disable_irq(priv->config->tx_irq);
  up_disable_irq(priv->config->rx_irq);
  up_disable_irq(priv->config->err_irq);
  if (priv->config->pt_enable)
    {
      up_disable_irq(priv->config->pt_irq);
    }

  priv->dev = NULL;
}

/****************************************************************************
 * Name: aurix_qspi_slave_qfull
 ****************************************************************************/

static bool aurix_qspi_slave_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct aurix_qspi_slave_priv_s *priv =
    (struct aurix_qspi_slave_priv_s *)ctrlr;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->txpos == priv->config->bufsize)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return true;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
  return false;
}

/****************************************************************************
 * Name: aurix_qspi_slave_enqueue
 ****************************************************************************/

static int aurix_qspi_slave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                                   const void *data, size_t nwords)
{
  struct aurix_qspi_slave_priv_s *priv =
    (struct aurix_qspi_slave_priv_s *)ctrlr;
  irqstate_t flags;
  size_t left;

  spiinfo("enqueue nwords %zu\n", nwords);
  flags = spin_lock_irqsave(&priv->lock);
  left = priv->config->bufsize - priv->txpos;
  if (left == 0)
    {
      spierr("there is no space left in the txbuf\n");
      spin_unlock_irqrestore(&priv->lock, flags);
      return 0;
    }

  nwords = nwords > AURIX_BYTES2WORDS(priv, left) ?
           AURIX_BYTES2WORDS(priv, left) : nwords;
  memcpy(priv->txbuf + priv->txpos, data, AURIX_WORDS2BYTES(priv, nwords));
  priv->txpos += AURIX_WORDS2BYTES(priv, nwords);
  if (!priv->slave.onTransfer)
    {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC3XX
      while (IfxQspi_SpiSlave_getStatus(&priv->slave) == SpiIf_Status_busy);
#else
      while (IfxQspi_SpiSlave_getStatus(&priv->slave) ==
             IfxQspi_Status_busy);
#endif
      IfxQspi_SpiSlave_exchange(&priv->slave, priv->txbuf,
                                priv->rxbuf,
                                AURIX_BYTES2WORDS(priv, priv->txpos));
      priv->txsend = priv->txpos;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return nwords;
}

/****************************************************************************
 * Name: aurix_qspi_slave_qpoll
 ****************************************************************************/

static size_t aurix_qspi_slave_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  /* Currently suppose the rx buffer can be received totally through
   * the SPIS_DEV_RECEIVE.
   */

  return 0;
}

/****************************************************************************
 * Name: aurix_qspi_slave_qflush
 ****************************************************************************/

static void aurix_qspi_slave_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct aurix_qspi_slave_priv_s *priv =
    (struct aurix_qspi_slave_priv_s *)ctrlr;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  priv->txpos = 0;
  spin_unlock_irqrestore(&priv->lock, flags);

  return;
}

/****************************************************************************
 * Name: aurix_qspi_slave_init
 ****************************************************************************/

int aurix_qspi_slave_init(struct aurix_qspi_slave_priv_s *priv,
  const struct aurix_qspi_slave_config_s *config)
{
  int ret;

  priv->txbuf = kmm_memalign(32, 2 * config->bufsize);
  if (priv->txbuf == NULL)
    {
      spierr("kmm_memalign failed\n");
      return -ENOMEM;
    }

  /* Attach to the irq handler */

#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
  ret = irq_attach_wqueue(config->err_irq, NULL, aurix_qspi_slave_error,
                          priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
  ret = irq_attach(config->err_irq, aurix_qspi_slave_error, priv);
#endif
  if (ret < 0)
    {
      spierr("irq attach interrupt %d failed!\n", config->err_irq);
      kmm_free(priv->txbuf);
      return ret;
    }

  if (config->pt_enable)
    {
#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config->pt_irq, NULL, aurix_qspi_cs_interrupt,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config->pt_irq, aurix_qspi_cs_interrupt, priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach interrupt %d failed\n", config->pt_irq);
          irq_detach(config->err_irq);
          kmm_free(priv->txbuf);
          return ret;
        }
    }
  else
    {
#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config->rx_irq, NULL, aurix_qspi_slave_receive,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config->rx_irq, aurix_qspi_slave_receive, priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach interrupt %d failed!\n", config->rx_irq);
          irq_detach(config->err_irq);
          kmm_free(priv->txbuf);
          return ret;
        }

#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config->tx_irq, NULL,
                              aurix_qspi_slave_transmit,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config->tx_irq, aurix_qspi_slave_transmit, priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach interrupt %d failed!\n", config->tx_irq);
          irq_detach(config->rx_irq);
          irq_detach(config->err_irq);
          kmm_free(priv->txbuf);
          return ret;
        }
    }

  priv->rxbuf = priv->txbuf + config->bufsize;
  priv->config = config;
  priv->ctrlr.ops = &aurix_qspi_slave_ops;
  spin_lock_init(&priv->lock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_qspi_slave_initialize
 *
 * Description:
 *   Initialize the qspi slave controllers according to the provided
 *   configuration.
 *
 * Input Parameters:
 *   ctrlr  - pointer to the controller structure
 *   config - pointer to the configuration structure for each controller
 *   count  - number of controllers to be initialized
 *
 * Returned Value:
 *   Zero OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int aurix_qspi_slave_initialize(struct spi_slave_ctrlr_s **ctrlr,
  const struct aurix_qspi_slave_config_s *config, size_t count)
{
  struct aurix_qspi_slave_priv_s *priv;
  int ret;
  int i;

  for (i = 0; i < count; i++)
    {
      if (config[i].qspi == NULL)
        {
          continue;
        }

      /* Currently, only DMA case is considered to enable PT interrupt */

      if (config[i].pt_enable && !config[i].usedma)
        {
          spierr("%d PT irq is not supported when dma is not enabled\n", i);
          continue;
        }

      priv = kmm_zalloc(sizeof(struct aurix_qspi_slave_priv_s));
      if (priv == NULL)
        {
          spierr("slave %d kmm_zalloc failed\n", i);
          return -ENOMEM;
        }

      ret = aurix_qspi_slave_init(priv, &config[i]);
      if (ret < 0)
        {
          spierr("slave %d aurix_qspi_slave_init failed ret %d\n", i, ret);
          kmm_free(priv);
          return ret;
        }

      ctrlr[i] = &priv->ctrlr;

#ifdef CONFIG_SPI_SLAVE_DRIVER
      ret = spi_slave_register(&priv->ctrlr, i);
      if (ret < 0)
        {
          spierr("slave %d spi_slave_register failed ret %d\n", i, ret);
        }
#endif

      spiinfo("slave %d initialize success\n", i);
    }

  return 0;
}
