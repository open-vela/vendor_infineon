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

#include <debug.h>
#include <errno.h>
#include <stddef.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi_transfer.h>
#include <arch/chip/chip.h>

#include "IfxPort.h"

#include "aurix_spi.h"
#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_spi_priv_s
{
  struct spi_dev_s          dev;
  struct aurix_spi_config_s *cfg;
  IfxAsclin_Spi             *ascspi;
  sem_t                     sem;
  mutex_t                   lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_spi_lock(struct spi_dev_s *dev, bool lock);
static void aurix_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected);
static uint32_t aurix_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency);
static void aurix_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void aurix_spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t aurix_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void aurix_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void aurix_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                               int nwords);
static void aurix_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                size_t nwords);
#endif
static int aurix_spi_initialize(struct spi_dev_s *dev,
                                const struct aurix_spi_config_s *config);
static uint8_t aurix_spi_getstatus(struct spi_dev_s *dev, uint32_t devid);

IfxAsclin_Spi g_aurix_spi_handler;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_aurix_spi_ops =
{
  .lock             = aurix_spi_lock,
  .select           = aurix_spi_select,
  .setfrequency     = aurix_spi_setfrequency,
  .setmode          = aurix_spi_setmode,
  .setbits          = aurix_spi_setbits,
  .status           = aurix_spi_getstatus,
  .send             = aurix_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = aurix_spi_exchange,
#else
  .sndblock         = aurix_spi_sndblock,
  .recvblock        = aurix_spi_recvblock,
#endif
  .registercallback = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_spi_lock
 ****************************************************************************/

static int aurix_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: aurix_spi_select
 ****************************************************************************/

static void aurix_spi_select(struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;
  IfxAsclin_Slso_Out *slso = NULL;

  switch (devid)
    {
#ifdef CONFIG_AURIX_SPI_CS0
      case 0:
        slso = priv->cfg->slso_0;
        break;
#endif
#ifdef CONFIG_AURIX_SPI_CS1
      case 1:
        slso = priv->cfg->slso_1;
        break;
#endif
#ifdef CONFIG_AURIX_SPI_CS2
      case 2:
        slso = priv->cfg->slso_2;
        break;
#endif
      default:
        spierr("unsupported SPI device: %ld\n", devid);
        return;
    }

  if (selected)
    {
      IfxPort_setPinHigh(slso->pin.port, slso->pin.pinIndex);
    }
  else
    {
      IfxPort_setPinLow(slso->pin.port, slso->pin.pinIndex);
    }
}

/****************************************************************************
 * Name: aurix_spi_setfrequency
 ****************************************************************************/

static uint32_t aurix_spi_setfrequency(struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;
  IfxAsclin_SamplePointPosition samplePointPosition;

  samplePointPosition = (IfxAsclin_SamplePointPosition)
    ((priv->cfg->config->baudrate.oversampling + 1) / 2);

  /* baudrate generation */

  IfxAsclin_setBitTiming(priv->ascspi->asclin, frequency,
                         priv->cfg->config->baudrate.oversampling,
                         samplePointPosition,
                         priv->cfg->config->bitSampling.medianFilter);

  return IfxAsclin_getShiftFrequency(priv->ascspi->asclin);
}

/****************************************************************************
 * Name: aurix_spi_setmode
 ****************************************************************************/

static void aurix_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;
  IfxAsclin_ClockPolarity cpol;
  IfxAsclin_SlavePolarity spol;

  cpol = ((mode >> 1) & 1) == 0 ? IfxAsclin_ClockPolarity_idleLow:
                                  IfxAsclin_ClockPolarity_idleHigh;
  spol = (mode & 1) == 0 ? IfxAsclin_SlavePolarity_idleLow:
                           IfxAsclin_SlavePolarity_idlehigh;

  IfxAsclin_setClockPolarity(priv->ascspi->asclin, cpol);
  IfxAsclin_setSlavePolarity(priv->ascspi->asclin, spol);
}

/****************************************************************************
 * Name: aurix_spi_setbits
 ****************************************************************************/

static void aurix_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;

  if (nbits <= 8)
    {
      IfxAsclin_setDataLength(priv->ascspi->asclin,
                              IfxAsclin_DataLength_8);

      /* setting Tx/Rx FIFO inlet width to 1 byte */

      IfxAsclin_setTxFifoInletWidth(priv->ascspi->asclin,
                                    IfxAsclin_TxFifoInletWidth_1);
      IfxAsclin_setRxFifoOutletWidth(priv->ascspi->asclin,
                                     IfxAsclin_RxFifoOutletWidth_1);

      /* echo the data width to module handle */

      priv->ascspi->dataWidth = 1;
    }
  else
    {
      IfxAsclin_setDataLength(priv->ascspi->asclin,
                              IfxAsclin_DataLength_16);

      /* setting Tx/Rx FIFO inlet width to 2 bytes */

      IfxAsclin_setTxFifoInletWidth(priv->ascspi->asclin,
                                    IfxAsclin_TxFifoInletWidth_2);
      IfxAsclin_setRxFifoOutletWidth(priv->ascspi->asclin,
                                     IfxAsclin_RxFifoOutletWidth_2);

      /* echo the data width to module handle */

      priv->ascspi->dataWidth = 2;
    }
}

/****************************************************************************
 * Name: aurix_spi_send
 ****************************************************************************/

static uint32_t aurix_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;

  aurix_spi_exchange(dev, &wd, NULL, (sizeof(uint32_t) / sizeof(uint8_t))
                     / priv->ascspi->dataWidth);
  return OK;
}

/****************************************************************************
 * Name: aurix_spi_isrReceive
 ****************************************************************************/

static int aurix_spi_isrReceive(int irq, void *context, void *arg)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)arg;

  IfxAsclin_Spi_isrReceive(priv->ascspi);

  IfxAsclin_Spi_isrTransmit(priv->ascspi);

  if (priv->ascspi->rxJob.pending == 0)
    {
      nxsem_post(&priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_spi_isrTransmit
 ****************************************************************************/

static int aurix_spi_isrTransmit(int irq, void *context, void *arg)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)arg;

  IfxAsclin_Spi_isrTransmit(priv->ascspi);

  if (priv->ascspi->txJob.pending == 0)
    {
      nxsem_post(&priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_spi_isrerror
 ****************************************************************************/

static int aurix_spi_isrerror(int irq, void *context, void *arg)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)arg;

  IfxAsclin_Spi_isrError(priv->ascspi);
  spierr("FrameErrorFlag: %d, RxFifoOverflowFlag: %d, \
         RxFifoUnderflowFlag: %d, TxFifoOverflowFlag %d\n",
         priv->ascspi->errorFlags.frameError,
         priv->ascspi->errorFlags.rxFifoOverflow,
         priv->ascspi->errorFlags.rxFifoUnderflow,
         priv->ascspi->errorFlags.txFifoOverflow);
  return OK;
}

/****************************************************************************
 * Name: aurix_spi_exchange
 ****************************************************************************/

static void aurix_spi_exchange(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;

  if (rxbuffer != NULL)
    {
      up_enable_irq(priv->cfg->rx_irq);
      IfxAsclin_enableRxFifoFillLevelFlag(priv->ascspi->asclin, TRUE);
    }
  else
    {
      up_enable_irq(priv->cfg->tx_irq);
      IfxAsclin_enableTxFifoFillLevelFlag(priv->ascspi->asclin, TRUE);
    }

  IfxAsclin_Spi_exchange(priv->ascspi, (void *)txbuffer, rxbuffer, nwords);

  nxsem_wait_uninterruptible(&priv->sem);
}

#ifndef CONFIG_SPI_EXCHANGE
/****************************************************************************
 * Name: aurix_spi_sndblock
 ****************************************************************************/

static void aurix_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                               int nwords)
{
  aurix_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: aurix_spi_recvblock
 ****************************************************************************/

static void aurix_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                size_t nwords)
{
  aurix_spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: aurix_spi_getstatus
 ****************************************************************************/

static uint8_t aurix_spi_getstatus(struct spi_dev_s *dev, uint32_t devid)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;

  return IfxAsclin_Spi_getStatus(priv->ascspi);
}

/****************************************************************************
 * Name: aurix_spi_initialize
 ****************************************************************************/

static int aurix_spi_initialize(struct spi_dev_s *dev,
                                const struct aurix_spi_config_s *config)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;
  IfxAsclin_Spi_Config spiMasterConfig;

  /* Initialize one instance of IfxAsclin_Spi_Config with default values */

  IfxAsclin_Spi_initModuleConfig(&spiMasterConfig, config->asclin);

  /* Default Values for fifo Control */

  spiMasterConfig.fifo.txFifoInterruptLevel =
    IfxAsclin_TxFifoInterruptLevel_15;
  spiMasterConfig.fifo.rxFifoInterruptLevel =
    IfxAsclin_RxFifoInterruptLevel_16;

  /* ISR priorities and service provider */

  spiMasterConfig.interrupt.rxPriority = IRQ_TO_NDX(priv->cfg->err_irq);
  spiMasterConfig.interrupt.typeOfService = IfxSrc_Tos_cpu0;

  /* Pin configuration */

  spiMasterConfig.pins = &config->pins;

  memcpy(priv->cfg->config, &spiMasterConfig, sizeof(IfxAsclin_Spi_Config));

  /* Initialize module with the above parameters */

  IfxAsclin_Spi_initModule(&g_aurix_spi_handler, &spiMasterConfig);
  return OK;
}

/****************************************************************************
 * Name: aurix_spibus_initialize
 ****************************************************************************/

static struct spi_dev_s *
aurix_spibus_initialize(struct aurix_spi_config_s *config)
{
  struct aurix_spi_priv_s *spi_priv;
  IfxAsclin_Spi_Config *ifxcfg;
  int ret;

  ifxcfg = kmm_zalloc(sizeof(IfxAsclin_Spi_Config));
  DEBUGASSERT(ifxcfg != NULL);

  spi_priv = kmm_zalloc(sizeof(struct aurix_spi_priv_s));
  DEBUGASSERT(spi_priv != NULL);

  spi_priv->dev.ops = &g_aurix_spi_ops;
  spi_priv->ascspi = &g_aurix_spi_handler;
  config->config = ifxcfg;
  spi_priv->cfg = config;

  nxsem_init(&spi_priv->sem, 0, 0);
  nxmutex_init(&spi_priv->lock);
  aurix_spi_initialize(&spi_priv->dev, config);
  ret = irq_attach(spi_priv->cfg->rx_irq, aurix_spi_isrReceive,
                   &spi_priv->dev);
  if (ret < 0)
    {
      spierr("irq attach interrupt %d failed!\n", spi_priv->cfg->rx_irq);
      goto err;
    }

  ret = irq_attach(spi_priv->cfg->err_irq, aurix_spi_isrerror,
                   &spi_priv->dev);
  if (ret < 0)
    {
      spierr("irq attach interrupt %d failed!\n", spi_priv->cfg->err_irq);
      goto err;
    }

  ret = irq_attach(spi_priv->cfg->tx_irq, aurix_spi_isrTransmit,
                   &spi_priv->dev);
  if (ret < 0)
    {
      spierr("irq attach interrupt %d failed!\n", spi_priv->cfg->tx_irq);
      goto err;
    }

  up_enable_irq(spi_priv->cfg->err_irq);

  /* Init cs pins */

#ifdef CONFIG_AURIX_SPI_CS0
  IfxAsclin_initSlsoPin(spi_priv->cfg->slso_0, spi_priv->cfg->slso_mode,
                        spi_priv->cfg->pinDriver);
#endif
#ifdef CONFIG_AURIX_SPI_CS1
  IfxAsclin_initSlsoPin(spi_priv->cfg->slso_1, spi_priv->cfg->slso_mode,
                        spi_priv->cfg->pinDriver);
#endif
#ifdef CONFIG_AURIX_SPI_CS2
  IfxAsclin_initSlsoPin(spi_priv->cfg->slso_2, spi_priv->cfg->slso_mode,
                        spi_priv->cfg->pinDriver);
#endif

  return &spi_priv->dev;

err:
  aurix_spi_deinit(&spi_priv->dev);
  kmm_free(spi_priv);
  kmm_free(ifxcfg);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_spi_deinit
 ****************************************************************************/

void aurix_spi_deinit(struct spi_dev_s *dev)
{
  struct aurix_spi_priv_s *priv = (struct aurix_spi_priv_s *)dev;

  IfxAsclin_Spi_disableModule(priv->ascspi);

  /* Disable interrupt */

  up_disable_irq(priv->cfg->tx_irq);
  up_disable_irq(priv->cfg->rx_irq);
  up_disable_irq(priv->cfg->err_irq);
}

/****************************************************************************
 * Name: aurix_all_spi_initialize
 *
 * Description:
 *   Initialize all spi deivce for aurix.
 *
 ****************************************************************************/

int aurix_all_spi_initialize(struct spi_dev_s **dev,
                             struct aurix_spi_config_s *config,
                             size_t count)
{
  size_t i;

  for (i = 0; i < count && dev[i] != DEV_END; i++)
    {
      dev[i] = aurix_spibus_initialize(&config[i]);
    }

  return OK;
}
