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
#include "aurix_qspi_tc3.h"
#include "tricore_internal.h"
#include "If/SpiIf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_CACHE_DELAY -1
#define SPI_CACHE_MODE -1
#define SPI_CACHE_NBIT 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_qspi_channel_s
{
  uint32_t startdelay;
  uint32_t stopdelay;
  uint32_t csdelay;
  uint32_t ifdelay;
  float baudrate;
  bool status;
  int mode;
  IfxQspi_SpiMaster_Channel channel;
};

struct aurix_qspi_cache_s
{
  int mode;
  int frequency;
  int startdelay;
  int stopdelay;
  int csdelay;
  int ifdelay;
  int nbits;
};

struct aurix_qspi_priv_s
{
  struct spi_dev_s  dev;
  IfxQspi_SpiMaster qspi;
  mutex_t           lock;
  sem_t             sem;
  struct aurix_qspi_channel_s *active_channel;
  struct aurix_qspi_channel_s spichannel[16];
  const IfxQspi_SpiMaster_Output *cs_pin;
  struct aurix_qspi_cache_s cache;
  const int *cs_active;
  int cs_num;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_qspi_lock(struct spi_dev_s *dev, bool lock);
static void aurix_qspi_select(struct spi_dev_s *dev, uint32_t devid,
                              bool selected);
static uint32_t aurix_qspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int aurix_qspi_setdelay(FAR struct spi_dev_s *dev,
                               uint32_t startdelay, uint32_t stopdelay,
                               uint32_t csdelay, uint32_t ifdelay);
#endif
static void aurix_qspi_setmode(struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void aurix_qspi_setbits(struct spi_dev_s *dev, int nbits);
static uint8_t aurix_qspi_getstatus(struct spi_dev_s *dev, uint32_t devid);
static uint32_t aurix_qspi_send(struct spi_dev_s *dev, uint32_t wd);
static void aurix_qspi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                                void *rxbuffer, size_t nwords);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_qspi_ops =
{
  .lock             = aurix_qspi_lock,
  .select           = aurix_qspi_select,
  .setfrequency     = aurix_qspi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay         = aurix_qspi_setdelay,
#endif
  .setmode          = aurix_qspi_setmode,
  .setbits          = aurix_qspi_setbits,
  .status           = aurix_qspi_getstatus,
  .send             = aurix_qspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = aurix_qspi_exchange,
#else
  .sndblock         = NULL,
  .recvblock        = NULL,
#endif
  .registercallback = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_qspi_lock
 ****************************************************************************/

static int aurix_qspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
      priv->active_channel = NULL;
      priv->cache.frequency = 0;
      priv->cache.startdelay = SPI_CACHE_DELAY;
      priv->cache.stopdelay  = SPI_CACHE_DELAY;
      priv->cache.csdelay    = SPI_CACHE_DELAY;
      priv->cache.ifdelay    = SPI_CACHE_DELAY;
      priv->cache.nbits      = SPI_CACHE_NBIT;
      priv->cache.mode       = SPI_CACHE_MODE;
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: aurix_qspi_select
 ****************************************************************************/

static void aurix_qspi_select(struct spi_dev_s *dev, uint32_t devid,
                              bool selected)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  const IfxQspi_SpiMaster_Output *slso;
  uint32 mask;
  uint32 oen;
  uint32 aol;
  int ret;

  if (devid >= priv->cs_num)
    {
      spierr("qspi devid exceed\n");
      return;
    }

  if (!selected)
    {
      return;
    }

  slso = &priv->cs_pin[devid];
  if (slso->pin == NULL_PTR)
    {
      spierr("qspi slso is error\n");

      /* priv->spichannel[devid].channel.useSlso = FALSE; */

      return;
    }

  if (!priv->spichannel[devid].status)
    {
      priv->spichannel[devid].channel.slso = slso->pin->pin;
      priv->spichannel[devid].channel.channelId =
                        (IfxQspi_ChannelId)slso->pin->slsoNr;
      priv->spichannel[devid].channel.bacon.B.CS =
                        priv->spichannel[devid].channel.channelId;
      priv->qspi.qspi->GLOBALCON.B.LB = 0;

      mask = 1U << priv->spichannel[devid].channel.channelId;
      oen  = mask << 16;
      aol  = (((priv->cs_active[devid] == Ifx_ActiveState_low) ? 0 : 1)
              << priv->spichannel[devid].channel.channelId);
      __ldmst(&priv->qspi.qspi->SSOC.U, (mask | (mask << 16)), aol | oen);
      IfxQspi_initSlso(slso->pin, slso->mode,
                      slso->driver, slso->pin->select);
      priv->spichannel[devid].status = true;
    }

  priv->active_channel = &priv->spichannel[devid];

  /* set frequency */

  if (priv->cache.frequency != 0)
    {
      ret = aurix_qspi_setfrequency(dev, priv->cache.frequency);
      if (ret != OK)
        {
          spierr("qspi setfrequency failed ret = %d\n", ret);
          return;
        }
    }

  /* set delay */

#ifdef CONFIG_SPI_DELAY_CONTROL
  if (priv->cache.startdelay != SPI_CACHE_DELAY ||
      priv->cache.stopdelay  != SPI_CACHE_DELAY ||
      priv->cache.csdelay    != SPI_CACHE_DELAY ||
      priv->cache.ifdelay    != SPI_CACHE_DELAY)
    {
      aurix_qspi_setdelay(dev, priv->cache.startdelay,
                          priv->cache.stopdelay, priv->cache.csdelay,
                          priv->cache.ifdelay);
    }
#endif

  /* set mode */

  if (priv->cache.mode >= SPIDEV_MODE0 && priv->cache.mode <= SPIDEV_MODE3)
    {
      aurix_qspi_setmode(dev, priv->cache.mode);
    }

  /* set bits */

  if (priv->cache.nbits != SPI_CACHE_NBIT)
    {
      aurix_qspi_setbits(dev, priv->cache.nbits);
    }
}

/****************************************************************************
 * Name: aurix_qspi_setfrequency
 ****************************************************************************/

static uint32_t aurix_qspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  IfxQspi_SpiMaster_Channel *channel;
  IfxQspi_DelayParameters params;
  int ret = OK;

  spiinfo("aurix_qspi_setfrequency: %ld\n", frequency);

  if (priv->active_channel == NULL)
    {
      priv->cache.frequency = frequency;
      return ret;
    }

  channel = &priv->active_channel->channel;
  if (priv->active_channel->baudrate != frequency)
    {
      while (IfxQspi_SpiMaster_getStatus(channel)
                                          == SpiIf_Status_busy);
      ret = IfxQspi_SpiMaster_setChannelBaudrate(channel, frequency);
      if (ret == OK)
        {
          priv->active_channel->baudrate = frequency;
        }
    }
  else
    {
      return OK;
    }

  /* set defalut leadingDelay and trailingDelay
   * time_clock = 1000000000 / (float)IfxScuCcu_getQspiFrequency(); = 5ns
   * leadingDelay = time_clock * 4 * 4 * (params.leadingPrescalar + 1)
   * leadingDelay = 180ns
   * trailingDelay = time_clock * 1 * (params.trailingPrescalar + 1)
   * trailingDelay = 5ns
   */

  params.leadingDelay = 2;
  params.leadingPrescalar = 1;
  params.trailingDelay = 0;
  params.trailingPrescalar = 0;
  params.idleDelay = 0;
  params.idlePrescalar = 0;
  IfxQspi_SpiMaster_updateDelayParameters(channel, &params);

  return ret;
}

/****************************************************************************
 * Name: aurix_qspi_setdelay
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL

static void get_parameters(int time_clock, int delay, int arg[2])
{
  int multiple;
  int time;
  int i;
  int j;

  arg[0] = IfxQspi_DelayLength_8;
  arg[1] = IfxQspi_DelayPrescalar_16384;
  for (i = IfxQspi_DelayLength_1; i <= IfxQspi_DelayLength_8; i++)
    {
      if (i == 0)
        {
          multiple = 1;
        }
      else
        {
          multiple = multiple * 4;
        }

      for (j = IfxQspi_DelayPrescalar_1; j <= IfxQspi_DelayPrescalar_16384;
           j++)
        {
          time = time_clock * multiple * (j + 1);
          if (time >= delay)
            {
              arg[0] = i;
              arg[1] = j;
              return;
            }
        }
    }
}

static int aurix_qspi_setdelay(FAR struct spi_dev_s *dev,
                               uint32_t startdelay, uint32_t stopdelay,
                               uint32_t csdelay, uint32_t ifdelay)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  IfxQspi_DelayParameters params;
  uint32_t time_clock;
  int arg[2];

  if (priv->active_channel == NULL)
    {
      priv->cache.startdelay = startdelay;
      priv->cache.stopdelay  = stopdelay;
      priv->cache.csdelay    = csdelay;
      priv->cache.ifdelay    = ifdelay;
      return 0;
    }

  if (startdelay == priv->active_channel->startdelay &&
      stopdelay  == priv->active_channel->stopdelay  &&
      csdelay    == priv->active_channel->csdelay    &&
      ifdelay    == priv->active_channel->ifdelay)
    {
      return OK;
    }

  priv->active_channel->startdelay = startdelay;
  priv->active_channel->stopdelay  = stopdelay;
  priv->active_channel->csdelay    = csdelay;
  priv->active_channel->ifdelay    = ifdelay;

  time_clock = 1000000000 / (float)IfxScuCcu_getQspiFrequency();

  if (startdelay <= time_clock)
    {
      params.leadingDelay = 0;
      params.leadingPrescalar = 0;
    }
  else
    {
        get_parameters(time_clock, startdelay, arg);
        params.leadingDelay = arg[1];
        params.leadingPrescalar = arg[0];
    }

  if (stopdelay <= time_clock)
    {
      params.trailingDelay = 0;
      params.trailingPrescalar = 0;
    }
  else
    {
      get_parameters(time_clock, stopdelay, arg);
      params.trailingDelay = arg[1];
      params.trailingPrescalar = arg[0];
    }

  params.idleDelay = 0;
  params.idlePrescalar = 0;

  IfxQspi_SpiMaster_updateDelayParameters(&priv->active_channel->channel,
                                          &params);
  return 0;
}
#endif

/****************************************************************************
 * Name: aurix_qspi_setmode
 ****************************************************************************/

static void aurix_qspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  uint8 cs;
  Ifx_QSPI *qspi;

  spiinfo("aurix_qspi_setmode: mode = %d\n", mode);

  if (priv->active_channel == NULL)
    {
      priv->cache.mode = mode;
      return;
    }

  priv->active_channel->mode = mode;

  /* Configuration extensions for channels 0 to 15.
   * Register cs defines several timing characteristics for two channels.
   * so We need to update the mode every time we communicate.
   */

  cs = priv->active_channel->channel.channelId % 8;
  qspi = priv->qspi.qspi;
  qspi->ECON[cs].B.CPOL  = ((mode >> 1) & 1) == 0 ? 0: 1;
  qspi->ECON[cs].B.CPH = (mode & 1) == 0 ? 0 : 1;
}

/****************************************************************************
 * Name: aurix_qspi_setbits
 ****************************************************************************/

static void aurix_qspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;

  spiinfo("aurix_qspi_setbits: nbits = %d\n", nbits);

  if (priv->active_channel == NULL)
    {
      priv->cache.nbits = nbits;
      return;
    }

  if (priv->active_channel->channel.dataWidth == nbits)
    {
      return;
    }

  priv->active_channel->channel.dataWidth = nbits;

  if (nbits <= 8)
    {
      priv->active_channel->channel.dataWidth = 8;
    }
  else if (nbits <= 16)
    {
      priv->active_channel->channel.dataWidth = 16;
    }
  else
    {
      priv->active_channel->channel.dataWidth = 32;
    }

  priv->active_channel->channel.bacon.B.DL =
                               priv->active_channel->channel.dataWidth - 1;
}

/****************************************************************************
 * Name: aurix_qspi_getstatus
 ****************************************************************************/

static uint8_t aurix_qspi_getstatus(struct spi_dev_s *dev, uint32_t devid)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  return IfxQspi_SpiMaster_getStatus(&priv->active_channel->channel);
}

/****************************************************************************
 * Name: aurix_qspi_send
 ****************************************************************************/

static uint32_t aurix_qspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  return IfxQspi_SpiMaster_exchange(&priv->active_channel->channel, &wd,
                                    NULL, sizeof(wd));
}

/****************************************************************************
 * Name: aurix_qspi_exchange
 ****************************************************************************/

static void aurix_qspi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)dev;
  int ret;

  while (IfxQspi_SpiMaster_getStatus(&priv->active_channel->channel)
                                          == SpiIf_Status_busy);

  IfxQspi_SpiMaster_exchange(&priv->active_channel->channel, txbuffer,
                              rxbuffer, nwords);
  ret = nxsem_wait_uninterruptible(&priv->sem);
  if (ret < 0)
    {
      spierr("qspi nxsem_wait_uninterruptible failed ret = %d\n", ret);
    }
}

/****************************************************************************
 * Name: aurix_qspi_receive
 ****************************************************************************/

static int aurix_qspi_receive(int irq, void *context, void *arg)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)arg;
  IfxQspi_SpiMaster_isrReceive(&priv->qspi);

  if (priv->qspi.base.activeChannel->flags.onTransfer == 0)
    {
      nxsem_post(&priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_qspi_transmit
 ****************************************************************************/

static int aurix_qspi_transmit(int irq, void *context, void *arg)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)arg;
  IfxQspi_SpiMaster_isrTransmit(&priv->qspi);
  return OK;
}

/****************************************************************************
 * Name: aurix_qspi_error
 ****************************************************************************/

static int aurix_qspi_error(int irq, void *context, void *arg)
{
  struct aurix_qspi_priv_s *priv = (struct aurix_qspi_priv_s *)arg;

  IfxQspi_SpiMaster_isrError(&priv->qspi);

  if (priv->qspi.base.activeChannel->flags.onTransfer == 0)
    {
      nxsem_post(&priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_qspi_initialize
 *
 * Description:
 *   Initialize qspi deivce for aurix.
 *
 ****************************************************************************/

int aurix_qspi_initialize(struct spi_dev_s **dev,
                          const struct aurix_qspi_config_s *config,
                          size_t count)
{
  IfxQspi_SpiMaster_ChannelConfig channel_config;
  IfxQspi_SpiMaster_Config qspi_config;
  struct aurix_qspi_priv_s *priv;
  int ret = OK;
  size_t i;
  size_t j;

  for (i = 0; i < count; i++)
    {
      /* Skip if no qspi configured */

      if (config[i].qspi == NULL)
        {
          continue;
        }

      priv = kmm_zalloc(sizeof(struct aurix_qspi_priv_s));
      if (priv == NULL)
        {
          spierr("qspi%ld kmm_zalloc failed\n", i);
          return -ENOMEM;
        }

      /* Initialize priv */

      priv->dev.ops = &g_qspi_ops;
      nxmutex_init(&priv->lock);
      nxsem_init(&priv->sem, 0, 0);

      /* Initialize QSPI module */

      memset(&qspi_config, 0, sizeof(IfxQspi_SpiMaster_Config));
      IfxQspi_SpiMaster_initModuleConfig(&qspi_config, config[i].qspi);

      qspi_config.base.mode = SpiIf_Mode_master;
      qspi_config.pins = &config[i].pins;
      qspi_config.base.isrProvider = IfxSrc_Tos_cpu0;
      qspi_config.base.txPriority = IRQ_TO_NDX(config[i].tx_irq);
      qspi_config.base.rxPriority = IRQ_TO_NDX(config[i].rx_irq);
      qspi_config.base.erPriority = IRQ_TO_NDX(config[i].err_irq);

      IfxQspi_SpiMaster_initModule(&priv->qspi, &qspi_config);

      /* Initialize QSPI channel */

      memset(&channel_config, 0, sizeof(IfxQspi_SpiMaster_ChannelConfig));
      IfxQspi_SpiMaster_initChannelConfig(&channel_config, &priv->qspi);
      priv->cs_pin = config[i].cs;
      priv->cs_num = config[i].cs_num;
      priv->cs_active = config[i].cs_active;
      for (j = 0; j < config[i].cs_num; j++)
        {
          priv->spichannel[j].mode = -1;
          priv->spichannel[j].baudrate = config[i].baudrate;
          channel_config.sls.output = config[i].cs[j];
          channel_config.base.baudrate = config[i].baudrate;
          channel_config.base.mode.dataWidth = config[i].datawidth;
          channel_config.base.mode.autoCS = true;
          IfxQspi_SpiMaster_initChannel(&priv->spichannel[j].channel,
                                        &channel_config);
        }

      /* Initialize QSPI interrupt */

#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config[i].rx_irq, NULL, aurix_qspi_receive,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config[i].rx_irq, aurix_qspi_receive,
                       priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach interrupt %d failed!\n", config[i].rx_irq);
          free(priv);
          return ret;
        }

#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config[i].tx_irq, NULL, aurix_qspi_transmit,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config[i].tx_irq, aurix_qspi_transmit,
                       priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach tx interrupt %d failed!\n", config[i].tx_irq);
          free(priv);
          return ret;
        }

#ifdef CONFIG_AURIX_QSPI_ISR_WQUEUE
      ret = irq_attach_wqueue(config[i].err_irq, NULL, aurix_qspi_error,
                              priv, CONFIG_AURIX_QSPI_ISR_WQUEUE_PRIORITY);
#else
      ret = irq_attach(config[i].err_irq, aurix_qspi_error,
                       priv);
#endif
      if (ret < 0)
        {
          spierr("irq attach err interrupt %d failed!\n", config[i].err_irq);
          free(priv);
          return ret;
        }

      up_enable_irq(config[i].tx_irq);
      up_enable_irq(config[i].rx_irq);
      up_enable_irq(config[i].err_irq);
      dev[config[i].dev_id] = &priv->dev;

      ret = spi_register(dev[config[i].dev_id], config[i].dev_id);
      if (ret < 0)
        {
          spierr("spi%d register failed ret = %d\n", (int)i, ret);
        }
    }

  return ret;
}
