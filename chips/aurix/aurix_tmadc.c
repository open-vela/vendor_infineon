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

#include <arch/barriers.h>
#include <stdio.h>
#include <debug.h>
#include <nuttx/kmalloc.h>

#include "aurix_tmadc.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CHECK_INTERNAL_CHEL_RESULT(x) \
  IfxAdc_Tmadc_isMonitorChannelResultAvailable(x)

#define CLEAR_INTERNAL_CHEL_RES_FLAG(x) \
  IfxAdc_Tmadc_clearMonitorChannelResultFlag(x)

#define TRIGGER_INTERNAL_CHEL(x) \
  IfxAdc_Tmadc_triggerMonitorChannel(x)

#define INIT_INTERNAL_CHEL(x,y) \
  IfxAdc_Tmadc_initMonitorChannel(x,y)

#define READ_INTERNAL_CHEL_RES(x) \
  IfxAdc_Tmadc_readMonitorChannelResult(x)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AURIX_HAVE_ADC_IRQ
static int adc_interrupt(int irq, void *context, void *arg);
static int adc_interrupt_cfg(struct adc_dev_s *dev, bool enable);
#endif

/* ADC module init */

static void init_module(struct aurix_adc_priv_s *priv,
                        const aurix_tmadc_module_config_s *config);

/* ADC channel init */

static IfxAdc_Status init_channels(struct aurix_adc_priv_s *priv);

/* ADC related fuction */

static int  adc_enable(struct aurix_adc_priv_s *priv, bool enable);

/* ADC Driver Methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_module
 *
 * Description:
 *   Get module config and initialize the module
 *
 ****************************************************************************/

static void init_module(struct aurix_adc_priv_s *priv,
                        const aurix_tmadc_module_config_s *config)
{
  uint8_t i;
  uint8_t len = config->module_config.srvReqCfg->numServReqNodes;
  bool use_direct_priority = false;

  IfxAdc_Tmadc_Config temp_config = config->module_config;
  for (i = 0; i < len; i++)
    {
      #ifdef CONFIG_ADC_USE_DMA
        use_direct_priority = (config->irq_nums[i] < ADC_IQR_MIN);
      #endif

      if (use_direct_priority)
        {
          temp_config.srvReqCfg->intConfig[i]->priority =
            config->irq_nums[i];
        }
      else
        {
          temp_config.srvReqCfg->intConfig[i]->priority =
            IRQ_TO_NDX(config->irq_nums[i]);
        }
    }

  IfxAdc_Tmadc_initModule(&priv->adc_handle, &temp_config);
}

/****************************************************************************
 * Name: init_channels
 *
 * Description:
 *   Get each channel config and initialize channels
 *
 ****************************************************************************/

static IfxAdc_Status init_channels(struct aurix_adc_priv_s *priv)
{
  /* Create channel configuration */

  IfxAdc_Status ret = IfxAdc_Status_success;
  uint8_t ch_id = 0;

  /* Initialize the channel */

  for (uint8_t i = 0; i < priv->nchannels; i++)
    {
      ch_id = priv->config->g_adc_channellist[i]->id;

      IfxAdc_Tmadc_initChannel(&priv->g_adc_channel[ch_id],
                               priv->config->g_adc_channellist[i]);
    }

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL

  for (uint8_t i = 0; i < priv->nchannels_monitor; i++)
    {
      ch_id = priv->config->adc_monitor_channellist[i]->id;
      INIT_INTERNAL_CHEL(&priv->g_adc_monitor_channel[ch_id],
                         priv->config->adc_monitor_channellist[i]);
    }

#endif
  return ret;
}

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables or disables the specified ADC peripheral.
 *
 ****************************************************************************/

static int adc_enable(struct aurix_adc_priv_s *priv, bool enable)
{
  DEBUGASSERT(priv != NULL);
  int ret = OK;

  if (enable)
    {
      /* Enable Adc channels and instance */

      IfxAdc_enableTmadcModule(priv->module_id);
    }
  else
    {
      IfxAdc_disableTmadcModule(priv->module_id);
    }

  return ret;
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.
 *   This is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;

  ainfo("instance: %d\n", priv->module_id);

  /* Only if this is the first initialzied ADC instance in the ADC block */

  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      aerr("ERROR: Failed to lock mutex\n");
      return;
    }

  /* Do nothing if ADC instance is currently in use */

  if (priv->cmn->refcount > 0)
    {
      goto out;
    }

  if (priv->cmn->refcount == 0)
    {
      /* Enable ADC reset state */

      up_disable_irq(priv->irq);
    }

out:
  nxmutex_unlock(&priv->cmn->lock);
  return;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;

  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      aerr("ERROR: Failed to lock mutex\n");
      return;
    }

  /* Decrement count only when ADC device is in use */

  if (priv->cmn->refcount > 0)
    {
      priv->cmn->refcount -= 1;
    }

  /* Shutdown the ADC device only when not in use */

  if (priv->cmn->refcount > 0)
    {
      goto out;
    }

  up_disable_irq(priv->irq);

out:
  nxmutex_unlock(&priv->cmn->lock);
  return;
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;
  int ret;

  ret = nxmutex_lock(&priv->cmn->lock);

  if (ret < 0)
    {
      return ret;
    }

  /* Do nothing when the ADC device is already set up */

  if (priv->cmn->refcount > 0)
    {
      nxmutex_unlock(&priv->cmn->lock);
      return OK;
    }

  /* Make sure that the ADC device is in the powered up, reset state */

  if (priv->cmn->refcount == 0)
    {
      /* Enable ADC reset state */

      up_disable_irq(priv->irq);
    }

  init_channels(priv);

  IfxAdc_triggerTmadcChannelSet(priv->group_handle.tmSFR,
                                priv->group_handle.channelset);

  /* The ADC device is ready */

  priv->cmn->refcount += 1;
  nxmutex_unlock(&priv->cmn->lock);
  return ret;
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
#ifdef CONFIG_AURIX_HAVE_ADC_IRQ
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;

  ainfo("instance: %d enable: %d\n", priv->module_id, enable ? 1 : 0);
  (void)adc_interrupt_cfg(dev, enable);

#endif
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
  {
    case ANIOC_TRIGGER:
    {
      if (priv->nchannels > 0)
      {
        IfxAdc_triggerTmadcChannelSet(priv->group_handle.tmSFR,
                                      priv->group_handle.channelset);
      }

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL
      int i   = 0;
      int ch_id = 0;

      for (i = 0; i < priv->nchannels_monitor; i++)
        {
          ch_id = priv->config->adc_monitor_channellist[i]->id;
          TRIGGER_INTERNAL_CHEL(&priv->g_adc_monitor_channel[ch_id]);
        }

#endif
      break;
    }

    case ANIOC_GET_NCHANNELS:
    {
      /* Return the number of configured channels */

      ret = priv->nchannels;
      break;
    }

    case ANIOC_WDOG_UPPER: /* Set watchdog upper threshold */
    {
      break;
    }

    case ANIOC_WDOG_LOWER:
    {
      break;
    }

    default:
    {
      aerr("ERROR: Unknown cmd: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }
  }

  return ret;
}

#ifdef CONFIG_AURIX_HAVE_ADC_IRQ

/****************************************************************************
 * Name: adc_getdata
 *
 * Description:
 *   Get adc data and channel without fifo.
 *
 * Input Parameters:
 *   priv - device structure
 *   data - adc data
 *
 * Returned Value: void.
 *
 ****************************************************************************/
#if defined(CONFIG_AURIX_ADC_USE_NO_FIFO)
static void adc_getdata(struct aurix_adc_priv_s *priv, uint32_t data)
{
  if (!(priv->fifo.fifochbuffer) || !(priv->fifo.fifodatabuffer))
    {
      return;
    }

  priv->fifo.fifodatabuffer[priv->current] = data;
  priv->fifo.fifochbuffer[priv->current]   =
    priv->g_adc_channellist[priv->current].priv_channel.id;

  if (priv->current == priv->nchannels - 1)
    {
      priv->adc_convert_flag = true;
    }

  return;
}
#endif

/****************************************************************************
 * Name: adc_interrupt_cfg
 *
 * Description:
 *   Config adcn interrupt.
 *
 * Input Parameters:
 *
 *   priv  - A reference to the ADC block status
 *   enable - enable or disable adcn interrupt
 *
 * Returned Value: void
 *
 ****************************************************************************/

static int adc_interrupt_cfg(struct adc_dev_s *dev, bool enable)
{
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;
  if (enable)
    {
      ainfo("Enable the ADC interrupt: irq=%lu\n", priv->irq);

      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  return OK;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value: always return OK.
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, void *arg)
{
  UNUSED(irq);
  UNUSED(context);
  struct adc_dev_s *dev = (struct adc_dev_s *) arg;
  struct aurix_adc_priv_s *priv =
    (struct aurix_adc_priv_s *)dev->ad_priv;
  int32_t data;
  uint8_t ch_id = 0;

#ifdef CONFIG_ADC_USE_DMA

  /* reset address */

  void  *address = priv->config->g_adc_channellist[0]->groupCfg->groupResPtr;

  if (address != NULL)
    {
      priv->group_handle.dma.channel.channel->SADR.U =
        (uint32)priv->group_handle.sourceAddress;
      priv->group_handle.dma.channel.channel->DADR.U =
        (uint32)address;

      IfxAdc_Tmadc_updateGroupResultAddress(&(priv->group_handle), address);
    }

  if (priv->call_back != NULL && address != NULL)
    {
      priv->call_back(address, priv);
    }

#endif

  for (priv->current = 0; priv->current < priv->nchannels;
        priv->current++)
    {
      ch_id = priv->config->g_adc_channellist[priv->current]->id;
      if (IfxAdc_Tmadc_isResultAvailable(&priv->g_adc_channel[ch_id]))
        {
          data = IfxAdc_Tmadc_readChannelResult(&priv->g_adc_channel[ch_id]);
          #if defined(CONFIG_AURIX_ADC_USE_NO_FIFO)
          adc_getdata(priv, data);
          #else
          if (priv->cb != NULL)
            {
              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(dev, ch_id, data);
            }

          #endif
          IfxAdc_Tmadc_clearResultFlag(&priv->g_adc_channel[ch_id]);
        }
    }

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL

  for (priv->current = 0; priv->current < priv->nchannels_monitor;
       priv->current++)
    {
      ch_id = priv->config->adc_monitor_channellist[priv->current]->id;
      if (CHECK_INTERNAL_CHEL_RESULT(&priv->g_adc_monitor_channel[ch_id]))
        {
          data = READ_INTERNAL_CHEL_RES(&priv->g_adc_monitor_channel[ch_id]);
          #if defined(CONFIG_AURIX_ADC_USE_NO_FIFO)
          adc_getdata(priv, data);
          #else
          if (priv->cb != NULL)
            {
              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(dev, ch_id + ADC_MAX_SAMPLES, data);
            }
          #endif
        }

      CLEAR_INTERNAL_CHEL_RES_FLAG(&priv->g_adc_monitor_channel[ch_id]);
    }

#endif
  priv->current = 0;

  return OK;
}
#endif

/****************************************************************************
 * Name: init_adc_dev
 *
 * Description:Initialize the adc dev.
 *
 ****************************************************************************/

static struct adc_dev_s *init_adc_dev(struct aurix_tmadc_config_s *config)
{
  uint8_t ch_id = 0;

  struct adc_dev_s *adc_dev = kmm_zalloc(sizeof(struct adc_dev_s));
  if (!adc_dev)
    {
      aerr("ERROR: Failed to allocate adc_dev_s memory\n");
      goto out;
    }

  struct aurix_adc_priv_s *priv =
    kmm_zalloc(sizeof(struct aurix_adc_priv_s));
  if (!priv)
    {
      aerr("ERROR: Failed to allocate aurix_adc_priv_s memory\n");
      goto free_adc_dev;
    }

  priv->fifo.fifosize = config->nchannels + 1 + ADC_MAX_MONITOR_NUM;

  priv->fifo.fifochbuffer = kmm_zalloc((priv->fifo.fifosize) *
    sizeof(uint8_t));

  if (!priv->fifo.fifochbuffer)
    {
      aerr("ERROR: Failed to allocate fifochbuffer memory\n");
      goto free_priv;
    }

  priv->fifo.fifodatabuffer = kmm_zalloc((priv->fifo.fifosize) *
    sizeof(uint32_t));
  if (!priv->fifo.fifodatabuffer)
    {
      aerr("ERROR: Failed to allocate fifodatabuffer memory\n");
      goto free_fifochbuffer;
    }

  priv->cmn = kmm_zalloc(sizeof(struct adccmn_data_s));
  if (!priv->cmn)
    {
      aerr("ERROR: Failed to allocate cmn memory\n");
      goto free_fifodatabuffer;
    }

  priv->config = config;
  priv->irq = config->irq;
  priv->module_id = config->module_id;
  priv->group_id = config->group_id;
  priv->nchannels = config->nchannels;
  priv->tmadc_module_handle = &MODULE_ADC;

  uint8_t i = 0;

  for (i = 0; i < config->nchannels; i++)
    {
        ch_id = config->g_adc_channellist[i]->id;

        priv->g_adc_channel[ch_id].id =
          (IfxAdc_TmadcChannel)ch_id;

        priv->g_adc_channel[ch_id].resultRegNum =
          (IfxAdc_TmadcResultReg)ch_id;
    }

#ifdef CONFIG_AURIX_TMADC_MONITOR_CHANNEL

  uint8_t sar_id = 0;
  priv->nchannels_monitor = config->nchannels_monitor;

  for (i = 0; i < config->nchannels_monitor; i++)
    {
      sar_id = config->g_adc_channellist[0]->core;

      priv->g_adc_monitor_channel[sar_id].id =
        config->adc_monitor_channellist[i]->id;
    }
#endif

  nxmutex_init(&priv->cmn->lock);
  priv->cmn->refcount = 0;

  adc_dev->ad_priv = priv;
  adc_dev->ad_ops = &g_adcops;
  adc_dev->ad_recv.af_data = priv->fifo.fifodatabuffer;
  adc_dev->ad_recv.af_channel = priv->fifo.fifochbuffer;
  adc_dev->ad_recv.af_fifosize = priv->fifo.fifosize;

#ifdef CONFIG_AURIX_ADC_ISR_WQUEUE
  priv->isr = adc_interrupt;
  irq_attach_wqueue(priv->irq, NULL, priv->isr,
                    adc_dev, CONFIG_AURIX_ADC_ISR_WQUEUE_PRIORITY);
#else
  priv->isr = adc_interrupt;
  irq_attach(priv->irq, priv->isr, adc_dev);
#endif
  adc_enable(priv, true);

  void  *group_res_ptr =
    priv->config->g_adc_channellist[0]->groupCfg->groupResPtr;

  IfxAdc_Tmadc_DmaConfig *dma_cfg =
    priv->config->g_adc_channellist[0]->groupCfg->dmaCfg;

  if ((group_res_ptr != NULL_PTR) && (dma_cfg != NULL_PTR))
    {
      dma_cfg->dmaSrvReqCfg->priority =
        IRQ_TO_NDX(dma_cfg->dmaSrvReqCfg->priority);
    }

  IfxAdc_Tmadc_initGroup(&priv->group_handle,
                         config->g_adc_channellist[0]->groupCfg);

  return adc_dev;

free_fifodatabuffer:
  kmm_free(priv->fifo.fifodatabuffer);
free_fifochbuffer:
  kmm_free(priv->fifo.fifochbuffer);
free_priv:
  kmm_free(priv);
free_adc_dev:
  kmm_free(adc_dev);
out:
  aerr("ERROR: Failed to allocate memory\n");
  return NULL;
}

/****************************************************************************
 * Name: aurix_adc_module_config
 *
 * Description:Config the adc module.
 *
 ****************************************************************************/

void aurix_adc_module_config(const aurix_tmadc_module_config_s *config,
                             size_t count)
{
  struct aurix_adc_priv_s *priv =
    kmm_zalloc(sizeof(struct aurix_adc_priv_s));
  if (!priv)
    {
      aerr("ERROR: Failed to allocate aurix_adc_priv_s memory\n");
      return;
    }

  IfxAdc_enableModule(&MODULE_ADC);

  uint8_t i;
  for (i = 0; i < count ; i++)
    {
      init_module(priv, &config[i]);

      IfxAdc_Tmadc_runModule(&priv->adc_handle);
    }

  kmm_free(priv);

  UP_DMB();
  shared_data_manual.tmadc_sync_barrier = 1;
}

/****************************************************************************
 * Name: aurix_adc_initialize
 *
 * Description:Initialize the ADC
 *
 ****************************************************************************/

size_t aurix_adc_initialize(struct adc_dev_s **dev,
                            const struct aurix_tmadc_config_s *config,
                            size_t count)
{
  uint8_t i;
  int     ret = OK;
  struct  aurix_tmadc_config_s *cur_config;

  while (!shared_data_manual.tmadc_sync_barrier)
    {
      up_udelay(CONFIG_USEC_PER_TICK);
    }

  UP_DMB();

  for (i = 0; i < count && dev[i] != DEV_END; i++)
    {
      cur_config = (struct aurix_tmadc_config_s *)&config[i];

      dev[i]     = init_adc_dev(cur_config);

      ret = adc_register(cur_config->path, dev[i]);

      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          continue;
        }
    }

  return ret;
}
