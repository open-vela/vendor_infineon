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

#include <stdio.h>
#include <debug.h>

#include <nuttx/kmalloc.h>

#include "aurix_evadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LOWER_16BITS_MASK (0x0000FFFF)

/* Get ADC int status */

#define CHECK_G_REFLAG(priv)                                     \
  ({                                                            \
    uint32_t extracted_bits = priv->group_handle.group->REFLAG.U & \
                              LOWER_16BITS_MASK;                \
    (extracted_bits != 0U) ? true : false;                      \
  })

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AURIX_HAVE_ADC_IRQ
static int adc_interrupt(int irq, void *context, void *arg);
static int adc_interrupt_cfg(struct adc_dev_s *dev, bool enable);
#endif

/* Group method */

static void init_group(struct aurix_evadc_config_s *priv);
static void deinit_group(struct aurix_evadc_config_s *priv);

/* Channel method */

static IfxEvadc_Status init_channels(struct aurix_evadc_config_s *priv);

/* ADC related method */

static void adc_reg_startconv(struct aurix_evadc_config_s *priv);
static int adc_enable(struct aurix_evadc_config_s *priv, bool enable);

/* ADC Driver Methods */

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind      = adc_bind,
  .ao_reset     = adc_reset,
  .ao_setup     = adc_setup,
  .ao_shutdown  = adc_shutdown,
  .ao_rxint     = adc_rxint,
  .ao_ioctl     = adc_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_group
 *
 * Description:
 *   Get group config and initialize the group
 *
 ****************************************************************************/

static void init_group(struct aurix_evadc_config_s *priv)
{
  IfxEvadc_Adc_GroupConfig *priv_group_config = &priv->adc_group_config;

  /* Create and initialize group configuration with default values */

  IfxEvadc_Adc_initGroupConfig(priv_group_config, &priv->evadc_handle);

  /* Setting user configuration using group 0 */

  priv_group_config->groupId = priv->group_id;
  priv_group_config->master  = priv->group_id;

  /* Set sample time */

  priv_group_config->inputClass[0].sampleTime = 9.0e-5;
  priv_group_config->inputClass[1].sampleTime = 9.0e-5;

  /* Enable queued source */

  priv_group_config->arbiter.requestSlotQueue0Enabled = TRUE;

  /* Enable all gates in "always" mode (no edge detection) */

  priv_group_config->queueRequest[0].triggerConfig.gatingMode =
    IfxEvadc_GatingMode_always;

  /* Initialize the group */

  IfxEvadc_Adc_initGroup(&priv->group_handle, priv_group_config);

  return;
}

/****************************************************************************
 * Name: deinit_group
 *
 * Description:
 *   deinit the group
 *
 ****************************************************************************/

static void deinit_group(struct aurix_evadc_config_s *priv)
{
  IfxEvadc_GroupId group_index = priv->group_handle.groupId;

  IfxEvadc_enableAccess(priv->evadc_handle.evadc,
                        (IfxEvadc_Protection)
                        (IfxEvadc_Protection_initGroup0 + group_index));

  IfxEvadc_resetGroup(priv->group_handle.group);

  IfxEvadc_disableAccess(priv->evadc_handle.evadc,
                         (IfxEvadc_Protection)
                         (IfxEvadc_Protection_initGroup0 + group_index));
}

/****************************************************************************
 * Name: init_channels
 *
 * Description:
 *   Get each channel config and initialize channels
 *
 ****************************************************************************/

static IfxEvadc_Status init_channels(struct aurix_evadc_config_s *priv)
{
  IfxEvadc_Status ret = IfxEvadc_Status_noError;

  /* Initialize the channel */

  for (uint8_t i = 0; i < priv->nchannels; i++)
    {
      if (NULL == priv->adc_channellist[i].channel_config)
        {
          IfxEvadc_Adc_ChannelConfig *temp_config =
            kmm_zalloc(sizeof(IfxEvadc_Adc_ChannelConfig));

          if (NULL == temp_config)
            {
              aerr("ERROR: Failed to allocate channel config memory");
              ret = IfxEvadc_Status_notInitialised;
              goto out;
            }

          IfxEvadc_Adc_initChannelConfig(temp_config, &priv->group_handle);

          /* Initialize the configuration with default values */

          priv->adc_channellist[i].channel_config = temp_config;

          priv->adc_channellist[i].channel_config->resultPriority =
            IRQ_TO_NDX(priv->irq);

          priv->adc_channellist[i].request_source =
            IfxEvadc_RequestSource_queue0;

          priv->adc_channellist[i].refill = 0;
        }

      /* Select the channel ID and the respective result register */

      priv->adc_channellist[i].channel_config->channelId =
        (IfxEvadc_ChannelId)priv->adc_channellist[i].priv_channel.channel;

      priv->adc_channellist[i].channel_config->resultRegister =
        (IfxEvadc_ChannelResult)i;

      ret = IfxEvadc_Adc_initChannel(&priv->adc_channellist[i].priv_channel,
        priv->adc_channellist[i].channel_config);

      if (IfxEvadc_Status_noError != ret)
        {
          goto out;
        }

      IfxEvadc_Adc_addToQueue(&priv->adc_channellist[i].priv_channel,
                                priv->adc_channellist[i].request_source,
                                priv->adc_channellist[i].refill);
    }

  return ret;

out:
  for (uint8_t i = 0; i < priv->nchannels; i++)
    {
      if (NULL == priv->adc_channellist[i].channel_config)
        {
          kmm_free(priv->adc_channellist[i].channel_config);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: adc_reg_startconv
 *
 * Description:
 *   Start (or stop) the ADC conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_reg_startconv(struct aurix_evadc_config_s *priv)
{
  DEBUGASSERT(priv != NULL);

  /* Start the conversion of channels */

  IfxEvadc_enableAccess(priv->evadc_handle.evadc,
                        (IfxEvadc_Protection)
                        (IfxEvadc_Protection_initGroup0 + priv->group_id));

  IfxEvadc_setAnalogConvertControl(priv->group_handle.group,
    IfxEvadc_AnalogConverterMode_normalOperation);

  IfxEvadc_disableAccess(priv->evadc_handle.evadc,
                         (IfxEvadc_Protection)
                         (IfxEvadc_Protection_initGroup0 + priv->group_id));

  for (uint8_t i = 0; i < priv->nchannels; i++)
    {
      IfxEvadc_Adc_addToQueue(&priv->adc_channellist[i].priv_channel,
                                priv->adc_channellist[i].request_source,
                                priv->adc_channellist[i].refill);
    }

  /* Start the queue0 */

  IfxEvadc_Adc_startQueue(&priv->group_handle,
                          IfxEvadc_RequestSource_queue0);
}

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables or disables the specified ADC peripheral.
 *
 ****************************************************************************/

static int adc_enable(struct aurix_evadc_config_s *priv, bool enable)
{
  DEBUGASSERT(priv != NULL);
  int ret = OK;

  if (enable)
    {
      /* Enable Adc channels and instance */

      init_channels(priv);
    }
  else
    {
      deinit_group(priv);
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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;

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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;

  ainfo("instance: %d\n", priv->group_id);

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

      deinit_group(priv);
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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;

  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
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

  deinit_group(priv);

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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;
  int ret = OK;

  ret = nxmutex_lock(&priv->cmn->lock);

  if (ret < 0)
    {
      return ret;
    }

  /* Do nothing when the ADC device is already set up */

  if (priv->cmn->refcount > 0)
    {
      return OK;
    }

  /* Make sure that the ADC device is in the powered up, reset state */

  adc_enable(priv, true);

  adc_reg_startconv(priv);

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
    struct aurix_evadc_config_s *priv =
      (struct aurix_evadc_config_s *)dev->ad_priv;
    ainfo("instance: %d enable: %d\n", priv->group_id, enable ? 1 : 0);
    adc_interrupt_cfg(dev, enable);
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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          if (priv->nchannels > 0)
            {
              adc_reg_startconv(priv);
            }
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
static void adc_getdata(struct aurix_evadc_config_s *priv, uint32_t data)
{
  if (!(priv->fifo.fifochbuffer) || !(priv->fifo.fifodatabuffer))
    {
      return;
    }

  priv->fifo.fifodatabuffer[priv->current] = data;
  priv->fifo.fifochbuffer[priv->current] =
    priv->adc_channellist[priv->current].priv_channel.channel;

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
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;
  int ret = OK;

  if (enable)
    {
      ainfo("Enable the ADC interrupt: irq=%lu\n", priv->irq);

      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev             = (struct adc_dev_s *) arg;
  struct aurix_evadc_config_s *priv =
    (struct aurix_evadc_config_s *)dev->ad_priv;
  int32_t data;

  if ((true == CHECK_G_REFLAG(priv)))
    {
      /* EOC:
       * End of conversion
       */

      priv->adc_convert_flag = false;

      for (priv->current = 0;
           priv->current < priv->nchannels;
           priv->current++)
        {
          /* Read the converted value and clear EOC bit
           * (It is cleared by reading the ADC_DR)
           */

          data = priv->group_handle.group->RES[priv->current].B.RESULT;
#if defined(CONFIG_AURIX_ADC_USE_NO_FIFO)
            adc_getdata(priv, data);
#else
          if (priv->cb != NULL)
            {
                DEBUGASSERT(priv->cb->au_receive != NULL);
                priv->cb->au_receive(dev,
                  priv->adc_channellist[priv->current].priv_channel.channel,
                                     data);
            }
#endif
        }

      /* Restart the conversion sequence
       * from the beginning
       */

      priv->current = 0;
      priv->group_handle.group->REFCLR.U = LOWER_16BITS_MASK;
    }

  return OK;
}

#endif

/****************************************************************************
 * Name: init_adc_dev
 *
 * Description:Initialize the adc dev.
 *
 ****************************************************************************/

static struct adc_dev_s * init_adc_dev(struct aurix_evadc_config_s *priv)
{
  struct adc_dev_s *adc_dev = kmm_zalloc(sizeof(struct adc_dev_s));
  if (!adc_dev)
    {
      aerr("ERROR: Failed to allocate adc_dev_s memory\n");
      goto out;
    }

  priv->fifo.fifosize = priv->nchannels + 1;

  priv->fifo.fifochbuffer =
    kmm_zalloc((priv->fifo.fifosize + 1) * sizeof(uint8_t));

  if (!priv->fifo.fifochbuffer)
    {
      aerr("ERROR: Failed to allocate fifochbuffer memory\n");
      goto out;
    }

  priv->fifo.fifodatabuffer =
    kmm_zalloc((priv->fifo.fifosize + 1) * sizeof(uint32_t));

  if (!priv->fifo.fifodatabuffer)
    {
      aerr("ERROR: Failed to allocate fifodatabuffer memory\n");
      goto out;
    }

  priv->cmn = kmm_zalloc(sizeof(struct adccmn_data_s));

  if (!priv->cmn)
    {
      aerr("ERROR: Failed to allocate cmn memory\n");
      goto out;
    }

  nxmutex_init(&priv->cmn->lock);
  priv->cmn->refcount = 0;

  priv->evadc_handle.evadc      = &MODULE_EVADC;
  adc_dev->ad_priv              = priv;
  adc_dev->ad_ops               = &g_adcops;
  adc_dev->ad_recv.af_data      = priv->fifo.fifodatabuffer;
  adc_dev->ad_recv.af_channel   = priv->fifo.fifochbuffer;
  adc_dev->ad_recv.af_fifosize  = priv->fifo.fifosize;

#ifdef CONFIG_AURIX_ADC_ISR_WQUEUE
  priv->isr = adc_interrupt;
  irq_attach_wqueue(priv->irq, NULL, priv->isr, adc_dev,
                    CONFIG_AURIX_ADC_ISR_WQUEUE_PRIORITY);
#else
  priv->isr = adc_interrupt;
  irq_attach(priv->irq, priv->isr, adc_dev);
#endif

  init_group(priv);
  return adc_dev;

out:
  aerr("ERROR: Failed to allocate memory\n");
  if (NULL != adc_dev)
    {
      kmm_free(adc_dev);
    }

  if (NULL != priv->fifo.fifochbuffer)
    {
      kmm_free(priv->fifo.fifochbuffer);
    }

  if (NULL != priv->fifo.fifodatabuffer)
    {
      kmm_free(priv->fifo.fifodatabuffer);
    }

  if (NULL != priv->cmn)
    {
      kmm_free(priv->cmn);
    }

  return NULL;
}

/****************************************************************************
 * Name: aurix_adc_initialize
 *
 * Description:
 *   Initialize the ADC
 *
 ****************************************************************************/

int aurix_adc_initialize(struct adc_dev_s **dev,
                         struct aurix_evadc_config_s *config,
                         size_t count)
{
  char    path[16];
  uint8_t i;
  int     ret = OK;
  struct  aurix_evadc_config_s * priv;

  for (i = 0; i < count && dev[i] != DEV_END; i++)
    {
      priv = (struct aurix_evadc_config_s *) &config[i];

      if (0 == i)
        {
          if (NULL == priv->adc_config)
            {
              priv->adc_config = kmm_zalloc(sizeof(IfxEvadc_Adc_Config));

              if (NULL == priv->adc_config)
                {
                  ret = -1;
                  aerr("ERROR: Init EVADC module failed: %d\n", ret);
                  break;
                }
            }

          IfxEvadc_Adc_initModuleConfig(priv->adc_config, &MODULE_EVADC);

          /* Initialize module */

          IfxEvadc_Adc_initModule(&priv->evadc_handle, priv->adc_config);
        }

      dev[i] = init_adc_dev(priv);

      snprintf(path, sizeof(path), "/dev/adc%d", priv->group_id);
      ret = adc_register(path, dev[i]);
      if (ret < 0)
        {
          aerr("ERROR: adc_register failed: %d\n", ret);
          break;
        }
    }

  return ret;
}
