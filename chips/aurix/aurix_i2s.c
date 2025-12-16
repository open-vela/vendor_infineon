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

#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <arch/chip/chip.h>
#include <Clock/Std/IfxClock.h>

#include "tricore_internal.h"
#include "aurix_i2s.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Currently only support 32 bit data width */

#define I2S_DATA_WIDTH        (32)

/* Currently only support 48000 samplerate */

#define I2S_SAMPLERATE        (48000)

/* The buffer size */

#define I2S_BUFFER_SIZE       (96)

/* The max buffer num in flight */

#define I2S_BUFFER_NUM        (2)

/* The num of data in one DMA transfer */

#define I2S_NUM_OF_DMA_DATA   (96)

/* The num of DMA transfer for one I2S Buffer */

#define I2S_NUM_OF_DMA_TRAN   (I2S_BUFFER_SIZE / I2S_NUM_OF_DMA_DATA)

/* The TDM TX FIFO Threshold */

#define I2S_TDM_TX_FIFO_TH    (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_i2s_buffer_s
{
  struct aurix_i2s_buffer_s *flink; /* Supports a singly linked list */

  /* The associated DMA in/outlink */

  IfxDma_ChannelId dma_ch;
  i2s_callback_t callback;      /* DMA completion callback */
  uint32_t timeout;             /* Timeout value of the DMA transfers */
  void *arg;                    /* Callback's argument */
  struct ap_buffer_s *apb;      /* The audio buffer */
  int result;                   /* The result of the transfer */
};

struct aurix_i2s_transport_s
{
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
};

struct aurix_i2s_s
{
  /* Vela Audio Upper-half Handler */

  struct i2s_dev_s                 dev;

  /* Audio Handler */

  IfxAudio_Audio                   ahandle;

  /* Audio TDM Internal Configs */

  IfxAudio_Audio_Config            audio_config;
  IfxAudio_Tdm_TxConfig            tdm_tx_config;
  IfxAudio_Tdm_RxConfig            tdm_rx_config;
  IfxAudio_Tdm_FifoConfig          tdm_tx_fifo_config;
  IfxAudio_Tdm_FifoConfig          tdm_rx_fifo_config;
  IfxClock_AudioConfig             apll_config;

  /* Aurix I2S External Config */

  struct aurix_i2s_config_s       *config;

  /* Dma related handles */

  IfxDma_Dma                       dma;
  IfxDma_Dma_Channel               dma_tx_ch;
  IfxDma_Dma_ChannelConfig         dma_tx_ch_cfg;

  /* TX transport state */

  struct aurix_i2s_transport_s     tx;

  /* Pre-allocated pool of buffer containers */

  sem_t                            bufsem;
  struct aurix_i2s_buffer_s       *bf_freelist;
  struct aurix_i2s_buffer_s        containers[I2S_BUFFER_NUM];

  /* Local used parameters */

  mutex_t                          lock;
  bool                             running;
  spinlock_t                       slock;

  /* param to record apb current ongoing */

  struct ap_buffer_s              *apb_ongoing;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Buffer container helpers */

static struct aurix_i2s_buffer_s *
                i2s_buf_allocate(struct aurix_i2s_s *priv);
static void     i2s_buf_free(struct aurix_i2s_s *priv,
                             struct aurix_i2s_buffer_s *bfcontainer);
static int      i2s_buf_initialize(struct aurix_i2s_s *priv);

/* DMA support */

static int      i2s_txdma_start(struct aurix_i2s_s *priv);
static int      i2s_txdma_setup(struct aurix_i2s_s *priv,
                                struct aurix_i2s_buffer_s *bfcontainer);
static void     i2s_tx_schedule(struct aurix_i2s_s *priv);
static void     i2s_tx_worker(void *arg);
static int      i2s_dma_tx_interrupt(int irq, void *context, void *arg);

/* I2S methods */

static int      i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                            i2s_callback_t callback, void *arg,
                            uint32_t timeout);

static int      i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels);
static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate);
static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                         i2s_callback_t callback, void *arg,
                         uint32_t timeout);

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev);
static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency);
static int      i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
  .i2s_rxchannels        = i2s_rxchannels,
  .i2s_rxsamplerate      = i2s_rxsamplerate,
  .i2s_rxdatawidth       = i2s_rxdatawidth,
  .i2s_receive           = i2s_receive,

  .i2s_txchannels        = i2s_txchannels,
  .i2s_txsamplerate      = i2s_txsamplerate,
  .i2s_txdatawidth       = i2s_txdatawidth,
  .i2s_send              = i2s_send,

  .i2s_getmclkfrequency  = i2s_getmclkfrequency,
  .i2s_setmclkfrequency  = i2s_setmclkfrequency,

  .i2s_ioctl             = i2s_ioctl,
};

volatile uint16_t g_i2s_dma_tran_cnt = 0;
volatile uint32_t g_i2s_underrun_count = 0;
volatile uint32_t g_i2s_overrun_count = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the I2S state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct aurix_i2s_buffer_s *i2s_buf_allocate(struct aurix_i2s_s *priv)
{
  struct aurix_i2s_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get the buffer from the head of the free list */

  flags = spin_lock_irqsave(&priv->slock);
  bfcontainer = priv->bf_freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->bf_freelist = bfcontainer->flink;
  spin_unlock_irqrestore(&priv->slock, flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct aurix_i2s_s *priv,
                         struct aurix_i2s_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list (circbuf) */

  flags = spin_lock_irqsave(&priv->slock);

  bfcontainer->apb = NULL;
  bfcontainer->flink  = priv->bf_freelist;
  priv->bf_freelist = bfcontainer;

  spin_unlock_irqrestore(&priv->slock, flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static int i2s_buf_initialize(struct aurix_i2s_s *priv)
{
  priv->bf_freelist = NULL;
  for (int i = 0; i < I2S_BUFFER_NUM; i++)
    {
      i2s_buf_free(priv, &priv->containers[i]);
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_txdma_start
 *
 * Description:
 *   Initiate the next TX DMA transfer. The DMA outlink was previously bound
 *   so it is safe to start the next DMA transfer at interrupt level.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int i2s_txdma_start(struct aurix_i2s_s *priv)
{
  struct aurix_i2s_buffer_s *bfcontainer;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->tx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->tx.pend))
    {
      return OK;
    }

  bfcontainer = (struct aurix_i2s_buffer_s *)sq_remfirst(&priv->tx.pend);

  /* If there isn't already an active transmission in progress,
   * then start it.
   */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

  priv->dma_tx_ch_cfg.sourceAddress = (uint32)bfcontainer->apb->samp;
  priv->dma_tx_ch_cfg.transferCount = I2S_NUM_OF_DMA_DATA;
  priv->apb_ongoing = bfcontainer->apb;

  g_i2s_dma_tran_cnt++;
  IfxDma_Dma_initChannel(&priv->dma_tx_ch, &priv->dma_tx_ch_cfg);
  IfxDma_Dma_startChannelTransaction(&priv->dma_tx_ch);

  return OK;
}

/****************************************************************************
 * Name: i2s_txdma_setup
 *
 * Description:
 *   Setup the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *   bfcontainer - The buffer container to be set up
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int i2s_txdma_setup(struct aurix_i2s_s *priv,
                           struct aurix_i2s_buffer_s *bfcontainer)
{
  int ret = OK;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->slock);

  /* Add the buffer container to the end of the TX pending queue */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);

  /* Trigger DMA transfer if no transmission is in progress */

  ret = i2s_txdma_start(priv);

  spin_unlock_irqrestore(&priv->slock, flags);

  return ret;
}

/****************************************************************************
 * Name: i2s_tx_schedule
 *
 * Description:
 *   An TX DMA completion has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - Initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *
 ****************************************************************************/

static void i2s_tx_schedule(struct aurix_i2s_s *priv)
{
  struct aurix_i2s_buffer_s *bfcontainer;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  if (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct aurix_i2s_buffer_s *)sq_peek(&priv->tx.act);

      sq_remfirst(&priv->tx.act);

      /* Report the result of the transfer */

      bfcontainer->result = OK;

      /* Add the completed buffer container to the tail of the tx.done
       * queue
       */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);

      /* Check if the DMA is IDLE */

      if (sq_empty(&priv->tx.act))
        {
          /* Then start the next DMA. */

          i2s_txdma_start(priv);
        }

      i2s_tx_worker(priv);
    }
}

/****************************************************************************
 * Name: i2s_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_tx_worker(void *arg)
{
  struct aurix_i2s_s *priv = (struct aurix_i2s_s *)arg;
  struct aurix_i2s_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when
   * the DMA is finished.
   *
   * In any case, the buffer containers in tx.act will be moved to the end
   * of the tx.done queue and tx.act will be emptied before this worker is
   * started.
   *
   */

  i2sinfo("tx.act.head=%p tx.done.head=%p\n",
          priv->tx.act.head, priv->tx.done.head);

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be disabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = spin_lock_irqsave(&priv->slock);
      bfcontainer =
        (struct aurix_i2s_buffer_s *)sq_remfirst(&priv->tx.done);
      spin_unlock_irqrestore(&priv->slock, flags);

      /* Perform the TX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: i2s_dma_tx_interrupt
 *
 * Description:
 *   Common I2S DMA interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info
 *   arg     - I2S controller private data
 *
 * Returned Value:
 *   Standard interrupt return value.
 *
 ****************************************************************************/

static int i2s_dma_tx_interrupt(int irq, void *context, void *arg)
{
  struct aurix_i2s_s *priv = (struct aurix_i2s_s *)arg;

  /* The first TX FIFO interrupt is triggered by underrun, in this time
   * The DMA transfer is not started, therefore return from interrupt.
   */

  if (!priv->running)
    {
      priv->running = true;
      return 0;
    }

  if (g_i2s_dma_tran_cnt < I2S_NUM_OF_DMA_TRAN)
    {
      /* samp is a uint8 array, the index should be multiplied by 4 */

      priv->dma_tx_ch_cfg.sourceAddress =
        (uint32)(&(priv->apb_ongoing->samp[
          g_i2s_dma_tran_cnt * I2S_NUM_OF_DMA_DATA * 4]));
      priv->dma_tx_ch_cfg.transferCount = I2S_NUM_OF_DMA_DATA;

      g_i2s_dma_tran_cnt++;
      IfxDma_Dma_initChannel(&priv->dma_tx_ch, &priv->dma_tx_ch_cfg);
      IfxDma_Dma_startChannelTransaction(&priv->dma_tx_ch);
    }
  else
    {
      g_i2s_dma_tran_cnt = 0;
      i2s_tx_schedule(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: i2s_tdm_err_interrupt
 *
 * Description:
 *   This is the interrupt handler for the I2S error interrupt.
 *
 ****************************************************************************/

static int i2s_tdm_err_interrupt(int irq, void *context, void *arg)
{
  struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)arg;
  uint8_t underrun_flags = 0;
  uint8_t overrun_flags = 0;

  underrun_flags = i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .INTR_TX_MASKED.B.FIFO_UNDERFLOW;
  overrun_flags = i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .INTR_TX_MASKED.B.FIFO_OVERFLOW;
  i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .INTR_TX.B.FIFO_TRIGGER = 0;

  /* count the number of times "overrun" and "underrun" appear */

  if (overrun_flags)
    {
      g_i2s_overrun_count++;
    }

  if (underrun_flags)
    {
      g_i2s_underrun_count++;
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_rxchannels
 *
 * Description:
 *   Set the I2S RX number of channels.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_rxchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  /* Simulation I2S not support rx function */

  return -ENOTTY;
}

/****************************************************************************
 * Name: i2s_rxsamplerate
 *
 * Description:
 *   Set the I2S RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t i2s_rxsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  /* Simulation I2S only support 48k sample rate */

  DEBUGASSERT(rate == I2S_SAMPLERATE);
  return I2S_SAMPLERATE;
}

/****************************************************************************
 * Name: i2s_rxdatawidth
 *
 * Description:
 *   Set the I2S RX data width.  The RX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_rxdatawidth(struct i2s_dev_s *dev, int bits)
{
  /* Simulation I2S only support 32-bit data width */

  DEBUGASSERT(bits == I2S_DATA_WIDTH);
  return I2S_DATA_WIDTH;
}

/****************************************************************************
 * Name: i2s_receive
 *
 * Description:
 *   Receive a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  /* Simulation I2S do not support receive */

  return -ENOTTY;
}

/****************************************************************************
 * Name: i2s_txchannels
 *
 * Description:
 *   Set the I2S TX number of channels.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)dev;

  DEBUGASSERT(channels == 4 || channels == 8);

  i2sdev->config->channels = channels;

  /* Set TDM TX configuration */

  i2sdev->tdm_tx_config.slaveOrMasterMode
                                        = IfxAudio_Tdm_ControlMode_master;
  i2sdev->tdm_tx_config.format
                                        = IfxAudio_Tdm_ControlFormat_left;
  i2sdev->tdm_tx_config.wordSize
                                        = IfxAudio_Tdm_ControlSize_32;
  i2sdev->tdm_tx_config.clockDiv
                                        = 64 / i2sdev->config->channels - 1;
  i2sdev->tdm_tx_config.mckClockDiv
                                        = IfxAudio_Tdm_IfControlMckDiv_1;
  i2sdev->tdm_tx_config.clockSource     = 0;
  i2sdev->tdm_tx_config.clockPolarity   = 0x0;
  i2sdev->tdm_tx_config.channelSyncPolarity
                                        = 0x0;
  i2sdev->tdm_tx_config.channelSyncPulseFormat
                                        = 0x1;
  i2sdev->tdm_tx_config.numChannels     = i2sdev->config->channels -1;
  i2sdev->tdm_tx_config.channelSize     = 0x1f;
  i2sdev->tdm_tx_config.tdmOrI2sMode    = 0x0;
  i2sdev->tdm_tx_config.enableChannels  = (i2sdev->config->channels == 4) ?
                                            0x0000000f : 0x000000ff;
  i2sdev->tdm_tx_config.txPinsRouteControl
                      = IfxAudio_Tdm_TxRouteControl_externalOrTransmitter;

  IfxAudio_Tdm_configureTx(&i2sdev->ahandle, i2sdev->config->tdm_index,
    &i2sdev->tdm_tx_config);

  return OK;
}

/****************************************************************************
 * Name: i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.  NOTE:  This will have no effect if (1) the
 *   driver does not support an I2S transmitter or if (2) the sample rate is
 *   driven by the I2S frame clock.  This may also have unexpected side-
 *   effects of the TX sample is coupled with the RX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static uint32_t i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  /* Simulation I2S only support 48k sample rate */

  DEBUGASSERT (rate == I2S_SAMPLERATE);
  return I2S_SAMPLERATE;
}

/****************************************************************************
 * Name: i2s_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting data width
 *
 ****************************************************************************/

static uint32_t i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  /* Simulation I2S only support 32-bit data width */

  DEBUGASSERT (bits == I2S_DATA_WIDTH);
  return I2S_DATA_WIDTH;
}

/****************************************************************************
 * Name: i2s_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be cancelled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  DEBUGASSERT (dev && apb);
  struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)dev;
  struct aurix_i2s_buffer_s *bfcontainer;
  int ret;

  nxmutex_lock(&i2sdev->lock);

  if (!i2sdev->running)
    {
      IfxAudio_Tdm_enableTx(&i2sdev->ahandle, i2sdev->config->tdm_index);
      IfxAudio_Tdm_activateTxFifo(&i2sdev->ahandle,
        i2sdev->config->tdm_index);
      IfxAudio_Tdm_startTx(&i2sdev->ahandle, i2sdev->config->tdm_index);
    }

  /* Allocate a buffer container in advance */

  bfcontainer = i2s_buf_allocate(i2sdev);
  DEBUGASSERT(bfcontainer);

  /* Initialize the buffer container structure */

  bfcontainer->callback = callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  ret = i2s_txdma_setup(i2sdev, bfcontainer);
  if (ret != OK)
    {
      goto errout_with_buf;
    }

  i2sinfo("Queued %d bytes into DMA buffers\n", apb->nbytes);

  nxmutex_unlock(&i2sdev->lock);

  return OK;

errout_with_buf:
  nxmutex_unlock(&i2sdev->lock);
  i2s_buf_free(i2sdev, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: i2s_getmclkfrequency
 *
 * Description:
 *   Get the current master clock frequency.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *
 * Returned Value:
 *   Returns the current master clock.
 *
 ****************************************************************************/

static uint32_t i2s_getmclkfrequency(struct i2s_dev_s *dev)
{
  struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)dev;

  return (I2S_DATA_WIDTH * I2S_SAMPLERATE * i2sdev->config->channels);
}

/****************************************************************************
 * Name: i2s_setmclkfrequency
 *
 * Description:
 *   Set the master clock frequency. Usually, the MCLK is a multiple of the
 *   sample rate. Most of the audio codecs require setting specific MCLK
 *   frequency according to the sample rate.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   frequency  - The I2S master clock's frequency
 *
 * Returned Value:
 *   Returns the resulting master clock or a negated errno value on failure.
 *
 ****************************************************************************/

static uint32_t i2s_setmclkfrequency(struct i2s_dev_s *dev,
                                     uint32_t frequency)
{
  struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)dev;

  DEBUGASSERT (frequency ==
    (I2S_DATA_WIDTH * I2S_SAMPLERATE * i2sdev->config->channels));
  return (I2S_DATA_WIDTH * I2S_SAMPLERATE * i2sdev->config->channels);
}

/****************************************************************************
 * Name: i2s_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 ****************************************************************************/

static int i2s_ioctl(struct i2s_dev_s *dev, int cmd, unsigned long arg)
{
  struct audio_buf_desc_s  *bufdesc;
  struct ap_buffer_info_s  *buf_info;
  int ret = -ENOTTY;

  switch (cmd)
    {
      /* AUDIOIOC_START - Start the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_START:
        {
          i2sinfo("AUDIOIOC_START\n");

          ret = OK;
        }
        break;

      /* AUDIOIOC_STOP - Stop the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_STOP:
        {
          i2sinfo("AUDIOIOC_STOP\n");

          struct aurix_i2s_s *i2sdev = (struct aurix_i2s_s *)dev;

          /* make sure the last transfer has finished */

          while (IfxAudio_Tdm_getTxFifoUsedStatus(&i2sdev->ahandle,
            i2sdev->config->tdm_index) != 0)
            {
            }

          IfxAudio_Tdm_stopTx(&i2sdev->ahandle, i2sdev->config->tdm_index);
          IfxAudio_Tdm_deactivateTxFifo(&i2sdev->ahandle,
            i2sdev->config->tdm_index);
          IfxAudio_Tdm_disableTx(&i2sdev->ahandle,
            i2sdev->config->tdm_index);

          i2sdev->running = false;

          ret = OK;
        }
        break;

      /* AUDIOIOC_GETBUFFERINFO - get the buffer info
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_GETBUFFERINFO:
        {
          i2sinfo("AUDIOIOC_GETBUFFERINFO\n");

          buf_info = (struct ap_buffer_info_s *) arg;

          buf_info->buffer_size = I2S_DATA_WIDTH *
                                  I2S_BUFFER_SIZE / 8;
          buf_info->nbuffers = I2S_BUFFER_NUM;

          ret = OK;
        }
        break;

      /* AUDIOIOC_ALLOCBUFFER - Allocate an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ALLOCBUFFER:
        {
          i2sinfo("AUDIOIOC_ALLOCBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          ret = apb_alloc(bufdesc);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          i2sinfo("AUDIOIOC_FREEBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *) arg;
          DEBUGASSERT(bufdesc->u.buffer != NULL);
          apb_free(bufdesc->u.buffer);
          ret = sizeof(struct audio_buf_desc_s);
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *aurix_i2sbus_initialize(struct aurix_i2s_config_s *config)
{
  struct aurix_i2s_s *i2sdev;
  Ifx_DMA *dmaSFR;

  DEBUGASSERT(config != NULL);

  i2sdev = kmm_zalloc(sizeof(*i2sdev));
  DEBUGASSERT(i2sdev != NULL);

  i2sdev->dev.ops = &g_i2sops;
  i2sdev->config = config;
  dmaSFR = (Ifx_DMA *)(IfxDma_cfg_indexMap[config->dma_idx].module);

  /* initialize buf containers */

  i2s_buf_initialize(i2sdev);

  /* fAUDIO_IN =  ((fIN / P) * N ) / K
   * fIN = 25MHz (fOSC)
   * P = 1
   * N = NINT + NFRAC = 39 + 0.3216 = 39.3216
   * NFRAC = 0.3216 * 2^12 = 1317 = 0x525
   * K = 10
   * fAUDIO_IN = ((25000000/1) * 39.3216) / 10 = 98304000 = 98.304MHz
   */

  if (IfxClock_isAudioPllDisabled())
    {
      IfxClock_audioInitConfig(&i2sdev->apll_config);
      i2sdev->apll_config.xtalFrequency = IFX_CFG_CLOCK_XTAL_FREQUENCY;
      i2sdev->apll_config.AudioClockSourceSelect
                                    = IfxClock_AudioClockSourceSelect_pll;
      i2sdev->apll_config.pllInputClockSelection
                                    = IfxClock_PllInputClockSelection_fOsc0;
      i2sdev->apll_config.audioPllConfig.pDivider = IfxClock_Pdivider_1;
      i2sdev->apll_config.audioPllConfig.nDivider = IfxClock_Ndivider_39;
      i2sdev->apll_config.audioPllConfig.nFracDivider = 1317;
      i2sdev->apll_config.audioPllConfig.kDivider = IfxClock_Kdivider_10;
      IfxClock_enableAudioPll(&i2sdev->apll_config);
    }

  /* Set Audio configuration */

  IfxAudio_Audio_initModuleConfig(&i2sdev->audio_config,
    i2sdev->config->amodule);
  IfxAudio_Audio_initModule(&i2sdev->ahandle, &i2sdev->audio_config);

  /* Enable Audio SFR access protection to Bus Masters */

  i2sdev->ahandle.audioSFR->ACCENGBL.WRA.U = 0xffffffff;
  i2sdev->ahandle.audioSFR->ACCENGBL.WRB.U = 0x000000ff;
  i2sdev->ahandle.audioSFR->ACCENGBL.RDA.U = 0xffffffff;
  i2sdev->ahandle.audioSFR->ACCENGBL.RDB.U = 0x000000ff;
  for (int i = 0; i < 1; i++)
    {
      i2sdev->ahandle.audioSFR->ACCENTDM[i].WRA.U = 0xffffffff;
      i2sdev->ahandle.audioSFR->ACCENTDM[i].WRB.U = 0x000000ff;
      i2sdev->ahandle.audioSFR->ACCENTDM[i].RDA.U = 0xffffffff;
      i2sdev->ahandle.audioSFR->ACCENTDM[i].RDB.U = 0x000000ff;
    }

  i2sdev->ahandle.audioSFR->ACCENMXR.WRA.U = 0xffffffff;
  i2sdev->ahandle.audioSFR->ACCENMXR.WRB.U = 0x000000ff;
  i2sdev->ahandle.audioSFR->ACCENMXR.RDA.U = 0xffffffff;
  i2sdev->ahandle.audioSFR->ACCENMXR.RDB.U = 0x000000ff;

  /* Set TDM TX configuration */

  i2sdev->tdm_tx_config.slaveOrMasterMode
                                      = IfxAudio_Tdm_ControlMode_master;
  i2sdev->tdm_tx_config.format        = IfxAudio_Tdm_ControlFormat_left;
  i2sdev->tdm_tx_config.wordSize      = IfxAudio_Tdm_ControlSize_32;
  i2sdev->tdm_tx_config.clockDiv      = 64 / i2sdev->config->channels - 1;
  i2sdev->tdm_tx_config.mckClockDiv   = IfxAudio_Tdm_IfControlMckDiv_1;
  i2sdev->tdm_tx_config.clockSource   = 0;
  i2sdev->tdm_tx_config.clockPolarity
                                      = 0x0;
  i2sdev->tdm_tx_config.channelSyncPolarity
                                      = 0x0;
  i2sdev->tdm_tx_config.channelSyncPulseFormat
                                      = 0x1;
  i2sdev->tdm_tx_config.numChannels   = i2sdev->config->channels -1;
  i2sdev->tdm_tx_config.channelSize   = 0x1f;
  i2sdev->tdm_tx_config.tdmOrI2sMode  = 0x0;
  i2sdev->tdm_tx_config.enableChannels
                                      = (i2sdev->config->channels == 4)
                                            ? 0x0000000f : 0x000000ff;
  i2sdev->tdm_tx_config.txPinsRouteControl
                    = IfxAudio_Tdm_TxRouteControl_externalOrTransmitter;

  /* Set TDM TXFIFO configuration */

  i2sdev->tdm_tx_fifo_config.triggerLevel
                                      = I2S_TDM_TX_FIFO_TH;
  i2sdev->tdm_tx_fifo_config.dmaChannelIndex
                                      = (IfxAudio_DmaCh)config->dma_channel;

  IfxAudio_Tdm_configureTx(&i2sdev->ahandle, i2sdev->config->tdm_index,
    &i2sdev->tdm_tx_config);
  IfxAudio_Tdm_configureTxFifo(&i2sdev->ahandle,
    i2sdev->config->tdm_index, &i2sdev->tdm_tx_fifo_config);
  IfxAudio_Tdm_configureRx(&i2sdev->ahandle, i2sdev->config->tdm_index,
    &i2sdev->tdm_rx_config);

  /* Init sck pin */

  IfxPort_setPinModeOutput(i2sdev->config->txsck->pin.port,
    i2sdev->config->txsck->pin.pinIndex, IfxPort_OutputMode_pushPull,
    i2sdev->config->txsck->select);
  IfxPort_setPinPadDriver(i2sdev->config->txsck->pin.port,
    i2sdev->config->txsck->pin.pinIndex,
    IfxPort_PadDriver_cmosAutomotiveSpeed1);

  /* Init fsync pin */

  IfxPort_setPinModeOutput(i2sdev->config->txfsync->pin.port,
    i2sdev->config->txfsync->pin.pinIndex, IfxPort_OutputMode_pushPull,
    i2sdev->config->txfsync->select);
  IfxPort_setPinPadDriver(i2sdev->config->txfsync->pin.port,
    i2sdev->config->txfsync->pin.pinIndex,
    IfxPort_PadDriver_cmosAutomotiveSpeed1);

  /* Init txsd pin */

  IfxPort_setPinPadDriver(i2sdev->config->txdata->pin.port,
    i2sdev->config->txdata->pin.pinIndex,
    IfxPort_PadDriver_cmosAutomotiveSpeed1);
  i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .TX_IOMUX_CTL.B.SDOUT_DRV_EN = 1;
  i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .TX_IOMUX_CTL.B.SDOUT_EN_SEL = i2sdev->config->txdata->select;

  /* enable the interrupt for underrun and overrun */

  i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
  .INTR_TX_MASK.U |= 0x06;

  /* DMA settings */

  if (dmaSFR->CLC.B.DISS)
    {
      IfxDma_Dma_Config dmaConfig;
      IfxDma_Dma_initModuleConfig(&dmaConfig, dmaSFR);
      IfxDma_Dma_initModule(&i2sdev->dma, &dmaConfig);
    }

  IfxDma_Dma_initChannelConfig(&i2sdev->dma_tx_ch_cfg, &i2sdev->dma);
  i2sdev->dma_tx_ch_cfg.channelId = config->dma_channel;
  i2sdev->dma_tx_ch_cfg.destinationAddress
    = (uint32)(&(i2sdev->ahandle.audioSFR->TDM[i2sdev->config->tdm_index]
      .TX_FIFO_WR.U));
  i2sdev->dma_tx_ch_cfg.destinationCircularBufferEnabled = TRUE;
  i2sdev->dma_tx_ch_cfg.destinationAddressCircularRange =
    IfxDma_ChannelIncrementCircular_none;
  i2sdev->dma_tx_ch_cfg.moveSize = IfxDma_ChannelMoveSize_32bit;
  i2sdev->dma_tx_ch_cfg.requestMode =
    IfxDma_ChannelRequestMode_completeTransactionPerRequest;
  i2sdev->dma_tx_ch_cfg.channelInterruptEnabled = TRUE;

  IfxDma_Dma_initChannel(&i2sdev->dma_tx_ch, &i2sdev->dma_tx_ch_cfg);

  /* enable DMA channel interrupt */

  irq_attach(config->tdm_tx_isr_irq, i2s_dma_tx_interrupt, i2sdev);
  up_enable_irq(config->tdm_tx_isr_irq);

  irq_attach(config->tdm_err_isr_irq, i2s_tdm_err_interrupt, i2sdev);
  up_enable_irq(config->tdm_err_isr_irq);

  i2sdev->running = false;
  nxmutex_init(&i2sdev->lock);

  return &i2sdev->dev;
}

/****************************************************************************
 * Name: aurix_all_i2sbus_initialize
 *
 * Description:
 *   Initialize all i2sbus deivce for aurix.
 *
 ****************************************************************************/

int aurix_all_i2sbus_initialize(struct i2s_dev_s **dev,
                                struct aurix_i2s_config_s *config,
                                size_t count)
{
  size_t i;

  for (i = 0; i < count && dev[i] != DEV_END; i++)
    {
      dev[i] = aurix_i2sbus_initialize(&config[i]);
    }

  return OK;
}
