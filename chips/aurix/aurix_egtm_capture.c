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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>

#include "aurix_egtm_capture.h"
#include "Cpu/Std/Ifx_Types.h"
#include "Egtm/Tim/In/IfxEgtm_Tim_In.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int egtm_capture_start(struct cap_lowerhalf_s *lower);
static int egtm_capture_stop(struct cap_lowerhalf_s *lower);
static int egtm_capture_getduty(struct cap_lowerhalf_s *lower,
                                uint8_t *duty);
static int egtm_capture_getfreq(struct cap_lowerhalf_s *lower,
                                uint32_t *freq);
static int egtm_capture_getedges(struct cap_lowerhalf_s *lower,
                                 uint32_t *edges);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half methods required by capture driver. */

static const struct cap_ops_s g_egtm_cap_ops =
{
  .start    = egtm_capture_start,
  .stop     = egtm_capture_stop,
  .getduty  = egtm_capture_getduty,
  .getfreq  = egtm_capture_getfreq,
  .getedges = egtm_capture_getedges,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: egtm_capture_tbu_tstamp
 *
 * Description:
 *   Get the TBU channel0 timestamp.
 *
 * Input Parameters:
 *   dev - The capture channel device.
 *
 * Returned Value:
 *   Returns the TBU channel0 timestamp.
 *
 ****************************************************************************/

static inline uint32_t
egtm_capture_tbu_tstamp(struct aurix_egtm_capture_dev_s *dev)
{
  const struct aurix_gtm_capture_config_s *info = dev->privinfo;
  IfxEgtm_Cluster cluster = IfxEgtm_Cluster_0;
  Ifx_EGTM *egtm = info->gtm_base;

  return egtm->CLS[cluster].TBU.CH0_BASE.U;
}

/****************************************************************************
 * Name: egtm_capture_channel
 *
 * Description:
 *   Get the TIM channel instance.
 *
 * Input Parameters:
 *   dev - The capture channel device.
 *
 * Returned Value:
 *   Returns the pointer of TIM channel.
 *
 ****************************************************************************/

static inline Ifx_EGTM_CLS_TIM_CH *
egtm_capture_channel(struct aurix_egtm_capture_dev_s *dev)
{
  IfxEgtm_Tim_In_Config *config;
  IfxEgtm_Tim_Ch channel_index;
  int cluster_index;
  IfxEgtm_Tim_TinMap *input_pin = NULL_PTR;

  config = &dev->egtm_tim_in_cfg;
#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
  input_pin = config->inputPin;
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
  input_pin = config->filter.inputPin;
#endif

  if (input_pin != NULL_PTR)
    {
      channel_index = input_pin->channel;
      cluster_index = input_pin->tim;
    }
  else
    {
      channel_index = config->channelIndex;
      cluster_index = config->cluster;
    }

  return IfxEgtm_Tim_getChannel(&config->egtm->CLS[cluster_index].TIM,
                                channel_index);
}

/****************************************************************************
 * Name: egtm_capture_start
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   enables the capture channel, interruption routine, sets the positive
 *   edge to trigger this interrupt and resets the frequency and duty
 *   values. The positive edge is always the first expected.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int egtm_capture_start(struct cap_lowerhalf_s *lower)
{
  const struct aurix_gtm_capture_config_s *info;
  struct aurix_egtm_capture_dev_s *dev;
  IfxEgtm_Tim_In_Config *tim_in_cfg;
  Ifx_EGTM_CLS_TIM_CH *tim_channel;
  IfxEgtm_Tim_In *tim_in;

  dev = (struct aurix_egtm_capture_dev_s *)lower;
  dev->capinfo.history_edges = 0;

  /* Initialize default parameters. */

  info = dev->privinfo;
  tim_in_cfg = &dev->egtm_tim_in_cfg;
  IfxEgtm_Tim_In_initConfig(tim_in_cfg, (Ifx_EGTM *)info->gtm_base);

  /* Set the TIM channel basic configuration. */

  tim_in_cfg->cluster       = (int)info->gtm_tim;
  tim_in_cfg->channelIndex  = info->gtm_tim_ch;
  tim_in_cfg->capture.clock = info->gtm_cmu_clk;
  tim_in_cfg->timeout.clock = info->gtm_cmu_clk;
  if (info->timeout_time != 0.0f)
    {
      tim_in_cfg->timeout.timeout = info->timeout_time;
      tim_in_cfg->timeout.irqOnTimeout = TRUE;
    }

#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
  tim_in_cfg->inputPin      = info->gtm_tim_pin;
  tim_in_cfg->inputPinMode  = IfxPort_InputMode_noPullDevice;
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
  tim_in_cfg->filter.inputPin      = info->gtm_tim_pin;
  tim_in_cfg->filter.inputPinMode  = IfxPort_InputMode_noPullDevice;
#endif

  /* Set the TIM channel filter configuration. */

  if (info->filter_time != 0.0f)
    {
      tim_in_cfg->filter.risingEdgeMode =
      IfxEgtm_Tim_In_ConfigFilterMode_individualDeglitchTimeUpDown;
      tim_in_cfg->filter.risingEdgeFilterTime = info->filter_time;
      tim_in_cfg->filter.fallingEdgeMode =
      IfxEgtm_Tim_In_ConfigFilterMode_individualDeglitchTimeUpDown;
      tim_in_cfg->filter.fallingEdgeFilterTime = info->filter_time;
    }

  /* Set the working mode of the TIM channel. */

  if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TPWM)
    {
      tim_in_cfg->mode = IfxEgtm_Tim_Mode_pwmMeasurement;
    }
  else if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TIEM)
    {
      tim_in_cfg->mode = IfxEgtm_Tim_Mode_inputEvent;
      tim_in_cfg->capture.activeEdge = IfxEgtm_Tim_In_ActiveEdge_both;
    }
  else
    {
      cpwarn("Unsupported capture mode: %d\n", dev->egtm_cap_mode);
      return -EINVAL;
    }

  IfxEgtm_Tbu_enableChannel((Ifx_EGTM *)info->gtm_base,
                            IfxEgtm_Cluster_0, IfxEgtm_Tbu_Ts_0);

  /* Set the TIM channel interrupt configuration. */

  tim_in_cfg->isrPriority         = 0;
  tim_in_cfg->capture.irqOnNewVal = TRUE;
  tim_in_cfg->irqMode             = IfxEgtm_IrqMode_pulseNotify;
  tim_in_cfg->isrProvider         = info->isr_provider;
  tim_channel                     = egtm_capture_channel(dev);

  IfxEgtm_Tim_Ch_setNotificationMode(tim_channel, tim_in_cfg->irqMode);

  IfxEgtm_Tim_Ch_setChannelNotification(tim_channel,
    tim_in_cfg->capture.irqOnNewVal,
    tim_in_cfg->capture.irqOnCntOverflow,
    tim_in_cfg->capture.irqOnEcntOverflow,
    tim_in_cfg->capture.irqOnDatalost);

  /* Initialize the TIM channel. */

  tim_in = &dev->egtm_tim_in;
  IfxEgtm_Tim_In_init(tim_in, tim_in_cfg);

  if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TIEM)
    {
      /* Use CLSi_TIM_INP_VAL as input for GPR0 */

      tim_channel->CTRL.B.GPR0_SEL = (uint8)0x1;
      tim_channel->CTRL.B.EGPR0_SEL = (uint8)0x1;
    }

  /* Get the capture clock frequency. */

  dev->clock = (uint32_t)tim_in->captureClockFrequency;
  dev->tbuclk = (uint32_t)IfxEgtm_Tbu_getClockFrequency(
                            (Ifx_EGTM *)info->gtm_base,
                            IfxEgtm_Cluster_0, IfxEgtm_Tbu_Ts_0);

  /* Initialize device status and sample data. */

  dev->freq      = 0;
  dev->duty      = 0;
  dev->edges     = 0;
  dev->isr_count = 0;
  dev->ready     = false;
  dev->enabled   = true;

  /* Enable the TIM channel interrupt. */

  up_enable_irq(info->gtm_tim_ch_irq);

  cpinfo("Channel TIM%d_CH%d enabled. \n", info->gtm_tim,
         info->gtm_tim_ch);

  return OK;
}

/****************************************************************************
 * Name: egtm_capture_stop
 *
 * Description:
 *   This function is a requirement of the upper-half driver. When called,
 *   disables the capture channel and the interrupt routine associated.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int egtm_capture_stop(struct cap_lowerhalf_s *lower)
{
  const struct aurix_gtm_capture_config_s *info;
  struct aurix_egtm_capture_dev_s *dev;
  Ifx_EGTM_CLS_TIM_CH *channel;

  dev = (struct aurix_egtm_capture_dev_s *)lower;
  info = dev->privinfo;

  /* Disable the TIM channel interrupt. */

  up_disable_irq(info->gtm_tim_ch_irq);

  /* Get the TIM channel instance. */

  channel = egtm_capture_channel(dev);

  /* Disable the TIM channel. */

  channel->CTRL.B.TIM_EN = 0;

  /* Update device status. */

  dev->enabled = false;
  dev->ready   = false;

  cpinfo("Channel TIM%d_CH%d disabled. \n", info->gtm_tim,
         info->gtm_tim_ch);

  return OK;
}

/****************************************************************************
 * Name: egtm_capture_getduty
 *
 * Description:
 *   This function is a requirement of the upper-half driver.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   duty  - uint8_t pointer where the duty cycle value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int egtm_capture_getduty(struct cap_lowerhalf_s *lower,
                                uint8_t *duty)
{
  struct aurix_egtm_capture_dev_s *dev;
  irqstate_t flags;
  int ret;

  dev = (struct aurix_egtm_capture_dev_s *)lower;
  ret = -EBUSY;

  /* The newly sampled frequency and duty are retrieved in the interrupt
   * service routine and saved to the device object. To obtain the sampled
   * data, you only need to read the data in the device object.
   */

  flags = enter_critical_section();

  /* Get the sampled data in interrupt mode. */

  if (dev->ready)
    {
      *duty = dev->duty;
      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: egtm_capture_getfreq
 *
 * Description:
 *   This function is a requirement of the upper-half driver.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   freq  - uint8_t pointer where the frequency value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int egtm_capture_getfreq(struct cap_lowerhalf_s *lower,
                                uint32_t *freq)
{
  struct aurix_egtm_capture_dev_s *dev;
  irqstate_t flags;
  int ret;

  dev = (struct aurix_egtm_capture_dev_s *)lower;
  ret = -EBUSY;

  /* The newly sampled frequency and duty are retrieved in the interrupt
   * service routine and saved to the device object. To obtain the sampled
   * data, you only need to read the data in the device object.
   */

  flags = enter_critical_section();

  /* Get the sampled data in interrupt mode. */

  if (dev->ready)
    {
      *freq = dev->freq;
      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: egtm_capture_getedges
 *
 * Description:
 *   This function is a requirement of the upper-half driver.
 *
 * Input Parameters:
 *   lower - Pointer to the capture channel lower-half data structure.
 *   edges - uint32_t pointer where the edges value is written.
 *
 * Returned Value:
 *   Returns OK on success.
 *
 ****************************************************************************/

static int egtm_capture_getedges(struct cap_lowerhalf_s *lower,
                                 uint32_t *edges)
{
  struct aurix_egtm_capture_dev_s *dev;
  irqstate_t flags;
  int ret;

  dev = (struct aurix_egtm_capture_dev_s *)lower;
  ret = -EBUSY;

  /* The newly sampled frequency and duty are retrieved in the interrupt
   * service routine and saved to the device object. To obtain the sampled
   * data, you only need to read the data in the device object.
   */

  flags = enter_critical_section();

  /* Get the sampled data in interrupt mode. */

  if (dev->ready)
    {
      *edges = dev->edges;
      ret = OK;
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: egtm_capture_isr
 *
 * Description:
 *   Default function called when a capture interrupt occurs.
 *   It reads the capture timer value and the interrupt edge. When positive
 *   edge triggered the interrupt, the current capture value is stored and
 *   the interrupt edge is modified to falling edge.
 *   When the negative edge triggers the interrupt, the timer count
 *   difference is calculated and the high time period is obtained.
 *
 *   Two pulses are required to properly calculate the frequency.
 *
 * Input Parameters:
 *   irq     - The interrupt request number.
 *   context - Pointer to the interrupt context.
 *   arg     - Pointer to the argument to be passed to the ISR.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int egtm_capture_isr(int irq, void *context, void *arg)
{
  const struct aurix_gtm_capture_config_s *info;
  struct aurix_egtm_capture_dev_s *dev;
  IfxEgtm_Tim_In *tim_in;

  dev = (struct aurix_egtm_capture_dev_s *)arg;
  tim_in = &dev->egtm_tim_in;
  info = dev->privinfo;

  /* Try to read the capture timer value. */

  IfxEgtm_Tim_In_update(tim_in);

  /* If there is new data to read, the new data is used;
   * If not, the old capture data is reserved.
   * If a timeout event was detected,
   * set freq and duty to 0, reset timeout flag;
   * Attention: if writing 1 to TODET, clear interrupt flag.
   */

  if (tim_in->channel->IRQ_NOTIFY.B.TODET == 1)
    {
      if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TPWM)
        {
          dev->duty = 0;
          dev->freq = 0;
          dev->edges = 0;
          dev->ready = true;
          if (tim_in->newData == true)
            {
              tim_in->channel->IRQ_NOTIFY.B.TODET = 1;
            }
        }
    }

  if (tim_in->newData)
    {
      if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TPWM)
        {
          dev->duty = (tim_in->pulseLengthTick * 100) / tim_in->periodTick;
          dev->freq = dev->clock / tim_in->periodTick;
          dev->edges = tim_in->edgeCount;
          dev->ready = true;
        }
      else if (dev->egtm_cap_mode == GTM_CAPTURE_MODE_TIEM)
        {
          uint32_t gpr0_value;
          int level;

          gpr0_value = tim_in->pulseLengthTick;
          level = (gpr0_value & (0x1 << (info->gtm_tim_ch + 16))) ? 1 : 0;

          tim_in->periodTick = egtm_capture_tbu_tstamp(dev);
          dev->edges = tim_in->edgeCount;

          if (tim_in->dataLost)
            {
              dev->capinfo.history_edges = 0;
            }

          if (dev->capinfo.history_edges < 2)
            {
              if ((level == 1) && (dev->capinfo.history_edges == 0))
                {
                  dev->capinfo.edge1_tstamp = tim_in->periodTick;
                  dev->capinfo.edge1_sequence = tim_in->edgeCount;
                  dev->capinfo.history_edges += 1;
                }
              else if ((level == 0) && (dev->capinfo.history_edges == 1))
                {
                  dev->capinfo.edge2_tstamp = tim_in->periodTick;
                  dev->capinfo.edge2_sequence = tim_in->edgeCount;
                  dev->capinfo.history_edges += 1;
                }
            }
          else
            {
              uint32_t period;
              uint32_t pulse;

              if (tim_in->periodTick < dev->capinfo.edge1_tstamp)
                {
                  period = (tim_in->periodTick + 0x7ffffff) -
                           dev->capinfo.edge1_tstamp;
                }
              else
                {
                  period = tim_in->periodTick -
                           dev->capinfo.edge1_tstamp;
                }

              if (dev->capinfo.edge2_tstamp < dev->capinfo.edge1_tstamp)
                {
                  pulse = (dev->capinfo.edge2_tstamp + 0x7ffffff) -
                          dev->capinfo.edge1_tstamp;
                }
              else
                {
                  pulse = dev->capinfo.edge2_tstamp -
                          dev->capinfo.edge1_tstamp;
                }

              dev->freq = dev->tbuclk / period;
              dev->duty = (pulse * 100) / period;

              dev->capinfo.edge1_tstamp = tim_in->periodTick;
              dev->capinfo.edge1_sequence = tim_in->edgeCount;
              dev->capinfo.history_edges = 1;

              dev->ready = true;
            }
        }

      if (dev->egtm_callback != NULL)
        {
          dev->egtm_callback();
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_capture_clear_data
 *
 * Description:
 *   Clear all the sampled data.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *
 ****************************************************************************/

void aurix_gtm_capture_clear_data(struct cap_lowerhalf_s *dev)
{
  struct aurix_egtm_capture_dev_s *capdev;
  irqstate_t flags;

  capdev = (struct aurix_egtm_capture_dev_s *)dev;
  flags = enter_critical_section();
  capdev->freq  = 0;
  capdev->duty  = 0;
  capdev->edges = 0;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: aurix_egtm_capture_get_pinstate
 *
 * Description:
 *   Obtain the status of the PWM input signal, whether it is high level
 *   or low level. This interface is only valid in the EGTM_CAPTURE_MODE_TIEM
 *   working mode.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *
 * Returned Value:
 *   0 represents a low level and 1 represents a high level.
 *
 ****************************************************************************/

bool aurix_gtm_capture_get_pinstate(struct cap_lowerhalf_s *dev)
{
  const struct aurix_gtm_capture_config_s *info;
  struct aurix_egtm_capture_dev_s *capdev;
  Ifx_EGTM_CLS_TIM_CH_GPR0 gpr0;
  Ifx_EGTM_CLS_TIM_CH *channel;
  bool level;

  capdev = (struct aurix_egtm_capture_dev_s *)dev;
  info = capdev->privinfo;

  /* In non-TIEM mode, the level value cannot be obtained. */

  if (capdev->egtm_cap_mode != GTM_CAPTURE_MODE_TIEM)
    {
      return 0;
    }

  /* Get the TIM channel instance. */

  channel = egtm_capture_channel(capdev);

  /* Obtain the level of the input signal. */

  gpr0.U = channel->GPR0.U;
  level = (gpr0.B.GPR0 & (0x1 << (info->gtm_tim_ch + 16))) ? 1 : 0;

  return level;
}

/****************************************************************************
 * Name: aurix_egtm_capture_set_callback
 *
 * Description:
 *   Register a callback function, and the callback function will be called
 *   in the interrupt handling function.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *   callback  - The callback function registered to the specified channel.
 *
 * Returned Value:
 *   NONE
 *
 ****************************************************************************/

void aurix_gtm_capture_set_callback(struct cap_lowerhalf_s *dev,
  gtm_capture_callback_t callback)
{
  struct aurix_egtm_capture_dev_s *capdev;

  DEBUGASSERT(dev != NULL);

  capdev = (struct aurix_egtm_capture_dev_s *)dev;
  capdev->egtm_callback = callback;
}

/****************************************************************************
 * Name: aurix_egtm_capture_set_capmode
 *
 * Description:
 *   Set the working mode of the specified channel. The setting can be
 *   successful only when the channel is in the closed state.
 *
 * Input Parameters:
 *   devs  - The pointer array of channel device to be initialized.
 *   mode  - The working mode of the specified eGTM TIM channel.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns error code.
 *
 ****************************************************************************/

int aurix_gtm_capture_set_capmode(struct cap_lowerhalf_s *dev,
  enum egtm_capture_mode_e mode)
{
  struct aurix_egtm_capture_dev_s *capdev;

  DEBUGASSERT(dev != NULL);

  capdev = (struct aurix_egtm_capture_dev_s *)dev;
  if (capdev->enabled)
    {
      return -EBUSY;
    }

  capdev->egtm_cap_mode = mode;
  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_capture_initialize
 *
 * Description:
 *   This function initializes the specified eGTM peripheral and the capture
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   dev - Channel device to be initialized.
 *   cfg - Configuration of the eGTM TIM channel.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

static int aurix_egtm_capture_initialize(
  struct aurix_egtm_capture_dev_s *dev,
  const struct aurix_gtm_capture_config_s *cfg)
{
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(cfg != NULL);
  DEBUGASSERT(cfg->gtm_tim_pin != NULL);
  DEBUGASSERT(cfg->capture_devpath != NULL);

  /* Initialize the lower half driver structure. */

  dev->privinfo      = cfg;
  dev->lowerhalf.ops = &g_egtm_cap_ops;
  dev->ready         = false;
  dev->enabled       = false;

  /* Register the eGTM TIM channel interrupt handler. */

  if (irq_attach(cfg->gtm_tim_ch_irq, egtm_capture_isr, dev) < 0)
    {
      cperr("Couldn't register IRQ handler.\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_egtm_capture_all_initialize
 *
 * Description:
 *   Initialize and register eGTM capture device.
 *
 * Input Parameters:
 *   devs - The pointer array of channel device to be initialized.
 *   cfgs - The list of configuration for the eGTM TIM channel.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns error code.
 *
 ****************************************************************************/

int aurix_egtm_capture_all_initialize(struct cap_lowerhalf_s **devs,
  const struct aurix_gtm_capture_config_s *cfgs, size_t count)
{
  size_t i;
  size_t ch;

  DEBUGASSERT(devs != NULL);
  DEBUGASSERT(cfgs != NULL);

  for (i = 0; i < count; i++)
    {
      /* Get the channel corresponding to the configuration. */

      ch = cfgs[i].gtm_tim_ch;
      if (ch > IfxEgtm_Tim_Ch_7)
        {
          cperr("ERROR: Invalid channel index\n");
          continue;
        }

      /* Allocate the capture channel device. */

      devs[i] = (struct cap_lowerhalf_s *)
        kmm_zalloc(sizeof(struct aurix_egtm_capture_dev_s));
      if (devs[i] == NULL)
        {
          cperr("ERROR: Error allocating capture\n");
          return -ENOMEM;
        }

      /* Initialize and register the capture channel device. */

      if (aurix_egtm_capture_initialize(
            (struct aurix_egtm_capture_dev_s *)devs[i], &cfgs[i]) < 0 ||
          cap_register(cfgs[i].capture_devpath, devs[i]) < 0)
        {
          kmm_free(devs[i]);
          devs[i] = NULL;
          cperr("ERROR: Error registering capture\n");
          return ERROR;
        }
    }

  return OK;
}
