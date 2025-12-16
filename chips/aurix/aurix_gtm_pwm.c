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

#include <nuttx/timers/pwm.h>
#include <arch/chip/chip.h>
#include "aurix_gtm_pwm.h"

#ifdef CONFIG_AURIX_GTM_TOM_PWM

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Upper-half functions required by pwm driver */

static int aurix_pwm_setup(struct pwm_lowerhalf_s *dev);
static int aurix_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int aurix_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int aurix_pwm_stop(struct pwm_lowerhalf_s *dev);
static int aurix_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half methods required by capture driver. */

static const struct pwm_ops_s g_pwmops =
{
  .setup    = aurix_pwm_setup,
  .shutdown = aurix_pwm_shutdown,
  .start    = aurix_pwm_start,
  .stop     = aurix_pwm_stop,
  .ioctl    = aurix_pwm_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 ****************************************************************************/

static int aurix_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct aurix_gtm_tom_pwm_dev_s *priv =
    (struct aurix_gtm_tom_pwm_dev_s *)dev;

  /* Update CM0/CM1 registers from Shadow at end of period */

  priv->tom_config.synchronousUpdateEnabled = TRUE;

  /* Start PWM at end of init */

  priv->tom_config.immediateStartEnabled = TRUE;

  return OK;
}

/****************************************************************************
 * Name: aurix_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int aurix_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct aurix_gtm_tom_pwm_dev_s *priv =
    (struct aurix_gtm_tom_pwm_dev_s *)dev;

  IfxGtm_Tom_Pwm_stop(&priv->tom_driver, TRUE);

  return OK;
}

/****************************************************************************
 * Name: aurix_pwm_start
 *
 * Description:
 *   Start the pulsed output
 *
 ****************************************************************************/

static int aurix_pwm_start(struct pwm_lowerhalf_s *dev,
  const struct pwm_info_s *info)
{
  struct aurix_gtm_tom_pwm_dev_s *priv =
    (struct aurix_gtm_tom_pwm_dev_s *)dev;

    DEBUGASSERT(info->frequency > 0);
    DEBUGASSERT(b16tof(info->duty) >= 0.0 && b16tof(info->duty) <= 100.0);

  /* Get the clock frequency of the tom_ch */

  IfxGtm_Cmu_Fxclk fxclk_src = (IfxGtm_Cmu_Fxclk)priv->tom_config.clock;

  float32 ch_src_freq =
      IfxGtm_Cmu_getFxClkFrequency(&MODULE_GTM, fxclk_src, FALSE);

  if ((info->frequency < (((uint32)ch_src_freq) / 65535)) ||
  (info->frequency > (((uint32)ch_src_freq) / 100)))
    {
      return -EINVAL;
    }

  priv->frequency = (float32)info->frequency;
  priv->duty = b16tof(info->duty);

  /* Period in ticks */

  priv->tom_config.period = (uint16)(ch_src_freq / priv->frequency);

  /* Duty cycle in ticks */

  priv->tom_config.dutyCycle = (uint16)(((uint32)priv->tom_config.period *
       (priv->duty * 100)) / 10000);

  if (priv->running == false)
    {
      /* Initialize the GTM TOM */

      IfxGtm_Tom_Pwm_init(&priv->tom_driver, &priv->tom_config);
      priv->running = true;
    }
  else
    {
      Ifx_GTM_TOM *tomSFR = &priv->tom_config.gtm->TOM[priv->tom_config.tom];

      IfxGtm_Tom_Ch_setCompareZeroShadow(tomSFR, priv->tom_config.tomChannel,
        priv->tom_config.period);
      IfxGtm_Tom_Ch_setCompareOneShadow(tomSFR, priv->tom_config.tomChannel,
        priv->tom_config.dutyCycle);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_pwm_stop
 *
 * Description:
 *   Stop the pulsed output
 *
 ****************************************************************************/

static int aurix_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct aurix_gtm_tom_pwm_dev_s *priv =
    (struct aurix_gtm_tom_pwm_dev_s *)dev;

  IfxGtm_Tom_Pwm_stop(&priv->tom_driver, TRUE);
  priv->running = false;

  return OK;
}

/****************************************************************************
 * Name: aurix_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int aurix_pwm_ioctl(struct pwm_lowerhalf_s *dev,
  int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gtm_tom_pwm_cfg_initialize
 *
 * Description:
 *   This function initializes the specified GTM peripheral and the pwm
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   cfgs  - Configuration for the GTM TOM channel.
 *   devs  - The pointer array of channel device to be initialized.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_gtm_tom_pwm_cfg_initialize(
  const struct aurix_pwm_cfg_s *cfgs,
  struct pwm_lowerhalf_s **devs, size_t count)
{
  struct aurix_gtm_tom_pwm_dev_s *priv;

  for (int i = 0; i < count; i++)
    {
      if (cfgs[i].module_type == GTM_Module_Type_TOM)
        {
          /* Allocate the pwm channel device. */

          devs[i] = (struct pwm_lowerhalf_s *)
            kmm_zalloc(sizeof(struct aurix_gtm_tom_pwm_dev_s));
          if (devs[i] == NULL)
            {
              cperr("ERROR: Error allocating pwm\n");
              return -ENOMEM;
            }

          priv = (struct aurix_gtm_tom_pwm_dev_s *)devs[i];

          /* Initialize the pwm channel device. */

          IfxGtm_Tom_Pwm_initConfig(&priv->tom_config, &MODULE_GTM);
          priv->tom_config.tom = (IfxGtm_Tom)cfgs[i].module_id;
          priv->tom_config.tomChannel = (IfxGtm_Tom_Ch)cfgs[i].channel_id;
          priv->tom_config.clock = (IfxGtm_Tom_Ch_ClkSrc)cfgs[i].ch_clk_id;
          priv->tom_config.pin.outputPin = cfgs[i].pin_map;
          priv->tom_config.pin.padDriver = cfgs[i].driver_strength;
          priv->tom_config.pin.outputMode = cfgs[i].output_mode;
          priv->tom_config.interrupt.isrProvider = cfgs[i].isr_provider;
          priv->running = false;
          priv->lowerhalf.ops = &g_pwmops;

          /* Then register the pwm sensor. */

          if (pwm_register(cfgs[i].devpath, &priv->lowerhalf) < 0)
            {
              kmm_free(devs[i]);
              devs[i] = NULL;
              syslog(LOG_ERR, "Error registering %s\n", cfgs[i].devpath);
              return ERROR;
            }
        }
    }

  return OK;
}

#endif /* CONFIG_AURIX_GTM_TOM_PWM */
