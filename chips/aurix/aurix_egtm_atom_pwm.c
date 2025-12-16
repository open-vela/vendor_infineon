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
#include <nuttx/kmalloc.h>
#include <arch/chip/chip.h>

#include "aurix_egtm_atom_pwm.h"
#include "Egtm/Atom/Pwm/IfxEgtm_Atom_Pwm.h"

#ifdef CONFIG_AURIX_EGTM_ATOM_PWM

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Upper-half functions required by pwm driver */

static int aurix_atom_pwm_setup(struct pwm_lowerhalf_s *dev);
static int aurix_atom_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int aurix_atom_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int aurix_atom_pwm_stop(struct pwm_lowerhalf_s *dev);
static int aurix_atom_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half methods required by capture driver. */

static const struct pwm_ops_s g_pwmops =
{
  .setup    = aurix_atom_pwm_setup,
  .shutdown = aurix_atom_pwm_shutdown,
  .start    = aurix_atom_pwm_start,
  .stop     = aurix_atom_pwm_stop,
  .ioctl    = aurix_atom_pwm_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_atom_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 ****************************************************************************/

static int aurix_atom_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct aurix_egtm_atom_pwm_dev_s *priv =
    (struct aurix_egtm_atom_pwm_dev_s *)dev;

  /* Update CM0/CM1 registers from Shadow at end of period */

  priv->atom_config.synchronousUpdateEnabled = TRUE;

  /* Start PWM at end of init */

  priv->atom_config.immediateStartEnabled = TRUE;

  return OK;
}

/****************************************************************************
 * Name: aurix_atom_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int aurix_atom_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct aurix_egtm_atom_pwm_dev_s *priv =
    (struct aurix_egtm_atom_pwm_dev_s *)dev;

  IfxEgtm_Atom_Pwm_stop(&priv->atom_driver, TRUE);

  return OK;
}

/****************************************************************************
 * Name: aurix_atom_pwm_start
 *
 * Description:
 *   Start the pulsed output
 *
 ****************************************************************************/

static int aurix_atom_pwm_start(struct pwm_lowerhalf_s *dev,
  const struct pwm_info_s *info)
{
  struct aurix_egtm_atom_pwm_dev_s *priv =
    (struct aurix_egtm_atom_pwm_dev_s *)dev;

  DEBUGASSERT(info->frequency > 0);
  DEBUGASSERT(b16tof(info->duty) >= 0.0 && b16tof(info->duty) <= 100.0);

  /* Get the clock frequency of the atom_ch */

  IfxEgtm_Cmu_Clk clk_src = (IfxEgtm_Cmu_Clk)priv->atom_config.clock;

  float32 ch_src_freq = IfxEgtm_Cmu_getClkFrequency
    (&MODULE_EGTM, clk_src, FALSE);

  /* Mininmum frequency range detection */

  if ((info->frequency < (((uint32)ch_src_freq) / 16777215)) ||
          (info->frequency > (((uint32)ch_src_freq) / 100)))
    {
      return -EINVAL;
    }

  priv->frequency = (float32)info->frequency;
  priv->duty = b16tof(info->duty);

  /* Period in ticks */

  priv->atom_config.period = (uint32)(ch_src_freq / priv->frequency);

  /* Duty cycle in ticks */

  priv->atom_config.dutyCycle = (uint32)(((uint32)priv->atom_config.period *
       (priv->duty * 100)) / 10000);

  if (priv->running == false)
    {
      /* Initialize the EGTM ATOM */

      IfxEgtm_Atom_Pwm_init(&priv->atom_driver, &priv->atom_config);
      priv->running = true;
    }
  else
    {
      Ifx_EGTM_CLS_ATOM *atomSFR =
          &priv->atom_config.egtm->CLS[priv->atom_config.cluster].ATOM;

      IfxEgtm_Atom_Ch_setCompareZeroShadow(atomSFR,
          priv->atom_config.atomChannel, priv->atom_config.period);
      IfxEgtm_Atom_Ch_setCompareOneShadow(atomSFR,
          priv->atom_config.atomChannel, priv->atom_config.dutyCycle);
    }

  return OK;
}

/****************************************************************************
 * Name: aurix_atom_pwm_stop
 *
 * Description:
 *   Stop the pulsed output
 *
 ****************************************************************************/

static int aurix_atom_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct aurix_egtm_atom_pwm_dev_s *priv =
    (struct aurix_egtm_atom_pwm_dev_s *)dev;

  IfxEgtm_Atom_Pwm_stop(&priv->atom_driver, TRUE);
  priv->running = false;

  return OK;
}

/****************************************************************************
 * Name: aurix_atom_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int aurix_atom_pwm_ioctl(struct pwm_lowerhalf_s *dev,
  int cmd, unsigned long arg)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_egtm_atom_pwm_cfg_initialize
 *
 * Description:
 *   This function initializes the specified eGTM peripheral and the pwm
 *   submodule with the provided configuration.
 *
 * Input Parameters:
 *   cfgs  - Configuration for the eGTM ATOM channel.
 *   devs  - The pointer array of channel device to be initialized.
 *   count - The number of channel to be initialized.
 *
 * Returned Value:
 *   On success, it returns OK. If fails, it returns ERROR.
 *
 ****************************************************************************/

int aurix_egtm_atom_pwm_cfg_initialize(
  const struct aurix_pwm_cfg_s * cfgs,
  struct pwm_lowerhalf_s** devs, size_t count)
{
  struct aurix_egtm_atom_pwm_dev_s * priv;
  /* int ch; */

  for (int i = 0; i < count; i++)
    {
      if (cfgs[i].module_type == GTM_Module_Type_ATOM)
        {
          /* Get the channel corresponding to the configuration. */

          /* ch = cfgs[i].channel_id; */

          /* Allocate the pwm channel device. */

          devs[i] = (struct pwm_lowerhalf_s *)
              kmm_zalloc(sizeof(struct aurix_egtm_atom_pwm_dev_s));
          if (devs[i] == NULL)
            {
              return -ENOMEM;
            }

          priv = (struct aurix_egtm_atom_pwm_dev_s *)devs[i];

          /* Initialize the pwm channel device. */

          IfxEgtm_Atom_Pwm_initConfig(&priv->atom_config, &MODULE_EGTM);
          priv->atom_config.cluster = cfgs[i].module_id;
          priv->atom_config.atomChannel = cfgs[i].channel_id;
          priv->atom_config.clock = cfgs[i].ch_clk_id;
          priv->atom_config.pin.outputPin = cfgs[i].pin_map;
          priv->atom_config.pin.padDriver = cfgs[i].driver_strength;
          priv->atom_config.pin.outputMode = cfgs[i].output_mode;
          priv->atom_config.interrupt.isrProvider = cfgs[i].isr_provider;
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
#endif /* CONFIG_AURIX_EGTM_ATOM_PWM */
