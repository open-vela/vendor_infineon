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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <arch/chip/chip.h>

#include "aurix_ioexpander.h"

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct aurix_ioexpander_dev_s
{
  struct ioexpander_dev_s  dev;
  mutex_t                  lock;
  Ifx_P                   *port;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_ioexpander_direction(struct ioexpander_dev_s *dev,
                                      uint8_t pin, int dir);
static int aurix_ioexpander_option(struct ioexpander_dev_s *dev,
                                   uint8_t pin, int opt, void *val);
static int aurix_ioexpander_writepin(struct ioexpander_dev_s *dev,
                                     uint8_t pin, bool value);
static int aurix_ioexpander_readpin(struct ioexpander_dev_s *dev,
                                    uint8_t pin, bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int aurix_ioexpander_multiwritepin(struct ioexpander_dev_s *dev,
                                          const uint8_t *pins,
                                          const bool *values, int count);
static int aurix_ioexpander_multireadpin(struct ioexpander_dev_s *dev,
                                         const uint8_t *pins,
                                         bool *values, int count);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O expander vtable */

static const struct ioexpander_ops_s g_aurix_ioexpander_ops =
{
  aurix_ioexpander_direction,
  aurix_ioexpander_option,
  aurix_ioexpander_writepin,
  aurix_ioexpander_readpin,
  aurix_ioexpander_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , aurix_ioexpander_multiwritepin
  , aurix_ioexpander_multireadpin
  , aurix_ioexpander_multireadpin
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_ioexpander_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_direction(struct ioexpander_dev_s *dev,
                                      uint8_t pin, int direction)
{
  struct aurix_ioexpander_dev_s *ioe =
                             (struct aurix_ioexpander_dev_s *)dev;
  nxmutex_lock(&ioe->lock);
  switch (direction)
    {
      case IOEXPANDER_DIRECTION_IN:
        {
          IfxPort_setPinModeInput(ioe->port, pin,
                                  IfxPort_InputMode_noPullDevice);
        }
        break;

      case IOEXPANDER_DIRECTION_IN_PULLUP:
        {
          IfxPort_setPinModeInput(ioe->port, pin,
                                  IfxPort_InputMode_pullUp);
        }
        break;

      case IOEXPANDER_DIRECTION_IN_PULLDOWN:
        {
          IfxPort_setPinModeInput(ioe->port, pin,
                                  IfxPort_InputMode_pullDown);
        }
        break;

      case IOEXPANDER_DIRECTION_OUT:
        {
          IfxPort_setPinModeOutput(ioe->port, pin,
                                   IfxPort_OutputMode_pushPull,
                                   IfxPort_OutputIdx_general);
        }
        break;

      case IOEXPANDER_DIRECTION_OUT_OPENDRAIN:
        {
          IfxPort_setPinModeOutput(ioe->port, pin,
                                   IfxPort_OutputMode_openDrain,
                                   IfxPort_OutputIdx_general);
        }
        break;

      default:
        {
          nxmutex_unlock(&ioe->lock);
          return -EINVAL; /* Return without changing the pin settings */
        }
        break;
    }

  nxmutex_unlock(&ioe->lock);
  return OK;
}

/****************************************************************************
 * Name: aurix_ioexpander_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_option(struct ioexpander_dev_s *dev,
                                   uint8_t pin, int opt, void *value)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: aurix_ioexpander_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_writepin(struct ioexpander_dev_s *dev,
                                     uint8_t pin, bool value)
{
  struct aurix_ioexpander_dev_s *ioe = (struct aurix_ioexpander_dev_s *)dev;

  nxmutex_lock(&ioe->lock);
  if (value)
    {
      IfxPort_setPinHigh(ioe->port, pin);
    }
  else
    {
      IfxPort_setPinLow(ioe->port, pin);
    }

  nxmutex_unlock(&ioe->lock);
  return OK;
}

/****************************************************************************
 * Name: aurix_ioexpander_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *      written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   value  - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_readpin(struct ioexpander_dev_s *dev,
                                    uint8_t pin, bool *value)
{
  struct aurix_ioexpander_dev_s *ioe = (struct aurix_ioexpander_dev_s *)dev;

  nxmutex_lock(&ioe->lock);
  *value = IfxPort_getPinState(ioe->port, pin);
  nxmutex_unlock(&ioe->lock);
  return OK;
}

#ifdef CONFIG_IOEXPANDER_MULTIPIN
/****************************************************************************
 * Name: aurix_ioexpander_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_multiwritepin(struct ioexpander_dev_s *dev,
                                          const uint8_t *pins,
                                          const bool *values, int count)
{
  struct aurix_ioexpander_dev_s *ioe = (struct aurix_ioexpander_dev_s *)dev;
  uint16_t data = 0;
  uint16_t mask = 0;
  uint8_t pin;
  int i;

  nxmutex_lock(&ioe->lock);

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&ioe->lock);
          return -ENXIO;
        }

      if (values[i])
        {
          data |= (1 << pin);
        }
      else
        {
          data &= ~(1 << pin);
        }

      mask |= (1 << pin);
    }

  IfxPort_setGroupState(ioe->port, 0, mask, data);
  nxmutex_unlock(&ioe->lock);
  return OK;
}

/****************************************************************************
 * Name: aurix_ioexpander_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int aurix_ioexpander_multireadpin(struct ioexpander_dev_s *dev,
                                         const uint8_t *pins,
                                         bool *values, int count)
{
  struct aurix_ioexpander_dev_s *ioe = (struct aurix_ioexpander_dev_s *)dev;
  uint32_t data;
  uint8_t pin;
  int i;

  nxmutex_lock(&ioe->lock);

  data = IfxPort_getGroupState(ioe->port, 0, 0xffff);

  for (i = 0; i < count; i++)
    {
      pin = pins[i];
      if (pin > 15)
        {
          nxmutex_unlock(&ioe->lock);
          return -ENXIO;
        }

      values[i] = (data >> pin) & 1;
    }

  nxmutex_unlock(&ioe->lock);
  return OK;
}
#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_ioexpander_initialize
 *
 * Description:
 *   Instantiate and configure the AURIX_IOEXPANDER device driver to use the
 *   provided configs.
 *
 * Input Parameters:
 *   ioe    - ioexpander device handle list
 *   config - The config list for ioexpander
 *   count  - the number of ioexpander devices to be initialized
 *
 * Returned Value:
 *   OK on success, errno on failure.
 *
 ****************************************************************************/

int aurix_ioexpander_initialize(struct ioexpander_dev_s **ioe,
  const struct aurix_ioexpander_config_s *config,
  size_t count)
{
  struct aurix_ioexpander_dev_s *ioedev;
  int i;

  DEBUGASSERT(config != NULL);

  for (i = 0; i < count && ioe[i] != DEV_END; i++)
    {
      ioedev = kmm_zalloc(sizeof(*ioedev));
      DEBUGASSERT(ioedev != NULL);

      ioedev->dev.ops = &g_aurix_ioexpander_ops;
      ioedev->port = config[i].port;
      nxmutex_init(&ioedev->lock);
      ioe[i] = (struct ioexpander_dev_s *)ioedev;
    }

  return 0;
}
