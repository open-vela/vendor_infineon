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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "aurix_i2c.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_i2c_dev_s
{
  struct i2c_master_s dev;       /* I2c private data */
  IfxI2c_I2c_Config i2c_config;  /* Structure to configure the module */
  IfxI2c_I2c_Device i2c_dev;     /* Structure with slave device data */
  IfxI2c_I2c i2c;                /* I2C handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int aurix_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int aurix_i2c_reset(FAR struct i2c_master_s *dev);
#endif
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer = aurix_i2c_transfer
#ifdef CONFIG_I2C_RESET
  , .reset  = aurix_i2c_reset
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aurix_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *msgs, int count)
{
  struct aurix_i2c_dev_s *priv = (struct aurix_i2c_dev_s *)dev;
  IfxI2c_I2c_Status status = IfxI2c_I2c_Status_ok;

  /* Because it is 7 bit long and bit 0 is R/W bit, the device address has
   * to be shifted by 1
   */

  priv->i2c_dev.deviceAddress = msgs->addr << 1;

  while (count--)
    {
      if (msgs->flags & I2C_M_READ)
        {
          status = IfxI2c_I2c_read(&priv->i2c_dev, msgs->buffer,
                                   msgs->length);
        }
      else
        {
          status = IfxI2c_I2c_write(&priv->i2c_dev, msgs->buffer,
                                    msgs->length);
        }

      if (status != IfxI2c_I2c_Status_ok)
        {
          i2cerr("aurix_i2c_transfer failed status=%d\n", status);
          break;
        }

       msgs++;
    }

  return status;
}

#ifdef CONFIG_I2C_RESET
static int aurix_i2c_reset(FAR struct i2c_master_s *dev)
{
  struct aurix_i2c_dev_s *priv = (struct aurix_i2c_dev_s *)dev;

  IfxI2c_resetModule(priv->i2c_config.i2c);
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_i2c_initialize
 *
 * Description:
 *   aurix i2c bus initialize function
 *
 ****************************************************************************/

int aurix_i2c_initialize(struct i2c_master_s **dev,
                         const struct aurix_i2c_config_s *config,
                         size_t num)
{
  IfxI2c_I2c_deviceConfig i2c_device_config;
  struct aurix_i2c_dev_s *priv;
  int ret = -EINVAL;
  size_t i;

  for (i = 0; i < num; i++)
    {
      priv = kmm_zalloc(sizeof(struct aurix_i2c_dev_s));
      if (priv == NULL)
        {
          i2cerr("i2c%ld kmm_zalloc failed\n", i);
          return -ENOMEM;
        }

      /* Initialize i2c module */

      IfxI2c_I2c_initConfig(&priv->i2c_config, config[i].module);
      priv->i2c_config.pins = &config[i].pins;
      priv->i2c_config.baudrate = config[i].baudrate;
      IfxI2c_I2c_initModule(&priv->i2c, &priv->i2c_config);

      /* Initialize i2c device */

      IfxI2c_I2c_initDeviceConfig(&i2c_device_config, &priv->i2c);
      IfxI2c_I2c_initDevice(&priv->i2c_dev, &i2c_device_config);

      /* Register i2c */

      priv->dev.ops = &g_i2c_ops;
      ret = i2c_register(&priv->dev, config[i].bus);
      if (ret < 0)
        {
          kmm_free(priv);
          i2cerr("I2c register failed, ret = %d\n", ret);
          break;
        }

      dev[config[i].bus] = &priv->dev;
    }

  return ret;
}

