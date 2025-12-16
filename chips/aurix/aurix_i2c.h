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

#ifndef __VENDOR_INFINEON_CHIP_AURIX_AURIX_I2C_H
#define __VENDOR_INFINEON_CHIP_AURIX_AURIX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>

#include "I2c/I2c/IfxI2c_I2c.h"

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct aurix_i2c_config_s
{
  Ifx_I2C *module;   /* I2C module */
  IfxI2c_Pins pins;  /* I2C pins */
  int bus;           /* I2C bus number */
  float32 baudrate;  /* I2C baudrate */
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

int aurix_i2c_initialize(struct i2c_master_s **dev,
                         const struct aurix_i2c_config_s *config,
                         size_t num);

#endif /* __VENDOR_INFINEON_CHIP_AURIX_AURIX_I2C_H */
