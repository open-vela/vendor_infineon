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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_IOEXPANDER__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_IOEXPANDER__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mutex.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <Port/Std/IfxPort.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_ioexpander_config_s
{
  Ifx_P *port;     /* Port address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
  size_t count);

#ifdef __cplusplus
}
#endif

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_IOEXPANDER__H */
