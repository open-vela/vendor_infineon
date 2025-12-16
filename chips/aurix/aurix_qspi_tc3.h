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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/spi/spi.h>

#include <Qspi/SpiMaster/IfxQspi_SpiMaster.h>
#include <IfxAsclin_PinMap.h>
#include <IfxQspi_regdef.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_qspi_config_s
{
  const IfxQspi_SpiMaster_Output *cs;
  const IfxQspi_SpiMaster_Pins pins;
  const int *cs_active;
  Ifx_QSPI *qspi;
  float32 baudrate;
  int datawidth;
  int err_irq;
  int tx_irq;
  int rx_irq;
  int cs_num;
  int dev_id;
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_qspi_initialize
 *
 * Description:
 *   Initialize qspi deivce for aurix.
 *
 ****************************************************************************/

int aurix_qspi_initialize(struct spi_dev_s **dev,
                          const struct aurix_qspi_config_s *config,
                          size_t count);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_H */
