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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_SPI__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_SPI__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <Asclin/Spi/IfxAsclin_Spi.h>
#include <nuttx/spi/spi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_spi_config_s
{
  IfxAsclin_Spi_Config *config;
  IfxAsclin_Slso_Out   *slso_0;
  IfxAsclin_Slso_Out   *slso_1;
  IfxAsclin_Slso_Out   *slso_2;
  IfxPort_OutputMode   slso_mode;
  Ifx_ASCLIN           *asclin;
  IfxPort_PadDriver    pinDriver;
  int                  tx_irq;
  int                  rx_irq;
  int                  err_irq;
  IfxAsclin_Spi_Pins   pins;
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
  IfxSrc_VmId          vm_id;
#endif
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_all_spi_initialize
 *
 * Description:
 *   Initialize all spi deivce for aurix.
 *
 ****************************************************************************/

int aurix_all_spi_initialize(struct spi_dev_s **dev,
                             struct aurix_spi_config_s *config,
                             size_t count);

/****************************************************************************
 * Name: aurix_spi_deinit
 *
 * Description:
 *   Uninitialize spi deivce for aurix.
 *
 ****************************************************************************/

void aurix_spi_deinit(struct spi_dev_s *dev);

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_SPI__H_ */
