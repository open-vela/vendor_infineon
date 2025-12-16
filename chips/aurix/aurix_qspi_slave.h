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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_SLAVE_H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <Qspi/SpiSlave/IfxQspi_SpiSlave.h>
#include <IfxQspi_regdef.h>

#include <nuttx/spi/slave.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct aurix_qspi_slave_config_s
{
  IfxQspi_SpiSlave_Pins pins; /* QSPI pins */
  Ifx_QSPI *qspi;             /* QSPI base address */
  int tx_irq;                 /* TX DMA interrupt, currently enable dma default */
  int rx_irq;                 /* RX DMA interrupt */
  int err_irq;                /* ERR interrupt */
  int pt_irq;                 /* PT2 interrupt */
  bool pt_enable;             /* Enalbe SLSI deactivated interrupt */
  bool usedma;
  int txdmachannel;
  int rxdmachannel;
  int dmaindex;
  float32 maxbaudrate;
  uint32_t bufsize;           /* Max data size to be sent/receive of one transfer process */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_qspi_slave_initialize
 *
 * Description:
 *   Initialize the qspi slave controllers according to the provided
 *   configuration.
 *
 * Input Parameters:
 *   ctrlr  - pointer to the controller structure
 *   config - pointer to the configuration structure for each controller
 *   count  - number of controllers to be initialized
 *
 * Returned Value:
 *   Zero OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int aurix_qspi_slave_initialize(struct spi_slave_ctrlr_s **ctrlr,
  const struct aurix_qspi_slave_config_s *config, size_t count);

#endif /*  __VENDOR_INFINEON_CHIPS_AURIX_AURIX_QSPI_SLAVE_H */
