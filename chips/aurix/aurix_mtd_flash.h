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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_MTD_FLASH__H
#define __VENDOR_INFINEON_CHIPS_AURIX_MTD_FLASH__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/rwsem.h>

#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxPort.h"

#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
#include "IfxFlash.h"
#include "IfxFlashCsrm.h"
#include "IfxPfrwb_reg.h"
#include "IfxFlash_cfg.h"
#else
#include "IfxNvmr.h"
#include "IfxNvmr_cfg.h"
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * LOCAL MACROS
 ****************************************************************************/

#define MEM(address)                 *((uint32 *)(address))
#define MTD_ERASED_STATE             (0x0)
#define NO_SWAP                      (0)
#define SLOT_A                       (1)
#define SLOT_B                       (2)
#define OTA_BANK_A                   (SLOT_A)
#define OTA_BANK_B                   (SLOT_B)
#define OFFSET_BETWEEN_BANKA_BANKB   (0x2000000)

#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
# define FLASH_TYPE                   IfxFlash_FlashType
# define FLASH_INVALID_TYPE           IfxFlash_FlashType_Invalid
# define PFLASH_SECTOR_SIZE           (IFXFLASH_PFLASH_SIZE \
                                       / IFXFLASH_PFLASH_NUM_LOG_SECTORS)
# define DFLASH_SECTOR_SIZE           (IFXFLASH_DFLASH_SIZE \
                                       / IFXFLASH_DFLASH_NUM_LOG_SECTORS)
# define DFLASH_PAGE_SIZE             IFXFLASH_DFLASH_PAGE_LENGTH
# define PFLASH_PAGE_SIZE             IFXFLASH_PFLASH_PAGE_LENGTH
# define DFLASH_SIZE                  IFXFLASH_DFLASH_SIZE
# define PFLASH_START                 IFXFLASH_PFLASH_START
# define DFLASH_START                 IFXFLASH_DFLASH_START
# define CSRM_PFLASH_START            IFXFLASHCSRM_PFLASH_START
# define CSRM_DFLASH_START            IFXFLASHCSRM_DFLASH_START
# define CSRM_PFLASH_SIZE             IFXFLASHCSRM_PFLASH_SIZE
# define CSRM_DFLASH_SIZE             IFXFLASHCSRM_DFLASH_SIZE
# define GET_BANK_FROM_ADDR(addr)     IfxFlash_getBankFromAddress(addr)
# define PFLASH_BANK_A_SIZE           (IFXFLASH_PFLASH_P00_SIZE \
                                       + IFXFLASH_PFLASH_P10_SIZE \
                                       + IFXFLASH_PFLASH_P20_SIZE \
                                       + IFXFLASH_PFLASH_P30_SIZE \
                                       + IFXFLASH_PFLASH_P40_SIZE \
                                       + IFXFLASH_PFLASH_P50_SIZE)

# define PFLASH_BANK_B_SIZE           (IFXFLASH_PFLASH_P01_SIZE \
                                       + IFXFLASH_PFLASH_P11_SIZE \
                                       + IFXFLASH_PFLASH_P21_SIZE \
                                       + IFXFLASH_PFLASH_P31_SIZE \
                                       + IFXFLASH_PFLASH_P41_SIZE \
                                       + IFXFLASH_PFLASH_P51_SIZE)
#define  PFLASH_BANKS                 IFXFLASH_PFLASH_BANKS
#define  DFLASH_BANKS                 IFXFLASH_DFLASH_BANKS
#define  CSRM_PFLASH_BANKS            IFXFLASHCSRM_PFLASH_BANKS
#define  CSRM_DFLASH_BANKS            IFXFLASHCSRM_DFLASH_BANKS

#else
# define FLASH_TYPE                   IfxNvmr_NvmrType
# define FLASH_INVALID_TYPE           IfxNvmr_NvmrType_Invalid
# define PFLASH_SECTOR_SIZE           IFXNVMR_PNVM_PAGE_LENGTH
# define DFLASH_SECTOR_SIZE           IFXNVMR_DNVM_PAGE_LENGTH
# define DFLASH_PAGE_SIZE             IFXNVMR_DNVM_PAGE_LENGTH
# define PFLASH_PAGE_SIZE             IFXNVMR_PNVM_PAGE_LENGTH
# define DFLASH_SIZE                  IFXNVMR_DNVM_SIZE
# define PFLASH_START                 IFXNVMR_PNVM00_START
# define DFLASH_START                 IFXNVMR_DNVM_START
# define CSRM_PFLASH_START            IFXNVMRCSRM_PNVM00_START
# define CSRM_DFLASH_START            IFXNVMRCSRM_DNVM_START
# define CSRM_PFLASH_SIZE             IFXNVMRCSRM_PNVM_SIZE
# define CSRM_DFLASH_SIZE             IFXNVMRCSRM_DNVM_SIZE

# define GET_BANK_FROM_ADDR(addr)     IfxNvmr_getBankFromAddress(addr)
# define PFLASH_BANK_A_SIZE           (IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE)

# define PFLASH_BANK_B_SIZE           (IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE \
                                       + IFXNVMR_PNVM_SIZE)
#define  PFLASH_BANKS                 IFXNVMR_PNVM_BANKS
#define  DFLASH_BANKS                 IFXNVMR_DNVM_BANKS
#define  CSRM_PFLASH_BANKS            IFXNVMRCSRM_PNVM_BANKS
#define  CSRM_DFLASH_BANKS            IFXNVMRCSRM_DNVM_BANKS
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* logic address to Pysical address map */

typedef struct
{
  FLASH_TYPE type;
  uint32_t start_addr;
  uint32_t ota_bank;
  uint32_t start_addr_map_a;
  uint32_t start_addr_map_b;
  uint32_t size;
}flash_map_t;

struct aurix_mtd_cfg_s
{
  uint32_t sectorsz;
  uint32_t nsectors;
  uint32_t pagesz;
  uint32_t baseaddr;
};

/* Aurix flash device private data  */

struct aurix_mtd_priv_s
{
  struct mtd_dev_s mtd;
  rw_semaphore_t lock[MAX(MAX(PFLASH_BANKS, DFLASH_BANKS),
                          MAX(CSRM_PFLASH_BANKS, CSRM_DFLASH_BANKS))];
  const struct aurix_mtd_cfg_s *cfg;
  bool is_dflash;
  bool is_csrm;
  const flash_map_t *flash_map;
};

/****************************************************************************
 * Name: aurix_flash_mtd
 *
 * Description:
 *   Get Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Flash MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *aurix_mtd_flash(const char *name,
                                  const struct aurix_mtd_cfg_s *cfg,
                                  bool is_dflash, bool is_csrm,
                                  const flash_map_t *flash_map);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __VENDOR_INFINEON_CHIPS_AURIX_MTD_FLASH__H */
