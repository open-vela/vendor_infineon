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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <sys/types.h>
#include <inttypes.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include "IfxCpu.h"
#include "IfxCpucfi_reg.h"
#include "IfxSmm_reg.h"
#include "aurix_mtd_flash.h"
#include "Ifx_Ssw_Compilers.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define CURRENT_SLOT (SMM_STMEM1.B.SCFG)
#define PFLASH_MAX_ERASE_SIZE (512*1024)
#define DFLASH_MAX_ERASE_SIZE (256*1024)
#define PFLASH_MAX_ERASE_TIME (300000)   /* 200ms * 1.5 */
#define DFLASH_MAX_ERASE_TIME (225000)   /* 150ms * 1.5 */
#define IDLE_WAIT_TIME        (10000)    /* 10ms */
#define FLASH_MAX_WRITE_TIME  (1000)     /* 1ms*/
#define RRAM_MAX_RETRY_CNT 1

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
typedef struct
{
  uint8_t (*waitunbusy)(uint32_t flash, FLASH_TYPE type, uint32_t timeout);
  void (*reset_to_read)(uint32_t flash);
  void (*clear_status)(uint32_t flash);
  void (*erase_multiple_sectors)(uint32 sector_addr, uint32 num_sector);
  boolean (*is_request_received)(void);
  boolean (*is_request_executed)(void);
  boolean (*is_error_detected)(IfxFlash_Error error);
  uint8 (*enter_page_mode)(uint32 page_addr);
  boolean (*is_dflash_in_page_mode)(void);
  boolean (*is_pflash_in_page_mode)(void);
  void (*write_page)(uint32 page_addr);
  void (*load_page_2x32)(uint32 page_addr, uint32 wordl, uint32 wordu);
} FLASH_OPS;

#else

typedef struct
{
  boolean (*write_data_page)(uint32 page_addr, uint8 *data);
  boolean (*write_prog_page)(uint32 page_addr, uint8 *data);
  boolean (*is_dmur_write_verify_error)(Ifx_DMUR *dmur);
  boolean (*is_dmur_protection_error)(Ifx_DMUR *dmur);
  boolean (*is_dmur_operation_erroro)(Ifx_DMUR *dmur);
  boolean (*is_pmur_writeverify_error)(Ifx_PMUR *pmur);
  boolean (*is_pmur_protection_erroro)(Ifx_PMUR *pmur);
  boolean (*is_pmur_operation_erroro)(Ifx_PMUR *pmur);
  boolean (*is_dmur_busy)(Ifx_DMUR *dmur);
  boolean (*is_pmur_busy)(Ifx_PMUR *pmur);
  volatile Ifx_DMUR *(*get_dmur)(uint32 page_addr);
  volatile Ifx_PMUR *(*get_pmur)(uint32 page_addr);
  void (*clear_pmur_error)(Ifx_PMUR *pmur, IfxNvmr_PmurError error);
  void (*clear_dmur_error)(Ifx_DMUR *dmur, IfxNvmr_DmurError error);
}FLASH_OPS;

typedef struct
{
  IfxCpu_Index core_id;
  volatile Ifx_CPUCFI_PFIBUFDIS *reg;
}CPU_CFIX_PFIBUFDIS;

#endif

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* MTD driver methods */

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

static int aurix_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks);
#endif

static ssize_t aurix_flash_read(struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, uint8_t *buffer);
static ssize_t aurix_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, uint8_t *buffer);
static ssize_t aurix_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, const uint8_t *buffer);
static int aurix_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

 #ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
static const FLASH_OPS host_flash_ops =
{
  .waitunbusy = IfxFlash_waitUnbusy,
  .reset_to_read = IfxFlash_resetToRead,
  .clear_status = IfxFlash_clearStatus,
  .erase_multiple_sectors = IfxFlash_eraseMultipleSectors,
  .is_request_received = IfxFlash_isRequestReceived,
  .is_request_executed = IfxFlash_isRequestExecuted,
  .is_error_detected = IfxFlash_isErrorDetected,
  .enter_page_mode = IfxFlash_enterPageMode,
  .is_dflash_in_page_mode = IfxFlash_isDflashInPageMode,
  .is_pflash_in_page_mode = IfxFlash_isPflashInPageMode,
  .write_page = IfxFlash_writePage,
  .load_page_2x32 = IfxFlash_loadPage2X32
};

static const FLASH_OPS csrm_flash_ops =
{
  .waitunbusy = IfxFlashCsrm_waitUnbusy,
  .reset_to_read = IfxFlashCsrm_resetToRead,
  .clear_status = IfxFlashCsrm_clearStatus,
  .erase_multiple_sectors = IfxFlashCsrm_eraseMultipleSectors,
  .is_request_received = IfxFlashCsrm_isRequestReceived,
  .is_request_executed = IfxFlashCsrm_isRequestExecuted,
  .is_error_detected = IfxFlashCsrm_isErrorDetected,
  .enter_page_mode = IfxFlashCsrm_enterPageMode,
  .is_dflash_in_page_mode = IfxFlashCsrm_isDflashInPageMode,
  .is_pflash_in_page_mode = IfxFlashCsrm_isPflashInPageMode,
  .write_page = IfxFlashCsrm_writePage,
  .load_page_2x32 = IfxFlashCsrm_loadPage2X32
};
#else
static const FLASH_OPS rram_ops =
{
  .write_data_page = IfxNvmr_writePageDataMemory,
  .write_prog_page = IfxNvmr_writePageProgramMemory,
  .is_dmur_write_verify_error = IfxNvmr_isDmurWriteVerifyErrorOccured,
  .is_dmur_operation_erroro = IfxNvmr_isDmurOperationErrorOccured,
  .is_dmur_protection_error = IfxNvmr_isDmurProtectionErrorOccured,
  .is_pmur_writeverify_error = IfxNvmr_isPmurWriteVerifyErrorOccured,
  .is_pmur_protection_erroro = IfxNvmr_isPmurProtectionErrorOccured,
  .is_pmur_operation_erroro = IfxNvmr_isPmurOperationErrorOccured,
  .is_dmur_busy = IfxNvmr_isDmurBusy,
  .is_pmur_busy = IfxNvmr_isPmurBusy,
  .get_dmur = IfxNvmr_getDmurBankAddress,
  .get_pmur = IfxNvmr_getPmurBankAddress,
  .clear_pmur_error = IfxNvmr_clearPmurError,
  .clear_dmur_error = IfxNvmr_clearDmurError,
};
#endif

#ifdef CONFIG_ARCH_CHIP_AURIX_TC48X
static const CPU_CFIX_PFIBUFDIS cpu_cfix_pribufdis[] =
{
    {IfxCpu_Index_0, &(CPU_CFI0_PFIBUFDIS)},
    {IfxCpu_Index_1, &(CPU_CFI1_PFIBUFDIS)},
    {IfxCpu_Index_2, &(CPU_CFI2_PFIBUFDIS)},
    {IfxCpu_Index_3, &(CPU_CFI3_PFIBUFDIS)},
    {IfxCpu_Index_6, &(CPU_CFICS_PFIBUFDIS)}
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t addr_convert(const uint8_t *buf)
{
  if (((uintptr_t)buf & 0x3) == 0)
    {
      return *(uint32_t *)buf;
    }
  else
    {
      return ((uint32_t)buf[0]) | (((uint32_t)buf[1]) << 8) |
              (((uint32_t)buf[2]) << 16) | (((uint32_t)buf[3]) << 24);
    }
}

static bool is_host_pflash_type(FLASH_TYPE type)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  return (type >= IfxFlash_FlashType_P00 && type <= IfxFlash_FlashType_P51);
#else
  return (type >= IfxNvmr_NvmrType_P00 && type <= IfxNvmr_NvmrType_P31);
#endif
}

static bool is_host_dflash_type(FLASH_TYPE type)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  return (type == IfxFlash_FlashType_DHost);
#else
  return (type == IfxNvmr_NvmrType_DHost);
#endif
}

static bool is_csrm_dflash_type(FLASH_TYPE type)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  return (type == IfxFlash_FlashType_DCsrm);
#else
  return (type == IfxNvmr_NvmrType_DCsrm);
#endif
}

static bool is_csrm_pflash_type(FLASH_TYPE type)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  return (type == IfxFlash_FlashType_PCsrm);
#else
  return (type == IfxNvmr_NvmrType_PCsrm);
#endif
}

static bool is_same_flash_type(FLASH_TYPE type1, FLASH_TYPE type2)
{
  return (is_host_pflash_type(type1) && is_host_pflash_type(type2)) ||
         (is_host_dflash_type(type1) && is_host_dflash_type(type2)) ||
         (is_csrm_dflash_type(type1) && is_csrm_dflash_type(type2)) ||
         (is_csrm_pflash_type(type1) && is_csrm_pflash_type(type2));
}

static int get_bank_count(const flash_map_t *map, uint32_t ota_bank,
                          uint32_t addr, uint32_t total_bytes)
{
  int start_bank = -1;
  int end_bank = -1;
  uint32_t bank_start;
  uint32_t bank_end;

  if (map)
    {
      for (int i = 0; i < PFLASH_BANKS; i++)
        {
          if (map[i].ota_bank != ota_bank)
            {
              continue;
            }

          bank_start = map[i].start_addr_map_a;
          bank_end = bank_start + map[i].size;

          if ((addr >= bank_start) && (addr < bank_end))
            {
              start_bank = i;
            }

          if ((addr + total_bytes -1 >= bank_start)
              && (addr + total_bytes -1 < bank_end))
            {
              end_bank = i;
            }
        }

      if (start_bank != -1 && end_bank != -1)
        {
          return end_bank - start_bank + 1;
        }
    }

  return 0;
}

static int get_bank_count_by_logic_addr(const flash_map_t *map,
                                        uint32_t addr, uint32_t total_bytes)
{
  int count = get_bank_count(map, OTA_BANK_A, addr, total_bytes);
  if (count > 0)
    {
      return count;
    }

  count =  get_bank_count(map, OTA_BANK_B, addr, total_bytes);
  return count;
}

static uint32_t logic_addr_to_pysical(const flash_map_t *map,
                                           uint32_t addr)
{
  uint32_t out_addr;
  int currunt_slot;
  out_addr = addr;

  if (map)
    {
      currunt_slot = CURRENT_SLOT;
      switch (currunt_slot)
        {
          case SLOT_A:
            for (int i = 0; i < PFLASH_BANKS; i++)
              {
                if ((addr >= map[i].start_addr_map_a)
                    && (addr < map[i].start_addr_map_a + map[i].size))
                  {
                    out_addr = map[i].start_addr +
                               (addr - map[i].start_addr_map_a);
                    return out_addr;
                  }
              }
            break;

          case SLOT_B:
            for (int i = 0; i < PFLASH_BANKS; i++)
              {
                if ((addr >= map[i].start_addr_map_b)
                    && (addr < map[i].start_addr_map_b + map[i].size))
                  {
                    out_addr =  map[i].start_addr +
                                (addr - map[i].start_addr_map_b);
                    return out_addr;
                  }
              }
            break;

          default:
            return out_addr;
        }
    }

  return out_addr;
}

static int flash_multi_bank_check(const flash_map_t *flash_map,
                                  uint32_t addr, uint32_t total_bytes,
                                  int *out_bank_cnt)
{
  FLASH_TYPE type_start;
  FLASH_TYPE type_end;
  int bank_cnt;

  if (out_bank_cnt == NULL)
    {
      ferr("invalid param\n");
      return -EINVAL;
    }

  type_start = GET_BANK_FROM_ADDR(logic_addr_to_pysical(flash_map, addr));
  if (type_start == FLASH_INVALID_TYPE)
    {
      ferr("invalid start addr 0x%" PRIx32 "\n", addr);
      return -EINVAL;
    }

  type_end = GET_BANK_FROM_ADDR(\
             logic_addr_to_pysical(flash_map, addr + total_bytes - 1));
  if (type_end == FLASH_INVALID_TYPE)
    {
      ferr("invalid end addr 0x%" PRIx32 "\n", addr + total_bytes - 1);
      return -EINVAL;
    }

  /* make sure flash type is correct and have
   * the same type for start and end flash bank
   */

  if ((type_start > type_end) || !is_same_flash_type(type_start, type_end))
    {
      ferr("invalid flash type, start is %d, end is %d\n",
           type_start, type_end);
      return -EINVAL;
    }

  if (CURRENT_SLOT != NO_SWAP && is_host_pflash_type(type_start))
    {
      bank_cnt = get_bank_count_by_logic_addr(flash_map, addr, total_bytes);
    }
  else
    {
      bank_cnt = type_end - type_start + 1;
    }

  finfo("bank_cnt is %d\n", bank_cnt);

  if ((false == is_host_pflash_type(type_start)) && (bank_cnt != 1))
    {
      ferr("we expect host dflash, csrm dflash, \
            csrm pflash only have %d bank,\
            but got %d bank\n", DFLASH_BANKS, bank_cnt);
      return -EINVAL;
    }

  if (bank_cnt <= 0)
    {
      ferr("invalid bank count %d\n", bank_cnt);
      return -EINVAL;
    }

  *out_bank_cnt = bank_cnt;
  return OK;
}

static int get_bank_lock(FLASH_TYPE type)
{
  if (is_host_pflash_type(type))
    {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
      return (type - IfxFlash_FlashType_P00) % PFLASH_BANKS;
#else
      return (type - IfxNvmr_NvmrType_P00) % PFLASH_BANKS;
#endif
    }

  return 0;
}

#ifdef CONFIG_ARCH_CHIP_AURIX_TC48X

static void aurix_dis_pfi_prefetch_buffer(bool disable)
{
  int coreid = IfxCpu_getCoreId();
  uint32_t wait_time = IDLE_WAIT_TIME;
  bool found = false;

  for (size_t i = 0;
       i < sizeof(cpu_cfix_pribufdis) / sizeof(cpu_cfix_pribufdis[0]); i++)
    {
      if (cpu_cfix_pribufdis[i].core_id == coreid)
        {
          cpu_cfix_pribufdis[i].reg->B.DISR = disable;
          while (cpu_cfix_pribufdis[i].reg->B.DISS != disable)
            {
              if (wait_time-- == 0)
                {
                  ferr("PFI buffer disable timeout on core %d\n", coreid);
                  break;
                }
            }

          Ifx_Ssw_ISYNC();
          found = true;
          break;
        }
    }

  if (!found)
    {
      ferr("Invalid cpu %d\n", coreid);
    }
}

#endif

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

static int aurix_get_errtype(const FLASH_OPS *flash_ops)
{
  if (flash_ops->is_error_detected(IfxFlash_Error_protection))
    {
      ferr("protection error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_programVerify))
    {
      ferr("program verify error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_eraseVerify))
    {
      ferr("erase verify error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_operation))
    {
      ferr("operation error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_address))
    {
      ferr("address error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_abort))
    {
      ferr("abort error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_clear))
    {
      ferr("clear error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_original))
    {
      ferr("original error\n");
      return -EIO;
    }

  if (flash_ops->is_error_detected(IfxFlash_Error_sequence))
    {
      ferr("sequence error\n");
      return -EIO;
    }

  return OK;
}
#else

static int aurix_get_errtype(const FLASH_OPS *flash_ops, uint32_t page_addr,
                             bool is_dflash, bool retry_allowed)
{
  if (is_dflash)
    {
      Ifx_DMUR *dmur = flash_ops->get_dmur(page_addr);
      if (flash_ops->is_dmur_protection_error(dmur))
        {
          ferr("drram protection error, page addr 0x%" PRIx32"\n",
               page_addr);
          return -EIO;
        }

      else if (flash_ops->is_dmur_write_verify_error(dmur))
        {
          ferr("drram write verify error, page addr 0x%" PRIx32"\n",
               page_addr);
          if (retry_allowed)
            {
              flash_ops->clear_dmur_error(dmur,
                                          IfxNvmr_DmurError_writeVerify);
              return -EAGAIN;
            }

          return -EIO;
        }

      else if (flash_ops->is_dmur_operation_erroro(dmur))
        {
          ferr("drram operation error, page addr 0x%" PRIx32"\n", page_addr);
          return -EIO;
        }

      return OK;
    }
  else
    {
      volatile Ifx_PMUR * pmur = flash_ops->get_pmur(page_addr);

      if (flash_ops->is_pmur_writeverify_error(pmur))
        {
          /* If the write verify error is detected, we should skip it
           * because it is a FAKE error for this version of silcon.
           *
           * we should use the following code to check the error when
           * the silicon is updated:
           *
           * ferr("drram write verify error, page addr 0x%" PRIx32"\n",
           *    page_addr);

           * ferr("prram write verify error, page addr 0x%" PRIx32"\n",
           *       page_addr);
           * if (retry_allowed)
           *   {
           *     flash_ops->clear_pmur_error(pmur,
           *                               IfxNvmr_PmurError_writeVerify);
           *     return -EAGAIN;
           *   }
           * return -EIO;
           */

           flash_ops->clear_pmur_error(pmur, IfxNvmr_PmurError_writeVerify);
        }

      if (flash_ops->is_pmur_protection_erroro(pmur))
        {
          ferr("prram protection error, page addr 0x%" PRIx32"\n",
               page_addr);
          return -EIO;
        }

      else if (flash_ops->is_pmur_operation_erroro(pmur))
        {
          ferr("prram operation error, page addr 0x%" PRIx32"\n", page_addr);
          return -EIO;
        }

      return OK;
    }
}

#endif

/****************************************************************************
 * Name: flash driver
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

static int aurix_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks)
{
  struct aurix_mtd_priv_s *priv = (struct aurix_mtd_priv_s *)dev;
  FLASH_TYPE type;
  uint32_t addr;
  size_t total_bytes;
  size_t bytes_per_bank;
  size_t nbytes_per_time;
  uint32_t nbytes_left;
  uint32_t physical_addr;
  size_t max_erase_size;
  uint32_t max_erase_time;
  uint32_t sector_per_erase;
  int erase_cnt;
  int bank_cnt = -1;
  int ret = -EINVAL;
  const flash_map_t * flash_map;
  const FLASH_OPS *flash_ops;

  if (nblocks > IFXFLASH_PFLASH_NUM_LOG_SECTORS)
    {
      ferr("nblocks is too large, max is %d\n",
           IFXFLASH_PFLASH_NUM_LOG_SECTORS);
      return -EINVAL;
    }

  total_bytes = nblocks * priv->cfg->sectorsz;
  addr = priv->cfg->baseaddr + startblock * priv->cfg->sectorsz;
  flash_map = priv->flash_map;

  if ((priv->is_dflash == false) && (priv->is_csrm == false)
      && (flash_map == NULL))
    {
      ferr("Pflash bank must have a flash map\n");
      return -EINVAL;
    }

  if (flash_multi_bank_check(flash_map, addr, total_bytes, &bank_cnt) != OK
      || bank_cnt <= 0)
    {
      ferr("get flash bank error for addr 0x%" PRIx32 ", \
            total bytes 0x%" PRIx32 ", \
            bank cnt is %d\n", addr, total_bytes, bank_cnt);
      return -EINVAL;
    }

  if (priv->is_csrm)
    {
      flash_ops = &csrm_flash_ops;
    }
  else
    {
      flash_ops = &host_flash_ops;
    }

  for (int i = 0; i < bank_cnt; i++)
    {
      if (0 == total_bytes)
        {
          break;
        }

      physical_addr = logic_addr_to_pysical(flash_map, addr);
      type = GET_BANK_FROM_ADDR(physical_addr);
      if (is_host_pflash_type(type))
        {
          bytes_per_bank = MIN(flash_map[type].size, total_bytes);
          max_erase_size = PFLASH_MAX_ERASE_SIZE;
          max_erase_time = PFLASH_MAX_ERASE_TIME;
        }
      else
        {
          bytes_per_bank = total_bytes;
          max_erase_size = DFLASH_MAX_ERASE_SIZE;
          max_erase_time = DFLASH_MAX_ERASE_TIME;
        }

      erase_cnt = bytes_per_bank / max_erase_size;
      nbytes_left = bytes_per_bank % max_erase_size;

      for (int j = 0; j <= erase_cnt; j++)
        {
          nbytes_per_time = (j == erase_cnt) ? nbytes_left : max_erase_size;

          if (0 == nbytes_per_time)
            {
              break;
            }

          if (up_interrupt_context())
            {
              if (flash_ops->waitunbusy(0, type, IDLE_WAIT_TIME))
                {
                  ferr("flash wait to erase timeout\n");
                  return -ETIMEDOUT;
                }
            }
          else
            {
              down_write(&priv->lock[get_bank_lock(type)]);
            }

          /* Reset the addressed command sequence interpreter */

          flash_ops->reset_to_read(0);

          /* Clear flags */

          flash_ops->clear_status(0);

          /* Erase the sector */

          finfo("addr 0x%" PRIx32 ", nbytes_per_time: 0x%" PRIx32 "\n",
            physical_addr, nbytes_per_time);

          sector_per_erase = nbytes_per_time / priv->cfg->sectorsz;
          max_erase_time = max_erase_time * sector_per_erase;

          /* Erase the given sector */

          flash_ops->erase_multiple_sectors(physical_addr, sector_per_erase);

          /* Wait until request acknowledged */

          while (!flash_ops->is_request_received())
            {
              if (0 == max_erase_time--)
                {
                  ret = -ETIMEDOUT;
                  break;
                }
            };

          /* Wait until request done */

          while (!flash_ops->is_request_executed() && ret != -ETIMEDOUT)
            {
              if (0 == max_erase_time--)
                {
                  ret = -ETIMEDOUT;
                  break;
                }
            };

          if (ret != -ETIMEDOUT)
            {
              ret = aurix_get_errtype(flash_ops);
            }

          flash_ops->clear_status(0);

          if (!up_interrupt_context())
            {
              up_write(&priv->lock[get_bank_lock(type)]);
            }

          if (ret != OK)
            {
              return ret;
            }

          physical_addr += nbytes_per_time;
        }

        total_bytes -= bytes_per_bank;
        addr += bytes_per_bank;
    }

  return (int)nblocks;
}
#endif

/****************************************************************************
 * Name: aurix_dflash_clear_eccerr
 ****************************************************************************/

static void aurix_dflash_clear_eccerr(bool is_csrm)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  if (is_csrm)
    {
      DMU_GP_CSRM_DFECCC.B.CLR = 3U;
    }
  else
    {
      DMU_GP_HOST_DFECCC.B.CLR = 3U;
    }
#else
  if (is_csrm)
    {
      DMUR1_UR_ECCC.B.ECCS_CLR = 3U;
    }
  else
    {
      DMUR0_UR_ECCC.B.ECCS_CLR = 3U;
    }
#endif
}

/****************************************************************************
 * Name: aurix_dflash_get_eccerr
 ****************************************************************************/

static int aurix_dflash_get_eccerr(FLASH_TYPE type)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  if (type == IfxFlash_FlashType_DHost && DMU_GP_HOST_DFECCS.B.AUCER == 1)
    {
      return -EBADMSG;
    }
  else if (type == IfxFlash_FlashType_DCsrm
           && DMU_GP_CSRM_DFECCS.B.AUCER == 1)
    {
      return -EBADMSG;
    }

#else
  if (type == IfxNvmr_NvmrType_DHost && DMUR0_UR_ECCS.B.AUCER == 1)
    {
      return -EBADMSG;
    }
  else if (type == IfxNvmr_NvmrType_DCsrm && DMUR1_UR_ECCS.B.AUCER == 1)
    {
      return -EBADMSG;
    }

#endif
  return OK;
}

static ssize_t aurix_flash_read(struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, uint8_t *buffer)
{
  struct aurix_mtd_priv_s *priv = (struct aurix_mtd_priv_s *)dev;
  FLASH_TYPE type;
  uint32_t addr;
  const flash_map_t * flash_map;
  uint32_t physical_addr;
  int bank_cnt = -1;
  size_t bytes_per_bank;
  size_t total_bytes = nbytes;
  bool ecc = false;
  const FLASH_OPS *flash_ops;
  uint32_t max_wait_time = IDLE_WAIT_TIME;

  addr = priv->cfg->baseaddr + offset;
  flash_map = priv->flash_map;

  if ((priv->is_dflash == false) && (flash_map == NULL))
    {
      ferr("Pflash bank must have a flash map\n");
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  if (priv->is_csrm)
    {
      flash_ops = &csrm_flash_ops;
    }
  else
    {
      flash_ops = &host_flash_ops;
    }
#else
    flash_ops = &rram_ops;
#endif

  if (flash_multi_bank_check(flash_map, addr, total_bytes, &bank_cnt) != OK
      || bank_cnt <= 0)
    {
      ferr("get flash bank error for addr 0x%" PRIx32 ", \
            total bytes 0x%" PRIx32 ", \
            bank cnt is %d\n", addr, total_bytes, bank_cnt);
      return -EINVAL;
    }

  if (priv->is_dflash)
    {
      aurix_dflash_clear_eccerr(priv->is_csrm);
    }

  for (int i = 0; i < bank_cnt; i++)
    {
      physical_addr = logic_addr_to_pysical(flash_map, addr);
      type = GET_BANK_FROM_ADDR(physical_addr);

      if (is_host_pflash_type(type))
        {
          bytes_per_bank = MIN(flash_map[type].size, total_bytes);
        }
      else
        {
          bytes_per_bank = total_bytes;
        }

      if (up_interrupt_context())
        {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
          if (flash_ops->waitunbusy(0, type, max_wait_time))
            {
              ferr("flash wait to read timeout\n");
              return -ETIMEDOUT;
            }
#else
          if (priv->is_dflash)
            {
              Ifx_DMUR *dmur = flash_ops->get_dmur(physical_addr);
              while (FALSE != flash_ops->is_dmur_busy(dmur))
                {
                  if (0 == max_wait_time--)
                    {
                      ferr("dflash wait to read timeout\n");
                      return -ETIMEDOUT;
                    }
                }
            }
          else
            {
              Ifx_PMUR *pmur = flash_ops->get_pmur(physical_addr);
              while (FALSE != flash_ops->is_pmur_busy(pmur))
                {
                  if (0 == max_wait_time--)
                    {
                      ferr("dflash wait to read timeout\n");
                      return -ETIMEDOUT;
                    }
                }
            }
#endif
        }
      else
        {
          down_read(&priv->lock[get_bank_lock(type)]);
        }

      finfo("read nbytes: 0x%"PRIx32" addr: 0x%"PRIx32"\n",
            bytes_per_bank, addr);

      memcpy(buffer, (void *)addr, bytes_per_bank);

      /* Dflash ecc check */

      if (priv->is_dflash && !ecc)
        {
          if (aurix_dflash_get_eccerr(type) == -EBADMSG)
            {
              ecc = true;
              aurix_dflash_clear_eccerr(priv->is_csrm);
            }
        }

      if (!up_interrupt_context())
        {
          up_read(&priv->lock[get_bank_lock(type)]);
        }

      total_bytes -= bytes_per_bank;
      addr += bytes_per_bank;
      buffer += bytes_per_bank;
    }

  return ecc ? -EBADMSG : nbytes;
}

static ssize_t aurix_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                                 size_t nblocks, uint8_t *buffer)
{
  struct aurix_mtd_priv_s *priv = (struct aurix_mtd_priv_s *)dev;
  uint32_t offset;
  uint32_t nbytes;
  ssize_t result;

  offset = startblock * priv->cfg->pagesz;
  nbytes = nblocks * priv->cfg->pagesz;
  result = aurix_flash_read(dev, offset, nbytes, buffer);
  return (result < 0) ? result : (result / priv->cfg->pagesz);
}

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

static int aurix_flash_write_page(const FLASH_OPS *flash_ops,
                                  uint32_t page_addr, const uint8_t *buffer,
                                  bool is_dflash)
{
  int ret;
  uint32_t max_write_time = FLASH_MAX_WRITE_TIME;

  /* Enter in page mode */

  flash_ops->enter_page_mode(page_addr);

  /* Wait until page mode is entered */

  while (FALSE == (is_dflash ? flash_ops->is_dflash_in_page_mode():
         flash_ops->is_pflash_in_page_mode()))
    {
      if (0 == max_write_time--)
        {
          return -ETIMEDOUT;
        }
    };

  ret = aurix_get_errtype(flash_ops);
  if (ret != OK)
    {
      return ret;
    }

  /* Clear flags */

  flash_ops->clear_status(0);

  if (is_dflash)
    {
      flash_ops->load_page_2x32(page_addr, addr_convert(buffer),
                                addr_convert(buffer + 4));
      buffer += 8;
    }
  else
    {
      /* Load data to be written in the page (32 bytes)
       * Divided into four rounds of writing with eight bytes written
       *  in each round
       * Load two words of 32 bits each
       */

      flash_ops->load_page_2x32(page_addr, addr_convert(buffer),
                                addr_convert(buffer + 4));
      buffer += 8;

      /* Load two words of 32 bits each */

      flash_ops->load_page_2x32(page_addr, addr_convert(buffer),
                                addr_convert(buffer + 4));
      buffer += 8;

      /* Load two words of 32 bits each */

      flash_ops->load_page_2x32(page_addr, addr_convert(buffer),
                                addr_convert(buffer + 4));
      buffer += 8;

      /* Load two words of 32 bits each */

      flash_ops->load_page_2x32(page_addr, addr_convert(buffer),
                                addr_convert(buffer + 4));
      buffer += 8;
    }

  /* Clear flags */

  flash_ops->clear_status(0);

  /* Write the loaded page */

  flash_ops->write_page(page_addr);

  /* Wait until request acknowledged */

  while (!flash_ops->is_request_received())
    {
      if (0 == max_write_time--)
        {
          return -ETIMEDOUT;
        }
    };

  /* Wait until request done */

  while (!flash_ops->is_request_executed())
    {
      if (0 == max_write_time--)
        {
          return -ETIMEDOUT;
        }
    };

  /* get error type */

  ret = aurix_get_errtype(flash_ops);

  if (ret != OK)
    {
      return ret;
    }

  /* Clear flags */

  flash_ops->clear_status(0);

  return OK;
}
#else

static int aurix_nvmr_write_page(const FLASH_OPS *flash_ops,
                                 uint32_t page_addr, const uint8_t *buffer,
                                 bool is_dflash, uint32_t readback_addr)
{
  int ret;
  int retry_cnt = 0;
  volatile uint32_t dummy IFX_ALIGN(4);

  do
    {
      if (is_dflash)
        {
          ret = flash_ops->write_data_page(page_addr, (uint8_t *)buffer);
          if (ret != true)
            {
              ferr("write drram page addr %" PRIx32 " failed\n", page_addr);
              return -EIO;
            }
        }
      else
        {
          aurix_dis_pfi_prefetch_buffer(true);

          /* write page */

          ret = flash_ops->write_prog_page(page_addr, (uint8_t *)buffer);

          if (ret != true)
            {
              ferr("write prram page addr %" PRIx32 " failed\n", page_addr);
                   aurix_dis_pfi_prefetch_buffer(false);
              return -EIO;
            }

          /* dummy read */

          for (int index = 0; index < PFLASH_PAGE_SIZE; index += 4)
            {
              dummy = *(uint32_t *)(readback_addr + index);
            }

          /* avoid compiler optimization */

          (void)dummy;

          aurix_dis_pfi_prefetch_buffer(false);
        }

      ret = aurix_get_errtype(flash_ops, page_addr, is_dflash,
                              retry_cnt < RRAM_MAX_RETRY_CNT);

      if (ret != -EAGAIN)
        {
          break;
        }

      retry_cnt++;
    }
  while (1);

  return ret;
}

#endif

static ssize_t aurix_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                  size_t nblocks, const uint8_t *buffer)
{
  struct aurix_mtd_priv_s *priv = (struct aurix_mtd_priv_s *)dev;

  FLASH_TYPE type;
  uint32_t offset;
  size_t total_bytes;
  size_t bytes_per_bank;
  uint32_t addr;
  uint32_t physical_addr;
  uint32_t page;
  int ret = -EINVAL;
  int bank_cnt = -1;
  const flash_map_t * flash_map;
  int lock_num;
  const FLASH_OPS *flash_ops;

  offset = startblock * priv->cfg->pagesz;
  total_bytes = nblocks * priv->cfg->pagesz;
  addr = priv->cfg->baseaddr + offset;
  flash_map = priv->flash_map;

  if ((priv->is_dflash == false) && (priv->is_csrm == false)
      && (flash_map == NULL))
    {
      ferr("Pflash bank must have a flash map\n");
      return -EINVAL;
    }

  if (flash_multi_bank_check(flash_map, addr, total_bytes, &bank_cnt) != OK
      || bank_cnt <= 0)
    {
      ferr("get flash bank error for addr 0x%" PRIx32 ", \
           total bytes 0x%" PRIx32 ", \
           bank cnt is %d\n", addr, total_bytes, bank_cnt);
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  if (priv->is_csrm)
    {
      flash_ops = &csrm_flash_ops;
    }
  else
    {
      flash_ops = &host_flash_ops;
    }
#else
    flash_ops = &rram_ops;
#endif

  for (int i = 0; i < bank_cnt; i++)
    {
      if (0 == total_bytes)
        {
          break;
        }

      physical_addr = logic_addr_to_pysical(flash_map, addr);
      type = GET_BANK_FROM_ADDR(physical_addr);
      lock_num = get_bank_lock(type);

      if (is_host_pflash_type(type))
        {
          bytes_per_bank = MIN(flash_map[type].size, total_bytes);
        }
      else
        {
          bytes_per_bank = total_bytes;
        }

      finfo("write start addr with addr:%" PRIx32", \
            logic addr:%" PRIx32", size %" PRIx32"\n", \
            physical_addr, addr, bytes_per_bank);

      if (up_interrupt_context())
        {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
          if (flash_ops->waitunbusy(0, type, IDLE_WAIT_TIME))
            {
              ferr("flash wait to write timeout\n");
              ret = -ETIMEDOUT;
              goto end;
            }
#else
          uint32_t max_wait_time = IDLE_WAIT_TIME;
          if (priv->is_dflash)
            {
              Ifx_DMUR *dmur = flash_ops->get_dmur(physical_addr);
              while (FALSE != flash_ops->is_dmur_busy(dmur))
                {
                  if (0 == max_wait_time--)
                    {
                      ferr("dflash wait to write timeout\n");
                      ret = -ETIMEDOUT;
                      goto end;
                    }
                }
            }
          else
            {
              Ifx_PMUR *pmur = flash_ops->get_pmur(physical_addr);
              while (FALSE != flash_ops->is_pmur_busy(pmur))
                {
                  if (0 == max_wait_time--)
                    {
                      ferr("pflash wait to write timeout\n");
                      ret = -ETIMEDOUT;
                      goto end;
                    }
                }
            }
#endif
        }
      else
        {
          down_write(&priv->lock[lock_num]);
        }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
      /*  Clear flags */

      flash_ops->clear_status(0);
#endif

      /* Loop over all the pages */

      for (page = 0; page < (bytes_per_bank / priv->cfg->pagesz); page++)
        {
          /* Get the address of the page */

          uint32_t page_addr = physical_addr + (page * priv->cfg->pagesz);
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
          ret = aurix_flash_write_page(flash_ops, page_addr, buffer,
                                       priv->is_dflash);
#else
          uint32_t readback_addr = addr + (page * priv->cfg->pagesz);
          ret = aurix_nvmr_write_page(flash_ops, page_addr, buffer,
                                      priv->is_dflash, readback_addr);
#endif
          if (ret != OK)
            {
              ferr("write page failed, ret: %d\n", ret);
              if (!up_interrupt_context())
                {
                  up_write(&priv->lock[lock_num]);
                }

              goto end;
            }

            buffer += priv->is_dflash ?
                      DFLASH_PAGE_SIZE : PFLASH_PAGE_SIZE;
        }

      if (!up_interrupt_context())
        {
          up_write(&priv->lock[lock_num]);
        }

      total_bytes -= bytes_per_bank;
      addr += bytes_per_bank;
    }

end:
  return ret < 0 ? ret : nblocks;
}

static int aurix_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                             unsigned long arg)
{
  struct aurix_mtd_priv_s *priv = (struct aurix_mtd_priv_s *)dev;
  int ret = -EINVAL;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)arg;
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              geo->blocksize    = priv->cfg->pagesz;
              geo->erasesize    = priv->cfg->sectorsz;
              geo->neraseblocks = priv->cfg->nsectors;

              ret  = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32 \
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          struct partition_info_s *info = (struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = (priv->cfg->sectorsz * priv->cfg->nsectors)
                                  / priv->cfg->pagesz;
              info->sectorsize  = priv->cfg->pagesz;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret = OK;
            }
        }
        break;

      case BIOC_XIPBASE:
        {
          void **ppv = (void**)arg;

          if (ppv)
            {
              /* Return (void*) base address of FLASH memory. */

              ret  = OK;
              *ppv = (void *)priv->cfg->baseaddr;
            }
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;
          *result = MTD_ERASED_STATE;
          ret = OK;
        }
        break;

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
      case MTDIOC_ERASESECTORS:
        {
          FAR struct mtd_erase_s *erase = (FAR struct mtd_erase_s *)arg;
          ret = aurix_flash_erase(dev, erase->startblock, erase->nblocks);
        }
        break;

      case MTDIOC_BULKERASE:
        {
          ret = aurix_flash_erase(dev, 0, priv->cfg->nsectors);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

struct mtd_dev_s *aurix_mtd_flash(const char *name,
                                  const struct aurix_mtd_cfg_s *cfg,
                                  bool is_dflash, bool is_csrm,
                                  const flash_map_t *flash_map)
{
  int bank_cnt;

  struct aurix_mtd_priv_s *mtd_dev =
    kmm_zalloc(sizeof(struct aurix_mtd_priv_s));

  if (!mtd_dev)
    {
      ferr("kmm zalloc failed for mtd device %s\n", name);
      return NULL;
    }

  if (is_dflash)
    {
      bank_cnt = is_csrm ? CSRM_DFLASH_BANKS : DFLASH_BANKS;
    }
  else
    {
      bank_cnt = is_csrm ? CSRM_PFLASH_BANKS : PFLASH_BANKS;
    }

  for (int i = 0; i < bank_cnt; i++)
    {
      init_rwsem(&mtd_dev->lock[i]);
    }

  DEBUGASSERT(DFLASH_BANKS == 1);
  DEBUGASSERT(CSRM_PFLASH_BANKS == 1);
  DEBUGASSERT(CSRM_DFLASH_BANKS == 1);

  /* Bus error trap is disabled for HOST/CSRM DFLASH,
   * UCB and CFS uncorrectable ECC errors
   */

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  if (is_dflash)
    {
      DMU_GP_HOST_DFECCC.B.TRAPDIS = 3U;
      DMU_GP_CSRM_DFECCC.B.TRAPDIS = 3U;
    }

  /* only eFLASH need erase interface, RRAM don't need */

  mtd_dev->mtd.erase  = aurix_flash_erase;
#else
  if (is_dflash)
    {
      DMUR0_UR_ECCC.B.TRAPDIS = 3U;
      DMUR1_UR_ECCC.B.TRAPDIS = 3U;
    }

#endif

  mtd_dev->mtd.bread  = aurix_flash_bread;
  mtd_dev->mtd.bwrite = aurix_flash_bwrite;
  mtd_dev->mtd.read   = aurix_flash_read;
  mtd_dev->mtd.ioctl  = aurix_flash_ioctl;
  mtd_dev->mtd.name   = name;
  mtd_dev->cfg = cfg;
  mtd_dev->is_dflash = is_dflash;
  mtd_dev->is_csrm = is_csrm;
  mtd_dev->flash_map = flash_map;
  return (struct mtd_dev_s *)mtd_dev;
}
