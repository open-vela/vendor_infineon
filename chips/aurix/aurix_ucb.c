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
#include <nuttx/board.h>
#include <nuttx/crc32.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <syslog.h>
#include <debug.h>
#include <sys/param.h>
#include <sys/types.h>
#include <arch/chip/chip.h>
#include "Ifx_Ssw.h"
#include "IfxCscu_reg.h"
#include "aurix_ucb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define WRITE_FORBIDEN                    (0)
#define UCB_CONFIRMATION_MODE             (0)
#define UCB_CS_SWAP_STATUS_USE_256KB      (1)

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
#  define UCB_SIZE                        (0x800)
#  define UCB_PASSWORD_OFFSET             (0x7D0)
#  define UCB_CONFIRMATION_OFFSET         (0x7F0)
#  define UCB_SWAP_INDEX_NUM              (32)
#  define UCB_CS_SWAP_INDEX_NUM           (32)
#  define UCB_ADDR(x)                     (IfxFlash_dFlashTableUcbLog[x].start)
#  define UCB_CS_ADDR(x)                  (IfxFlashCsrm_dFlashTableUcbLog[x].start)
#else
#  define UCB_SIZE                        (0x100)
#  define UCB_PASSWORD_OFFSET             (0xD0)
#  define UCB_CONFIRMATION_OFFSET         (0xF0)
#  define UCB_SWAP_INDEX_NUM              (13)
#  define UCB_CS_SWAP_INDEX_NUM           (13)
#  define UCB_ADDR(x)                     ((uint32_t)(IFXNVMR_DNVM_UCB0_START + (x * UCB_SIZE)))
#  define UCB_CS_ADDR(x)                  ((uint32_t)(0xAEC00000 + (x * UCB_SIZE)))
#endif

#define UCB_PASSWORD_SIZE                 (32)
#define UCB_CONFIRMATION_SIZE             (8)
#define UCB_STATUS_VALID                  (0x43211234)
#define UCB_RTC_SWAP_STATUS_MARKER_A      (0x00000055)
#define UCB_RTC_SWAP_STATUS_MARKER_B      (0x000000AA)

#define UCB_SWAP_CONFIG_SIZE              (sizeof(UCB_SWAP_CONFIG_T))
#define UCB_SWAP_CONFIG_TOTAL_SIZE        (UCB_SWAP_INDEX_NUM * UCB_SWAP_CONFIG_SIZE)
#define UCB_CS_SWAP_STATUS_MARKER_A       (0x00000044)
#define UCB_CS_SWAP_STATUS_MARKER_B_256KB (0x00000011)
#define UCB_CS_SWAP_STATUS_MARKER_B_512KB (0x00000022)
#define USRCFG_BUF_SIZE                   (48)
#define UCB_ORIG                          (true)
#define UCB_COPY                          (false)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  SWAP_BASED_ON_ORIG = 0,
  SWAP_BASED_ON_COPY,
} SWAP_TARGET_E;

typedef struct
{
  uint32_t sal;
  uint32_t status;
  uint32_t marker;
  uint32_t crcse;
} UCB_SWAP_CONFIG_T;

typedef struct
{
  UCB_SWAP_CONFIG_T config[UCB_PASSWORD_SIZE];
  uint32_t password[8];
  uint8_t confirmation[8];
} UCB_SWAP_T;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool ucb_disable_protection_status(UCB_T *ucb);
static UCB_CONFIRMATION_STATE_E ucb_confirmation_status(UCB_T *ucb,
                                                        bool is_orig);
static void disable_protection(UCB_T *ucb);
static void hexdump(const void *data, size_t size);
static UCB_T *get_ucb_from_map(char *ucb_name);
static UCB_CONFIRMATION_STATE_E dual_ucb_confirmation_status(UCB_T *ucb);
static uint8_t ucb_read(UCB_REGION_E region, uint32_t addr, size_t nbytes, uint8_t *buffer);
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
static uint8_t ucb_erase(UCB_REGION_E region, uint32_t addr);
#endif
static uint8_t ucb_write(UCB_REGION_E region, uint32_t addr, size_t nbytes,
                         const uint8_t *buffer);
static uint32_t crc32_software(const uint32_t *data, size_t length);
static uint32_t crc32_hardware(const uint32_t *data, const size_t length);
static bool is_swap_valid(uint32_t addr, UCB_SWAP_CONFIG_T *swap,
                          UCB_REGION_E region);
static SLOT_STATUS_E get_slot_from_swap_cfg(UCB_SWAP_CONFIG_T *swap,
                                            UCB_REGION_E region);
static int dump_ucb_swap(uint32_t addr, UCB_SWAP_T *read_buf,
                        UCB_REGION_E region);
static int check_ucb_swap(uint32_t addr, UCB_SWAP_T *read_buf, UCB_T *ucb,
                          int *out_valid_index, int *out_slot);
static int setup_one_swap_config(UCB_SWAP_CONFIG_T *swap,
                                 uint32_t swap_sal_addr, UCB_T *ucb,
                                 SLOT_STATUS_E slot);
static int get_next_swap_config(UCB_SWAP_T *read_buf);
static int read_ucb_swap(UCB_REGION_E region, uint32_t addr, UCB_SWAP_T *read_buf);
static int reinit_ucb_swap(uint32_t addr, UCB_T *ucb);
static int ucb_add_swap_config(uint32_t addr, UCB_SWAP_T *read_buf,
                               UCB_T *ucb, SLOT_STATUS_E slot);
static int a_b_swap_enable(uint32_t addr, UCB_T *ucb, bool enable);

/****************************************************************************
 * Public Data
 ****************************************************************************/

UCB_T *ucb_map = NULL;
static int ucb_map_size = 0;
static UCB_SWAP_T g_ucb_swap IFX_ALIGN(32) =
  {
    0
  };

/* ucb conformation code */

#if UCB_CONFIRMATION_MODE
static const uint8_t confirmation_code[8] IFX_ALIGN(32) = \
  {
    0x7f, 0x32, 0xb5, 0x57, 0x00, 0x00, 0x00, 0x00
  };
#else
static const uint8_t confirmation_code[8] IFX_ALIGN(32) = \
  {
    0x34, 0x12, 0x21, 0x43, 0x00, 0x00, 0x00, 0x00
  };
#endif

char *swap_status_str[SWAP_MAX] =
{
    "no swap status",
    "runing in slot A",
    "runing in slot B",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int get_swap_valid_index(UCB_REGION_E region)
{
  if (region == UCB_REGION_RTC)
    {
      return SMM_STMEM1.B.SIDX;
    }

  else
    {
      return CSCU_STMEM1.B.SIDX;
    }

  return 0;
}

static int get_swap_target(UCB_REGION_E region)
{
  if (region == UCB_REGION_RTC)
    {
      return SMM_STMEM1.B.ST;
    }

  else
    {
      return CSCU_STMEM1.B.ST;
    }

  return 0;
}

static UCB_T *get_ucb_from_map(char *ucb_name)
{
  int i;

  if (ucb_map && ucb_map_size)
    {
      for (i = 0; i < ucb_map_size; i++)
        {
          if (strcmp(ucb_map[i].ucb_name, ucb_name) == 0)
            {
              return &(ucb_map[i]);
            }
        }
    }

  return NULL;
}

static uint32_t ucb_addr(UCB_T *ucb, int ucb_no)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  if (ucb->region == UCB_REGION_RTC)
    {
      return IfxFlash_dFlashTableUcbLog[ucb_no].start;
    }
  else
    {
      return IfxFlashCsrm_dFlashTableUcbLog[ucb_no].start;
    }

#else

  if (ucb->region == UCB_REGION_RTC)
    {
      return (IFXNVMR_DNVM_UCB0_START + (ucb_no * UCB_SIZE));
    }
  else
    {
      return (0xaec00000 + (ucb_no * UCB_SIZE));
    }

#endif
}

static bool ucb_disable_protection_status(UCB_T *ucb)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  if (ucb->region == UCB_REGION_RTC)
    {
      switch (ucb->type)
        {
          case IfxFlash_UcbType_ucbSwap:
            return DMU_GP_HOST_PROTECT.B.PRODISSWAP;

          case IfxFlash_UcbType_ucbUsercfg:
            return DMU_GP_HOST_PROTECT.B.PRODISUSERCFG;

          default:
            return false;
        }
    }
  else
    {
        switch (ucb->type)
        {
          case IfxFlashCsrm_UcbType_csrmucbSwap:
            return DMU_GP_CSRM_PROTECT.B.PRODISSWAP;

          case IfxFlashCsrm_UcbType_csrmucbUsercfg:
            return DMU_GP_CSRM_PROTECT.B.PRODISUSERCFG;

          default:
            return false;
        }
    }
#else

  /* no IFX lld provide this function,
   * so Temporarily not processing.
   */

  return true;
#endif
}

static UCB_CONFIRMATION_STATE_E ucb_confirmation_status(UCB_T *ucb,
                                                        bool is_orig)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  int ucb_no = is_orig ? ucb->ucb_orig_no : ucb->ucb_copy_no;

  if (ucb->region == UCB_REGION_RTC)
    {
      switch (ucb_no)
      {
          case UCB_RTC_SWAP_ORIG:
            return DMU_GP_HOST_CONFIRMB.B.PROINSWAPO;

          case UCB_RTC_SWAP_COPY:
            return DMU_GP_HOST_CONFIRMB.B.PROINSWAPC;

          case UCB_RTC_USERCFG_ORIG:
            return DMU_GP_HOST_CONFIRMB.B.PROINUSERCFGO;

          case UCB_RTC_USERCFG_COPY:
            return DMU_GP_HOST_CONFIRMB.B.PROINUSERCFGC;

          default:
              return UCB_STATE_ERRORED;
      }
    }
  else
    {
      switch (ucb_no)
      {
        case UCB_CS_SWAP_ORIG:
          return DMU_GP_CSRM_CONFIRMB.B.PROINSWAPO;

        case UCB_CS_SWAP_COPY:
          return DMU_GP_CSRM_CONFIRMB.B.PROINSWAPC;

        case UCB_CS_USERCFG_ORIG:
          return DMU_GP_CSRM_CONFIRMB.B.PROINUSERCFGO;

        case UCB_CS_USERCFG_COPY:
          return DMU_GP_CSRM_CONFIRMB.B.PROINUSERCFGC;

        default:
          return UCB_STATE_ERRORED;
      }
    }
#else
  return UCB_STATE_UNLOCKED;
#endif
}

static void disable_protection(UCB_T *ucb)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  if (ucb->region == UCB_REGION_RTC)
    {
      IfxFlash_disableWriteProtection((uint32_t)NULL, ucb->type,
                                      (uint32_t *)(ucb->password));
    }
  else
    {
      IfxFlashCsrm_disableWriteProtection((uint32_t)NULL, ucb->type,
                                          (uint32_t *)(ucb->password));
    }
#else

  /* no IFX lld provide this function,
   * so Temporarily not processing.
   */

  (void)(ucb);
  return;
#endif
}

static void hexdump(const void *data, size_t size)
{
  const unsigned char *byte_data = data;
  char line_buffer[16 * 3 + 1];
  size_t i;

  for (i = 0; i < size; i += 16)
    {
      size_t len = 0;
      for (size_t j = 0; j < 16; ++j)
        {
          if (i + j < size)
            {
              len += sprintf(&line_buffer[len], "%02x ", byte_data[i + j]);
            }
          else
            {
              len += sprintf(&line_buffer[len], "   ");
            }
        }

      line_buffer[len] = '\0';

      syslog(LOG_INFO, "%s\n", line_buffer);
    }
}

/* status of confirmation of ucb swap orig and copy */

static UCB_CONFIRMATION_STATE_E dual_ucb_confirmation_status(UCB_T *ucb)
{
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  if (ucb_confirmation_status(ucb, UCB_ORIG) == UCB_STATE_ERRORED)
    {
      return ucb_confirmation_status(ucb, UCB_COPY);
    }

  else
    {
      return ucb_confirmation_status(ucb, UCB_ORIG);
    }

  return UCB_STATE_ERRORED;

#else

  /* no IFX lld provide this function,
   * so Temporarily not processing.
   */

  return UCB_STATE_UNLOCKED;

#endif
}

static uint8_t ucb_read(UCB_REGION_E region, uint32_t addr, size_t nbytes, uint8_t *buffer)
{
  off_t offset;
  struct mtd_dev_s *mtd_dev_ptr;
  uint32_t dflash_start_addr;

  if (region == UCB_REGION_RTC)
    {
      mtd_dev_ptr = g_mtd_dflash;
      dflash_start_addr = DFLASH_START;
    }

#ifdef CONFIG_AURIX_CSMTD_DFLASH
  else if (region == UCB_REGION_CS)
    {
      mtd_dev_ptr = g_mtd_csdflash;
      dflash_start_addr = CSRM_DFLASH_START;
    }

#endif

  else
    {
      return 1;
    }

  if (!mtd_dev_ptr || (addr < dflash_start_addr) || !buffer)
    {
      return 1;
    }

  offset = addr - dflash_start_addr;

  if (nbytes != MTD_READ(mtd_dev_ptr, offset, nbytes, buffer))
    {
      return 1;
    }

  return 0;
}

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

static uint8_t ucb_erase(UCB_REGION_E region, uint32_t addr)
{
  off_t startblock;
  struct mtd_dev_s *mtd_dev_ptr;
  uint32_t dflash_start_addr;

  if (region == UCB_REGION_RTC)
    {
      mtd_dev_ptr = g_mtd_dflash;
      dflash_start_addr = DFLASH_START;
    }

#ifdef CONFIG_AURIX_CSMTD_DFLASH
  else if (region == UCB_REGION_CS)
    {
      mtd_dev_ptr = g_mtd_csdflash;
      dflash_start_addr = CSRM_DFLASH_START;
    }

#endif

  else
    {
      return 1;
    }

  if (!mtd_dev_ptr || (addr < dflash_start_addr) || (addr % UCB_SIZE))
    {
      ferr("ERROR: ucb erase para error, addr=0x%08lx\n", addr);
      return 1;
    }

  startblock = (addr - dflash_start_addr) / UCB_SIZE;

#if WRITE_FORBIDEN
  finfo("erase at 0x%08lx\n", addr);
#else
  if (MTD_ERASE(mtd_dev_ptr, startblock, 1) < 0)
    {
      ferr("ERROR: ucb erase failed, addr 0x%08lx\n", addr);
      return 1;
    }
#endif

  return 0;
}
#endif

static uint8_t ucb_write(UCB_REGION_E region, uint32_t addr, size_t nbytes, const uint8_t *buffer)
{
  int ret = 0;
  off_t offset;
  struct mtd_dev_s *mtd_dev_ptr;
  uint32_t dflash_start_addr;

  if (region == UCB_REGION_RTC)
    {
      mtd_dev_ptr = g_mtd_dflash;
      dflash_start_addr = DFLASH_START;
    }

#ifdef CONFIG_AURIX_CSMTD_DFLASH
  else if (region == UCB_REGION_CS)
    {
      mtd_dev_ptr = g_mtd_csdflash;
      dflash_start_addr = CSRM_DFLASH_START;
    }

#endif

  else
    {
      return 1;
    }

  if (!mtd_dev_ptr || (addr < dflash_start_addr) || !buffer)
    {
      ferr("ERROR: ucb write para error, addr 0x%08lx\n", addr);
      return 1;
    }

  if (nbytes % DFLASH_PAGE_SIZE)
    {
      ferr("ERROR: \
        ucb write nbytes 0x%08lx should can be divided by %d\n", \
          nbytes, DFLASH_PAGE_SIZE);
      return 1;
    }

  offset = addr - dflash_start_addr;

  if (offset % DFLASH_PAGE_SIZE)
    {
      ferr("ERROR: \
        ucb write offset 0x%08lx should can be divided by %d\n", \
          offset, DFLASH_PAGE_SIZE);
      return 1;
    }

#if WRITE_FORBIDEN
  finfo("write at 0x%08lx, size 0x%08lx, dump as:\n", addr, nbytes);
  hexdump(buffer, nbytes);
#else
  ret = MTD_BWRITE(mtd_dev_ptr, offset / DFLASH_PAGE_SIZE, \
    nbytes / DFLASH_PAGE_SIZE, buffer);
  if (ret < 0)
    {
      ferr("ERROR: \
        ucb write failed, addr=0x%08lx nbytes=0x%08lx\n", addr, nbytes);
      return 1;
    }
#endif

  return 0;
}

static uint32_t crc32_software(const uint32_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  uint32_t value;

  for (size_t i = 0; i < length; i++)
    {
      value =  (((data[i] & 0xff000000) >> 24) | \
                ((data[i] & 0x00ff0000) >> 8) | \
                ((data[i] & 0x0000ff00) << 8) | \
                ((data[i] & 0x000000ff) << 24));

      crc = crc32part((const uint8_t *)&value, 4, crc);
    }

  return crc ^ 0xffffffff;
}

static uint32_t crc32_hardware(const uint32_t *data, const size_t length)
{
  uint32_t index;
  uint32_t crc32 = 0;

  for (index = 0; index < length; index++)
    {
      crc32 = __crc32(crc32, data[index]);
    }

  return crc32;
}

/****************************************************************************
 * Name: is_swap_valid
 *
 * Description:
 *   This function checks if a swap configuration is valid for the given
 *   address and region. It verifies the status, swap address, and marker,
 *   as well as compares the CRC checks to ensure integrity.
 *
 * Input Parameters:
 *   - addr: The expected swap address to validate against.
 *   - swap: Pointer to the swap configuration structure to validate.
 *   - region: The region of the UCB (RTC or CS) that the swap configuration
 *             belongs to.
 *
 * Returned Value:
 *   Returns true if the swap configuration is valid, false otherwise.
 *
 * Assumptions/Limitations:
 *   Assumes that the `swap` pointer is valid and initialized.
 ****************************************************************************/

static bool is_swap_valid(uint32_t addr, UCB_SWAP_CONFIG_T *swap,
                          UCB_REGION_E region)
{
  uint32_t crc1;
  uint32_t crc2;

  if (swap)
    {
      if ((swap->status == UCB_STATUS_VALID) && (swap->sal == addr))
        {
          switch (region)
            {
              case UCB_REGION_RTC:
                if ((swap->marker != UCB_RTC_SWAP_STATUS_MARKER_A)
                   && (swap->marker != UCB_RTC_SWAP_STATUS_MARKER_B))
                  {
                    return false;
                  }
                break;
              case UCB_REGION_CS:
                if ((swap->marker != UCB_CS_SWAP_STATUS_MARKER_A)
                   && (swap->marker != UCB_CS_SWAP_STATUS_MARKER_B_256KB)
                   && (swap->marker != UCB_CS_SWAP_STATUS_MARKER_B_512KB))
                  {
                    return false;
                  }
                break;
              default:
                return false;
            }

          crc1 = crc32_software((const uint32_t *)swap,
                                (sizeof(swap->sal) + sizeof(swap->status) +
                                sizeof(swap->marker)) / sizeof(uint32_t));

          crc2 = crc32_hardware((const uint32_t *)swap,
                                (sizeof(swap->sal) + sizeof(swap->status) +
                                sizeof(swap->marker)) / sizeof(uint32_t));
          if ((swap->crcse == crc1) && (swap->crcse == crc2))
            {
              return true;
            }
        }
    }

  return false;
}

/****************************************************************************
 * Name: get_slot_from_swap_cfg
 *
 * Description:
 *   get the slot information from swap configuration
 *
 * Input Parameters:
 *   - swap: Pointer to the UCB_SWAP_CONFIG_T structure representing
 *            the swap configuration.
 *   - region: An enumerator indicating the UCB region (RTC or CS)
 *             associated with the swap configuration.
 *
 * Returned Value:
 *   - Returns SWAP_A_STATUS if the configuration is valid and corresponds
 *     to Swap A.
 *   - Returns SWAP_B_STATUS if the configuration is valid and corresponds
 *     to Swap B.
 *   - Returns NO_SWAP_STATUS if the configuration is invalid or does not
 *     match the expected criteria.
 *
 * Assumptions/Limitations:
 *   - Assumes that the `swap` pointer is valid and properly initialized
 *     before calling this function.
 ****************************************************************************/

static SLOT_STATUS_E get_slot_from_swap_cfg(UCB_SWAP_CONFIG_T *swap,
                                            UCB_REGION_E region)
{
  switch (region)
    {
      case UCB_REGION_RTC:
        if (swap->marker == UCB_RTC_SWAP_STATUS_MARKER_A)
          {
            return SWAP_A_STATUS;
          }
        else if (swap->marker == UCB_RTC_SWAP_STATUS_MARKER_B)
          {
            return SWAP_B_STATUS;
          }
        break;
      case UCB_REGION_CS:
        if (swap->marker == UCB_CS_SWAP_STATUS_MARKER_A)
          {
            return SWAP_A_STATUS;
          }
        else if ((swap->marker == UCB_CS_SWAP_STATUS_MARKER_B_256KB) ||
                (swap->marker == UCB_CS_SWAP_STATUS_MARKER_B_512KB))
          {
            return SWAP_B_STATUS;
          }
        break;
      default:
        break;
    }

  return NO_SWAP_STATUS;
}

/****************************************************************************
 * Name: dump_ucb_swap
 *
 * Description:
 *   This function dumps the details of UCB swap configurations, including
 *   the validity of each swap configuration, its password, and confirmation
 *   data. It prints the information to the logs.
 *
 * Input Parameters:
 *   - addr: The base address to read the UCB swap configurations from.
 *   - read_buf: Pointer to the buffer containing the swap configurations
 *                that will be dumped.
 *   - region: The region of the UCB that the swap configurations belong to.
 *
 * Returned Value:
 *   Returns 0 on success.
 *
 * Assumptions/Limitations:
 *   Assumes that `read_buf` is initialized and contains valid data.
 ****************************************************************************/

static int dump_ucb_swap(uint32_t addr, UCB_SWAP_T *read_buf,
                         UCB_REGION_E region)
{
  bool valid = false;

  for (int i = 0; i < UCB_SWAP_INDEX_NUM; i++)
    {
      valid = is_swap_valid(addr + i * UCB_SWAP_CONFIG_SIZE,
                            &(read_buf->config[i]), region);
      if (valid)
        {
          finfo("swap%d at 0x%08lx, valid is %d\n", i,
                addr + i * UCB_SWAP_CONFIG_SIZE, valid);
          hexdump(&(read_buf->config[i]), UCB_SWAP_CONFIG_SIZE);
        }
    }

  finfo("password at 0x%08lx, dump as:\n", addr + UCB_PASSWORD_OFFSET);
  hexdump(&(read_buf->password[0]), UCB_PASSWORD_SIZE);

  finfo("confirmation at 0x%08lx, dump as:\n",
        addr + UCB_CONFIRMATION_OFFSET);
  hexdump(&(read_buf->confirmation[0]), UCB_CONFIRMATION_SIZE);

  return 0;
}

/****************************************************************************
 * Name: check_ucb_swap
 *
 * Description:
 *   This function checks the UCB swap configurations for validity.
 *   It verifies the password and confirmation code, and finds the
 *   largest valid swap  index. The function allows for empty UCB
 *   configurations but ensures that the swap format is correct.
 *
 * Input Parameters:
 *   - addr: The base address of the UCB swap configurations to check.
 *   - read_buf: Pointer to the buffer containing the current UCB swap
 *                configuration data.
 *   - ucb: Pointer to the UCB structure containing expected password data.
 *   - out_valid_index: Pointer to an integer that will hold the index of
 *                      the largest valid swap configuration.
 *
 *   - out_slot: Pointer to an integer that will hold the
 *               the slot information from the
 *               swap_configuration[out_valid_index].
 * Returned Value:
 *   Returns 0 on success, -1 on error, logging specific errors encountered.
 *
 * Assumptions/Limitations:
 *   Assumes that `read_buf`, `ucb`, and `out_valid_index` are valid and
 *   properly initialized.
 ****************************************************************************/

static int check_ucb_swap(uint32_t addr, UCB_SWAP_T *read_buf, UCB_T *ucb,
  int *out_valid_index, int *out_slot)
{
  if (!read_buf || !out_valid_index || !ucb || !out_slot)
    {
      ferr("ERROR: swap config or swap index is null\n");
      return -1;
    }

  *out_valid_index = -1;
  *out_slot = NO_SWAP_STATUS;

  if (memcmp(&(read_buf->password[0]), &(ucb->password[0]),
      sizeof(read_buf->password)) != 0)
    {
      ferr("Password check error, dump as:\n");
      hexdump(&(read_buf->password[0]), UCB_PASSWORD_SIZE);
      return -1;
    }

  if (memcmp(&(read_buf->confirmation[0]), confirmation_code,
      sizeof(read_buf->confirmation)) != 0)
    {
      ferr("Confirmation check error dump as:\n");
      hexdump(&(read_buf->confirmation[0]), UCB_CONFIRMATION_SIZE);
      return -1;
    }

  for (int i = (UCB_SWAP_INDEX_NUM - 1); i >= 0; i--)
    {
      if (is_swap_valid(addr + i * UCB_SWAP_CONFIG_SIZE, \
        &(read_buf->config[i]), ucb->region) == true)
        {
          /* Update the output index to valid swap index */

          *out_valid_index = i;

          /* We found the largest valid swap index */

          *out_slot = get_slot_from_swap_cfg(&(read_buf->config[i]),
                                             ucb->region);
          return 0;
        }
    }

  /* No valid swap index found, but return
   * 0 to indicate correct swap format
   */

  return 0;
}

/****************************************************************************
 * Name: setup_one_swap_config
 *
 * Description:
 *   This function configures a single swap configuration for a given UCB
 *   (User Configuration Block). It sets the swap address, status, and
 *   marker based on the specified slot. It also calculates and verifies the
 *   CRC checks of the configuration.
 *
 * Input Parameters:
 *   - swap: Pointer to the swap configuration structure to be initialized.
 *   - swap_sal_addr: The swap address for the configuration.
 *   - ucb: Pointer to the UCB structure that holds configuration data.
 *   - slot: The specific slot (A or B) for which the configuration is being
 *           set.
 *
 * Returned Value:
 *   Returns 0 on success, -1 on error, logging any issues encountered.
 *
 * Assumptions/Limitations:
 *   Assumes that both `swap` and `ucb` pointers are valid.
 ****************************************************************************/

static int setup_one_swap_config(UCB_SWAP_CONFIG_T *swap,
                                 uint32_t swap_sal_addr, UCB_T *ucb,
                                 SLOT_STATUS_E slot)
{
  uint32_t crc1;
  uint32_t crc2;

  /* Check for null pointers */

  if (!swap || !ucb)

    {
      ferr("ERROR: swap config or UCB is null\n");
      return -1;
    }

  /* Set the swap address and status */

  swap->sal = swap_sal_addr;
  swap->status = UCB_STATUS_VALID;

  /* Configure the marker based on the slot and UCB region */

  if (slot == SLOT_B)
    {
      if (ucb->region == UCB_REGION_RTC)
        {
          swap->marker = UCB_RTC_SWAP_STATUS_MARKER_B;
        }
      else
        {
#if UCB_CS_SWAP_STATUS_USE_256KB
          swap->marker = UCB_CS_SWAP_STATUS_MARKER_B_256KB;
#else
          swap->marker = UCB_CS_SWAP_STATUS_MARKER_B_512KB;
#endif
        }
    }
  else if (slot == SLOT_A)
    {
      if (ucb->region == UCB_REGION_RTC)
        {
          swap->marker = UCB_RTC_SWAP_STATUS_MARKER_A;
        }
      else
        {
          swap->marker = UCB_CS_SWAP_STATUS_MARKER_A;
        }
    }
  else
    {
      ferr("ERROR: slot %d must be SLOT_A or SLOT_B\n", slot);
      return -1;
    }

  /* Calculate CRCs */

  crc1 = crc32_software((const uint32_t *)swap, (sizeof(swap->sal) + \
    sizeof(swap->status) + sizeof(swap->marker)) / sizeof(uint32_t));
  crc2 = crc32_hardware((const uint32_t *)swap, (sizeof(swap->sal) + \
    sizeof(swap->status) + sizeof(swap->marker)) / sizeof(uint32_t));

  /* Verify CRCs match */

  if (crc1 != crc2)
    {
      ferr("ERROR: crc_soft = 0x%08lx, crc_hard = 0x%08lx, crc1 != crc2\n",
           crc1, crc2);
      return -1;
    }

  swap->crcse = crc1;
  finfo("crc1 = 0x%08lx, crc2 = 0x%08lx\n", crc1, crc2);
  return 0;
}

/****************************************************************************
 * Name: get_next_swap_config
 *
 * Description:
 *   This function retrieves the next available index for a swap
 *   configuration in the provided read buffer. It checks for
 *   empty slots marked by zero configurations.
 *
 * Input Parameters:
 *   - read_buf: Pointer to the buffer containing the
 *               current swap configurations.
 *
 * Returned Value:
 *   Returns the index of the next available swap configuration, or -1
 *   if an error occurs.
 *
 * Assumptions/Limitations:
 *   Assumes that `read_buf` is properly initialized and valid.
 ****************************************************************************/

static int get_next_swap_config(UCB_SWAP_T *read_buf)
{
  int i;
  int ret = UCB_SWAP_INDEX_NUM;
  UCB_SWAP_CONFIG_T zero_config =
    {
      0
    };

  if (!read_buf)
    {
      ferr("ERROR: read_buf is null\n");
      return -1;
    }

  for (i = 31; i >= 0; i--)
    {
      if (memcmp(&(read_buf->config[i]), &zero_config,
          UCB_SWAP_CONFIG_SIZE) == 0)
        {
          ret = i;
        }
      else
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: read_ucb_swap
 *
 * Description:
 *   This function reads the UCB swap configuration, password, and
 *   confirmation information from the specified memory address into
 *   the provided buffer.
 *
 * Input Parameters:
 *   - region: The value determines whether the addr belongs to HOST or CS.
 *   - addr: The address from which to read the UCB swap information.
 *   - read_buf: Pointer to the buffer that will store the read swap data.
 *
 * Returned Value:
 *   Returns 0 on success, or -1 on failure, logging specific errors
 *   encountered.
 *
 * Assumptions/Limitations:
 *   Assumes that `read_buf` is valid and initialized.
 ****************************************************************************/

static int read_ucb_swap(UCB_REGION_E region, uint32_t addr, UCB_SWAP_T *read_buf)
{
  int ret;

  if (!read_buf)
    {
      ferr("ERROR: UCB swap target is null\n");
      return -1;
    }

  /* Read the swap configuration into the buffer */

  ret = ucb_read(region, addr, UCB_SWAP_CONFIG_TOTAL_SIZE,
                 (uint8_t *)&(read_buf->config[0]));
  if (ret != 0)
    {
      ferr("ERROR: Read UCB swap config error\n");
      return ret;
    }

  /* Read the password into the buffer */

  ret = ucb_read(region, addr + UCB_PASSWORD_OFFSET, UCB_PASSWORD_SIZE, \
                 (uint8_t *)&(read_buf->password[0]));
  if (ret != 0)
    {
      ferr("ERROR: Read UCB swap password error\n");
      return ret;
    }

  /* Read the confirmation into the buffer */

  ret = ucb_read(region, addr + UCB_CONFIRMATION_OFFSET, UCB_CONFIRMATION_SIZE, \
                 (uint8_t *)&(read_buf->confirmation[0]));
  if (ret != 0)
    {
      ferr("ERROR: Read UCB swap confirmation error\n");
      return ret;
    }

  return ret; /* Return success */
}

/****************************************************************************
 * Name: reinit_ucb_swap
 *
 * Description:
 *   This function erases the swap UCB at the specified address and
 *   reinitializes it by writing a confirmation code and the
 *   associated password.
 *
 * Input Parameters:
 *   - addr: The address of the UCB to erase and reinitialize.
 *   - ucb: Pointer to the UCB structure containing relevant data.
 *
 * Returned Value:
 *   Returns 0 on success, or -1 on error, logging any issues encountered.
 *
 * Assumptions/Limitations:
 *   Assumes that the `ucb` pointer is valid and that the necessary data is
 *   initialized.
 ****************************************************************************/

static int reinit_ucb_swap(uint32_t addr, UCB_T *ucb)
{
  int ret;

  /* Check for null pointer */

  if (!ucb)
    {
      ferr("ERROR: UCB config is null\n");
      return -1;
    }

  finfo("Erase this UCB and re-init it\n");

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  /* Erase the UCB at the specified address */

  ret = ucb_erase(ucb->region, addr);
  if (ret != 0)
    {
      ferr("ERROR: Erase UCB SWAP at 0x%08lx failed\n", addr);
      return ret;
    }

  /* Write the confirmation code to the UCB */

  ret = ucb_write(ucb->region, (addr + UCB_CONFIRMATION_OFFSET), UCB_CONFIRMATION_SIZE, \
                  confirmation_code);
  if (ret != 0)
    {
      ferr("ERROR: Write UCB SWAP confirmation failed\n");
      return ret;
    }

  /* Write the password to the UCB */

  ret = ucb_write(ucb->region, addr + UCB_PASSWORD_OFFSET, UCB_PASSWORD_SIZE, \
                  (const uint8_t *)&(ucb->password[0]));
  if (ret != 0)
    {
      ferr("ERROR: Write UCB SWAP password failed\n");
      return ret;
    }

#else
  UCB_SWAP_CONFIG_T zero_config =
    {
      0
    };

  for (int i = 0; i < UCB_SWAP_INDEX_NUM; i++)
    {
      /* write all swap index as zero */

      ret = ucb_write(ucb->region, addr, UCB_SWAP_CONFIG_SIZE,
                      (const uint8_t *)&zero_config);
      if (ret != 0)
        {
          ferr("ERROR: Write UCB SWAP confirmation failed\n");
          return ret;
        }

      addr += UCB_SWAP_CONFIG_SIZE;
    }

#endif
  return ret;
}

/****************************************************************************
 * Name: ucb_add_swap_config
 *
 * Description:
 *   This function adds or updates the swap configuration for a specified UCB
 *   (User Configuration Block) at a given memory address. It handles the
 *   indexing of swap configurations and ensures that the swap configurations
 *   follow the defined limits. It also performs necessary reinitialization
 *   if the index exceeds the configured limits.
 *
 * Input Parameters:
 *   - addr: The memory address where the swap configuration is stored.
 *   - read_buf: Pointer to the buffer containing the current swap
 *               configuration.
 *   - ucb: Pointer to the UCB structure being modified.
 *   - slot: The specific slot where the swap configuration is being applied.
 *
 * Returned Value:
 *   Returns 0 on success, -1 on error, with appropriate logging of errors.
 *
 * Assumptions/Limitations:
 *   Assumes that `read_buf` is properly initialized and that the UCB
 *   structure has a valid region with a correctly initialized swap
 *   configuration.
 ****************************************************************************/

static int ucb_add_swap_config(uint32_t addr, UCB_SWAP_T *read_buf,
                               UCB_T *ucb, SLOT_STATUS_E slot)
{
  int ret;
  int swap_index;

  /* Retrieve the next swap configuration index */

  swap_index = get_next_swap_config(read_buf);
  finfo("swap_index is %d\n", swap_index);

  /* swap_index error */

  if (swap_index < 0)
    {
      ferr("ERROR: get swap index failed\n");
      return -1;
    }

  /* Check for swap index exceeding defined limits */

  if (swap_index >= UCB_SWAP_INDEX_NUM)
    {
      ret = reinit_ucb_swap(addr, ucb);
      swap_index = 0;
      if (ret != 0)
        {
          ferr("ERROR:Reinitialize UCB swap error, addr 0x%08lx\n", addr);
          return -1;
        }

      memset(&(read_buf->config[0]), 0, UCB_SWAP_CONFIG_TOTAL_SIZE);
    }

  /* Set up the configuration for the current swap index */

  ret = setup_one_swap_config(&(read_buf->config[swap_index]), \
    addr + swap_index * sizeof(UCB_SWAP_CONFIG_T), ucb, slot);
  if (ret != 0)
    {
      ferr("ERROR: Setup one swap config failed\n");
      return -1;
    }

  /* Write the swap configuration back to the specified address */

  ret = ucb_write(ucb->region, (addr + swap_index * sizeof(UCB_SWAP_CONFIG_T)),
                  sizeof(UCB_SWAP_CONFIG_T),
                  (uint8_t *)&(read_buf->config[swap_index]));
  if (ret != 0)
    {
      ferr("ERROR: Write a UCB configuration failed\n");
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: a_b_swap_enable
 *
 * Description:
 *   This function enables or disables A/B swapping for the specified UCB
 *   (User Configuration Block). It verifies the UCB state, calculates
 *   and checks the CRC32 checksum, writes the swap configuration, and
 *   verifies that the write operations were successful.
 *
 * Input Parameters:
 *   - addr: The address of the UCB in memory.
 *   - ucb: Pointer to the UCB structure.
 *   - enable: Boolean value indicating whether to enable (true) or
 *     disable (false) the A/B swap.
 *
 * Returned Value:
 *   Returns 0 on success, -1 on error, and logs any errors.
 *
 * Assumptions/Limitations:
 *   Assumes that `ucb` is a valid pointer and that the UCB structure is
 *   properly initialized.
 *
 ****************************************************************************/

static int a_b_swap_enable(uint32_t addr, UCB_T *ucb, bool enable)
{
  int ret;
  uint32_t crc32;
  uint32_t swap_en_offset;
  uint32_t crc32_offset;
  uint32_t crc32_size = sizeof(uint32_t);
  uint8_t read_buf[USRCFG_BUF_SIZE] IFX_ALIGN(4);

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  uint32_t password_offset;
  uint32_t confirmation_offset;
#endif

  /* Check if UCB pointer is valid */

  if (ucb == NULL)
    {
      ferr("ERROR: UCB is not found or UCB swap is null\n");
      return -1;
    }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
  /* Determine offsets based on the UCB region */

  if (ucb->region == UCB_REGION_RTC)
    {
      swap_en_offset = offsetof(Ifx_Ssw_UserCfg, swapEna);
      crc32_offset = offsetof(Ifx_Ssw_UserCfg, crcRtcUserCfg);
      password_offset = offsetof(Ifx_Ssw_UserCfg, pw);
      confirmation_offset = offsetof(Ifx_Ssw_UserCfg, confirmation);
    }
  else
    {
      swap_en_offset = offsetof(Ifx_Ssw_CsUserCfg, swapCsEna);
      crc32_offset = offsetof(Ifx_Ssw_CsUserCfg, crcCsUserCfg);
      password_offset = offsetof(Ifx_Ssw_CsUserCfg, pw);
      confirmation_offset = offsetof(Ifx_Ssw_CsUserCfg, confirmation);
    }
#else
  if (ucb->region == UCB_REGION_RTC)
    {
      swap_en_offset = offsetof(Ifx_Ssw_UserCfg, swapEna);
      crc32_offset = offsetof(Ifx_Ssw_UserCfg, crcRtcUserCfg);
    }
  else
    {
      swap_en_offset = offsetof(Ifx_Ssw_CsUserCfg, apuBypass);
      crc32_offset = offsetof(Ifx_Ssw_CsUserCfg, crcCsUserCfg);
    }
#endif

  /* Check buffer size to ensure it can read necessary data */

  if (sizeof(read_buf) < (crc32_offset + crc32_size))
    {
      ferr("ERROR: Buffer is not enough to read usercfg UCB\n");
      return -1;
    }

  /* Initialize the read buffer with zeros */

  memset(read_buf, 0, crc32_offset + crc32_size);

  /* Read data from the UCB */

  ret = ucb_read(ucb->region, addr, crc32_offset + crc32_size, read_buf);
  if (ret != 0)
    {
      ferr("ERROR: Read UCB SWAP at 0x%08lx failed\n", addr);
      return -1;
    }

  /* Calculate CRC32 and check against the read data */

  crc32 = crc32_hardware((const uint32_t *)read_buf,
                         crc32_offset / crc32_size);
  if (crc32 != *(uint32_t *)&read_buf[crc32_offset])
    {
      ferr("ERROR: CRC32 check failed\n");
      return -1;
    }

  /* Enable or disable the swap based on the input parameter */

  if (enable)
    {
      if (read_buf[swap_en_offset] != 0x0a)
        {
          read_buf[swap_en_offset] = 0x0a;
        }
      else
        {
          finfo("Swap is already enabled\n");
          return 0;
        }
    }
  else
    {
      if (read_buf[swap_en_offset] != 0x05)
        {
          read_buf[swap_en_offset] = 0x05;
        }
      else
        {
          finfo("Swap is already disabled\n");
          return 0;
        }
    }

  /* Update CRC32 in the read buffer */

  crc32 = crc32_hardware((const uint32_t *)&read_buf,
                         crc32_offset / sizeof(uint32_t));
  memcpy(read_buf + crc32_offset, &crc32, crc32_size);

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  /* Erase the UCB sector before writing */

  ret = ucb_erase(ucb->region, addr);
  if (ret != 0)
    {
      ferr("ERROR: Erase at 0x%08lx failed\n", addr);
      return -1;
    }

#endif

  /* Write the updated buffer back to the UCB area */

  ret = ucb_write(ucb->region, addr, sizeof(read_buf), (const uint8_t *)read_buf);
  if (ret != 0)
    {
      ferr("ERROR: \
        Write at 0x%08lx size 0x%08lx failed\n", addr, sizeof(read_buf));
      return -1;
    }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  /* Write the password to the UCB */

  ret = ucb_write(ucb->region, addr + password_offset, UCB_PASSWORD_SIZE, \
    (const uint8_t *)(&(ucb->password[0])));
  if (ret != 0)
    {
      ferr("ERROR: \
        Write at 0x%08lx size UCB_PASSWORD_SIZE failed\n", addr);
      return -1;
    }

  /* Write the confirmation code to the UCB */

  ret = ucb_write(ucb->region, addr + confirmation_offset, sizeof(confirmation_code),
                  (const uint8_t *)confirmation_code);
  if (ret != 0)
    {
      ferr("ERROR: \
        Write UCB RTC SWAP confirmation failed\n");
      return -1;
    }
#endif

  /* Read back data to recheck CRC */

  memset(read_buf, 0, crc32_offset + crc32_size);
  ret = ucb_read(ucb->region, addr, crc32_offset + crc32_size, read_buf);
  if (ret != 0)
    {
      ferr("ERROR: Reread UCB SWAP at 0x%08lx failed\n", addr);
      return -1;
    }

  /* Verify the CRC after the write operation */

  crc32 = crc32_hardware((const uint32_t *)read_buf,
                         crc32_offset / crc32_size);
  if (crc32 != *(uint32_t *)&read_buf[crc32_offset])
    {
      ferr("ERROR: Recheck CRC32 check failed\n");
      return -1;
    }

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

  /* Verify the written password */

  ret = ucb_read(ucb->region, addr + password_offset, UCB_PASSWORD_SIZE, read_buf);
  if (ret != 0)
    {
      ferr("ERROR: Reread UCB SWAP at 0x%08lx failed\n", \
           addr + password_offset);
      return -1;
    }

  /* Compare the read password with the original */

  if (memcmp(read_buf, &(ucb->password[0]), UCB_PASSWORD_SIZE) != 0)
    {
      ferr("ERROR: Password check error, dump as:\n");
      hexdump(read_buf, UCB_PASSWORD_SIZE);
      return -1;
    }

  /* Verify the written confirmation code */

  ret = ucb_read(ucb->region, addr + confirmation_offset, sizeof(confirmation_code),
                 read_buf);
  if (ret != 0)
    {
      ferr("ERROR: Reread UCB SWAP at 0x%08lx failed\n", \
           addr + confirmation_offset);
      return -1;
    }

  /* Compare the read confirmation code with the original */

  if (memcmp(read_buf, confirmation_code, sizeof(confirmation_code)) != 0)
    {
      ferr("ERROR: Confirmation check error dump as:\n");
      hexdump(confirmation_code, sizeof(confirmation_code));
      return -1;
    }
#endif

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4_ab_swap_enable
 *
 * Description:
 *   This function enables or disables A/B swapping for the specified UCB
 *   (User Configuration Block).
 *
 * Input Parameters:
 *   - ucb_name: The name of the UCB to operate on.
 *   - enable: Boolean value to indicate whether to enable (true) or
 *     disable (false) the A/B swap.
 *
 * Returned Value:
 *   Returns 0 on success and -1 on error, with appropriate logging.
 *
 * Assumptions/Limitations:
 *   Assumes that the UCB has been properly initialized and the provided
 *   parameters are valid.
 *
 ****************************************************************************/

int tc4_ab_swap_enable(char *ucb_name, bool enable)
{
  int ret;
  uint32_t addr;
  uint32_t ucb_addr_inactive;
  uint32_t ucb_addr_active;
  UCB_CONFIRMATION_STATE_E ucb_status;
  UCB_T *ucb;

  /* Retrieve the User Configuration Block from the map */

  ucb = get_ucb_from_map(ucb_name);
  if (ucb == NULL)
    {
      ferr("ERROR: UCB %s is not found or UCB swap is null\n", ucb_name);
      return -1;
    }

  /* Check the UCB confirmation status */

  ucb_status = dual_ucb_confirmation_status(ucb);
  if ((ucb_status == UCB_STATE_ERRORED) || (ucb_status == UCB_STATE_UNKNOWN))
    {
      ferr("ERROR: RTC UCB status error, status is %d\n", ucb_status);
      return -1;
    }

  /* Disable protection to access UCB area if the UCB status is confirmed */

  if (ucb_status == UCB_STATE_CONFIRMED)
    {
      disable_protection(ucb);

      /* Check if protection was successfully disabled */

      if (ucb_disable_protection_status(ucb) != true)
        {
          ferr("ERROR: Protection is enabled, expected disabled status\n");
          return -1;
        }
    }

  /* Determine UCB addresses based on the status of the original UCB */

  if ((ucb_confirmation_status(ucb, UCB_ORIG) == UCB_STATE_ERRORED) ||
      (ucb_confirmation_status(ucb, UCB_ORIG) == UCB_STATE_UNKNOWN))
    {
      /* Swap based on UCB COPY */

      ucb_addr_active = ucb_addr(ucb, ucb->ucb_copy_no);
      ucb_addr_inactive = ucb_addr(ucb, ucb->ucb_orig_no);

      /* Validate the status of the UCB COPY */

      if ((ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_ERRORED) ||
          (ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_UNKNOWN))
        {
          ferr("ERROR: Swap status based on UCB COPY, \
            but status of UCB COPY is ERROR.\n");
          ret = -1;
          goto end;
        }
    }
  else
    {
      /* Swap based on UCB ORIG */

      ucb_addr_active = ucb_addr(ucb, ucb->ucb_orig_no);
      ucb_addr_inactive = ucb_addr(ucb, ucb->ucb_copy_no);
    }

  /* Log UCB addresses */

  finfo("UCB address active is 0x%08lx\n", ucb_addr_active);
  finfo("UCB address inactive is 0x%08lx\n", ucb_addr_inactive);

  /* Enable or disable the A/B swap for both active
   * and inactive UCB addresses
   */

  for (int i = 0; i < 2; i++)
    {
      addr = (i == 0) ? ucb_addr_inactive : ucb_addr_active;

      ret = a_b_swap_enable(addr, ucb, enable);
      if (ret != 0)
        {
          ferr("ERROR: Enable swap in 0x%08lx failed\n", addr);
          goto end;
        }
    }

end:

  /* Re-enable the password protection if the UCB status was confirmed */

  if (ucb_status == UCB_STATE_CONFIRMED)
    {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

      IfxFlash_resumeProtection((uint32_t)NULL);

#else
      /* no IFX rram lld provide this function,
       *  so Temporarily not processing.
       */

#endif
    }

  return ret;
}

/****************************************************************************
 * Name: tc4_rtc_ab_swap
 *
 * Description:
 *   This function handles the swapping of UCB (User Configuration Block)
 *   between two slots (A and B). It performs necessary checks and
 *   validations before proceeding with the swap operation.
 *
 * Input Parameters:
 *   - ucb_name: The name of the UCB to operate on.
 *   - slot: The target slot to switch to (e.g., SLOT_A or SLOT_B).
 *   - read_buf: Buffer to store the configuration read from the active UCB.
 *
 * Returned Value:
 *   Returns 0 on success, -1 on error with appropriate logging.
 *
 ****************************************************************************/

int tc4_rtc_ab_swap(char *ucb_name, SLOT_STATUS_E slot)
{
  int i;
  int ret;
  int ab_status;
  int valid_swap_index;
  int valid_swap_index_from_ucb;
  int valid_slot_from_ucb;
  uint32_t addr;
  uint32_t ucb_addr_inactive;
  uint32_t ucb_addr_active;
  bool reinit_inactive_flag = false;
  bool swap_active_flag = false;
  UCB_CONFIRMATION_STATE_E ucb_status;
  UCB_T *ucb;
  UCB_SWAP_T *read_buf = &g_ucb_swap;

  /* Retrieve the User Configuration Block from the map */

  ucb = get_ucb_from_map(ucb_name);
  if (ucb == NULL)
    {
      ferr("ERROR: UCB %s is not found or UCB swap is null\n", ucb_name);
      return -1;
    }

  /* Check the current swap status */

  ab_status = get_swap_status(ucb->region);
  if (ab_status == slot)
    {
      finfo("Already in %d slot, no need to switch\n", ab_status);
      return 0;
    }

  /* Check the UCB confirmation status */

  ucb_status = dual_ucb_confirmation_status(ucb);
  if (ucb_status == UCB_STATE_ERRORED || ucb_status == UCB_STATE_UNKNOWN)
    {
      ferr("ERROR: RTC UCB status error, status is %d\n", ucb_status);
      return -1;
    }

  /* Disable protection to access UCB area */

  if (ucb_status == UCB_STATE_CONFIRMED)
    {
      disable_protection(ucb);

      if (ucb_disable_protection_status(ucb) != true)
        {
          ferr("ERROR: Protection is enabled, expected disabled status\n");
          ret = -1;
          goto end;
        }
    }

  /* Set UCB addresses based on the current swap target */

  if (get_swap_target(ucb->region) == SWAP_BASED_ON_COPY)
    {
      ucb_addr_active = ucb_addr(ucb, ucb->ucb_copy_no);
      ucb_addr_inactive = ucb_addr(ucb, ucb->ucb_orig_no);

      /* Validate copy UCB status */

      if (ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_ERRORED ||
          ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_UNKNOWN)
        {
          ferr("ERROR: swap status based on ucb COPY, "\
               "but status of ucb COPY is ERROR, this is an error\n");
          ret = -1;
          goto end;
        }

      if (ucb_confirmation_status(ucb, UCB_ORIG) != UCB_STATE_ERRORED)
        {
          ferr("ERROR: swap status based on ucb COPY, "\
               "but status of ucb ORIG is CORRECT, this is an error\n");
          ret = -1;
          goto end;
        }

      reinit_inactive_flag = true;
    }
  else
    {
      ucb_addr_active = ucb_addr(ucb, ucb->ucb_orig_no);
      ucb_addr_inactive = ucb_addr(ucb, ucb->ucb_copy_no);

      if (ucb_confirmation_status(ucb, UCB_ORIG) == UCB_STATE_ERRORED ||
          ucb_confirmation_status(ucb, UCB_ORIG) == UCB_STATE_UNKNOWN)
        {
          ferr("ERROR: swap status based on ucb ORIG, "\
               "but status of ucb ORIG is ERROR, this is an error\n");
          ret = -1;
          goto end;
        }

      if (ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_ERRORED ||
          ucb_confirmation_status(ucb, UCB_COPY) == UCB_STATE_UNKNOWN)
        {
          reinit_inactive_flag = true;
        }

      swap_active_flag = true;
    }

  valid_swap_index = get_swap_valid_index(ucb->region);
  finfo("ucb addr active is 0x%08lx\n", ucb_addr_active);
  finfo("ucb addr inactive is 0x%08lx, \
         reinit flag is %d\n", ucb_addr_inactive, reinit_inactive_flag);

  /* Read the active UCB swap */

  ret = read_ucb_swap(ucb->region, ucb_addr_active, read_buf);
  if (ret != 0)
    {
      ferr("ERROR: Failed to read UCB swap at addr 0x%08lx\n",
           ucb_addr_active);
      ret = -1;
      goto end;
    }

  /* Verify the UCB swap */

  ret = check_ucb_swap(ucb_addr_active, read_buf, ucb,
                       &valid_swap_index_from_ucb, &valid_slot_from_ucb);
  if (ret != 0)
    {
      ferr("ERROR: UCB swap check failed at addr 0x%08lx\n",
           ucb_addr_active);
      ret = -1;
      goto end;
    }

  /* Ensure the swap index is valid */

  if ((valid_swap_index != valid_swap_index_from_ucb) &&
      (ab_status != NO_SWAP_STATUS))
    {
      ferr("ERROR: Swap index mismatch. UCB: %d, Register: %d\n",
           valid_swap_index_from_ucb, valid_swap_index);
      ret = -1;
      goto end;
    }

  /* when the slot cfg is already the value we want, don't do anything */

  if ((slot == valid_slot_from_ucb) && (valid_swap_index_from_ucb >= 0) &&
      (valid_swap_index_from_ucb < UCB_SWAP_INDEX_NUM))
    {
      finfo("the slot cfg:%s is already the value we want: %s, don't do \
             anything\n", swap_status_str[valid_slot_from_ucb],
             swap_status_str[slot]);
      ret = 0;
      goto end;
    }

  /* Re-initialize the inactive UCB if needed */

  if (reinit_inactive_flag)
    {
      ret = reinit_ucb_swap(ucb_addr_inactive, ucb);
      if (ret != 0)
        {
          ferr("ERROR: Re-initialization of UCB swap failed at 0x%08lx\n",
               ucb_addr_inactive);
          ret = -1;
          goto end;
        }
    }

  /* Update the UCB configuration */

  for (i = 0; i < 2; i++)
    {
      addr = (i == 0) ? ucb_addr_inactive : ucb_addr_active;
      if (i == 1 && (swap_active_flag == false))
        {
          continue;
        }

      finfo("Enabling swap at 0x%08lx\n", addr);
      ret = read_ucb_swap(ucb->region, addr, read_buf);
      if (ret != 0)
        {
          ferr("ERROR: read UCB swap error at \
               addr 0x%08lx\n", addr);
          ret = -1;
          goto end;
        }

      ret = ucb_add_swap_config(addr, read_buf, ucb, slot);
      if (ret != 0)
        {
          ferr("ERROR: Swap error occurred\n");
          ret = -1;
          goto end;
        }

      /* Recheck the inactive UCB */

      ret = read_ucb_swap(ucb->region, addr, read_buf);
      if (ret != 0)
        {
          ferr("ERROR: Re-reading UCB swap error at \
                addr 0x%08lx\n", addr);
          ret = -1;
          goto end;
        }

      ret = check_ucb_swap(addr, read_buf, ucb, &valid_swap_index_from_ucb,
                           &valid_slot_from_ucb);
      if (ret != 0)
        {
          ferr("ERROR: Re-check UCB swap error at addr 0x%08lx\n", addr);
          ret = -1;
          goto end;
        }

      if (valid_slot_from_ucb != slot)
        {
          ferr("ERROR: slot re-read from ucb is %d, \
               not equal to the value we set: %d\n",
               valid_slot_from_ucb, slot);
          ret = -1;
          goto end;
        }

      if (valid_swap_index_from_ucb < 0 ||
          valid_swap_index_from_ucb >= UCB_SWAP_INDEX_NUM)
        {
          ferr("ERROR: Invalid swap index: %d\n", valid_swap_index_from_ucb);
          ret = -1;
          goto end;
        }
    }

end:

  /* Re-enable the password protection if confirmation was successful */

  if (ucb_status == UCB_STATE_CONFIRMED)
    {
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX

      IfxFlash_resumeProtection((uint32_t)NULL);

#else
      /* no IFX rram lld provide this function,
       * so Temporarily not processing.
       */

#endif
    }

  return ret;
}

/****************************************************************************
 * Name: get_swap_status
 *
 * Description:
 *  Determine which partition the current A/B state is in.
 *
 * Input Parameters:
 *   - region: Host cpu reigon or CS cpu region.
 *
 * Returned Value:
 *  0 : a non-A/B partition
 *  1 : A partition
 *  2 : B partition
 *
 ****************************************************************************/

int get_swap_status(int region)
{
  if ((UCB_REGION_E)region == UCB_REGION_RTC)
    {
      return SMM_STMEM1.B.SCFG;
    }

  else if ((UCB_REGION_E)region == UCB_REGION_CS)
    {
      return CSCU_STMEM1.B.SCFG;
    }

  else
    {
      return NO_SWAP_STATUS;
    }
}

/****************************************************************************
 * Name: dump_swap_status
 *
 * Description:
 *  dump some swap status and swap-UCB content.
 *
 ****************************************************************************/

void dump_swap_status(void)
{
  int swap_status = get_swap_status(UCB_REGION_RTC);
  if (swap_status >= NO_SWAP_STATUS && swap_status <= SWAP_B_STATUS)
    {
      syslog(LOG_INFO, "RTC A/B swap status: %s\n",
             swap_status_str[swap_status]);
    }
  else
    {
      syslog(LOG_INFO, "RTC A/B swap status error\n");
    }

  syslog(LOG_INFO, "UCB RTC USERCFG ORIG : 0x%08lx\n",
         UCB_ADDR(UCB_RTC_USERCFG_ORIG));
  hexdump((uint8_t *)UCB_ADDR(UCB_RTC_USERCFG_ORIG), 64);
  syslog(LOG_INFO, "UCB RTC USERCFG COPY : 0x%08lx\n",
         UCB_ADDR(UCB_RTC_USERCFG_COPY));
  hexdump((uint8_t *)UCB_ADDR(UCB_RTC_USERCFG_COPY), 64);
  syslog(LOG_INFO, "UCB RTC SWAP ORIG : 0x%08lx dump as:\n",
         UCB_ADDR(UCB_RTC_SWAP_ORIG));
  read_ucb_swap(UCB_REGION_RTC, UCB_ADDR(UCB_RTC_SWAP_ORIG), &g_ucb_swap);
  dump_ucb_swap(UCB_ADDR(UCB_RTC_SWAP_ORIG), &g_ucb_swap, UCB_REGION_RTC);
  syslog(LOG_INFO, "UCB RTC SWAP COPY : 0x%08lx dump as:\n",
         UCB_ADDR(UCB_RTC_SWAP_COPY));
  read_ucb_swap(UCB_REGION_RTC, UCB_ADDR(UCB_RTC_SWAP_COPY), &g_ucb_swap);
  dump_ucb_swap(UCB_ADDR(UCB_RTC_SWAP_COPY), &g_ucb_swap, UCB_REGION_RTC);

  if (IfxCpu_getCoreIndex() == IfxCpu_ResourceCpu_6)
    {
      swap_status = get_swap_status(UCB_REGION_CS);
      if (swap_status >= NO_SWAP_STATUS && swap_status <= SWAP_B_STATUS)
        {
          syslog(LOG_INFO, "CS A/B swap status: %s\n",
                swap_status_str[swap_status]);
        }
      else
        {
          syslog(LOG_INFO, "CS A/B swap status error\n");
        }

        syslog(LOG_INFO, "UCB CS USERCFG ORIG : 0x%08lx\n",
              UCB_CS_ADDR(UCB_CS_USERCFG_ORIG));
        hexdump((uint8_t *)UCB_CS_ADDR(UCB_CS_USERCFG_ORIG), 64);

        syslog(LOG_INFO, "UCB CS USERCFG COPY : 0x%08lx\n",
              UCB_CS_ADDR(UCB_CS_USERCFG_COPY));
        hexdump((uint8_t *)UCB_CS_ADDR(UCB_CS_USERCFG_COPY), 64);

        syslog(LOG_INFO, "UCB CS SWAP ORIG : 0x%08lx dump as:\n",
              UCB_CS_ADDR(UCB_CS_SWAP_ORIG));
        read_ucb_swap(UCB_REGION_CS, UCB_CS_ADDR(UCB_CS_SWAP_ORIG), &g_ucb_swap);
        dump_ucb_swap(UCB_CS_ADDR(UCB_CS_SWAP_ORIG), &g_ucb_swap, UCB_REGION_CS);

        syslog(LOG_INFO, "UCB CS SWAP COPY : 0x%08lx dump as:\n",
              UCB_CS_ADDR(UCB_CS_SWAP_COPY));
        read_ucb_swap(UCB_REGION_CS, UCB_CS_ADDR(UCB_CS_SWAP_COPY), &g_ucb_swap);
        dump_ucb_swap(UCB_CS_ADDR(UCB_CS_SWAP_COPY), &g_ucb_swap, UCB_REGION_CS);
    }

  return;
}

/****************************************************************************
 * Name: aurix_ucb_initialize
 *
 * Description:
 *  initialize the ucb map
 *
 ****************************************************************************/

void aurix_ucb_initialize(UCB_T *map, int size)
{
  ucb_map = map;
  ucb_map_size = size;
}
