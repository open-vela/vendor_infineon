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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_UCB__H
#define __VENDOR_INFINEON_CHIPS_AURIX_UCB__H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "aurix_mtd_flash.h"

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
#ifdef CONFIG_ARCH_CHIP_AURIX_TC4DX
#define UCB_RTC_USERCFG_ORIG              (17)
#define UCB_RTC_USERCFG_COPY              (18)
#define UCB_RTC_SWAP_ORIG                 (19)
#define UCB_RTC_SWAP_COPY                 (20)
#define UCB_CS_SWAP_ORIG                  (12)
#define UCB_CS_SWAP_COPY                  (13)
#define UCB_CS_USERCFG_ORIG               (15)
#define UCB_CS_USERCFG_COPY               (16)
#else
#define UCB_RTC_USERCFG_ORIG              (2)
#define UCB_RTC_USERCFG_COPY              (3)
#define UCB_RTC_SWAP_ORIG                 (6)
#define UCB_RTC_SWAP_COPY                 (7)
#define UCB_CS_USERCFG_ORIG               (7)
#define UCB_CS_USERCFG_COPY               (8)
#define UCB_CS_SWAP_ORIG                  (12)
#define UCB_CS_SWAP_COPY                  (13)
#endif
/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  UCB_STATE_UNKNOWN = 0,
  UCB_STATE_UNLOCKED = 1,
  UCB_STATE_CONFIRMED = 2 ,
  UCB_STATE_ERRORED = 3 ,
} UCB_CONFIRMATION_STATE_E;

typedef enum
{
  UCB_REGION_RTC = 0,
  UCB_REGION_CS = 1,
  UCT_TYPE_MAX,
} UCB_REGION_E;

/* Slot information */

typedef enum
{
  NO_SWAP_STATUS = NO_SWAP,
  SWAP_A_STATUS = SLOT_A,
  SWAP_B_STATUS = SLOT_B,
  SWAP_MAX = 3,
} SLOT_STATUS_E;

typedef struct
{
  const char * ucb_name;
  UCB_REGION_E region;
  int type;
  int ucb_orig_no;
  int ucb_copy_no;
  uint32_t (*ucb_addr)(int ucb_no);
  uint32_t password[8] IFX_ALIGN(4);
}UCB_T;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern char *swap_status_str[];
extern int get_swap_status(int region);
extern int tc4_rtc_ab_swap(char *ucb_name, SLOT_STATUS_E slot);
extern int tc4_ab_swap_enable(char *ucb_name, bool enable);
extern void dump_swap_status(void);
extern void aurix_ucb_initialize(UCB_T *map, int size);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __VENDOR_INFINEON_CHIPS_AURIX_MTD_FLASH__H */
