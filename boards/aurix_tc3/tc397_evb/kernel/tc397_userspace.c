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

#include "aurix_userspace.h"
#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/mm/mm.h>

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NUTTX_USERSPACE
#error "CONFIG_NUTTX_USERSPACE not defined"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct userspace_data_s g_userspace_data =
{
  .us_heap = &g_mmheap,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These 'addresses' of these values are setup by the linker script.
 * They are not actual uint32_t storage locations!
 * They are only used meaningfully in the following way:
 *
 *  - The linker script defines, for example, the symbol_sdata.
 *  - The declaration extern uint32_t _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a uint32_t variable _data (it
 *    is not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  uint32_t *pdata = &_sdata;
 */

/* These symbols are setup by the linker script. */

extern uint8_t _stext[]; /* Start of .text */
extern uint8_t _etext[]; /* End_1 of .text + .rodata */
extern const uint8_t _eronly[]; /* End+1 of read only section (.text + .rodata) */
extern uint8_t _sdata_pflash[]; /* Start of the loading address of the .data section */
extern uint8_t _sdata[]; /* Start of .data */
extern uint8_t _edata[]; /* End+1 of .data */
extern uint8_t _sbss[]; /* Start of .bss */
extern uint8_t _ebss[]; /* End+1 of .bss */
extern uint8_t _scalibra[]; /* Start of .calibra */
extern uint8_t _ecalibra[]; /* End+1 of .calibra */
extern uint8_t _calib_pflash[]; /* End+1 of calibra in flash */

const struct aurix_userspace userspace locate_data(".userspace") = {
    /* General memory map */

    .common.us_entrypoint = CONFIG_INIT_ENTRYPOINT,
    .common.us_textstart = (uintptr_t)&_stext,
    .common.us_textend = (uintptr_t)&_etext,
    .common.us_datasource = (uintptr_t)&_sdata_pflash,
    .common.us_datastart = (uintptr_t)&_sdata,
    .common.us_dataend = (uintptr_t)&_edata,
    .common.us_bssstart = (uintptr_t)&_sbss,
    .common.us_bssend = (uintptr_t)&_ebss,

    /* User data memory structure */

    .common.us_data = &g_userspace_data,

    /* Task/thread startup routines */

    .common.task_startup = nxtask_startup,

    /* Signal handler trampoline */

#ifndef CONFIG_DISABLE_SIGNALS
    .common.signal_handler = up_signal_handler,
#endif

/* User-space work queue support (declared in include/nuttx/wqueue.h) */

#ifdef CONFIG_LIBC_USRWORK
    .common.work_usrstart = work_usrstart,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
