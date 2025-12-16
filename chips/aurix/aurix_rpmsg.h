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

#ifndef __VENDOR_INFINEON_CHIPS_AURIX_AURIX_RPMSG__H
#define __VENDOR_INFINEON_CHIPS_AURIX_AURIX_RPMSG__H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/rpmsg/rpmsg.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_AURIX_RPMSG

struct aurix_rpmsg_config_s
{
  uint32_t   vring_num;
  uint32_t   vring_align;
  uint32_t   vring_buf_size;
  int        master_cpu;
  int        slave_cpu;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_rpmsg_serialinit
 ****************************************************************************/

int aurix_rpmsg_serialinit(const struct aurix_rpmsg_config_s *cfg,
                           int num);

/****************************************************************************
 * Name: aurix_rpmsg_init
 ****************************************************************************/

int aurix_rpmsg_init(const struct aurix_rpmsg_config_s *cfg, int num,
                     void *base, size_t len);

#endif /* CONFIG_AURIX_RPMSG */

#endif /* __VENDOR_INFINEON_CHIPS_AURIX_AURIX_RPMSG__H */
