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
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <nuttx/fs/fs.h>
#include <sys/param.h>

#include <debug.h>
#include <stdio.h>

#include "aurix_mtd_partition.h"
#include "aurix_mtd_flash.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* partition table, first entry *must always* be program flash memory */

/* Define default values to silent compiler warning about undefined macro */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_partition_init
 *
 *   Initialize aurix partition. Read partition information, and use
 *   these data for creating MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

void aurix_partition_init(const struct partition_s *ptabs, size_t count,
                          struct mtd_dev_s *mtd_flash)
{
  struct mtd_dev_s *mtd_part;
  struct mtd_geometry_s geo;
  int ret = 0;
  int i;

  ret = MTD_IOCTL(mtd_flash, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      ferr("ERROR: Failed to get info from MTD\n");
      return;
    }

  for (i = 0; i != count; i++)
    {
      mtd_part = mtd_partition(mtd_flash, ptabs[i].firstblock,
                               ptabs[i].nblocks);

      if (mtd_part == NULL)
        {
          ferr("[%s]ERROR: mtd_partition() failed %d\n",
               ptabs[i].name, errno);
          continue;
        }

#ifndef CONFIG_MTD_CONFIG_NONE
      if (!strcmp(ptabs[i].name, CONFIG_AURIX_MTD_CFG_PATH_FOR_NVM))
        {
          ret = mtdconfig_register_by_path(mtd_part, ptabs[i].name);
        }
      else
 #endif
        {
          ret = register_mtddriver(ptabs[i].name, mtd_part, 0755, NULL);
        }

      if (ret != 0)
        {
          ferr("register_mtddriver failed: %d\n", ret);
        }
    }

  return;
}

