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

#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/wqueue.h>
#include <memory_layout.h>

#include "aurix_coredump.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct work_s g_coredump_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void aurix_coredump_worker(void *arg)
{
  uint8_t *buffer = (uint8_t *)COREDUMP_RAM_START;
  size_t size = (size_t)COREDUMP_RAM_SIZE;
  size_t written = 0;
  ssize_t nbytes;
  int fd;

  /* Check if there has coredump data */

  if (buffer[0] == 0x00)
    {
      return;
    }

  /* Save the coredump data to flash */

  fd = open("/dev/trapinfo", O_RDWR);
  if (fd < 0)
    {
      return;
    }

  do
    {
      nbytes = write(fd, buffer, size - written);
      if (nbytes < 0)
        {
          close(fd);
          return;
        }

      written += nbytes;
      buffer += nbytes;
    }
  while (written < size);

  close(fd);

  /* Clear the coredump header */

  memset((void *)COREDUMP_RAM_START, 0x00, 16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tricore_save_coredump(void)
{
  work_queue(LPWORK, &g_coredump_work, aurix_coredump_worker, NULL, 0);
}
