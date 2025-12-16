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

#include <nuttx/mutex.h>
#include <nuttx/arch.h>
#include <sys/param.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>

#include "aurix_crypto.h"
#include "Rng/Rng/IfxRng_Rng.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_rng_dev_s
{
  mutex_t rd_lock; /* mutex for read RNG data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t aurix_rng_read(struct file *filep, char *buffer,
                              size_t buflen);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static const struct file_operations g_aurix_rngops =
{
  NULL,           /* open */
  NULL,           /* close */
  aurix_rng_read, /* read */
};

static struct aurix_rng_dev_s g_aurix_rng_dev =
{
  .rd_lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_rng_read
 *
 * Description:
 *
 *
 ****************************************************************************/

static ssize_t aurix_rng_read(struct file *filep, char *buffer,
                              size_t buflen)
{
  struct aurix_rng_dev_s *rng_lock = &g_aurix_rng_dev;
  ssize_t read_len = 0;
  size_t copied = 0;
  uint32 word = 0;

  IfxRng_Rng_Config config;
  IfxRng_Rng rng;
  IfxRng_status status;

  if (nxmutex_lock(&rng_lock->rd_lock) != OK)
    {
      return -EBUSY;
    }

  /* Check if RNG module is enabled */

  if (IfxRng_isModuleEnabled(&MODULE_RNG) == FALSE)
    {
      /* Enable Module clock */

      IfxRng_enableModule(&MODULE_RNG);
    }

  /* Initialize the config to default values */

  IfxRng_Rng_initConfig(&MODULE_RNG, &config);

  /* TODO: use param in g_aurix_rng_dev to change rng mode */

  /* Configuration of TRNG Mode */

  config.mode = IfxRng_operationMode_TRNG;

  /* Initialize the Rng module to start generation */

  status = IfxRng_Rng_init(&rng, &config);
  if (status != IfxRng_status_success)
    {
      read_len = -EIO;
      goto exit;
    }

  /* Check if module is in error */

  if (IfxRng_Rng_getAndUpdateError(&rng) != IfxRng_status_success)
    {
      read_len = -EIO;
      goto exit;
    }

  /* Wait until the buffer is filled */

  while (read_len < buflen)
    {
      copied = MIN(sizeof(word), buflen - read_len);

      word = IfxRng_Rng_readResult(&rng);

      if (IfxRng_Rng_getAndUpdateError(&rng) != IfxRng_status_success)
        {
          goto exit;
        }

      memcpy(buffer, (char *)&word, copied);
      buffer += copied;
      read_len += copied;
    }

exit:

  /* Go to config mode and stop generating random words */

  IfxRng_setModuleState(rng.rngSFR, IfxRng_state_config);

  /* Release rd_lock for next read */

  nxmutex_unlock(&rng_lock->rd_lock);

  return read_len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *   Must be called BEFORE devurandom_register.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void devrandom_register(void)
{
  if (register_driver("/dev/random", &g_aurix_rngops, 0444, NULL) == 0)
    {
      aurix_set_hsmstatus(CSRM2HT_RNG_INITED);
    }
}
#endif

/****************************************************************************
 * Name: devurandom_register
 *
 * Description:
 *   Register /dev/urandom
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_URANDOM_ARCH
void devurandom_register(void)
{
  register_driver("/dev/urandom", &g_aurix_rngops, 0444, NULL);
}
#endif
