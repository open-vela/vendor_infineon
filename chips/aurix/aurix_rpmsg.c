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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun_bmp.h>
#include <nuttx/serial/uart_rpmsg.h>

#include "aurix_rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AURIX_RPMSG_CSCORE_ID         6
#define AURIX_RPMSG_CARVEOUT_RESERVED 128

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void aurix_rpmsg_get_name(char *cpuname, size_t size,
                                 int cpu, bool uart)
{
  if (cpu == AURIX_RPMSG_CSCORE_ID)
    {
      snprintf(cpuname, size, "%s", uart ? "CORECS": "corecs");
    }
  else
    {
      snprintf(cpuname, size, "%s%u", uart ? "CORE": "core", cpu);
    }
}

/****************************************************************************
 * Name: aurix_rptun_rsc_init
 ****************************************************************************/

static void
aurix_rptun_rsc_init(const struct aurix_rpmsg_config_s *config,
                     void *base, size_t len)
{
  FAR struct rptun_rsc_s *rsc;

  memset(base, 0, len);

  rsc = base;

  rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                           rpmsg_vdev);
  rsc->rpmsg_vdev.type          = RSC_VDEV;
  rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
  rsc->rpmsg_vdev.notifyid      = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS |
                                  1 << VIRTIO_RPMSG_F_ACK |
                                  1 << VIRTIO_RPMSG_F_BUFSZ |
                                  1 << VIRTIO_RPMSG_F_CPUNAME;
  rsc->rpmsg_vdev.num_of_vrings = 2;
  rsc->rpmsg_vdev.notifyid      = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
  rsc->rpmsg_vdev.reserved[0]   = VIRTIO_DEV_DRIVER;
  rsc->rpmsg_vdev.reserved[1]   = 0;
  rsc->rpmsg_vring0.da          = 0;
  rsc->rpmsg_vring0.align       = config->vring_align;
  rsc->rpmsg_vring0.num         = config->vring_num;
  rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vring1.da          = 0;
  rsc->rpmsg_vring1.align       = config->vring_align;
  rsc->rpmsg_vring1.num         = config->vring_num;
  rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->config.r2h_buf_size      = config->vring_buf_size;
  rsc->config.h2r_buf_size      = config->vring_buf_size;
  aurix_rpmsg_get_name((FAR char *)rsc->config.remote_cpuname,
                       sizeof(rsc->config.remote_cpuname),
                       config->slave_cpu, false);
  aurix_rpmsg_get_name((FAR char *)rsc->config.host_cpuname,
                       sizeof(rsc->config.host_cpuname),
                       config->master_cpu, false);

  rsc->offset[1]                = offsetof(struct rptun_rsc_s,
                                           carveout);
  rsc->carveout.da              = (metal_phys_addr_t)rsc +
                                  ALIGN_UP(sizeof(struct rptun_rsc_s),
                                           config->vring_align);
  rsc->carveout.len             = len - ALIGN_UP(sizeof(struct rptun_rsc_s),
                                                 config->vring_align);
  rsc->carveout.pa              = FW_RSC_U32_ADDR_ANY;
  memcpy(rsc->carveout.name, "vdev0buffer", 11);

  UP_DMB();
  rsc->rsc_tbl_hdr.ver          = 1;
  rsc->rsc_tbl_hdr.num          = 2;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_rpmsg_serialinit
 ****************************************************************************/

int aurix_rpmsg_serialinit(const struct aurix_rpmsg_config_s *cfg,
                           int num)
{
  int cpuid = sched_getcpu();
  int ret1 = OK;
  int ret = 0;
  int i;

  for (i = 0; i < num; i++)
    {
      int remote_cpuid;
      int uart_cpuid;
      char uartname[7];
      char cpuname[7];
      bool umaster;

      if ((cfg[i].master_cpu != cpuid && cfg[i].slave_cpu != cpuid) ||
          (cfg[i].master_cpu != CONFIG_AURIX_RPMSG_UART_MASTER_CPUID &&
           cfg[i].slave_cpu != CONFIG_AURIX_RPMSG_UART_MASTER_CPUID))
        {
          continue;
        }

      umaster = cpuid == CONFIG_AURIX_RPMSG_UART_MASTER_CPUID;

      remote_cpuid = cpuid == cfg[i].master_cpu ? cfg[i].slave_cpu :
                     cfg[i].master_cpu;
      uart_cpuid = umaster ? remote_cpuid : cpuid;

      aurix_rpmsg_get_name(cpuname, sizeof(cpuname), remote_cpuid , false);
      aurix_rpmsg_get_name(uartname, sizeof(uartname), uart_cpuid , true);

      ret = uart_rpmsg_init(cpuname, uartname, 1024, !umaster);

      if (ret < 0)
        {
          ret1 = ret;
          rpmsgerr("ERROR: uart_rpmsg_init failed %d\n", ret);
        }
    }

  return ret1;
}

/****************************************************************************
 * Name: aurix_rpmsg_init
 ****************************************************************************/

int aurix_rpmsg_init(const struct aurix_rpmsg_config_s *cfg, int num,
                     void *base, size_t len)
{
  FAR void *rsc = base;
  size_t size;
  int cpuid = sched_getcpu();
  int ret1 = OK;
  int ret;
  int i;

  for (i = 0; i < num; i++, rsc += size)
    {
      cpu_set_t cpuset;
      char cpuname[7];
      int irq_trigger;
      int irq_event;
      bool master;

      if ((cfg[i].master_cpu >= CONFIG_NCPUS &&
           cfg[i].master_cpu != AURIX_RPMSG_CSCORE_ID) ||
          (cfg[i].slave_cpu >= CONFIG_NCPUS &&
           cfg[i].slave_cpu != AURIX_RPMSG_CSCORE_ID))
        {
          size = 0;
          continue;
        }

      size = ALIGN_UP(sizeof(FAR struct rptun_rsc_s),
                      cfg[i].vring_align);
      size += 2 * ALIGN_UP(vring_size(cfg[i].vring_num,
                                      cfg[i].vring_align),
                                      cfg[i].vring_align);
      size += 2 * ALIGN_UP(cfg[i].vring_num * cfg[i].vring_buf_size,
                           cfg[i].vring_align);
      size += AURIX_RPMSG_CARVEOUT_RESERVED;

      if (rsc + size > base + len)
        {
          rpmsgerr("ERROR: Not enough shared memory for rpmsg\n"
                   "rsc: %p, size: %zu, base: %p, len: %zu, cfg index: %d\n",
                    rsc, size, base, len, i);
          return -ENOMEM;
        }

      CPU_ZERO(&cpuset);
      if (cpuid == cfg[i].master_cpu)
        {
          /* setup resource table */

          aurix_rptun_rsc_init(&cfg[i], rsc, size);

          CPU_SET(cfg[i].slave_cpu, &cpuset);
          master = true;
          aurix_rpmsg_get_name(cpuname, sizeof(cpuname),
                               cfg[i].slave_cpu, false);

          irq_trigger = TRICORE_GPSR_IRQNUM(cfg[i].master_cpu,
                                            cfg[i].slave_cpu);
          irq_event = TRICORE_GPSR_IRQNUM(cfg[i].slave_cpu,
                                          cfg[i].master_cpu);
        }
      else if (cpuid == cfg[i].slave_cpu)
        {
          CPU_SET(cfg[i].master_cpu, &cpuset);
          master = false;
          aurix_rpmsg_get_name(cpuname, sizeof(cpuname),
                               cfg[i].master_cpu, false);

          irq_trigger = TRICORE_GPSR_IRQNUM(cfg[i].slave_cpu,
                                            cfg[i].master_cpu);
          irq_event = TRICORE_GPSR_IRQNUM(cfg[i].master_cpu,
                                          cfg[i].slave_cpu);
        }
      else
        {
          continue;
        }

      ret = rptun_bmp_init(cpuname, master,
                           (FAR struct rptun_rsc_s *)rsc,
                           irq_event, irq_trigger, cpuset);
      if (ret < 0)
        {
          rpmsgerr("ERROR: Failed to init rpmsg virtio bmp: %d\n", ret);
          ret1 = ret;
        }
    }

  return ret1;
}
