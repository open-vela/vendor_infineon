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

#ifndef __VENDOR_INFINEON_CHIP_AURIX_INCLUDE_CHIP_H
#define __VENDOR_INFINEON_CHIP_AURIX_INCLUDE_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/fs/partition.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAN_MODULE_NUM 5
#define DEV_END        (void *)0xffffffff

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct can_module_s
{
  volatile uint8      can_module_ref;
  volatile spinlock_t can_module_lock;
};

typedef struct
{
  volatile int       tmadc_sync_barrier;
  volatile int       eth_sync_barrier;
  volatile bool      eth_mac_ready_flag;
  struct can_module_s can_module[CAN_MODULE_NUM];
}shared_data_manual_s;

extern shared_data_manual_s shared_data_manual;

extern struct ioexpander_dev_s *g_ioe[];
extern struct uart_dev_s *g_uart[];

#if defined(CONFIG_AURIX_I2C)
extern struct i2c_master_s *g_i2c[];
#endif

#ifdef CONFIG_AURIX_SPI
extern struct spi_dev_s *g_spi[];
#endif

#ifdef CONFIG_AURIX_EVADC
extern struct adc_dev_s *g_evadc[];
#endif

#ifdef CONFIG_AURIX_EGTM_TOM_PWM_TOM0
extern struct pwm_lowerhalf_s *g_pwm_channels[];
#endif

#ifdef CONFIG_AURIX_EGTM_ATOM_PWM_ATOM0
extern struct pwm_lowerhalf_s *g_pwm_channels[];
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM0_TIMER) || defined(CONFIG_AURIX_GTM_ATOM_ATOM0_TIMER)
extern struct timer_lowerhalf_s *g_atom0_timer[];
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM1_TIMER) || defined(CONFIG_AURIX_GTM_ATOM_ATOM1_TIMER)
extern struct timer_lowerhalf_s *g_atom1_timer[];
#endif

#if defined(CONFIG_AURIX_EGTM_ATOM_ATOM2_TIMER) || defined(CONFIG_AURIX_GTM_ATOM_ATOM2_TIMER)
extern struct timer_lowerhalf_s *g_atom2_timer[];
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM0_TIMER) || defined(CONFIG_AURIX_GTM_TOM_TOM0_TIMER)
extern struct timer_lowerhalf_s *g_tom0_timer[];
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM1_TIMER) || defined(CONFIG_AURIX_GTM_TOM_TOM1_TIMER)
extern struct timer_lowerhalf_s *g_tom1_timer[];
#endif

#if defined(CONFIG_AURIX_EGTM_TOM_TOM2_TIMER) || defined(CONFIG_AURIX_GTM_TOM_TOM2_TIMER)
extern struct timer_lowerhalf_s *g_tom2_timer[];
#endif

#ifdef CONFIG_AURIX_PWM
extern struct pwm_lowerhalf_s *g_pwm_channels[];
#endif

#ifdef CONFIG_AURIX_CAPTURE
extern struct cap_lowerhalf_s *g_capture_channels[];
#endif

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM0
extern struct pwm_lowerhalf_s *g_gtm_tom0_channels[];
#endif

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM1
extern struct pwm_lowerhalf_s *g_gtm_tom1_channels[];
#endif

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM2
extern struct pwm_lowerhalf_s *g_gtm_tom2_channels[];
#endif

#if defined(CONFIG_AURIX_MTD_PFLASH)
extern struct mtd_dev_s *g_mtd_pflash;
#endif

#if defined(CONFIG_AURIX_MTD_DFLASH)
extern struct mtd_dev_s *g_mtd_dflash;
#endif

#ifdef CONFIG_AURIX_CSMTD_PFLASH
extern struct mtd_dev_s *g_mtd_cspflash;
#endif

#ifdef CONFIG_AURIX_CSMTD_DFLASH
extern struct mtd_dev_s *g_mtd_csdflash;
#endif

#if defined(CONFIG_AURIX_I2S) || defined(CONFIG_AURIX_I2S_SIM)
extern struct audio_lowerhalf_s *g_audio_i2s[];
#endif

#if defined(CONFIG_AURIX_QSPI)
extern struct spi_dev_s *g_qspi[];
#endif

#if defined(CONFIG_AURIX_QSPI_SLAVE)
extern struct spi_slave_ctrlr_s *g_qspislave[];
#endif

#ifdef CONFIG_AURIX_TMADC
extern struct  adc_dev_s *g_adc[];
#endif

#if defined(CONFIG_AURIX_I2S) || defined(CONFIG_AURIX_I2S_SIM)
extern struct i2s_dev_s *g_i2s[];
extern struct audio_lowerhalf_s *g_audio_i2s[];
#endif

#ifdef CONFIG_AURIX_MCMCAN
  #ifdef CONFIG_AURIX_MCMCAN_CHARDRIVER
    extern struct can_dev_s *g_mcmcan_devs[];
  #else
    extern struct net_driver_s *g_mcmcan_devs[];
  #endif
#endif

#ifdef CONFIG_AURIX_LIN
extern struct net_driver_s *g_lin_devs[];
#endif

#if defined CONFIG_AURIX_ENET
extern struct net_driver_s *g_enet_devs[];
#endif

#ifdef CONFIG_AURIX_QBVSCH
extern struct timer_lowerhalf_s *g_qbv_schedu_timer[];
#endif

#endif /* __VENDOR_INFINEON_CHIP_AURIX_INCLUDE_CHIP_H */
