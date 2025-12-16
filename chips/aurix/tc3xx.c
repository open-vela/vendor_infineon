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

#include <nuttx/spi/spi_transfer.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/chip.h>
#include <sys/param.h>

#include <IfxStm.h>

#include "aurix_systimer.h"
#include "tricore_internal.h"
#include "aurix_evadc.h"
#include "aurix_i2c.h"
#include "aurix_uart.h"
#include "aurix_ioexpander.h"
#include "aurix_spi.h"
#include "aurix_gtm.h"
#include "aurix_gtm_pwm.h"
#include "aurix_qspi_slave.h"

#ifdef CONFIG_AURIX_GTM_ATOM_TIMER
#include "aurix_gtm_atom_timer.h"
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM
#include "aurix_gtm_atom_pwm.h"
#endif

#ifdef CONFIG_BUILD_PROTECTED
#include "aurix_userspace.h"
#endif

#ifdef CONFIG_ARCH_USE_MPU
#include "tricore_mpu.h"
#endif

#include "memory_layout.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_IRQ_GET(SRC_ADDR) (((uintptr_t)&SRC_ADDR - (uintptr_t)&SRC_CPU_CPU0_SB) / 4)

#define ENET_MODULE_NUMBER 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_AURIX_IOEXPANDER)
static struct aurix_ioexpander_config_s g_aurix_ioe_config[] =
{
    {&MODULE_P00},
    {&MODULE_P01},
    {&MODULE_P02},
    {&MODULE_P10},
    {&MODULE_P11},
    {&MODULE_P12},
    {&MODULE_P13},
    {&MODULE_P14},
    {&MODULE_P15},
    {&MODULE_P20},
    {&MODULE_P21},
    {&MODULE_P22},
    {&MODULE_P23},
    {&MODULE_P24},
    {&MODULE_P25},
    {&MODULE_P26},
    {&MODULE_P30},
    {&MODULE_P31},
    {&MODULE_P32},
    {&MODULE_P33},
    {&MODULE_P34},
    {&MODULE_P40},
    {&MODULE_P41}
};
#endif

#if defined(CONFIG_AURIX_UART0)
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_AURIX_UART
static struct uart_dev_s g_aurix_uart[] =
{
#if defined(CONFIG_AURIX_UART0)
  [0] =
    {
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
    .isconsole = 1,
#  endif
    .recv      =
    {
      .size    = CONFIG_UART0_RXBUFSIZE,
      .buffer  = g_uart0rxbuffer,
    },
    .xmit      =
    {
      .size    = CONFIG_UART0_TXBUFSIZE,
      .buffer  = g_uart0txbuffer,
    },
    .priv = (void *)&g_aurix_uart_config[0],
  },
#endif
};
#endif

#if defined(CONFIG_AURIX_I2C)

static const struct aurix_i2c_config_s g_aurix_i2c_config[] =
{
# if defined(CONFIG_AURIX_I2C_BUS0)
  [0] =
    {
      .module = &MODULE_I2C0,
      .pins   =
        {
          .scl       = &IfxI2c0_SCL_P15_4_INOUT,
          .sda       = &IfxI2c0_SDA_P15_5_INOUT,
          .padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
      .bus    = 0,
    },
#endif // CONFIG_AURIX_I2C_BUS0

# if defined(CONFIG_AURIX_I2C_BUS1)
  [1] =
    {
      .module = &MODULE_I2C1,
      .pins   =
        {
          .scl       = &IfxI2c1_SCL_P11_14_INOUT,
          .sda       = &IfxI2c1_SDA_P11_13_INOUT,
          .padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
      .bus    = 1,
    },
#endif // CONFIG_AURIX_I2C_BUS1
};

#endif // CONFIG_AURIX_I2C

#if defined(CONFIG_AURIX_SPI)

struct aurix_spi_config_s g_aurix_spi_config[] =
{
#ifdef CONFIG_AURIX_SPI0
  [0] =
    {
    .asclin             = &MODULE_ASCLIN1,
    .slso_0             = &IfxAsclin1_SLSO_P14_3_OUT,
    .slso_1             = &IfxAsclin1_SLSO_P20_8_OUT,
    .slso_2             = &IfxAsclin1_SLSO_P33_10_OUT,
    .slso_mode          = IfxPort_OutputMode_pushPull,
    .pinDriver          = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    .tx_irq             = 23,
    .rx_irq             = 24,
    .err_irq            = 25,
    .pins               =
    {
      .sclk             = &IfxAsclin1_SCLK_P20_10_OUT,
      .sclkMode         = IfxPort_OutputMode_pushPull,
      .rx               = &IfxAsclin1_RXB_P15_5_IN,
      .rxMode           = IfxPort_InputMode_pullUp,
      .tx               = &IfxAsclin1_TX_P15_4_OUT,
      .txMode           = IfxPort_OutputMode_pushPull,
      .pinDriver        = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
  }
#endif
};

#endif // CONFIG_AURIX_SPI

static const struct aurix_systimer_config_s g_aurix_systimer_config =
{
#if CONFIG_CPU_COREID == 0
  .tbase      = &MODULE_STM0,
  .irq        = TRICORE_IRQ_GET(SRC_STM0SR0),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 1
  .tbase      = &MODULE_STM1,
  .irq        = TRICORE_IRQ_GET(SRC_STM1SR0),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 2
  .tbase      = &MODULE_STM2,
  .irq        = TRICORE_IRQ_GET(SRC_STM2SR0),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 3
  .tbase      = &MODULE_STM3,
  .irq        = TRICORE_IRQ_GET(SRC_STM3SR0),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 4
  .tbase      = &MODULE_STM4,
  .irq        = TRICORE_IRQ_GET(SRC_STM4SR0),
  .freq       = SCU_FREQUENCY,

#elif CONFIG_CPU_COREID == 5
  .tbase      = &MODULE_STM5,
  .irq        = TRICORE_IRQ_GET(SRC_STM5SR0),
  .freq       = SCU_FREQUENCY,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ioexpander_dev_s *g_ioe[] =
{
  [23] = DEV_END,
};

#ifdef CONFIG_AURIX_UART
struct uart_dev_s *g_uart[] =
{
  [0] = &g_aurix_uart[0],
  [1] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_I2C)
struct i2c_master_s *g_i2c[] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_SPI
struct spi_dev_s *g_spi[] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_QSPI
struct spi_dev_s *g_qspi[] =
{
  [6] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_QSPI_SLAVE
struct spi_slave_ctrlr_s *g_qspislave[] =
{
  [6] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_EVADC
struct adc_dev_s *g_evadc[] =
{
  [12] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_PWM
struct pwm_lowerhalf_s *g_pwm_channels[] =
{
  [192] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_CAPTURE
struct cap_lowerhalf_s *g_capture_channels[] =
{
  [64] = DEV_END,
};
#endif

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM0
struct pwm_lowerhalf_s *g_gtm_tom0_channels[] =
{
  [16] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM0 */

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM1
struct pwm_lowerhalf_s *g_gtm_tom1_channels[] =
{
  [16] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM1 */

#ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM2
struct pwm_lowerhalf_s *g_gtm_tom2_channels[] =
{
  [16] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_TOM_PWM_TOM2 */

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM0
struct pwm_lowerhalf_s *g_gtm_atom0_channels[] =
{
    [8] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM0 */

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM1
struct pwm_lowerhalf_s *g_gtm_atom1_channels[] =
{
    [8] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM1 */

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM2
struct pwm_lowerhalf_s *g_gtm_atom2_channels[] =
{
    [8] = DEV_END,
};
#endif /* #ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM2 */

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM0_TIMER)
struct timer_lowerhalf_s *g_atom0_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM1_TIMER)
struct timer_lowerhalf_s *g_atom1_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM2_TIMER)
struct timer_lowerhalf_s *g_atom2_timer[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_GTM_CAPTURE_TIM0)
struct cap_lowerhalf_s *g_tim0_channels[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_GTM_CAPTURE_TIM1)
struct cap_lowerhalf_s *g_tim1_channels[] =
{
  [8] = DEV_END,
};
#endif

#if defined(CONFIG_AURIX_GTM_CAPTURE_TIM2)
struct cap_lowerhalf_s *g_tim2_channels[] =
{
  [8] = DEV_END,
};
#endif

#if defined CONFIG_AURIX_ENET
struct net_driver_s *g_enet_devs[ENET_MODULE_NUMBER];
#endif

#ifdef CONFIG_ARCH_USE_MPU
static unsigned int kstack_mpu_region;

#ifdef CONFIG_BUILD_PROTECTED
static unsigned int ustack_mpu_region;
#endif

static struct mpu_region_s g_mpu_regions[] =
{
  #include "mpuconfig.h"
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that tricore_earlyserialinit was called previously.
 *
 ****************************************************************************/

void tricore_serialinit(void)
{
#ifdef CONFIG_AURIX_UART
  aurix_uart_allinitialize(g_uart);
#endif
}

/****************************************************************************
 * Name: up_lateinitialize
 *
 * Description:
 *   Initialize all chip driver at board_lateinitialize/board_app_initialize
 *
 ****************************************************************************/

void up_lateinitialize(void)
{
  /* Initialize GTM clock source. */

  aurix_gtm_initialize();

#if defined(CONFIG_AURIX_IOEXPANDER)
  aurix_ioexpander_initialize(g_ioe, g_aurix_ioe_config,
                              nitems(g_aurix_ioe_config));
#endif

#if defined(CONFIG_AURIX_I2C)
  /* Initialize and register i2c bus */

  aurix_i2c_initialize(g_i2c, g_aurix_i2c_config,
                       nitems(g_aurix_i2c_config));
#endif

#ifdef CONFIG_AURIX_SPI
  aurix_all_spi_initialize(g_spi, g_aurix_spi_config,
                           nitems(g_aurix_spi_config));
#endif
}

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower;

  DEBUGASSERT(g_aurix_systimer_config.tbase);

  lower = tricore_systimer_initialize(g_aurix_systimer_config.tbase,
              g_aurix_systimer_config.irq,
              IfxStm_getFrequency(g_aurix_systimer_config.tbase));

  DEBUGASSERT(lower != NULL);

  up_alarm_set_lowerhalf(lower);
}

#ifdef CONFIG_ARCH_USE_MPU

/****************************************************************************
 * Name: os_kstack_protect
 ****************************************************************************/

#ifdef CONFIG_ARCH_KSTACK_PROTECT
void os_kstack_protect(void)
{
  mpu_modify_region(g_mpu_kset, kstack_mpu_region,
                    GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);
}
#endif

/****************************************************************************
 * Name: tricore_mpuinit
 ****************************************************************************/

void tricore_mpuinit(void)
{
  mpu_initialize(g_mpu_regions,
                 sizeof(g_mpu_regions) / sizeof(g_mpu_regions[0]));

  kstack_mpu_region =
    mpu_configure_region(g_mpu_kset,
                         GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                         GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                         REGION_TYPE_DATA | REGION_ATTR_RW);

#ifdef CONFIG_BUILD_PROTECTED
  mpu_modify_region(g_mpu_uset, kstack_mpu_region,
                    GENERATE_CORE_STACK_KERNEL_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_KERNEL_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);

  ustack_mpu_region =
    mpu_configure_region(g_mpu_kset,
                         GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                         GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                         REGION_TYPE_DATA | REGION_ATTR_RW);

#ifdef CONFIG_ARCH_STACK_PROTECT
  mpu_modify_region(g_mpu_uset, ustack_mpu_region,
                    GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RO);
#else
  mpu_modify_region(g_mpu_uset, ustack_mpu_region,
                    GENERATE_CORE_STACK_USER_START(CONFIG_CPU_COREID),
                    GENERATE_CORE_STACK_USER_SIZE(CONFIG_CPU_COREID),
                    REGION_TYPE_DATA | REGION_ATTR_RW);
#endif
#endif
}
#endif

#ifdef CONFIG_BUILD_PROTECTED
void tricore_userspace(void)
{
  aurix_userspace();
}
#endif
