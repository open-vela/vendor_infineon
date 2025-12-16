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

#include <nuttx/fs/fs.h>
#include <nuttx/board.h>
#include <arch/chip/chip.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/userspace.h>

#include <sys/types.h>
#include <sys/param.h>
#include <syslog.h>

#include "tricore_internal.h"

#include "aurix_evadc.h"
#include "aurix_uart.h"
#include "aurix_lin.h"
#include "aurix_qspi_tc3.h"
#include "aurix_qspi_slave.h"
#include "aurix_gtm.h"
#include "aurix_gtm_pwm.h"
#include "aurix_gtm_capture.h"
#include "tc397.h"

#ifdef CONFIG_AURIX_GTM_ATOM_TIMER
#include "aurix_gtm_atom_timer.h"
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM
#include "aurix_gtm_atom_pwm.h"
#endif

#include "memory_layout.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_IRQ_GET(SRC_ADDR) (((uintptr_t)&SRC_ADDR - (uintptr_t)&SRC_CPU_CPU0_SB) / 4)
#define TLF_INIT_DELAY_TIME_US      60                          /* Delay time of 60us as defined in the Data Sheet   */

/****************************************************************************
 * Private type
 ****************************************************************************/

#ifdef CONFIG_GPIO_LOWER_HALF
struct tc397_pin_config_s
{
  uint8_t                    numOfIoe;
  uint8_t                    pin;
  enum gpio_pintype_e        type;
  int                        minor;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Please refer to g_ioe_config for
 * corresponding ioe num for each port
 */

#ifdef CONFIG_GPIO_LOWER_HALF
static const struct tc397_pin_config_s g_pin_config[] =
{
  /* ioe num | pin | pin type         | gpio minor */

  {
    6, 0, GPIO_OUTPUT_PIN, 0
  },
  {
    0, 11, GPIO_OUTPUT_PIN, 1
  },
  {
    0, 12, GPIO_INPUT_PIN,  2
  },
};
#endif

#ifdef CONFIG_AURIX_PWM
const struct aurix_pwm_cfg_s g_aurix_pwm_channel_cfgs[] =
{
#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM0_CH2
  {
    .devpath        = "/dev/pwm0",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 0,
    .channel_id      = 2,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM0_2_TOUT83_P14_3_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM2_CH1
  {
    .devpath        = "/dev/pwm1",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 2,
    .channel_id      = 1,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM2_1_TOUT43_P23_2_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM3_CH1
  {
    .devpath        = "/dev/pwm2",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 3,
    .channel_id      = 1,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM3_1N_TOUT79_P15_8_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM3_CH4
  {
    .devpath        = "/dev/pwm3",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 3,
    .channel_id      = 4,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM3_4_TOUT131_P22_5_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM4_CH6
  {
    .devpath        = "/dev/pwm4",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 4,
    .channel_id      = 6,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM4_6N_TOUT134_P22_8_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM4_CH7
  {
    .devpath        = "/dev/pwm5",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 4,
    .channel_id      = 7,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM4_7N_TOUT135_P22_9_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM5_CH1
  {
    .devpath        = "/dev/pwm6",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 5,
    .channel_id      = 1,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM5_1_TOUT175_P31_1_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM5_CH3
  {
    .devpath        = "/dev/pwm7",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 5,
    .channel_id      = 3,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM5_3_TOUT255_P13_6_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM5_CH4
  {
    .devpath        = "/dev/pwm8",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 5,
    .channel_id      = 4,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM5_4_TOUT123_P11_7_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM5_CH5
  {
    .devpath        = "/dev/pwm9",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 5,
    .channel_id      = 5,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM5_5_TOUT269_P10_11_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM5_CH7
  {
    .devpath        = "/dev/pwm10",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 5,
    .channel_id      = 7,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM5_7_TOUT248_P13_9_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM6_CH1
  {
    .devpath        = "/dev/pwm11",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 6,
    .channel_id      = 1,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM6_1N_TOUT158_P01_12_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM6_CH2
  {
    .devpath        = "/dev/pwm12",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 6,
    .channel_id      = 2,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM6_2_TOUT252_P13_14_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM6_CH5
  {
    .devpath        = "/dev/pwm13",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 6,
    .channel_id      = 5,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM6_5_TOUT262_P13_13_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM6_CH6
  {
    .devpath        = "/dev/pwm14",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 6,
    .channel_id      = 6,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM6_6_TOUT188_P31_14_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif

#ifdef CONFIG_AURIX_GTM_ATOM_PWM_ATOM7_CH6
  {
    .devpath        = "/dev/pwm15",
    .module_type    = GTM_Module_Type_ATOM,
    .module_id         = 7,
    .channel_id      = 6,
    .ch_clk_id  = 0,
    .pin_map        = &IfxGtm_ATOM7_6_TOUT153_P02_13_OUT,
    .driver_strength = IfxPort_PadDriver_cmosAutomotiveSpeed3,
    .output_mode    = IfxPort_OutputMode_pushPull,
    .isr_provider    = IfxSrc_Tos_cpu0,
  },
#endif
};
#endif  // CONFIG_AURIX_PWM

#ifdef CONFIG_AURIX_CAPTURE
const struct aurix_gtm_capture_config_s g_aurix_capture_channel_cfgs[] =
{
#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM1_CH1
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_1,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_1,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM1_1_P14_6_IN,
    .gtm_tim_ch_irq = 1954,
    .capture_devpath = "/dev/capture1_1",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM1_CH4
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_1,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_4,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM1_4_P20_0_IN,
    .gtm_tim_ch_irq = 1955,
    .capture_devpath = "/dev/capture1_4",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM2_CH4
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_2,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_4,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM2_4_P31_4_IN,
    .gtm_tim_ch_irq = 1960,
    .capture_devpath = "/dev/capture2_4",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM3_CH0
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_3,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_0,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM3_0_P13_3_IN,
    .gtm_tim_ch_irq = 1954,
    .capture_devpath = "/dev/capture3_0",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM3_CH6
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_3,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_6,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM3_6_P13_1_IN,
    .gtm_tim_ch_irq = 1955,
    .capture_devpath = "/dev/capture3_6",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM3_CH7
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_3,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_7,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM3_7_P13_2_IN,
    .gtm_tim_ch_irq = 1955,
    .capture_devpath = "/dev/capture3_7",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM4_CH4
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_4,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_4,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM4_4_P24_4_IN,
    .gtm_tim_ch_irq = 1954,
    .capture_devpath = "/dev/capture4_4",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM4_CH5
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_4,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_5,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM4_5_P01_2_IN,
    .gtm_tim_ch_irq = 1955,
    .capture_devpath = "/dev/capture4_5",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM5_CH1
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_5,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_1,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM5_1_P31_9_IN,
    .gtm_tim_ch_irq = 1954,
    .capture_devpath = "/dev/capture5_1",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM5_CH4
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_5,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_4,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM5_4_P00_0_IN,
    .gtm_tim_ch_irq = 1955,
    .capture_devpath = "/dev/capture5_4",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif

#ifdef CONFIG_AURIX_GTM_CAPTURE_TIM6_CH3
  {
    .gtm_base       = &MODULE_GTM,
    .gtm_tim        = IfxGtm_Tim_6,
    .gtm_tim_ch     = IfxGtm_Tim_Ch_3,
    .gtm_cmu_clk    = IfxGtm_Cmu_Clk_0,
    .gtm_tim_pin    = &IfxGtm_TIM6_3_P23_4_IN,
    .gtm_tim_ch_irq = 1954,
    .capture_devpath = "/dev/capture6_3",
    .isr_provider    = IfxSrc_Tos_cpu0,
    .filter_time     = 5e-07,
    .timeout_time    = 0.012,
  },
#endif
};
#endif //CONFIG_AURIX_CAPTURE

#ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_TIMER
static const struct aurix_gtm_atom_timer_config_s
g_aurix_gtm_atom0_timer_config[] =
{
  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH0_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_0,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_0),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_0",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH1_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_1,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_0),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_1",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH2_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_2,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_1),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_2",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH3_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_3,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_1),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_3",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH4_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_4,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_2),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_4",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH5_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_5,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_2),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_5",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH6_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_6,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_3),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_6",
  },
  #endif

  #ifdef CONFIG_AURIX_GTM_ATOM_ATOM0_CH7_TIMER
  {
    .atom                    = IfxGtm_Atom_0,
    .channel                 = IfxGtm_Atom_Ch_7,
    .clock                   = IfxGtm_Cmu_Clk_0,
    .isrIrq                  = TRICORE_IRQ_GET(SRC_GTM_ATOM0_3),
    .base.frequency          = 100.0f,
    .irqModeTimer            = IfxGtm_IrqMode_pulseNotify,
    .irqModeTrigger          = IfxGtm_IrqMode_pulseNotify,
    .devpath                 = "/dev/timer0_7",
  },
  #endif
};
#endif/* CONFIG_AURIX_GTM_ATOM_ATOM0_TIMER */

#ifdef CONFIG_AURIX_QSPI

#ifdef CONFIG_AURIX_QSPI0

const int g_qspi0_cs_active[9] =
{
  [0] = 0,
  [1] = 0,
  [2] = 0,
  [3] = 0,
  [4] = 0,
  [5] = 0,
  [6] = 0,
  [7] = 0,
  [8] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi0_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi0_SLSO2_P20_13_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [1] =
    {
      .pin = &IfxQspi0_SLSO4_P11_11_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [2] =
    {
      .pin = &IfxQspi0_SLSO7_P33_5_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [3] =
    {
      .pin = &IfxQspi0_SLSO8_P20_6_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [4] =
    {
      .pin = &IfxQspi0_SLSO9_P20_3_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [5] =
    {
      .pin = &IfxQspi0_SLSO10_P22_11_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [6] =
    {
      .pin = &IfxQspi0_SLSO11_P23_6_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [7] =
    {
      .pin = &IfxQspi0_SLSO12_P22_4_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [8] =
    {
      .pin = &IfxQspi0_SLSO6_P20_10_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};
#endif

#ifdef CONFIG_AURIX_QSPI1

const int g_qspi1_cs_active[6] =
{
  [0] = 0,
  [1] = 0,
  [2] = 0,
  [3] = 0,
  [4] = 0,
  [5] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi1_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi1_SLSO0_P20_8_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [1] =
    {
      .pin = &IfxQspi1_SLSO1_P20_9_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [2] =
    {
      .pin = &IfxQspi1_SLSO6_P33_10_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [3] =
    {
      .pin = &IfxQspi1_SLSO8_P10_4_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [4] =
    {
      .pin = &IfxQspi1_SLSO9_P10_5_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [5] =
    {
      .pin = &IfxQspi1_SLSO10_P10_0_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};
#endif

#ifdef CONFIG_AURIX_QSPI2

const int g_qspi2_cs_active[2] =
{
  [0] = 0,
  [1] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi2_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi2_SLSO1_P14_2_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [1] =
    {
      .pin = &IfxQspi2_SLSO12_P32_6_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};
#endif

#ifdef CONFIG_AURIX_QSPI3

const int g_qspi3_cs_active[6] =
{
  [0] = 0,
  [1] = 0,
  [2] = 0,
  [3] = 0,
  [4] = 0,
  [5] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi3_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi3_SLSO0_P02_4_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [1] =
    {
      .pin = &IfxQspi3_SLSO1_P02_0_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [2] =
    {
      .pin = &IfxQspi3_SLSO2_P02_1_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [3] =
    {
      .pin = &IfxQspi3_SLSO3_P02_2_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [4] =
    {
      .pin = &IfxQspi3_SLSO5_P02_8_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [5] =
    {
      .pin = &IfxQspi3_SLSO6_P02_15_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};
#endif

#ifdef CONFIG_AURIX_QSPI4

const int g_qspi4_cs_active[2] =
{
  [0] = 0,
  [1] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi4_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi4_SLSO3_P22_2_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },

  [1] =
    {
      .pin = &IfxQspi4_SLSO4_P23_5_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    },
};
#endif

#ifdef CONFIG_AURIX_QSPI5

const int g_qspi5_cs_active[1] =
{
  [0] = 0
};

static const IfxQspi_SpiMaster_Output g_qspi5_cs[] =
{
  [0] =
    {
      .pin = &IfxQspi5_SLSO0_P15_13_OUT,
      .mode = IfxPort_OutputMode_pushPull,
      .driver = IfxPort_PadDriver_cmosAutomotiveSpeed1,
    }
};
#endif

static const struct aurix_qspi_config_s g_aurix_qspi_config[] =
{
  #ifdef CONFIG_AURIX_QSPI0
  [0] =
    {
      .cs        = g_qspi0_cs,
      .cs_active  = g_qspi0_cs_active,
      .cs_num    = nitems(g_qspi0_cs),
      .pins      =
        {
          &IfxQspi0_SCLK_P20_11_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi0_MTSR_P20_14_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi0_MRSTA_P20_12_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 32,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI0TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI0RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI0ERR),
      .qspi      = &MODULE_QSPI0,
      .dev_id   = 0,
    },
  #endif

  #ifdef CONFIG_AURIX_QSPI1
  [1] =
    {
      .cs        = g_qspi1_cs,
      .cs_active  = g_qspi1_cs_active,
      .cs_num    = nitems(g_qspi1_cs),
      .pins      =
        {
          &IfxQspi1_SCLK_P10_2_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi1_MTSR_P10_3_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi1_MRSTA_P10_1_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 32,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI1TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI1RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI1ERR),
      .qspi      = &MODULE_QSPI1,
      .dev_id   = 1,
    },
  #endif

  #ifdef CONFIG_AURIX_QSPI2
  [2] =
    {
      .cs        = g_qspi2_cs,
      .cs_active  = g_qspi2_cs_active,
      .cs_num    = nitems(g_qspi2_cs),
      .pins      =
        {
          &IfxQspi2_SCLK_P15_8_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi2_MTSR_P15_6_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi2_MRSTB_P15_7_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 16,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI2TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI2RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI2ERR),
      .qspi      = &MODULE_QSPI2,
      .dev_id   = 2,
    },
  #endif

  #ifdef CONFIG_AURIX_QSPI3
  [3] =
    {
      .cs        = g_qspi3_cs,
      .cs_active  = g_qspi3_cs_active,
      .cs_num    = nitems(g_qspi3_cs),
      .pins      =
        {
          &IfxQspi3_SCLK_P01_7_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi3_MTSR_P01_6_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi3_MRSTC_P01_5_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 32,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI3TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI3RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI3ERR),
      .qspi      = &MODULE_QSPI3,
      .dev_id   = 3,
    },
  #endif

  #ifdef CONFIG_AURIX_QSPI4
  [4] =
    {
      .cs        = g_qspi4_cs,
      .cs_active  = g_qspi4_cs_active,
      .cs_num    = nitems(g_qspi4_cs),
      .pins      =
        {
          &IfxQspi4_SCLK_P22_3_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi4_MTSR_P22_0_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi4_MRSTB_P22_1_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 32,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI4TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI4RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI4ERR),
      .qspi      = &MODULE_QSPI4,
      .dev_id   = 4,
    },
  #endif

  #ifdef CONFIG_AURIX_QSPI5
  [5] =
    {
      .cs        = g_qspi5_cs,
      .cs_active  = g_qspi5_cs_active,
      .cs_num    = nitems(g_qspi5_cs),
      .pins      =
        {
          &IfxQspi5_SCLK_P14_13_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi5_MTSRA_P15_14_IN, IfxPort_OutputMode_pushPull,
          &IfxQspi5_MRSTA_P15_10_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed3
        },
      .baudrate = 1000000,
      .datawidth = 32,
      .tx_irq    = TRICORE_IRQ_GET(SRC_QSPI5TX),
      .rx_irq    = TRICORE_IRQ_GET(SRC_QSPI5RX),
      .err_irq   = TRICORE_IRQ_GET(SRC_QSPI5ERR),
      .qspi      = &MODULE_QSPI5,
      .dev_id   = 5,
    },
  #endif
  };

#endif /* CONFIG_AURIX_QSPI */

#ifdef CONFIG_AURIX_QSPI_SLAVE
static const struct aurix_qspi_slave_config_s g_aurix_qspislave_config[] =
{
  #ifdef CONFIG_AURIX_QSPISLAVE4
  [4] =
    {
      .pins =
        {
          &IfxQspi4_SCLKB_P22_3_IN, IfxPort_InputMode_pullDown,
          &IfxQspi4_MTSRB_P22_0_IN, IfxPort_InputMode_pullDown,
          &IfxQspi4_MRST_P22_1_OUT, IfxPort_OutputMode_pushPull,
          &IfxQspi4_SLSIB_P22_2_IN, IfxPort_InputMode_pullDown,
          IfxPort_PadDriver_cmosAutomotiveSpeed1,
        },
      .qspi = &MODULE_QSPI4,
      .tx_irq = TRICORE_IRQ_GET(SRC_DMACH0),
      .rx_irq = TRICORE_IRQ_GET(SRC_DMACH1),
      .err_irq = TRICORE_IRQ_GET(SRC_QSPI4ERR),
      .pt_irq = TRICORE_IRQ_GET(SRC_QSPI4PT),
      .pt_enable = true,
      .usedma = true,
      .txdmachannel = IfxDma_ChannelId_0,
      .rxdmachannel = IfxDma_ChannelId_1,
      .maxbaudrate = 10000000,
      .bufsize = 512,
    },
  #endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct aurix_uart_config_s g_aurix_uart_config[] =
{
#if defined(CONFIG_AURIX_UART0)
  [0] =
    {
      .base = &MODULE_ASCLIN0,
      .pins =
      {
        .cts       = NULL,
        .ctsMode   = IfxPort_InputMode_pullUp,
        .rx        = &IfxAsclin0_RXA_P14_1_IN,
        .rxMode    = IfxPort_InputMode_pullUp,
        .rts       = NULL,
        .rtsMode   = IfxPort_OutputMode_pushPull,
        .tx        = &IfxAsclin0_TX_P14_0_OUT,
        .txMode    = IfxPort_OutputMode_pushPull,
        .pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1
      },
      .baud = CONFIG_UART0_BAUD,
      .irq  = 21,
      .bus  = 0,
    },
#endif
};

#if defined(CONFIG_AURIX_EVADC)
static struct aurix_evadc_config_s g_aurix_evadc_config[] =
{
  #ifdef CONFIG_AURIX_EVADC0
    {
      .group_id = IfxEvadc_GroupId_0,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 412,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC1
    {
      .group_id = IfxEvadc_GroupId_1,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 416,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC2
    {
      .group_id = IfxEvadc_GroupId_2,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 420,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC3
    {
      .group_id = IfxEvadc_GroupId_3,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 424,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC4
    {
      .group_id = IfxEvadc_GroupId_4,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 428,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC5
    {
      .group_id = IfxEvadc_GroupId_5,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 432,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC6
    {
      .group_id = IfxEvadc_GroupId_6,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 436,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC7
    {
      .group_id = IfxEvadc_GroupId_7,
      .nchannels = 8,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 440,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC8
    {
      .group_id = IfxEvadc_GroupId_8,
      .nchannels = 16,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 444,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC9
    {
      .group_id = IfxEvadc_GroupId_9,
      .nchannels = 16,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 448,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC10
    {
      .group_id = IfxEvadc_GroupId_10,
      .nchannels = 16,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 452,
      #endif
    },
  #endif

  #ifdef CONFIG_AURIX_EVADC11
    {
      .group_id = IfxEvadc_GroupId_11,
      .nchannels = 16,
      #ifdef CONFIG_AURIX_HAVE_ADC_IRQ
      .irq = 456,
      #endif
  },
  #endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use
 *
 ****************************************************************************/

#ifdef CONFIG_GPIO_LOWER_HALF
static void aurix_gpio_initialize(void)
{
  int i;

  for (i = 0; i < nitems(g_pin_config); i++)
    {
      gpio_lower_half(g_ioe[g_pin_config[i].numOfIoe],
                      g_pin_config[i].pin,
                      g_pin_config[i].type,
                      g_pin_config[i].minor);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_earlyinitialize(void)
{
}

void board_lateinitialize_phaseA(void)
{
  int ret;

  UNUSED(ret);
}

void board_lateinitialize_phaseB(void)
{
  int ret;

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM0_TIMER)
  /* Initialize and register ATOM0 Timer Device */

  aurix_gtm_timer_initialize(g_atom0_timer,
                              g_aurix_gtm_atom0_timer_config,
                              nitems(g_aurix_gtm_atom0_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM1_TIMER)
  /* Initialize and register ATOM1 Timer Device */

  aurix_gtm_timer_initialize(g_atom1_timer,
                              g_aurix_gtm_atom1_timer_config,
                              nitems(g_aurix_gtm_atom1_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM2_TIMER)
  /* Initialize and register ATOM2 Timer Device */

  aurix_gtm_timer_initialize(g_atom2_timer,
                              g_aurix_gtm_atom2_timer_config,
                              nitems(g_aurix_gtm_atom2_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM3_TIMER)
  /* Initialize and register ATOM3 Timer Device */

  aurix_gtm_timer_initialize(g_atom3_timer,
                              g_aurix_gtm_atom3_timer_config,
                              nitems(g_aurix_gtm_atom3_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM4_TIMER)
  /* Initialize and register ATOM4 Timer Device */

  aurix_gtm_timer_initialize(g_atom4_timer,
                              g_aurix_gtm_atom4_timer_config,
                              nitems(g_aurix_gtm_atom4_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM5_TIMER)
  /* Initialize and register ATOM5 Timer Device */

  aurix_gtm_timer_initialize(g_atom5_timer,
                              g_aurix_gtm_atom5_timer_config,
                              nitems(g_aurix_gtm_atom5_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM6_TIMER)
  /* Initialize and register ATOM6 Timer Device */

  aurix_gtm_timer_initialize(g_atom6_timer,
                              g_aurix_gtm_atom6_timer_config,
                              nitems(g_aurix_gtm_atom6_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM7_TIMER)
  /* Initialize and register ATOM7 Timer Device */

  aurix_gtm_timer_initialize(g_atom7_timer,
                              g_aurix_gtm_atom7_timer_config,
                              nitems(g_aurix_gtm_atom7_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM8_TIMER)
  /* Initialize and register ATOM8 Timer Device */

  aurix_gtm_timer_initialize(g_atom8_timer,
                              g_aurix_gtm_atom8_timer_config,
                              nitems(g_aurix_gtm_atom8_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM9_TIMER)
  /* Initialize and register ATOM9 Timer Device */

  aurix_gtm_timer_initialize(g_atom9_timer,
                              g_aurix_gtm_atom9_timer_config,
                              nitems(g_aurix_gtm_atom9_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM10_TIMER)
  /* Initialize and register ATOM10 Timer Device */

  aurix_gtm_timer_initialize(g_atom10_timer,
                              g_aurix_gtm_atom10_timer_config,
                              nitems(g_aurix_gtm_atom10_timer_config));
#endif

#if defined(CONFIG_AURIX_GTM_ATOM_ATOM11_TIMER)
  /* Initialize and register ATOM11 Timer Device */

  aurix_gtm_timer_initialize(g_atom11_timer,
                              g_aurix_gtm_atom11_timer_config,
                              nitems(g_aurix_gtm_atom11_timer_config));
#endif

#ifdef CONFIG_AURIX_PWM
#ifdef CONFIG_AURIX_GTM_TOM_PWM
    aurix_gtm_tom_pwm_cfg_initialize(g_aurix_pwm_channel_cfgs,
                                    g_pwm_channels,
                                    nitems(g_aurix_pwm_channel_cfgs));
#endif /* CONFIG_AURIX_GTM_TOM_PWM */

#ifdef CONFIG_AURIX_GTM_ATOM_PWM
    aurix_gtm_atom_pwm_cfg_initialize(g_aurix_pwm_channel_cfgs,
                                    g_pwm_channels,
                                    nitems(g_aurix_pwm_channel_cfgs));
#endif /* CONFIG_AURIX_GTM_ATOM_PWM */
#endif /* CONFIG_AURIX_PWM */

#if defined(CONFIG_AURIX_CAPTURE)

  /* Initialize and register GTM TIM0 capture devices. */

  aurix_gtm_capture_all_initialize(g_capture_channels,
      g_aurix_capture_channel_cfgs,
      nitems(g_aurix_capture_channel_cfgs));
#endif

#ifdef CONFIG_AURIX_QSPI
  aurix_qspi_initialize(g_qspi, g_aurix_qspi_config,
                        nitems(g_aurix_qspi_config));
#endif

#ifdef CONFIG_AURIX_QSPI_SLAVE
  aurix_qspi_slave_initialize(g_qspislave, g_aurix_qspislave_config,
                              nitems(g_aurix_qspislave_config));
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#if defined(CONFIG_GPIO_LOWER_HALF)

  /* Initialize and register the GPIO driver */

  aurix_gpio_initialize();
#endif /* CONFIG_GPIO_LOWER_HALF */

#ifdef CONFIG_AURIX_EVADC
  aurix_adc_initialize(g_evadc, g_aurix_evadc_config,
                       nitems(g_aurix_evadc_config));
#endif

  UNUSED(ret);
}

void board_lateinitialize(void)
{
    int cpuid = sched_getcpu();

#ifdef CONFIG_AUTOCORE_SYNBARRIER
    autocore_sync_barrier_wait(cpuid);
#endif
    board_lateinitialize_phaseA();
#ifdef CONFIG_AUTOCORE_SYNBARRIER
    autocore_sync_barrier_wait(cpuid);
#endif
    board_lateinitialize_phaseB();

    UNUSED(cpuid);
}

void board_finalinitialize(void)
{
}

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the user-space heap.
 *
 ****************************************************************************/

void up_allocate_heap(void** heap_start, size_t* heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

    /* Get the unaligned size and position of the user-space heap.
     * This heap begins after the user-space .bss section.
     */

    uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend;
    uintptr_t udlmu_start = GENERATE_CORE_DLMU_USER_START(CONFIG_CPU_COREID);
    uintptr_t udlmu_size = GENERATE_CORE_DLMU_USER_SIZE(CONFIG_CPU_COREID);

    /* Return the user-space heap settings */

    DEBUGASSERT(udlmu_start + udlmu_size > ubase);

    *heap_start = (void*)ubase;
    *heap_size = udlmu_start + udlmu_size - ubase;
#else

    /* Return the heap settings */

    *heap_start = _sheap;
    *heap_size = (uintptr_t)_eheap - (uintptr_t)_sheap;
#endif
}