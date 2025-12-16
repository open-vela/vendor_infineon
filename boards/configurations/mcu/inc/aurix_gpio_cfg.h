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

#ifndef ARUIX_GPIO_CFG_H
#define ARUIX_GPIO_CFG_H

#include <nuttx/ioexpander/gpio.h>

#define _PIN_SHIFT (0) /* Bits 0-4: pin number */
#define _PIN_MASK (31 << _PIN_SHIFT)

#define _PORT_SHIFT (5) /* Bits 5-11: port number */
#define _PORT_MASK (63 << _PORT_SHIFT)

#define _TYPE_SHIFT (12) /* Bits 12-16: type number */
#define _TYPE_MASK (31 << _TYPE_SHIFT)

#define _SPEED_SHIFT (17) /* Bits 17-20: sPEED number */
#define _SPEED_MASK (15 << _SPEED_SHIFT)

#define _INITVALUE_SHIFT (21) /* Bits 21: iNITVALUE number */
#define _INITVALUE_MASK (1 << _INITVALUE_SHIFT)

#define _OUTPUTMODE_SHIFT (22) /* Bits 22: oUTPUTMODE number */
#define _OUTPUTMODE_MASK (1 << _OUTPUTMODE_SHIFT)

#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
#define _PORT00 0x00
#define _PORT01 0x01
#define _PORT02 0x02
#define _PORT03 0x03
#define _PORT04 0x04
#define _PORT10 0x05
#define _PORT13 0x06
#define _PORT14 0x07
#define _PORT15 0x08
#define _PORT16 0x09
#define _PORT20 0x0A
#define _PORT21 0x0B
#define _PORT22 0x0C
#define _PORT23 0x0D
#define _PORT25 0x0E
#define _PORT30 0x0F
#define _PORT31 0x10
#define _PORT32 0x11
#define _PORT33 0x12
#define _PORT34 0x13
#define _PORT35 0x14
#define _PORT40 0x15
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
#define _PORT00 0x00
#define _PORT01 0x01
#define _PORT02 0x02
#define _PORT03 0x03
#define _PORT10 0x04
#define _PORT11 0x05
#define _PORT12 0x06
#define _PORT13 0x07
#define _PORT14 0x08
#define _PORT15 0x09
#define _PORT16 0x0A
#define _PORT20 0x0B
#define _PORT21 0x0C
#define _PORT22 0x0D
#define _PORT23 0x0E
#define _PORT25 0x0F
#define _PORT30 0x10
#define _PORT31 0x11
#define _PORT32 0x12
#define _PORT33 0x13
#define _PORT34 0x14
#define _PORT35 0x15
#define _PORT40 0x16
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC39X)
#define _PORT00 0x00
#define _PORT01 0x01
#define _PORT02 0x02
#define _PORT10 0x03
#define _PORT11 0x04
#define _PORT12 0x05
#define _PORT13 0x06
#define _PORT14 0x07
#define _PORT15 0x08
#define _PORT20 0x09
#define _PORT21 0x0A
#define _PORT22 0x0B
#define _PORT23 0x0C
#define _PORT24 0x0D
#define _PORT25 0x0E
#define _PORT26 0x0F
#define _PORT30 0x10
#define _PORT31 0x11
#define _PORT32 0x12
#define _PORT33 0x13
#define _PORT34 0x14
#define _PORT40 0x15
#define _PORT41 0x16
#endif


#define _PIN0 0x00
#define _PIN1 0x01
#define _PIN2 0x02
#define _PIN3 0x03
#define _PIN4 0x04
#define _PIN5 0x05
#define _PIN6 0x06
#define _PIN7 0x07
#define _PIN8 0x08
#define _PIN9 0x09
#define _PIN10 0x0A
#define _PIN11 0x0B
#define _PIN12 0x0C
#define _PIN13 0x0D
#define _PIN14 0x0E
#define _PIN15 0x0F
#define _PIN16 0x10
#define _PIN17 0x11
#define _PIN18 0x12
#define _PIN19 0x13
#define _PIN20 0x14
#define _PIN21 0x15
#define _PIN22 0x16
#define _PIN23 0x17
#define _PIN24 0x18
#define _PIN25 0x19
#define _PIN26 0x1A
#define _PIN27 0x1B
#define _PIN28 0x1C
#define _PIN29 0x1D
#define _PIN30 0x1E
#define _PIN31 0x1F


#if defined(CONFIG_ARCH_CHIP_AURIX_TC4DX)
#define PORT_00 (_PORT00 << _PORT_SHIFT)
#define PORT_01 (_PORT01 << _PORT_SHIFT)
#define PORT_02 (_PORT02 << _PORT_SHIFT)
#define PORT_03 (_PORT03 << _PORT_SHIFT)
#define PORT_04 (_PORT04 << _PORT_SHIFT)
#define PORT_10 (_PORT10 << _PORT_SHIFT)
#define PORT_13 (_PORT13 << _PORT_SHIFT)
#define PORT_14 (_PORT14 << _PORT_SHIFT)
#define PORT_15 (_PORT15 << _PORT_SHIFT)
#define PORT_16 (_PORT16 << _PORT_SHIFT)
#define PORT_20 (_PORT20 << _PORT_SHIFT)
#define PORT_21 (_PORT21 << _PORT_SHIFT)
#define PORT_22 (_PORT22 << _PORT_SHIFT)
#define PORT_23 (_PORT23 << _PORT_SHIFT)
#define PORT_25 (_PORT25 << _PORT_SHIFT)
#define PORT_30 (_PORT30 << _PORT_SHIFT)
#define PORT_31 (_PORT31 << _PORT_SHIFT)
#define PORT_32 (_PORT32 << _PORT_SHIFT)
#define PORT_33 (_PORT33 << _PORT_SHIFT)
#define PORT_34 (_PORT34 << _PORT_SHIFT)
#define PORT_35 (_PORT35 << _PORT_SHIFT)
#define PORT_40 (_PORT40 << _PORT_SHIFT)
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC48X)
#define PORT_00 (_PORT00 << _PORT_SHIFT)
#define PORT_01 (_PORT01 << _PORT_SHIFT)
#define PORT_02 (_PORT02 << _PORT_SHIFT)
#define PORT_03 (_PORT03 << _PORT_SHIFT)
#define PORT_10 (_PORT10 << _PORT_SHIFT)
#define PORT_11 (_PORT11 << _PORT_SHIFT)
#define PORT_12 (_PORT12 << _PORT_SHIFT)
#define PORT_13 (_PORT13 << _PORT_SHIFT)
#define PORT_14 (_PORT14 << _PORT_SHIFT)
#define PORT_15 (_PORT15 << _PORT_SHIFT)
#define PORT_16 (_PORT16 << _PORT_SHIFT)
#define PORT_20 (_PORT20 << _PORT_SHIFT)
#define PORT_21 (_PORT21 << _PORT_SHIFT)
#define PORT_22 (_PORT22 << _PORT_SHIFT)
#define PORT_23 (_PORT23 << _PORT_SHIFT)
#define PORT_25 (_PORT25 << _PORT_SHIFT)
#define PORT_30 (_PORT30 << _PORT_SHIFT)
#define PORT_31 (_PORT31 << _PORT_SHIFT)
#define PORT_32 (_PORT32 << _PORT_SHIFT)
#define PORT_33 (_PORT33 << _PORT_SHIFT)
#define PORT_34 (_PORT34 << _PORT_SHIFT)
#define PORT_35 (_PORT35 << _PORT_SHIFT)
#define PORT_40 (_PORT40 << _PORT_SHIFT)
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC39X)
#define PORT_00 (_PORT00 << _PORT_SHIFT)
#define PORT_01 (_PORT01 << _PORT_SHIFT)
#define PORT_02 (_PORT02 << _PORT_SHIFT)
#define PORT_10 (_PORT10 << _PORT_SHIFT)
#define PORT_11 (_PORT11 << _PORT_SHIFT)
#define PORT_12 (_PORT12 << _PORT_SHIFT)
#define PORT_13 (_PORT13 << _PORT_SHIFT)
#define PORT_14 (_PORT14 << _PORT_SHIFT)
#define PORT_15 (_PORT15 << _PORT_SHIFT)
#define PORT_20 (_PORT20 << _PORT_SHIFT)
#define PORT_21 (_PORT21 << _PORT_SHIFT)
#define PORT_22 (_PORT22 << _PORT_SHIFT)
#define PORT_23 (_PORT23 << _PORT_SHIFT)
#define PORT_24 (_PORT24 << _PORT_SHIFT)
#define PORT_25 (_PORT25 << _PORT_SHIFT)
#define PORT_26 (_PORT26 << _PORT_SHIFT)
#define PORT_30 (_PORT30 << _PORT_SHIFT)
#define PORT_31 (_PORT31 << _PORT_SHIFT)
#define PORT_32 (_PORT32 << _PORT_SHIFT)
#define PORT_33 (_PORT33 << _PORT_SHIFT)
#define PORT_34 (_PORT34 << _PORT_SHIFT)
#define PORT_40 (_PORT40 << _PORT_SHIFT)
#define PORT_41 (_PORT41 << _PORT_SHIFT)
#endif

#define PIN_0 (_PIN0 << _PIN_SHIFT)
#define PIN_1 (_PIN1 << _PIN_SHIFT)
#define PIN_2 (_PIN2 << _PIN_SHIFT)
#define PIN_3 (_PIN3 << _PIN_SHIFT)
#define PIN_4 (_PIN4 << _PIN_SHIFT)
#define PIN_5 (_PIN5 << _PIN_SHIFT)
#define PIN_6 (_PIN6 << _PIN_SHIFT)
#define PIN_7 (_PIN7 << _PIN_SHIFT)
#define PIN_8 (_PIN8 << _PIN_SHIFT)
#define PIN_9 (_PIN9 << _PIN_SHIFT)
#define PIN_10 (_PIN10 << _PIN_SHIFT)
#define PIN_11 (_PIN11 << _PIN_SHIFT)
#define PIN_12 (_PIN12 << _PIN_SHIFT)
#define PIN_13 (_PIN13 << _PIN_SHIFT)
#define PIN_14 (_PIN14 << _PIN_SHIFT)
#define PIN_15 (_PIN15 << _PIN_SHIFT)
#define PIN_16 (_PIN16 << _PIN_SHIFT)
#define PIN_17 (_PIN17 << _PIN_SHIFT)
#define PIN_18 (_PIN18 << _PIN_SHIFT)
#define PIN_19 (_PIN19 << _PIN_SHIFT)
#define PIN_20 (_PIN20 << _PIN_SHIFT)
#define PIN_21 (_PIN21 << _PIN_SHIFT)
#define PIN_22 (_PIN22 << _PIN_SHIFT)
#define PIN_23 (_PIN23 << _PIN_SHIFT)
#define PIN_24 (_PIN24 << _PIN_SHIFT)
#define PIN_25 (_PIN25 << _PIN_SHIFT)
#define PIN_26 (_PIN26 << _PIN_SHIFT)
#define PIN_27 (_PIN27 << _PIN_SHIFT)
#define PIN_28 (_PIN28 << _PIN_SHIFT)
#define PIN_29 (_PIN29 << _PIN_SHIFT)
#define PIN_30 (_PIN30 << _PIN_SHIFT)
#define PIN_31 (_PIN31 << _PIN_SHIFT)

#ifdef CONFIG_GPIO_LOWER_HALF
struct tc4d9_pin_config_s
{
  uint8_t                    numOfIoe;
  uint8_t                    pin;
  enum gpio_pintype_e        type;
  bool                       initValue;
  int                        minor;
};

extern const struct tc4d9_pin_config_s g_pin_config[];
#define AURIX_GPIO_COUNTS 0
#if (defined (CONFIG_CPU_COREID) && (defined AURIX_GPIO_COUNTS) && (CONFIG_CPU_COREID == 1))
#undef AURIX_GPIO_COUNTS
#define AURIX_GPIO_COUNTS 4
#endif

#endif //CONFIG_GPIO_LOWER_HALF

#endif //ARUIX_GPIO_CFG_H
