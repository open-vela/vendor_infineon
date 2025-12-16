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

#ifndef __AURIX_GPIOIF_H__
#define __AURIX_GPIOIF_H__

#include <stdint.h>
#include "aurix_gpio_cfg.h"

extern void gpio_service_write(uint32_t pin_set, uint8_t value);
extern _Bool gpio_service_read(uint32_t pin_set);
/****************************************************************
prifex:
    GPIOIF:GPIO interface
****************************************************************/
#define GPIO_P33_0 (PORT_33 | PIN_0)
#define GPIO_P33_5 (PORT_33 | PIN_5)
#define GPIO_P00_5 (PORT_00 | PIN_5)
#define GPIO_P00_6 (PORT_00 | PIN_6)
#define GPIO_DI_LIN1_RX (IfxAsclin5_RXH_P35_4_IN.pin)
#define GPIO_DI_LIN2_RX (IfxAsclin4_RXF_P35_1_IN.pin)
#define GPIO_DI_LIN3_RX (IfxAsclin1_RXG_P02_3_IN.pin)
#define GPIO_DI_LIN4_RX (IfxAsclin26_RXD_P03_13_IN.pin)
#define GPIO_DI_LIN5_RX (IfxAsclin11_RXD_F_P21_1_IN.pin)
#define GPIO_DI_LIN6_RX (IfxAsclin18_RXB_F_P21_4_IN.pin)
#define GPIO_DI_LIN7_RX (IfxAsclin6_RXB_P01_0_IN.pin)

#define GPIOIF_MCU_DI_LIN_RX(x) ({                          \
    int result;                                             \
    switch (x) {                                            \
        case 1: \
        result = gpio_service_lin_read(GPIO_DI_LIN1_RX.port,GPIO_DI_LIN1_RX.pinIndex); \
        break; \
    case 2: \
        result = gpio_service_lin_read(GPIO_DI_LIN2_RX.port,GPIO_DI_LIN2_RX.pinIndex); \
        break; \
    case 3: \
        result = gpio_service_lin_read(GPIO_DI_LIN3_RX.port,GPIO_DI_LIN3_RX.pinIndex); \
        break; \
    case 4: \
        result = gpio_service_lin_read(GPIO_DI_LIN4_RX.port,GPIO_DI_LIN4_RX.pinIndex); \
        break; \
    case 5: \
        result = gpio_service_lin_read(GPIO_DI_LIN5_RX.port,GPIO_DI_LIN5_RX.pinIndex); \
        break; \
    case 6: \
        result = gpio_service_lin_read(GPIO_DI_LIN6_RX.port,GPIO_DI_LIN6_RX.pinIndex); \
        break; \
    case 7: \
        result = gpio_service_lin_read(GPIO_DI_LIN7_RX.port,GPIO_DI_LIN7_RX.pinIndex); \
        break; \                                                  \
    default:                                            \
        result = 0;                                     \
        break;                                          \
    }                                                       \
    result;                                                 \
})


#define GPIOIF_WRITE(pinset, value) gpio_service_write(pinset, value)
#define GPIOIF_READ(pinset) gpio_service_read(pinset)

#endif  /* __AURIX_GPIOIF_H__ */
