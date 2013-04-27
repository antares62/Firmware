/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
/**
 * @file PX4IO hardware definitions.
 */

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_internal.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/
/******************************************************************************
 * GPIOS
 ******************************************************************************/

#define BOARD_GPIO_OUTPUT(pin) (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                                GPIO_OUTPUT_CLEAR|(pin))
#define BOARD_GPIO_INPUT_FLOAT(pin)  (GPIO_INPUT|GPIO_CNF_INFLOAT|\
                                      GPIO_MODE_INPUT|(pin))
#define BOARD_GPIO_INPUT_PUP(pin) (GPIO_INPUT|GPIO_CNF_INPULLUP|\
                                   GPIO_MODE_INPUT|(pin))
#define BOARD_GPIO_INPUT_ANALOG(pin) (GPIO_INPUT|GPIO_CNF_ANALOGIN|\
                                   GPIO_MODE_INPUT|(pin))


/* LEDs *********************************************************************/

#define BOARD_GPIO_LED1 (GPIO_PORTB|GPIO_PIN14)
#define BOARD_GPIO_LED2 (GPIO_PORTB|GPIO_PIN15)
#define BOARD_GPIO_LED3 (GPIO_PORTB|GPIO_PIN10)

#define BOARD_GPIO_LED_BLUE   BOARD_GPIO_LED1
#define BOARD_GPIO_LED_AMBER  BOARD_GPIO_LED2
#define BOARD_GPIO_LED_SAFETY BOARD_GPIO_LED3

/* Safety switch button *************************************************************/

// float
#define BOARD_GPIO_BTN_SAFETY (GPIO_PORTB|GPIO_PIN5)

/* Power switch controls ************************************************************/

#define BOARD_GPIO_ACC1_PWR_EN  (GPIO_PORTC|GPIO_PIN13)
#define BOARD_GPIO_ACC2_PWR_EN  (GPIO_PORTC|GPIO_PIN14)
#define BOARD_GPIO_SERVO_PWR_EN (GPIO_PORTC|GPIO_PIN15)

// pullup
#define BOARD_GPIO_ACC_OC_DETECT   (GPIO_PORTB|GPIO_PIN12)
#define BOARD_GPIO_SERVO_OC_DETECT (GPIO_PORTB|GPIO_PIN13)

#define BOARD_GPIO_RELAY1_EN (GPIO_PORTA|GPIO_PIN12)
#define BOARD_GPIO_RELAY2_EN (GPIO_PORTA|GPIO_PIN11)

/* Analog inputs ********************************************************************/

#define BOARD_GPIO_ADC_VBATT (GPIO_PORTA|GPIO_PIN4)
#define BOARD_GPIO_ADC_IN5	 (GPIO_PORTA|GPIO_PIN5)

