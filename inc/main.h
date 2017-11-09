/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif
#ifndef STM32L476xx
#define STM32L476xx
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx.h"
#include "stm32l4xx_ll_gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CLAW_RELEASE_BUT_PIN LL_GPIO_PIN_13
#define CLAW_GRAB_BUT_PIN LL_GPIO_PIN_12
#define LEFT_RIGHT_POT_PIN LL_GPIO_PIN_0
#define SHOULDER_POT_PIN LL_GPIO_PIN_1
#define ELBOW_POT_PIN LL_GPIO_PIN_2
#define CLAW_ROTATION_POT_PIN LL_GPIO_PIN_3
#define CLAW_UP_BUT_PIN LL_GPIO_PIN_10
#define CLAW_DOWN_BUT_PIN LL_GPIO_PIN_11
#define AUTO_MODE_PIN LL_GPIO_PIN_9
#define MANUAL_MODE_PIN LL_GPIO_PIN_8
#define JOYSTICK_CONNECTED_PIN LL_GPIO_PIN_7

#define IOW_PIN LL_GPIO_PIN_1
#define IOR_PIN LL_GPIO_PIN_2
#define D0_PIN LL_GPIO_PIN_8
#define D1_PIN LL_GPIO_PIN_9
#define D2_PIN LL_GPIO_PIN_10
#define D3_PIN LL_GPIO_PIN_11
#define D4_PIN LL_GPIO_PIN_12
#define D5_PIN LL_GPIO_PIN_13
#define D6_PIN LL_GPIO_PIN_14
#define D7_PIN LL_GPIO_PIN_15
#define A0_PIN LL_GPIO_PIN_3
#define A1_PIN LL_GPIO_PIN_4
#define A2_PIN LL_GPIO_PIN_5
#define ENABLE_PIN LL_GPIO_PIN_6

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1 */

/* USER CODE BEGIN Private defines */

#define ADDR_DATA_PORT GPIOB
#define INPUT_PORT GPIOC

#define SERIAL_BUFFER_LEN 100
#define FULL_STEP 2
#define HALF_STEP 1
#define STEP_SIZE FULL_STEP
#define STEP_FWD 1
#define STEP_REV -1
#define ROTATION_MOTOR
#define SHOULDER_MOTOR
#define ELBOW_MOTOR
#define CLAW_GRAB_MOTOR
#define CLAW_ROT_MOTOR_L
#define CLAW_ROT_MOTOR_R

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

void LED_wave();
void LED_Blink();
int8_t send_string(const char *msg);
int8_t receive_string(char *buffer, uint16_t buff_len);
void serial_test();
void motor_test();
int8_t set_addr(uint8_t addr);
int8_t step_motor(uint8_t motor, int8_t dir);
void check_mode();
void manual_control();
void remote_control();

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
