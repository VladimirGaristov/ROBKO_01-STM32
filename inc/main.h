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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CLAW_RELEASE_BUT_PIN LL_GPIO_PIN_13
#define CLAW_GRAB_BUT_PIN LL_GPIO_PIN_12
#define LEFT_RIGHT_POT_PIN LL_GPIO_PIN_1
#define SHOULDER_POT_PIN LL_GPIO_PIN_0
#define ELBOW_POT_PIN LL_GPIO_PIN_3
#define CLAW_ROTATION_POT_PIN LL_GPIO_PIN_2
#define CLAW_UP_BUT_PIN LL_GPIO_PIN_10
#define CLAW_DOWN_BUT_PIN LL_GPIO_PIN_11
#define AUTO_MODE_PIN LL_GPIO_PIN_9
#define MANUAL_MODE_PIN LL_GPIO_PIN_8
#define JOYSTICK_CONNECTED_PIN LL_GPIO_PIN_7
#define STEP_SIZE_PIN LL_GPIO_PIN_14
#define POT_MODE_PIN LL_GPIO_PIN_4

#define IOW_PIN LL_GPIO_PIN_1
#define IOR_PIN LL_GPIO_PIN_2
#define D0_PIN LL_GPIO_PIN_9
#define D1_PIN LL_GPIO_PIN_8
#define D2_PIN LL_GPIO_PIN_11
#define D3_PIN LL_GPIO_PIN_10
#define D4_PIN LL_GPIO_PIN_12
#define D5_PIN LL_GPIO_PIN_13
#define D6_PIN LL_GPIO_PIN_14
#define D7_PIN LL_GPIO_PIN_15
#define A0_PIN LL_GPIO_PIN_3
#define A1_PIN LL_GPIO_PIN_4
#define A2_PIN LL_GPIO_PIN_5
#define ENABLE_PIN LL_GPIO_PIN_6

#define TEST_LED_PIN LL_GPIO_PIN_5
#define LED0_PIN LL_GPIO_PIN_13
#define LED1_PIN LL_GPIO_PIN_14
#define LED2_PIN LL_GPIO_PIN_15
#define LED3_PIN LL_GPIO_PIN_0
#define LED4_PIN LL_GPIO_PIN_1
#define LED5_PIN LL_GPIO_PIN_4
#define LED6_PIN LL_GPIO_PIN_5
#define LED7_PIN LL_GPIO_PIN_6
#define LED8_PIN LL_GPIO_PIN_7
#define LED9_PIN LL_GPIO_PIN_8
#define LED10_PIN LL_GPIO_PIN_2
#define LED11_PIN LL_GPIO_PIN_3

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

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1 */

/* USER CODE BEGIN Private defines */

#define ADDR_DATA_PORT GPIOB
#define INPUT_PORT GPIOC
#define LED_PORT GPIOA

#define SERIAL_BUFFER_LEN 100
#define FULL_STEP 2
#define HALF_STEP 1
#define STEP_SIZE FULL_STEP	//Da se chete ot DIP switch
#define STEP_TIME 300		//miliseconds
#define STEP_FWD 1
#define STEP_REV -1

//Stepper motors addresses and directions
#define ROTATION_MOTOR 0	//FWD=left
#define SHOULDER_MOTOR 1	//FWD=forward
#define ELBOW_MOTOR 2		//FWD=up
#define CLAW_GRAB_MOTOR 5	//FWD=open
#define CLAW_ROT_MOTOR_L 4	//FWD=up
#define CLAW_ROT_MOTOR_R 3	//FWD=down
#define ALL_MOTORS 6

//Commands for remote control
#define MOV_FWD 1
#define MOV_REV 2
#define OFF 3
#define OPEN_FILE 4
#define GOTO_POS 5
#define TOGETHER 6
#define KILL 7
#define CLEAR 8
#define FREEZE 9
#define RESUME 10

//Enabling and disabling ROBKO 01
#define ENABLE_ROBKO() LL_GPIO_SetOutputPin(ADDR_DATA_PORT, ENABLE_PIN)
#define DISABLE_ROBKO() LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, ENABLE_PIN)

/* USER CODE END Private defines */

//Exported function prototypes ----------------------------------------------

void _Error_Handler(char *, int);

void DWT_Init(void);
void DWT_Delay(uint32_t us);

void LED_wave(void);
void LED_Blink(void);
int32_t send_string(const char *msg);
int32_t receive_string(char *buffer, uint32_t buff_len);
void serial_test(void);
void motor_test(void);
int32_t set_addr(uint32_t addr);
int32_t step_motor(uint32_t motor, int32_t dir);
int32_t stop_motor(uint32_t motor);
void check_mode(void);
void manual_control(void);
int32_t remote_control(void);
void read_cmd(void);
void set_LEDs(void);

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
