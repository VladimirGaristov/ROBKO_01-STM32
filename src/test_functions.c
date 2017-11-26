/*
 * test_functions.c
 *
 *  Created on: Oct 11, 2017
 *      Author: cartogan
 */

#include "main.h"

void LED_Blink()
{
	//Unusable after SB21 was opened
	LL_GPIO_TogglePin(LED_PORT, TEST_LED_PIN);
	/* Insert delay 250 ms */
	LL_mDelay(250);
}

void LED_wave()
{
	int active_led=0x00000100;	//D0_Pin
	while(1)
	{
		LL_GPIO_WriteOutputPort(ADDR_DATA_PORT, active_led);
		active_led=active_led << 1;
		if(active_led>0x00008000)	//D7_Pin
			active_led=0x00000100;	//D0_Pin
		LL_mDelay(250);
	}
}

int32_t send_string(const char *msg)
{
	if(!LL_USART_IsActiveFlag_TC(USART1))
		return -1;
	uint16_t l, i=0;
	l=strlen(msg);
	while(i<l)
	{
		LL_USART_TransmitData9(USART1, msg[i]);
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		i++;
	}
	return 0;
}

int32_t receive_string(char *buffer, uint32_t buff_len)
{
	if(!LL_USART_IsActiveFlag_RXNE(USART1))
		return -1;
	uint16_t i=0;
	while(i<buff_len)
	{
		while((!LL_USART_IsActiveFlag_RXNE(USART1)));	//add timeout
		buffer[i]=LL_USART_ReceiveData9(USART1);
		if(buffer[i]=='\0')
			break;
		i++;
	}
	return i;
}

void serial_test()
{
	char input_buffer[SERIAL_BUFFER_LEN];
	input_buffer[0]='\0';
	int8_t lastbit;
	while(!LL_USART_IsActiveFlag_RXNE(USART1));
	lastbit=receive_string(input_buffer, SERIAL_BUFFER_LEN);
	if(!(lastbit<=0))
	{
		input_buffer[lastbit]='\0';

		if(!strcmp(input_buffer, "on"))
		{
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
			send_string("Brightening up the world!\n");
		}
		else if(!strcmp(input_buffer, "off"))
		{
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
			send_string("Getting dark...\n");
		}
		else
		{
			send_string("What did you say?\n");
		}
		while(!LL_USART_IsActiveFlag_TC(USART1));
	}
}

void motor_test()
{
	static int8_t m=2;
	step_motor(m, STEP_REV);
	//m++;
	if(m>5)
		m=0;
	LL_mDelay(STEP_TIME);
}
