/*
 * test_functions.c
 *
 *  Created on: Oct 11, 2017
 *      Author: Vladimir Garistov
 */

#include "main.h"

void LED_Blink(void)
{
	//Unusable after SB21 was opened
	LL_GPIO_TogglePin(LED_PORT, TEST_LED_PIN);
	/* Insert delay 250 ms */
	LL_mDelay(250);
}

void LED_wave(void)
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

//Send a string through USART
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

//Receive a string from USART
//Blocks with no timeout!
int32_t receive_string(char *buffer, uint32_t buff_len)
{
	if(!LL_USART_IsActiveFlag_RXNE(USART1))
		return -1;
	uint16_t i=0;
	while(i<buff_len)
	{
		while((!LL_USART_IsActiveFlag_RXNE(USART1)));
		buffer[i]=LL_USART_ReceiveData9(USART1);
		if(buffer[i]=='\0')
			break;
		i++;
	}
	return i;
}

//Toggles the test LED based on USART commands and replies accordingly
void serial_test(void)
{
	char input_buffer[SERIAL_BUFFER_LEN];
	input_buffer[0]='\0';
	int32_t lastbit;
	while(!LL_USART_IsActiveFlag_RXNE(USART1));
	lastbit=receive_string(input_buffer, SERIAL_BUFFER_LEN);
	if(!(lastbit<=0))
	{
		input_buffer[lastbit]='\0';		//is this necessary?
		if(!strcmp(input_buffer, "on"))
		{
			LL_GPIO_SetOutputPin(LED_PORT, TEST_LED_PIN);
			send_string("Brightening up the world!\n");
		}
		else if(!strcmp(input_buffer, "off"))
		{
			LL_GPIO_ResetOutputPin(LED_PORT, TEST_LED_PIN);
			send_string("Getting dark...\n");
		}
		else
		{
			send_string("What did you say?\n");
		}
		while(!LL_USART_IsActiveFlag_TC(USART1));
	}
}

//Checks the condition of the motors
void motor_test(void)
{
	static int8_t m=2;
	extern uint16_t local_step_time;
	step_motor(m, STEP_FWD);
	m++;
	if(m>5)
		m=0;
	LL_mDelay(local_step_time);
}

extern uint32_t SystemCoreClock;

//Initialize the modules used by DWT_Delay()
void DWT_Init(void)
{
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
	{
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

//Provides delay in microseconds
void DWT_Delay(uint32_t us)
{
	int32_t tp = DWT->CYCCNT + us * (SystemCoreClock/1000000);
	while (((int32_t)DWT->CYCCNT - tp) < 0);
}

inline int heap_overflow(void *new_alloc, size_t size)
{
	if (new_alloc + size > (void *) HEAP_LIMIT)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 * Sends reply_len number of bytes from memory, pointed to by reply through UART
 * Each call to the function sends only one byte. Call it again as send_reply(NULL, 0) to send the next byte.
 */
int send_reply(uint8_t *reply, uint8_t reply_len)
{
	static uint8_t reply_buffer[MAX_REPLY_LEN] = {0};
	static uint8_t reply_lenght = 0;
	static uint8_t current_byte = 0;
	uint8_t i;
	if (reply != NULL)
	{
		if (reply_len == 0 || reply_len > MAX_REPLY_LEN)
		{
			return -2;
		}
		if (current_byte < reply_lenght)
		{
			return -1;
		}
		reply_lenght = reply_len;
		current_byte = 0;
		// copy data to buffer to avoid corruption
		for (i = 0; i < reply_len; i++)
		{
			reply_buffer[i] = reply[i];
		}
		if (LL_USART_IsActiveFlag_TXE(USART1))
		{
			LL_USART_TransmitData9(USART1, reply_buffer[current_byte]);
			current_byte++;
		}
		return 0;
	}
	else
	{
		if (current_byte < reply_lenght)
		{
			if (LL_USART_IsActiveFlag_TXE(USART1))
			{
				LL_USART_TransmitData9(USART1, reply_buffer[current_byte]);
				current_byte++;
			}
			return 0;
		}
		else
		{
			return 0;
		}
	}
}
