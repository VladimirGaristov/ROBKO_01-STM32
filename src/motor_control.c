/*
 * motor_control.c
 *
 *  Created on: Nov 1, 2017
 *      Author: cartogan
 */

#include "main.h"

typedef struct cmd_buffer_t
{
	uint8_t cmd[4];
	struct cmd_buffer_t *next_cmd;
}
cmd_buffer_t;

cmd_buffer_t *current_cmd=NULL, *last_cmd;
const uint8_t coil_current[4][8]={{1, 1, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 1, 1, 1, 0, 0},
								  {0, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 1, 1}};
int32_t motor_pos[6]={1, 1, 1, 1, 1, 1};

int32_t set_addr(uint32_t addr)
{
	if(addr>7)
		return -1;
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, A0_PIN | A1_PIN | A2_PIN);
	if(addr>3)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A2_PIN);
		addr-=4;
	}
	if(addr>1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A1_PIN);
		addr-=2;
	}
	if(addr>0)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A0_PIN);
	return 0;
}

int32_t step_motor(uint32_t motor, int32_t dir)
{
	if(motor>5)
		return -1;
	//Increment/decrement motor position
	motor_pos[motor]+=dir*STEP_SIZE;
	//Overflow and underflow protection
	uint32_t new_pos=motor_pos[motor];
	new_pos%=8;
	if(new_pos<0)
		new_pos+=8;
	//Select this motor's register
	set_addr(motor);
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
	//Set register bits to represent the active coils for the next step
	if(coil_current[0][new_pos]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D0_PIN);
	if(coil_current[1][new_pos]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D1_PIN);
	if(coil_current[2][new_pos]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D2_PIN);
	if(coil_current[3][new_pos]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D3_PIN);
	//Send the data to ROBKO-01
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	DWT_Delay(10);
	LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	return 0;
}

void check_mode(void)
{
	uint32_t auto_mode, manual_mode, joystick;
	auto_mode=LL_GPIO_IsInputPinSet(INPUT_PORT, AUTO_MODE_PIN);
	manual_mode=LL_GPIO_IsInputPinSet(INPUT_PORT, MANUAL_MODE_PIN);
	if(!auto_mode)
		if(manual_mode)
			manual_control();
		else
			remote_control();
	else
	{
		joystick=LL_GPIO_IsInputPinSet(INPUT_PORT, JOYSTICK_CONNECTED_PIN);
		if(joystick)
			manual_control();
		else
			remote_control();
	}
	LL_mDelay(STEP_TIME);
}

void manual_control(void)
{

}

int32_t remote_control(void)
{
	int32_t n=0;
	cmd_buffer_t *old;
	if(current_cmd==NULL || LL_GPIO_IsOutputPinSet(ADDR_DATA_PORT, ENABLE_PIN)==0)
		return -1;
	switch(current_cmd->cmd[0])
	{
		case MOV_FWD:
			n=step_motor(current_cmd->cmd[1], STEP_FWD);
			current_cmd->cmd[2]--;
			if(current_cmd->cmd[2]==0)
				n=1;
			break;
		case MOV_REV:
			n=step_motor(current_cmd->cmd[1], STEP_REV);
			current_cmd->cmd[2]--;
			if(current_cmd->cmd[2]==0)
				n=1;
			break;
		case OFF:
			stop_motor(current_cmd->cmd[1]);
			n=1;
			break;
		case FREEZE:
			LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, ENABLE_PIN);
			n=1;
			break;
		//case GOTO_POS:
		//case TOGETHER:
		default: n=1;
	}
	if(n)
	{
		old=current_cmd;
		current_cmd=current_cmd->next_cmd;
		free(old);
	}
	return 0;
}

int32_t stop_motor(uint32_t motor)
{
	uint32_t n;
	if(motor<6)
	{
		set_addr(motor);
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		DWT_Delay(10);
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		return 0;
	}
	else if(motor==6)
	{
		for(n=0;n<6;n++)
			stop_motor(n);
		return 0;
	}
	else
	{
		return -1;
	}
}

void read_cmd(void)
{
	static uint8_t rem_bytes=0, current_byte=0;
	uint8_t read_buffer;
	cmd_buffer_t *eraser;
	read_buffer=LL_USART_ReceiveData9(USART1);
	if(rem_bytes>0)
	{
		last_cmd->cmd[current_byte]=read_buffer;
		current_byte++;
		rem_bytes--;
	}
	else
	{
		//Start of new command
		current_byte=1;
		rem_bytes=0;
		switch(read_buffer)
		{
			case KILL:
				stop_motor(6);
				LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, ENABLE_PIN);
				break;
			case RESUME:
				LL_GPIO_SetOutputPin(ADDR_DATA_PORT, ENABLE_PIN);
				break;
			case CLEAR:
				do
				{
					eraser=current_cmd;
					current_cmd=current_cmd->next_cmd;
					free(eraser);
				}
				while(current_cmd!=NULL);
				break;
			case TOGETHER: case OFF: rem_bytes+=1;
			/* no break */
			case MOV_FWD: case MOV_REV: rem_bytes+=1;
			/* no break */
			case GOTO_POS: rem_bytes+=1;
				if(current_cmd==NULL)
				{
					current_cmd=malloc(sizeof(cmd_buffer_t));
					last_cmd=current_cmd;
				}
				else
				{
					last_cmd->next_cmd=malloc(sizeof(cmd_buffer_t));
					last_cmd=last_cmd->next_cmd;
				}
				if(last_cmd!=NULL)
				{
					memset(last_cmd, 0, sizeof(cmd_buffer_t));
					last_cmd->cmd[0]=read_buffer;
				}
				break;
			default:;
		}
	}
}

void set_LEDs(void)
{

}
