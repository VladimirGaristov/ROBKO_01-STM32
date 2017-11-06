/*
 * motor_control.c
 *
 *  Created on: Nov 1, 2017
 *      Author: cartogan
 */

#include "main.h"

const uint8_t coil_current[4][8]={{1, 1, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 1, 1, 1, 0, 0},
								  {0, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 1, 1}};
int8_t motor_pos[6]={1, 1, 1, 1, 1, 1};

int8_t set_addr(uint8_t addr)
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

int8_t step_motor(uint8_t motor, int8_t dir)
{
	if(motor>5)
		return -1;
	motor_pos[motor]+=dir*STEP_SIZE;
	//Overflow and underflow protection
	if(motor_pos[motor]<0)
		motor_pos[motor]=7;
	else if(motor_pos[motor]>7)
		motor_pos[motor]=-1+STEP_SIZE;
	set_addr(motor);
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
	if(coil_current[0][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D0_PIN);
	if(coil_current[1][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D1_PIN);
	if(coil_current[2][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D2_PIN);
	if(coil_current[3][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D3_PIN);
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	LL_mDelay(1);
	LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	return 0;
}
