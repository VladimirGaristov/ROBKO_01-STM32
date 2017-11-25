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
	if(motor_pos[motor]<0)
		motor_pos[motor]=7;
	else if(motor_pos[motor]>7)
		motor_pos[motor]=-1+STEP_SIZE;
	//Select this motor's register
	set_addr(motor);
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
	//Set register bits to represent the active coils for the next step
	if(coil_current[0][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D0_PIN);
	if(coil_current[1][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D1_PIN);
	if(coil_current[2][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D2_PIN);
	if(coil_current[3][motor_pos[motor]]==1)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D3_PIN);
	//Send the data to ROBKO-01
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	LL_mDelay(1);
	LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
	return 0;
}

void check_mode()
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
}

void manual_control()
{

}

void remote_control()
{

}
