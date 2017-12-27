/*
 * motor_control.c
 *
 *  Created on: Nov 1, 2017
 *      Author: cartogan
 */

#include "main.h"

//Stores pending commands, used to implement a linked list
typedef struct cmd_buffer_t
{
	uint8_t cmd[4];
	struct cmd_buffer_t *next_cmd;
}
cmd_buffer_t;

cmd_buffer_t *current_cmd=NULL, *last_cmd;

//Describes the sequence for driving the 4 coils in half-step mode of 2-phase unipolar stepper motor
//For full-step mode only the every even numbered state is skipped
const uint8_t coil_current[4][8]={{1, 1, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 1, 1, 1, 0, 0},
								  {0, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 1, 1}};
//Stores how many steps have the motors moved
int32_t motor_pos[6]={1, 1, 1, 1, 1, 1};
//Stores the direction of the last steps. Used to determine which status LEDs to illuminate
int8_t motor_status[6]={0};
//This is needed for looping
const uint32_t LED_pins[12]={LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN,
							 LED6_PIN, LED7_PIN, LED8_PIN, LED9_PIN, LED10_PIN, LED11_PIN};

//Translates an address of ROBKO register to states of the address pins
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

//Moves a motor one step in a specified direction
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
	//Pass the direction of movement to set_LEDs()
	motor_status[motor]=dir;
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

//Determines the mode of operation (manual/remote), sets the STEP_SIZE and STEP_TIME
//and calls remote_control() or manual_control()
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
	//Wait for the motors to finish moving
	LL_mDelay(STEP_TIME);
}

//
void manual_control(void)
{

}

//Executes a stored command
int32_t remote_control(void)
{
	//n is a flag, raised if the current command has been completed and can be deleted
	int32_t n=0;
	cmd_buffer_t *old;
	//Check if any commands are pending
	if(current_cmd==NULL || LL_GPIO_IsOutputPinSet(ADDR_DATA_PORT, ENABLE_PIN)==0)
		return -1;
	//Decode command
	switch(current_cmd->cmd[0])
	{
		case MOV_FWD:
			n=step_motor(current_cmd->cmd[1], STEP_FWD);
			//Decrement number of steps
			current_cmd->cmd[2]--;
			//Check if command has been finished
			if(current_cmd->cmd[2]==0)
				n=1;
			break;
		case MOV_REV:
			n=step_motor(current_cmd->cmd[1], STEP_REV);
			//Decrement number of steps
			current_cmd->cmd[2]--;
			//Check if command has been finished
			if(current_cmd->cmd[2]==0)
				n=1;
			break;
		case OFF:
			stop_motor(current_cmd->cmd[1]);
			n=1;
			break;
		case FREEZE:
			DISABLE_ROBKO();
			n=1;
			break;
		//case GOTO_POS:
		//case TOGETHER:
		//Unknown command
		default: n=1;
	}
	if(n)
	{
		//Advance the current command to the next and delete the old one
		old=current_cmd;
		current_cmd=current_cmd->next_cmd;
		free(old);
	}
	return 0;
}

//Turns off all coils in a motor, allowing to move it by hand
int32_t stop_motor(uint32_t motor)
{
	uint32_t n;
	if(motor<6)
	{
		set_addr(motor);
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
		//Send the data to ROBKO 01
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		DWT_Delay(10);
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		return 0;
	}
	else if(motor==ALL_MOTORS)
	{
		//Recursion
		for(n=0;n<6;n++)
			stop_motor(n);
		return 0;
	}
	else
	{
		return -1;
	}
}

//Called by interrupt when a byte of data has been received by USART
void read_cmd(void)
{
	static uint8_t rem_bytes=0, current_byte=0;
	uint8_t read_buffer;
	cmd_buffer_t *eraser;
	//Read a byte of data
	read_buffer=LL_USART_ReceiveData9(USART1);
	//Check if there is a command not fully received
	if(rem_bytes>0)
	{
		//Add the new byte to the unfinished command
		last_cmd->cmd[current_byte]=read_buffer;
		current_byte++;
		rem_bytes--;
	}
	else
	{
		//Start of new command
		current_byte=1;
		rem_bytes=0;
		//KILL, RESUME and CLEAR commands are not added to the queue but executed with priority
		switch(read_buffer)
		{
			case KILL:
				//Emergency stop
				stop_motor(6);
				DISABLE_ROBKO();
				break;
			case RESUME:
				ENABLE_ROBKO();
				break;
			case CLEAR:
				//Clear the command queue
				do
				{
					eraser=current_cmd;
					current_cmd=current_cmd->next_cmd;
					free(eraser);
				}
				while(current_cmd!=NULL);
				break;
			case GOTO_POS: rem_bytes+=1;
			/* no break */
			case MOV_FWD: case MOV_REV: rem_bytes+=1;
			/* no break */
			case TOGETHER: case OFF: rem_bytes+=1;
				//Allocate memory for the new command and add it to the linked list
				//Check if the queue is empty
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
				//Check if memory was allocated successfully
				if(last_cmd!=NULL)
				{
					//Zero-out the new block of memory
					memset(last_cmd, 0, sizeof(cmd_buffer_t));
					//Write the byte of data that was read
					last_cmd->cmd[0]=read_buffer;
				}
				break;
			default:;
		}
	}
}

//Illuminate the status LEDs corresponding to the direction of movement of the motors
void set_LEDs(void)
{
	uint8_t i;
	//Turn off all LEDs
	for(i=0;i<12;i++)
		LL_GPIO_ResetOutputPin(LED_PORT, LED_pins[i]);
	//Turn on the LEDs corresponding to the last steps made
	for(i=0;i<6;i++)
	{
		if(motor_status[i]==STEP_FWD)
			LL_GPIO_SetOutputPin(LED_PORT, LED_pins[i*2]);
		else if(motor_status[i]==STEP_REV)
			LL_GPIO_SetOutputPin(LED_PORT, LED_pins[i*2+1]);
	}
	//Clear the motor_status array so the step_motor() function can safely set it
	memset(motor_status, 0, sizeof(motor_status));
}
