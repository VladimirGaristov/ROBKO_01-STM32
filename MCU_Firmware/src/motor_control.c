/*
 * motor_control.c
 *
 *  Created on: Nov 1, 2017
 *      Author: Vladimir Garistov
 */

#include "main.h"

//Stores pending commands, used to implement a linked list
typedef struct cmd_buffer_t
{
	uint8_t cmd[13];
	uint8_t incomplete;
	struct cmd_buffer_t *next_cmd;
}
cmd_buffer_t;

cmd_buffer_t *current_cmd = NULL, *last_cmd = NULL;

//Describes the sequence for driving the 4 coils in half-step mode of 2-phase unipolar stepper motor
//For full-step mode only the every even numbered state is skipped
const uint8_t coil_current[4][8] = {{1, 1, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 1, 1, 1, 0, 0},
									{0, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 1, 1}};
//Stores how many steps have the motors moved
int16_t motor_pos[6] = {1, 1, 1, 1, 1, 1};
//Stores the direction of the last steps. Used to determine which status LEDs to illuminate
int8_t motor_status[6] = {0};
//Step size and speed parameters
uint8_t move_until = MOVE_FREELY, local_step_size = FULL_STEP, remote_step_size = FULL_STEP;
uint16_t local_step_time = SLOW_STEP, remote_step_time = USE_LOCAL_TIME;
//This is needed for looping
const uint32_t LED_pins[12] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN, LED5_PIN,
							   LED6_PIN, LED7_PIN, LED8_PIN, LED9_PIN, LED10_PIN, LED11_PIN};
//Stores the potentiometer values after ADC conversion
__IO uint16_t adc_pot_vals[4] = {POT_INIT_VAL};

//Translates an address of ROBKO register to states of the address pins
int32_t set_addr(uint8_t addr)
{
	if (addr > 7)
	{
		return -1;
	}
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, A0_PIN | A1_PIN | A2_PIN);
	if (addr > 3)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A2_PIN);
		addr -= 4;
	}
	if (addr > 1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A1_PIN);
		addr -= 2;
	}
	if (addr > 0)
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, A0_PIN);
	return 0;
}

//Moves a motor one step in a specified direction
int32_t step_motor(uint8_t motor, int8_t dir)
{
	if (motor > 5)
	{
		return -1;
	}
	//Increment/decrement motor position
	if (remote_step_size == USE_LOCAL_STEP)
	{
		motor_pos[motor] += dir * local_step_size;
	}
	else if (remote_step_size == FULL_STEP || remote_step_size == HALF_STEP)
	{
		motor_pos[motor] += dir * remote_step_size;
	}
	if ((motor_pos[motor] %2 == 0 && remote_step_size == FULL_STEP) ||
		(motor_pos[motor] %2 ==0 && remote_step_size == USE_LOCAL_STEP && local_step_size == FULL_STEP))
	{
		motor_pos[motor] -= dir;
	}
	//Overflow and underflow protection
	int16_t new_pos = motor_pos[motor];
	new_pos %= 8;
	if (new_pos < 0)
	{
		new_pos += 8;
	}
	//Pass the direction of movement to set_LEDs()
	motor_status[motor] = dir;
	//Select this motor's register
	set_addr(motor);
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
	//Set register bits to represent the active coils for the next step
	if (coil_current[0][new_pos] == 1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D0_PIN);
	}
	if (coil_current[1][new_pos] == 1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D1_PIN);
	}
	if (coil_current[2][new_pos] == 1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D2_PIN);
	}
	if (coil_current[3][new_pos] == 1)
	{
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, D3_PIN);
	}
	//Send the data to ROBKO 01
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
	auto_mode = LL_GPIO_IsInputPinSet(INPUT_PORT, AUTO_MODE_PIN);
	manual_mode = LL_GPIO_IsInputPinSet(INPUT_PORT, MANUAL_MODE_PIN);
	if (LL_GPIO_IsInputPinSet(INPUT_PORT, STEP_SIZE_PIN))
	{
		local_step_size=FULL_STEP;
	}
	else
	{
		local_step_size=FULL_STEP;
	}
	if (LL_GPIO_IsInputPinSet(INPUT_PORT, STEP_TIME_PIN))
	{
		local_step_time=FAST_STEP;
	}
	else
	{
		local_step_time=SLOW_STEP;
	}
	if (!auto_mode)
	{
		if (manual_mode)
		{
			manual_control();
		}
		else
		{
			remote_control();
		}
	}
	else
	{
		joystick = LL_GPIO_IsInputPinSet(INPUT_PORT, JOYSTICK_CONNECTED_PIN);
		if (joystick)
		{
			manual_control();
		}
		else
		{
			remote_control();
		}
	}
	set_LEDs();
	//Wait for the motors to finish moving
	if (remote_step_time==USE_LOCAL_TIME)
	{
		LL_mDelay(local_step_time);
	}
	else
	{
		LL_mDelay(remote_step_time);
	}
}

void manual_control(void)
{
	int8_t upper_but, lower_but;
	upper_but = LL_GPIO_IsInputPinSet(INPUT_PORT, CLAW_GRAB_BUT_PIN);
	lower_but = LL_GPIO_IsInputPinSet(INPUT_PORT, CLAW_RELEASE_BUT_PIN);
	if (upper_but && !lower_but)
	{
		step_motor(CLAW_GRAB_MOTOR, STEP_REV);
	}
	else if (!upper_but && lower_but)
	{
		step_motor(CLAW_GRAB_MOTOR, STEP_FWD);
	}
	upper_but = LL_GPIO_IsInputPinSet(INPUT_PORT, CLAW_UP_BUT_PIN);
	lower_but = LL_GPIO_IsInputPinSet(INPUT_PORT, CLAW_DOWN_BUT_PIN);
	if (upper_but && !lower_but)
	{
		step_motor(CLAW_ROT_MOTOR_L, STEP_FWD);
		step_motor(CLAW_ROT_MOTOR_R, STEP_REV);
	}
	else if (!upper_but && lower_but)
	{
		step_motor(CLAW_ROT_MOTOR_L, STEP_REV);
		step_motor(CLAW_ROT_MOTOR_R, STEP_FWD);
	}
	if (adc_pot_vals[LEFT_RIGHT_POT_NUM] > LEFT_THR)
	{
		step_motor(ROTATION_MOTOR, STEP_FWD);
	}
	else if (adc_pot_vals[LEFT_RIGHT_POT_NUM] < RIGHT_THR)
	{
		step_motor(ROTATION_MOTOR, STEP_REV);
	}
	if (adc_pot_vals[SHOULDER_POT_NUM] > UP_THR)
	{
		step_motor(SHOULDER_MOTOR, STEP_FWD);
	}
	else if (adc_pot_vals[SHOULDER_POT_NUM] < DOWN_THR)
	{
		step_motor(SHOULDER_MOTOR, STEP_REV);
	}
	if (adc_pot_vals[ELBOW_POT_NUM] > UP_THR)
	{
		step_motor(ELBOW_MOTOR, STEP_FWD);
	}
	else if (adc_pot_vals[ELBOW_POT_NUM] < DOWN_THR)
	{
		step_motor(ELBOW_MOTOR, STEP_REV);
	}
	if (adc_pot_vals[CLAW_ROTATION_POT_NUM] > LEFT_THR)
	{
		step_motor(CLAW_ROT_MOTOR_R, STEP_REV);
		step_motor(CLAW_ROT_MOTOR_L, STEP_REV);
	}
	else if (adc_pot_vals[CLAW_ROTATION_POT_NUM] < RIGHT_THR)
	{
		step_motor(CLAW_ROT_MOTOR_R, STEP_FWD);
		step_motor(CLAW_ROT_MOTOR_L, STEP_FWD);
	}
}

//Executes a stored command
int32_t remote_control(void)
{
	//n is a flag, raised if the current command has been completed and can be deleted
	int8_t n = 0, i;
	cmd_buffer_t *old;
	//Check if any commands are pending
	if (current_cmd == NULL || LL_GPIO_IsOutputPinSet(ADDR_DATA_PORT, ENABLE_PIN) == 0)
	{
		return -1;
	}
	//Check if the entire command has been buffered
	if (current_cmd->incomplete)
	{
		return -2;
	}
	//Decode command
	switch (current_cmd->cmd[0])
	{
		case MOV:
			if ((* (int16_t *) (current_cmd->cmd + 2)) > 0)
			{
				n = step_motor(current_cmd->cmd[1], STEP_FWD);
				//Decrement number of steps
				(* (int16_t *) (current_cmd->cmd + 2))--;
			}
			else if ((* (int16_t *) (current_cmd->cmd + 2)) < 0)
			{
				n = step_motor(current_cmd->cmd[1], STEP_REV);
				//Decrement number of steps
				(* (int16_t *) (current_cmd->cmd + 2))++;
			}
			//Checks if the robot should stop moving before finishing the command
			if (check_opto_flag())
			{
				n = 1;
				move_until = MOVE_FREELY;
			}
			//Check if command has been finished
			if ((* (int16_t *) (current_cmd->cmd + 2)) == 0)
			{
				n = 1;
				move_until = MOVE_FREELY;
			}
			break;
		case MOVE:
			for (i = 0; i < 12; i += 2)
			{
				if ((* (int16_t *) (current_cmd->cmd + 1 + i)) > 0)
				{
					n = step_motor(i / 2, STEP_FWD);
					//Decrement number of steps
					(* (int16_t *) (current_cmd->cmd + 1 + i))--;
				}
				else if ((* (int16_t *) (current_cmd->cmd + 1 + i)) < 0)
				{
					n = step_motor(i / 2, STEP_REV);
					//Decrement number of steps
					(* (int16_t *) (current_cmd->cmd + 1 + i))++;
				}
			}
			//Checks if the robot should stop moving before finishing the command
			if (check_opto_flag())
			{
				n = 1;
				move_until = MOVE_FREELY;
			}
			//Check if command has been finished
			if (0 == ((* (int16_t *) (current_cmd->cmd + 1)) ||
					  (* (int16_t *) (current_cmd->cmd + 3)) ||
					  (* (int16_t *) (current_cmd->cmd + 5)) ||
					  (* (int16_t *) (current_cmd->cmd + 7)) ||
					  (* (int16_t *) (current_cmd->cmd + 9)) ||
					  (* (int16_t *) (current_cmd->cmd + 11))))
			{
				n = 1;
				move_until = MOVE_FREELY;
			}
			break;
		case OFF:
			stop_motor(current_cmd->cmd[1]);
			n = 1;
			break;
		case FREEZE:
			DISABLE_ROBKO();
			n = 1;
			break;
		//TODO
		//case GOTO_POS:
		//case SAVE_POS:
		case SET_STEP:
			if (current_cmd->cmd[1] <= FULL_STEP)		//FULL_STEP has the highest value from
			{
				remote_step_size = current_cmd->cmd[1];	//the possible values of this flag
			}
			n = 1;
			break;
		case SET_SPEED:
			if ((* (uint16_t *) (current_cmd->cmd + 1)) >= MAX_STEP_SPEED || (* (uint16_t *) (current_cmd->cmd + 1)) == USE_LOCAL_TIME)
			{
				remote_step_time=(* (uint16_t *) (current_cmd->cmd+1));
			}
			n = 1;
			break;
		case OPTO:
			//Sets a flag to stop movement at detection/loss of object
			if (current_cmd->cmd[1] <= MOVE_UNTIL_NO_DETECTION)	//MOVE_UNTIL_NO_DETECTION has the highest value from
			{
				move_until=current_cmd->cmd[1];					//the possible values of this flag
			}
			n = 1;
			break;
		//Unknown command
		default: n = 1;
	}
	if (n)
	{
		//Advance the current command to the next and delete the old one
		old = current_cmd;
		current_cmd = current_cmd->next_cmd;
		free(old);
	}
	return 0;
}

//Turns off all coils in a motor, allowing to move it by hand
int32_t stop_motor(uint8_t motor)
{
	uint8_t n;
	if (motor < 6)
	{
		set_addr(motor);
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, D0_PIN | D1_PIN | D2_PIN | D3_PIN);
		//Send the data to ROBKO 01
		LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		DWT_Delay(10);
		LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOW_PIN);
		return 0;
	}
	else if (motor == ALL_MOTORS)
	{
		//Recursion
		for (n = 0; n < 6; n++)
		{
			stop_motor(n);
		}
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
	static uint8_t rem_bytes = 0, current_byte = 0;
	uint8_t read_buffer;
	cmd_buffer_t *eraser;
	//Read a byte of data
	read_buffer = LL_USART_ReceiveData9(USART1);
	//Check if there is a command not fully received
	if (rem_bytes > 0)
	{
		//Add the new byte to the unfinished command
		last_cmd->cmd[current_byte] = read_buffer;
		current_byte++;
		rem_bytes--;
		if (!rem_bytes)
		{
			last_cmd->incomplete = 0;
		}
	}
	else
	{
		//Start of new command
		current_byte = 1;
		rem_bytes = 0;
		//KILL, RESUME, GET_POS and CLEAR commands are not added to the queue. Instead they are executed with priority.
		switch (read_buffer)
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
					eraser = current_cmd;
					current_cmd = current_cmd->next_cmd;
					free(eraser);
				}
				while (current_cmd != NULL);
				break;
			case GET_POS:
				//TODO
				break;
			case GOTO_POS: case MOVE: rem_bytes += 9;
			/* no break*/
			case MOV: rem_bytes += 1;
			/* no break */
			case SET_SPEED: rem_bytes += 1;
			/* no break */
			case OPTO: case OFF: case SET_STEP: rem_bytes += 1;
			/* no break */
			case FREEZE:
				//Allocate memory for the new command and add it to the linked list
				//Check if the queue is empty
				if (current_cmd == NULL)
				{
					current_cmd = malloc(sizeof(cmd_buffer_t));
					last_cmd = current_cmd;
				}
				else
				{
					last_cmd->next_cmd = malloc(sizeof(cmd_buffer_t));
					last_cmd = last_cmd->next_cmd;
				}
				//Check if memory was allocated successfully
				if (last_cmd != NULL)
				{
					//Zero-out the new block of memory
					memset(last_cmd, 0, sizeof(cmd_buffer_t));
					//Write the byte of data that was read
					last_cmd->cmd[0] = read_buffer;
					if (read_buffer != FREEZE)
					{
						last_cmd->incomplete = 1;
					}
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
	for (i = 0; i < 12; i++)
	{
		LL_GPIO_ResetOutputPin(LED_PORT, LED_pins[i]);
	}
	//Turn on the LEDs corresponding to the last steps made
	for (i = 0; i < 6; i++)
	{
		if (motor_status[i] == STEP_FWD)
		{
			LL_GPIO_SetOutputPin(LED_PORT, LED_pins[i * 2]);
		}
		else if (motor_status[i] == STEP_REV)
		{
			LL_GPIO_SetOutputPin(LED_PORT, LED_pins[i * 2 + 1]);
		}
	}
	//Clear the motor_status array so the step_motor() function can safely set it
	memset(motor_status, 0, sizeof(motor_status[0]) * 6);
}

//Check if an object is detected inside the claw
int8_t get_opto(void)
{
	uint8_t opto_state;
	set_addr(7);
	//Read the state of pin IN6 on ROBKO 01
	LL_GPIO_ResetOutputPin(ADDR_DATA_PORT, IOR_PIN);
	DWT_Delay(10);
	opto_state = LL_GPIO_IsInputPinSet(ADDR_DATA_PORT, D6_PIN);
	LL_GPIO_SetOutputPin(ADDR_DATA_PORT, IOR_PIN);
	if (opto_state)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//Checks if the conditions to stop moving are met.
//Command OPTO sets the move_until flag when the robot must move until it detects/loses an object.
int8_t check_opto_flag(void)
{
	int8_t opto_state = get_opto();
	if (move_until == MOVE_FREELY)
	{
		return 0;
	}
	else if (move_until == MOVE_UNTIL_DETECTION && opto_state)
	{
		return 1;
	}
	else if (move_until == MOVE_UNTIL_NO_DETECTION && !opto_state)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
