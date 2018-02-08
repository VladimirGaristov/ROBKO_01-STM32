#include "robko_decode.h"
//Includes string.h, stdlib.h and stdint.h

void decode_cmd(char *buffer, uint8_t *cmd)
{
	char *next_val, i;
	int l;
	//Dobavi proverki dali ne sa vavedeni krivi stoinosti
	//The first byte is the lenght
	if (strstr(buffer, "MOVE"))
	{
		cmd[0] = 14;
		cmd[1] = MOVE;
		l=strlen(buffer);
		* (int16_t *) (cmd+2) = (int16_t) strtol(buffer+5, &next_val, 10);
		for(i=0;i<10;i+=2)
		{
			//Prevents reading from adresses beyond the end of the buffer
			if(next_val > buffer+l)
				break;
			* (int16_t *) (cmd+4+i) = (int16_t) strtol(next_val+1, &next_val, 10);
		}
	}
	else if (strstr(buffer, "MOV"))
	{
		cmd[0] = 5;
		cmd[1] = MOV;
		* (int16_t *) (cmd+3) = (int16_t) strtol(buffer+6, NULL, 10);
		buffer[5] = '\0';
		cmd[2] = (uint8_t) strtol(buffer+4, NULL, 10);
	}
	else if (strstr(buffer, "OFF"))
	{
		cmd[0] = 3;
		cmd[1] = OFF;
		cmd[2] = (uint8_t) strtol(buffer+4, NULL, 10);
	}
	else if (strstr(buffer, "OPEN_FILE"))
	{
		cmd[0] = strlen(buffer) - 9;
		cmd[1] = OPEN_FILE;
		strcpy((char *) cmd+2, buffer+10);
	}
	/*else if (strstr(buffer, "GOTO_POS"))
	{
		cmd[0] = 12;
		cmd[1] = GOTO_POS;
		*(short int *) (cmd+2) = (short int) strtol(buffer+9, NULL, 10);
	}*/
	//dobavi GET_POS SAVE_POS
	else if (strstr(buffer, "KILL"))
	{
		cmd[0] = 2;
		cmd[1] = KILL;
	}
	else if (strstr(buffer, "CLEAR"))
	{
		cmd[0] = 2;
		cmd[1] = CLEAR;
	}
	else if (strstr(buffer, "FREEZE"))
	{
		cmd[0] = 2;
		cmd[1] = FREEZE;
	}
	else if (strstr(buffer, "RESUME"))
	{
		cmd[0] = 2;
		cmd[1] = RESUME;
	}
	else if (strstr(buffer, "GET_POS"))
	{
		cmd[0] = 2;
		cmd[1] = GET_POS;
	}
	else if (strstr(buffer, "SAVE_POS"))
	{
		cmd[0] = 2;
		cmd[1] = SAVE_POS;
	}
	else if (strstr(buffer, "OPTO"))
	{
		cmd[0] = 3;
		cmd[1] = OPTO;
		cmd[2] = (uint8_t) strtol(buffer+5, NULL, 10);
	}
	else if (strstr(buffer, "SET_STEP"))
	{
		cmd[0] = 3;
		cmd[1] = SET_STEP;
		cmd[2] = (uint8_t) strtol(buffer+9, NULL, 10);
	}
	else if (strstr(buffer, "SET_SPEED"))
	{
		cmd[0] = 4;
		cmd[1] = SET_SPEED;
		* (uint16_t *) (cmd+2) = (uint16_t) strtol(buffer+10, NULL, 10);
	}
}
