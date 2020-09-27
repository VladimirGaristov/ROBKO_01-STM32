#ifndef ROBKO_DEODE_H
#define ROBKO_DEODE_H

//Commands for remote control
#define MOV			1
#define MOVE		2
#define OFF			3
#define OPEN_FILE	4
#define GOTO_POS	5
#define GET_POS		6
#define KILL		7
#define CLEAR		8
#define FREEZE		9
#define RESUME		10
#define SAVE_POS	11
#define OPTO		12
#define SET_STEP	13
#define SET_SPEED	14
#define GET_STEP	15
#define GET_SPEED	16
#define SET_HOME	17

#define REPEAT		25

#define GET_POS_REPLY	18
#define SAVE_POS_REPLY	69
#define STEP_REPLY		20
#define SPEED_REPLY		21
#define ACK				22
#define ERROR_REPLY		23
#define LAST_CMD		24

#define FULL_RAM		101
#define UNKNOWN_CMD		102

#define FULL_STEP		2
#define HALF_STEP		1
#define USE_LOCAL_STEP	0
#define USE_LOCAL_TIME	0

//Socket parameters
#define SOCK_PORT		55321
#define SOCK_PROTOCOL	SOCK_STREAM	//SOCK_STREAM=TCP

#define MAX_FILENAME_SIZE	256

int decode_cmd(char *buffer, uint8_t *cmd);

#endif