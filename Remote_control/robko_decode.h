#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//Commands for remote control
#define MOV 1
#define MOVE 2
#define OFF 3
#define OPEN_FILE 4
#define GOTO_POS 5
#define GET_POS 6
#define KILL 7
#define CLEAR 8
#define FREEZE 9
#define RESUME 10
#define SAVE_POS 11
#define OPTO 12
#define SET_STEP 13
#define SET_SPEED 14

//Socket parameters
#define SOCK_PORT 55321
#define SOCK_PROTOCOL SOCK_STREAM	//SOCK_STREAM=TCP

void decode_cmd(char *buffer, uint8_t *cmd);