#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define SOCK_PORT 55321
#define SOCK_PROTOCOL SOCK_STREAM	//SOCK_STREAM=TCP
#define BUFFER_SIZE (512)
//#define SERVER_IP_ADDR "192.168.1.4"

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

void error(char *msg);
void decode_cmd(char *buffer, uint8_t *cmd);

void error(char *msg)
{
	perror(msg);
	exit(-1);
}

int main(int argc, const char *argv[])
{
	int sockfd, n;
	struct sockaddr_in serv_addr;
	char buffer[BUFFER_SIZE];
	uint8_t cmd[BUFFER_SIZE];
	if (argc < 2)
	{
		error("ERROR - plese specify the server IP address\n");
	}
	//Open a socket
	sockfd = socket(AF_INET, SOCK_PROTOCOL, 0);
	if (sockfd < 0)
		error("ERROR opening socket\n");
	memset(&serv_addr, 0, sizeof(serv_addr));
	//Set the server address and port
	serv_addr.sin_family = AF_INET;
	if (!(inet_aton(argv[1], (struct in_addr *) &serv_addr.sin_addr.s_addr)))
	error("ERROR converting server address\n");
	serv_addr.sin_port = htons(SOCK_PORT);
	printf("Attempting to connect...\n");
	//Connect to the server
	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		error("ERROR connecting\n");
	while (1)
	{
		printf("Please enter a command: ");
		memset(buffer, 0, BUFFER_SIZE);
		fgets(buffer, BUFFER_SIZE, stdin);
		//Add command verification
		decode_cmd(buffer, cmd);
		//Send data to the server
		n = write(sockfd, cmd, cmd[0]);
		if (n < 0)
			 error("ERROR writing to socket\n");
		printf("%d bytes sent.\n", n);
		memset(buffer, 0, BUFFER_SIZE);
		memset(cmd, 0, BUFFER_SIZE);
	}
	//shutdown(sockfd, SHUT_WR);
	return 0;
}

void decode_cmd(char *buffer, uint8_t *cmd)
{
	//Dobavi proverki dali ne sa vavedeni krivi stoinosti
	//The first byte is the lenght
	if (strstr(buffer, "MOV"))
	{
		cmd[0] = 5;
		cmd[1] = MOV;
		* (int16_t *) (cmd+3) = (int16_t) strtol(buffer+5, NULL, 10);
		buffer[5] = '\0';
		cmd[2] = (uint8_t) strtol(buffer+4, NULL, 10);
	}
	/*else if (strstr(buffer, "MOVE"))
	{
		cmd[0] = 13;
		cmd[1] = MOVE;
		cmd[3] = (uint8_t) strtol(buffer+10, NULL, 10);
		buffer[9] = '\0';
		cmd[2] = (uint8_t) strtol(buffer+8, NULL, 10);
	}*/
	else if (strstr(buffer, "OFF"))
	{
		cmd[0] = 3;
		cmd[1] = OFF;
		cmd[2] = (uint8_t) strtol(buffer+4, NULL, 10);
	}
	else if (strstr(buffer, "OPEN_FILE"))
	{
		cmd[0] = strlen(buffer) - 10;
		cmd[1] = OPEN_FILE;
		strcpy((char *) cmd+2, buffer+10);
	}
	/*else if (strstr(buffer, "GOTO_POS"))
	{
		cmd[0] = 12;
		cmd[1] = GOTO_POS;
		*(short int *) (cmd+2) = (short int) strtol(buffer+9, NULL, 10);
	}*/
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
