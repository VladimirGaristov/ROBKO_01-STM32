#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#define SOCK_PORT 55321
#define SOCK_PROTOCOL SOCK_STREAM	//SOCK_STREAM=TCP
#define BUFFER_SIZE (256)
//#define SERVER_IP_ADDR "192.168.1.4"

//Commands for remote control
#define MOV_FWD 1
#define MOV_REV 2
#define OFF 3
#define OPEN_FILE 4
#define GOTO_POS 5
#define TOGETHER 6
#define KILL 7
#define CLEAR 8
#define FREEZE 9
#define RESUME 10

void error(char *msg);
void decode_cmd(char *buffer, char *cmd);

void error(char *msg)
{
	perror(msg);
	exit(-1);
}

int main(int argc, const char *argv[])
{
	int sockfd, n;
	struct sockaddr_in serv_addr;
	char buffer[BUFFER_SIZE], cmd[BUFFER_SIZE];
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

void decode_cmd(char *buffer, char *cmd)
{
	//The first byte is the lenght
	if (strstr(buffer, "MOV_FWD"))
	{
		cmd[0] = 4;
		cmd[1] = MOV_FWD;
		cmd[3] = (char) strtol(buffer+10, NULL, 10);
		buffer[9] = '\0';
		cmd[2] = (char) strtol(buffer+8, NULL, 10);
	}
	else if (strstr(buffer, "MOV_REV"))
	{
		cmd[0] = 4;
		cmd[1] = MOV_REV;
		cmd[3] = (char) strtol(buffer+10, NULL, 10);
		buffer[9] = '\0';
		cmd[2] = (char) strtol(buffer+8, NULL, 10);
	}
	else if (strstr(buffer, "OFF"))
	{
		cmd[0] = 3;
		cmd[1] = OFF;
		cmd[2] = (char) strtol(buffer+4, NULL, 10);
	}
	else if (strstr(buffer, "OPEN_FILE"))
	{
		cmd[0] = strlen(buffer) - 9;
		cmd[1] = OPEN_FILE;
		strcpy(cmd+2, buffer+10);
	}
	else if (strstr(buffer, "GOTO_POS"))
	{
		cmd[0] = 5;
		cmd[1] = GOTO_POS;
		*(short int *) (cmd+2) = (short int) strtol(buffer+9, NULL, 10);
	}
	else if (strstr(buffer, "TOGETHER"))
	{
		cmd[0] = 3;
		cmd[1] = TOGETHER;
		cmd[2] = (char) strtol(buffer+9, NULL, 10);
	}
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
}