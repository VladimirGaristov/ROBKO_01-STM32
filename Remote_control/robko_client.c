#define _GNU_SOURCE		//Checked for by one of the included files

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "robko_decode.h"
//Includes string.h, stdlib.h and stdint.h

#define BUFFER_SIZE (512)

void error(char *msg);

void error(char *msg)
{
	perror(msg);
	exit(-1);
}

int main(int argc, const char *argv[])
{
	int sockfd, n, bytes_available = 0, sock_flags;
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
	{
		error("ERROR opening socket\n");
	}
	memset(&serv_addr, 0, sizeof(serv_addr));
	//Set the server address and port
	serv_addr.sin_family = AF_INET;
	if (!(inet_aton(argv[1], (struct in_addr *) &serv_addr.sin_addr.s_addr)))
	{
		error("ERROR converting server address\n");
	}
	serv_addr.sin_port = htons(SOCK_PORT);
	printf("Attempting to connect...\n");
	//Connect to the server
	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		error("ERROR connecting\n");
	}
	//Set the socket file descriptor to non-blocking mode
	sock_flags = fcntl(sockfd, F_GETFL, 0);
	fcntl(sockfd, F_SETFL, sock_flags | O_NONBLOCK);
	printf("Please enter a command: ");
	while (1)
	{	
		//File descriptor 0 is stdin
		if (ioctl(0, FIONREAD, &bytes_available) == 0 && bytes_available > 0)
		{
			memset(buffer, 0, BUFFER_SIZE);
			memset(cmd, 0, BUFFER_SIZE);
			fgets(buffer, BUFFER_SIZE, stdin);
			printf("Please enter a command: ");
			//Add command verification?
			decode_cmd(buffer, cmd);
			//Send data to the server
			n = write(sockfd, cmd, cmd[0]);
			if (n < 0)
			{
				error("ERROR writing to socket\n");
			}
			printf("%d bytes sent.\n", n);
		}
		if (ioctl(sockfd, FIONREAD, &bytes_available) == 0 && bytes_available > 0)
		{
			memset(buffer, 0, BUFFER_SIZE);
			n = read(sockfd, buffer, BUFFER_SIZE - 1);
			if (n < 0)
			{
				printf("ERROR reading from socket\n");
			}
			printf("Reply from server:\t%s\n", buffer);
		}
	}
	//shutdown(sockfd, SHUT_WR);
	return 0;
}
