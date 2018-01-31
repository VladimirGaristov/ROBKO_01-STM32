#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <libserialport.h>
#include <time.h>

#define SOCK_PORT 55321
#define SOCK_PROTOCOL SOCK_STREAM	//SOCK_STREAM=TCP
#define SOCK_BUFFER_SIZE 255
#define SER_BUFFER_SIZE 100
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
#define STOP_BITS 1
#define WORD_LENGHT 8
#define PARITY SP_PARITY_ODD

void delay(int msec);
int port_configuration(struct sp_port **ser_port);
void transmit_string(struct sp_port *ser_port, char *s_out);
void error(const char *);

int main(void)
{
	struct sockaddr_in serv_addr, cli_addr;
	int sockfd, newsockfd, clilen, n;
	char sock_buffer[SOCK_BUFFER_SIZE], serial_buffer[SER_BUFFER_SIZE];
	struct sp_port *serial_port;
	//Open serial connection
	if(port_configuration(&serial_port))
		error("Serial port error\n");
	//Create a socket for receiving incoming requests
	sockfd=socket(AF_INET, SOCK_PROTOCOL, 0);
	if(sockfd<0)
		error("ERROR opening socket\n");
	//Set all bits in serv_addr to 0
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	//INADDR_ANY is the IP address of the host, running this code
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(SOCK_PORT);
	//Bind the socket to the host IP address and port number
	//sockaddr is a generic socket address, sockaddr_in is for IPv4
	if(bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))<0)
		error("ERROR on binding\n");
	listen(sockfd,5);
	clilen=sizeof(cli_addr);
	while(1)
	{
		printf("\nAwaiting connection...\n");
		//Open a socket to communicate with the client
		newsockfd=accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t *) &clilen);
		if(newsockfd<0)
			error("ERROR on accept\n");
		while(1)
		{
			memset(sock_buffer, 0, SOCK_BUFFER_SIZE);
			memset(serial_buffer, 0, SER_BUFFER_SIZE);
			n=read(newsockfd, sock_buffer, SOCK_BUFFER_SIZE);
			//add data parsing?
			//If FIN has been received, close the socket
			if(!n)
			{
				//add a console message
				shutdown(newsockfd, SHUT_RDWR);
				close(newsockfd);
				break;
			}
			printf("Received %d bytes\n", n);
			transmit_string(serial_port, sock_buffer);
			if(n<0)
				error("ERROR writing to socket\n");
		}
	}
	sp_free_port(serial_port);
	return 0;
}

void error(const char *error_msg)
{
	perror(error_msg);
	exit(-1);
}

void delay(int msec)
{
	clock_t start, current=0;
	start=clock();
	while((((float)(current-start))/CLOCKS_PER_SEC)*1000.0<msec)
		current=clock();
}

int port_configuration(struct sp_port **ser_port)
{
	int check;
	sp_get_port_by_name(SERIAL_PORT, ser_port);
	if(ser_port==NULL)
	{
		printf("Port is unavailable or doesn't exist.\n");
		return -1;
	}
	check=sp_open(*ser_port, SP_MODE_READ_WRITE);
	if(check)
	{
		printf("Port is busy.\n");
		return -3;
	}
	check=sp_set_baudrate(*ser_port, BAUDRATE);
	check|=sp_set_bits(*ser_port, WORD_LENGHT);
	check|=sp_set_stopbits(*ser_port, STOP_BITS);
	check|=sp_set_parity(*ser_port, PARITY);
	if(check)
	{
		printf("Configuration failed.\n");
		return -2;
	}
	return 0;
}

void transmit_string(struct sp_port *ser_port, char *s_out)
{
	int i=1;
	//int l=strlen(s_out);
	int l=s_out[0];
	while(i<l)
	{
		sp_nonblocking_write(ser_port, s_out+i, 1);
		i++;
		delay(20);
	}
	printf("Bytes written: %d\n", i);
}
