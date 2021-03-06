#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <libserialport.h>
#include <time.h>
#include <bsd/string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include "robko_decode.h"

#define SOCK_BUFFER_SIZE 1501
#define SER_BUFFER_SIZE 100
#define REPLY_MSG_SIZE 150
#define FILE_BUFFER_SIZE 256
#define CMD_MAX_SIZE 14
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
#define STOP_BITS 1
#define WORD_LENGHT 8
#define PARITY SP_PARITY_ODD
#define OUTPUT_FILE "/home/cartogan/Ac6/workspace/ROBKO_01/Remote_control/position_log.robko"
#define READ_TIMEOUT 25		//ms

#define PARSED_REPEAT 2
#define PARSED_REPLY 1

#define EN_DEBUG

typedef struct
{
	uint8_t repeat;
	char filename[MAX_FILENAME_SIZE];
}
script_file_t;

typedef struct
{
	int sockfd;
	struct sp_port *ser_port;
	script_file_t *script_file;
}
thread_args_t;

pthread_mutex_t serial_port_mutex;

void delay(int msec);
int port_configuration(struct sp_port **ser_port);
void transmit_string(struct sp_port *ser_port, char *s_out);
void error(const char *);
int execute_file(script_file_t *file, struct sp_port *ser_port);
int get_client_commands(int client_sockfd, struct sp_port *ser_port);
int parse_reply(struct sp_port *ser_port, char *msg);
int save_position(int16_t *position);
int send_reply(int client_sockfd, char *reply_msg);
void *reply_thread(void *args);

int main(void)
{
	struct sockaddr_in serv_addr, cli_addr;
	int sockfd, clilen;
	struct sp_port *serial_port;

	//Open serial connection
	if (port_configuration(&serial_port))
	{
		error("Serial port error\n");
	}

	//Create a socket for receiving incoming requests
	sockfd = socket(AF_INET, SOCK_PROTOCOL, 0);
	if (sockfd < 0)
	{
		error("ERROR opening socket\n");
	}
	//Set all bits in serv_addr to 0
	memset((char *) &serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	//INADDR_ANY is the IP address of the host, running this code
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(SOCK_PORT);
	//Bind the socket to the host IP address and port number
	//sockaddr is a generic socket address, sockaddr_in is for IPv4
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		error("ERROR on binding\n");
	}
	listen(sockfd, 5);
	clilen = sizeof(cli_addr);

	while (1)
	{
		printf("\nAwaiting connection...\n");
		//Open a socket to communicate with the client
		get_client_commands(accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t *) &clilen), serial_port);
	}

	sp_free_port(serial_port);
	return 0;
}

int get_client_commands(int client_sockfd, struct sp_port *ser_port)
{
	int n, sock_flags;
	//int bytes_available = 0;
	char sock_buffer[SOCK_BUFFER_SIZE];
	script_file_t script_file =
	{
		.repeat = 0, .filename[0] = '\0'
	};
	pthread_attr_t attributes;
	pthread_t reply_thread_handle;
	thread_args_t arguments;

	if (client_sockfd < 0)
	{
		printf("ERROR on accept!\n");
		return -1;
	}
	//Set the socket file descriptor to non-blocking mode
	sock_flags = fcntl(client_sockfd, F_GETFL, 0);
	fcntl(client_sockfd, F_SETFL, sock_flags | O_NONBLOCK);
	puts("Client connected.");

	pthread_mutex_init(&serial_port_mutex, NULL);
	pthread_attr_init(&attributes);
	pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
	arguments.sockfd = client_sockfd;
	arguments.ser_port = ser_port;
	arguments.script_file = &script_file;
	if (pthread_create(&reply_thread_handle, &attributes, reply_thread, (void *) &arguments))
	{
		printf("ERROR creating secondary thread!\n");
		pthread_attr_destroy(&attributes);
		shutdown(client_sockfd, SHUT_RDWR);
		close(client_sockfd);
		return -2;
	}
	pthread_attr_destroy(&attributes);

	while (1)
	{
		/*
		//Moved to secondary thread
		switch (parse_reply(ser_port, reply_msg))
		{
			case 2:
				if (script_file.repeat)
				{
					execute_file(&script_file, ser_port);
				}
				break;
			case 1:
				puts(reply_msg);
				send_reply(client_sockfd, reply_msg);
				break;
			default:;
		}
		*/

		//if (ioctl(client_sockfd, FIONREAD, &bytes_available) == 0 && bytes_available > 0)
		//{
			memset(sock_buffer, 0, SOCK_BUFFER_SIZE);
			errno = 0;
			n = read(client_sockfd, sock_buffer, SOCK_BUFFER_SIZE - 1);
			//if file, add null byte?
			
			//If FIN has been received, close the socket
			if (!n)
			{
				//add a console message
				shutdown(client_sockfd, SHUT_RDWR);
				close(client_sockfd);
				pthread_cancel(reply_thread_handle);
				pthread_mutex_destroy(&serial_port_mutex);
				return 0;
			}

			if (n < 0)
			{
				if (errno == EAGAIN || errno == EWOULDBLOCK)
				{
					continue;
				}
				else
				{
					printf("ERROR reading from socket\n");
					shutdown(client_sockfd, SHUT_RDWR);
					close(client_sockfd);
					pthread_cancel(reply_thread_handle);
					pthread_mutex_destroy(&serial_port_mutex);
					return -2;
				}
			}

			#ifdef EN_DEBUG
			printf("Received %d bytes\n", n);
			#endif

			if (sock_buffer[1] == OPEN_FILE)
			{
				script_file.repeat = 0;
				strlcpy(script_file.filename, sock_buffer + 2, MAX_FILENAME_SIZE);
				pthread_mutex_lock(&serial_port_mutex);
				execute_file(&script_file, ser_port);
				pthread_mutex_unlock(&serial_port_mutex);
			}
			else
			{
				script_file.repeat = 0;
				pthread_mutex_lock(&serial_port_mutex);
				transmit_string(ser_port, sock_buffer);
				pthread_mutex_unlock(&serial_port_mutex);
			}
		//}
	}
}

void error(const char *error_msg)
{
	perror(error_msg);
	exit(-1);
}

void delay(int msec)
{
	clock_t start, current = 0;
	start = clock();
	while ((((float) (current - start)) / CLOCKS_PER_SEC) * 1000.0 < msec)
	{
		current = clock();
	}
}

//Initialize the serial port
int port_configuration(struct sp_port **ser_port)
{
	int check;
	//Decode port name
	sp_get_port_by_name(SERIAL_PORT, ser_port);
	if (ser_port == NULL)
	{
		printf("Port is unavailable or doesn't exist.\n");
		return -1;
	}
	check = sp_open(*ser_port, SP_MODE_READ_WRITE);
	if (check)
	{
		printf("Port is busy.\n");
		return -3;
	}
	check = sp_set_baudrate(*ser_port, BAUDRATE);
	check |= sp_set_bits(*ser_port, WORD_LENGHT);
	check |= sp_set_stopbits(*ser_port, STOP_BITS);
	check |= sp_set_parity(*ser_port, PARITY);
	if (check)
	{
		printf("Configuration failed.\n");
		return -2;
	}
	return 0;
}

void transmit_string(struct sp_port *ser_port, char *s_out)
{
	int i = 1;
	int l = s_out[0];
	while (i < l)
	{
		sp_nonblocking_write(ser_port, s_out + i, 1);
		i++;
	}
	printf("Bytes written: %d\n", i - 1);
}

int execute_file(script_file_t *file, struct sp_port *ser_port)
{
	FILE *fd = NULL;
	char file_buffer[FILE_BUFFER_SIZE];
	uint8_t cmd[CMD_MAX_SIZE], decode_status;
	memset(file_buffer, 0, FILE_BUFFER_SIZE);
	fd = fopen(file->filename, "r");
	if (fd == NULL)
	{
		printf("Error when opening file \"%s\". It may not exist.\n", file->filename);
		return -1;
	}
	printf("Executing %s\n", file->filename);
	//Reads one line at a time
	while (fgets(file_buffer, FILE_BUFFER_SIZE - 1, fd))
	{
		decode_status = decode_cmd(file_buffer, cmd);
		if (!decode_status)
		{
			transmit_string(ser_port, (char *) cmd);
		}
		else if (decode_status == REPEAT)
		{
			file->repeat = 1;
		}
		else
		{
			printf("Invalid command - \"%s\"\n", file_buffer);
		}
		//TODO parse reply and check for error or ACK
	}
	printf("Done\n");
	fclose(fd);
	return 0;
}

int parse_reply(struct sp_port *ser_port, char *msg)
{
	uint8_t reply[13] = {0}, read_status = 0;
	uint16_t speed = 0;
	//pthread_mutex_lock(&serial_port_mutex);
	read_status = sp_blocking_read(ser_port, reply, 1, READ_TIMEOUT);
	//pthread_mutex_unlock(&serial_port_mutex);
	if (!read_status)
	{
		return 0;
	}
	else if (read_status == 1)
	{
		#ifdef EN_DEBUG
		printf("--- %d ---\n", reply[0]);
		fflush(stdout);
		#endif

		switch (reply[0])
		{
			case ACK:
				return 0;
			case STEP_REPLY:
				pthread_mutex_lock(&serial_port_mutex);
				read_status = sp_blocking_read(ser_port, reply + 1, 1, READ_TIMEOUT);
				pthread_mutex_unlock(&serial_port_mutex);
				if (read_status < 1)
				{
					return -1;
				}
				switch (reply[1])
				{
					case USE_LOCAL_STEP:
						sprintf(msg, "ROBKO 01 is set to use local step setting.\n");
						return PARSED_REPLY;
					case FULL_STEP:
						sprintf(msg, "Step mode is set to FULL_STEP.\n");
						return PARSED_REPLY;
					case HALF_STEP:
						sprintf(msg, "Step mode is set to HALF_STEP.\n");
						return PARSED_REPLY;
					default:
						return -1;
				}
			case SPEED_REPLY:
				pthread_mutex_lock(&serial_port_mutex);
				read_status = sp_blocking_read(ser_port, reply + 1, 2, READ_TIMEOUT);
				pthread_mutex_unlock(&serial_port_mutex);
				if (read_status < 2)
				{
					return -1;
				}
				speed = * (uint16_t *) (reply + 1);
				if (speed == USE_LOCAL_TIME)
				{
					sprintf(msg, "ROBKO 01 is set to use local speed setting.\n");
					return PARSED_REPLY;
				}
				else
				{
					sprintf(msg, "Time between steps is %d miliseconds.\n", speed);
					return PARSED_REPLY;
				}
			case GET_POS_REPLY:
			case SAVE_POS_REPLY:
				pthread_mutex_lock(&serial_port_mutex);
				read_status = sp_blocking_read(ser_port, reply + 1, 12, READ_TIMEOUT);
				pthread_mutex_unlock(&serial_port_mutex);
				if (read_status < 12)
				{
					return -1;
				}
				int16_t *reply_pos = (int16_t *) (reply + 1);
				//No hate pls
				sprintf(
					msg,
					"Current motor positions:\n"
					"\tMotor 0: %d\n"
					"\tMotor 1: %d\n"
					"\tMotor 2: %d\n"
					"\tMotor 3: %d\n"
					"\tMotor 4: %d\n"
					"\tMotor 5: %d\n",
					reply_pos[0],
					reply_pos[1],
					reply_pos[2],
					reply_pos[3],
					reply_pos[4],
					reply_pos[5]);
				if (reply[0] == SAVE_POS_REPLY)
				{
					save_position(reply_pos);
				}
				return PARSED_REPLY;
			case LAST_CMD:
				return PARSED_REPEAT;
			case ERROR_REPLY:
				pthread_mutex_lock(&serial_port_mutex);
				read_status = sp_blocking_read(ser_port, reply + 1, 1, READ_TIMEOUT);
				pthread_mutex_unlock(&serial_port_mutex);
				if (read_status < 1)
				{
					return -1;
				}
				switch (reply[1])
				{
					case UNKNOWN_CMD:
						sprintf(msg, "Error - unknown command!\n");
						return PARSED_REPLY;
					case FULL_RAM:
						sprintf(msg, "Error - MCU RAM is full!\n");
						return PARSED_REPLY;
				}
			default:;
		}
	}
	return -1;
}

int save_position(int16_t *position)
{
	FILE *output_fp = NULL;
	uint8_t i;
	output_fp = fopen(OUTPUT_FILE, "a");
	if (output_fp == NULL)
	{
		puts("ERROR opening output file!\n");
		return -1;
	}
	fputs("GOTO_POS ", output_fp);
	for (i = 0; i < 6; i++)
	{
		fprintf(output_fp, "%d ", position[i]);
	}
	fputs("\n", output_fp);
	fflush(output_fp);
	return 0;
}

int send_reply(int client_sockfd, char *reply_msg)
{
	int bytes_sent;
	bytes_sent = write(client_sockfd, reply_msg, strlen(reply_msg + 1));
	if (bytes_sent < 0)
	{
		puts("ERROR writing to socket\n");
		return -1;
	}
	printf("%d bytes sent.\n", bytes_sent);
	return 0;
}

// Secondary thread handles replies from the MCU to the client
// and repeating files
void *reply_thread(void *args)
{
	char reply_msg[REPLY_MSG_SIZE];
	thread_args_t *arguments = (thread_args_t *) args;
	while (1)
	{
		switch (parse_reply(arguments->ser_port, reply_msg))
		{
			case PARSED_REPEAT:
				if (arguments->script_file->repeat)
				{
					pthread_mutex_lock(&serial_port_mutex);
					execute_file(arguments->script_file, arguments->ser_port);
					pthread_mutex_unlock(&serial_port_mutex);					
				}
				break;
			case PARSED_REPLY:
				#ifdef EN_DEBUG
				fputs(reply_msg, stdout);
				fflush(stdout);
				#endif
				send_reply(arguments->sockfd, reply_msg);
				break;
			default:;
		}

	}
	return NULL;
}