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
#define BUFFER_SIZE 256
#define SERVER_IP_ADDR "192.168.1.4"

void error(char *msg);

void error(char *msg)
{
    perror(msg);
    exit(-1);
}

int main(void)
{
    int sockfd, n;
    struct sockaddr_in serv_addr;
    char buffer[BUFFER_SIZE];
    //Open a socket
    sockfd = socket(AF_INET, SOCK_PROTOCOL, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    memset(&serv_addr, 0, sizeof(serv_addr));
    //Set the server address and port
    serv_addr.sin_family = AF_INET;
    if (!(inet_aton(SERVER_IP_ADDR, (struct in_addr *) &serv_addr.sin_addr.s_addr)))
	error("ERROR converting server address");
    serv_addr.sin_port = htons(SOCK_PORT);
    printf("Attempting to connect...\n");
    //Connect to the server
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting\n");
    printf("Please enter the message: ");
    memset(buffer, 0, BUFFER_SIZE);
    scanf("%s", buffer);
    //Send data to the server
    //add syntax checks
    n = write(sockfd, buffer, strlen(buffer));
    if (n < 0)
         error("ERROR writing to socket\n");
    printf("%d bytes sent.\n", n);
    memset(buffer, 0, BUFFER_SIZE);
    //Receive data from the server
    n = read(sockfd, buffer, BUFFER_SIZE);
    if (n < 0)
         error("ERROR reading from socket\n");
    printf("%d bytes received: '%s'\n", n, buffer);
    //add shutdown, close? and loop
    shutdown(sockfd, SHUT_WR);

    return 0;
}
