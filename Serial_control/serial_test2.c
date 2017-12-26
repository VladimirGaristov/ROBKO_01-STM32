#include <stdio.h>
#include <libserialport.h>
#include <time.h>
#include <string.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 115200
#define STOP_BITS 2
#define WORD_LENGHT 8
#define PARITY SP_PARITY_ODD

void delay(int msec);
int port_configuration(struct sp_port **ser_port);
void transmit_string(struct sp_port *ser_port, char *s_out);

int main(void)
{
	char msg[3]={1, 0, 10};
	struct sp_port *serial_port;
	if(port_configuration(&serial_port))
		return -1;
	transmit_string(serial_port, msg);
	sp_free_port(serial_port);
	return 0;
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
	int i=0;
	int l=strlen(s_out);
	while(i<=2)
	{
		//scanf("%d", &l);
		sp_nonblocking_write(ser_port, s_out+i, 1);
		i++;
		//delay(10);
	}
	printf("Bytes written: %d\n", i);
}
