# ROBKO_01-STM32
Software for controlling the bulgarian robot ROBKO 01 (1986) with MCU board STM32L476RG and commands over serial communication by UART. Includes client and server programs for remote control over the Internet. Developed as diplom work at ELSYS (ТУЕС), Sofia.

The project consists of:
* firmware for the microcontroller
* client and server programs for remote control over the Internet
* schematic and layout files for adapter PCB (EDA program: Proteus 8.4)
* STM CubeMX files
* documentation

Used libraries and APIs:
* ST Low-Layer drivers
* ST Hardware Abstraction Layer
* libserialport (for the server)

IDE and compiler:
* Ac6 System Workbench
* MCU GCC

ST's APIs make up most of the source code. The MCU firmware should run on most STM32xx Nucleo boards with small changes.

Instructions for remote control:
* manufacture the boards from the provided gerber files
* set solder bridges, according to inc/main.h
* assemble the hardware as shown in the documentation
* set the DIP switches to enable remote control
* import, compile and upload the MCU firmware
* modify the macro SERIAL_PORT in Remote_control/robko_server.c to match your serial port
* compile the server and client programs (makefiles provided)
* launch the server on a PC connected to the MCU
* launch the client, providing the server IP address as a parameter
* send commands to the MCU

Command syntax:\
MOV *motor* *num_of_steps*\
MOVE *motor0_steps* *motor1_steps* *motor2_steps* *motor3_steps* *motor4_steps* *motor5_steps*\
OFF *motor*\
OPEN_FILE *filename*\
KILL\
CLEAR\
FREEZE\
RESUME\
OPTO *mode*\
SET_STEP *step_mode*\
SET_SPEED *delay_btw_steps*

Words in *italics* are parameters. Negative numbers of steps cause movement in reverse. 'OFF 6' turns off all motors.
The file, specified by OPEN_FILE must be located in the same directory as the server executable and it must contain
commands in this syntax. KILL command performs emergency shutdown. CLEAR clears the command queue. FREEZE halts
the robot until it receives RESUME command. OPTO 1 will cause the next movement command to finish when the optical
sensor on the claw detects an object (or when it completes its steps). OPTO 2 acts the same way but it stops the
movement command when an object is no longer being detected. SET_STEP 1 will set the step mode to half step
and SET_STEP 2 to full step. SET_SPEED sets the delay between steps in miliseconds.
