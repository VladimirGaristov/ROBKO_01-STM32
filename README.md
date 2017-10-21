# ROBKO_01-STM32
Software for controlling the bulgarian robot Робко01 (1986) with MCU board STM32L476RG and commands over serial communication by UART.  Developed as diplom work at ELSYS (ТУЕС), Sofia.

The software is divided in two modules:

* firmware for the microcontroller
* an example program for sending commands to the MCU over UART

Used libraries and APIs:
* ST Low-Layer drivers
* ST Hardware Abstraction Layer
* libserialport

ST's APIs make up most of the source code. The MCU firmware should run on all STM32xx boards with minimal changes.
