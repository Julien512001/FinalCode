#ifndef UART_GR5_H
#define UART_GR5_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pigpio.h>

#include "../../include/main/CtrlStruct_gr5.h"


#define ADDR_FRONT "/dev/ttyACM0" 
#define ADDR_REAR "/dev/ttyACM1"

typedef struct UART
{
    int handle_front;
    int handle_rear;
} UART;


void init_uart(CtrlStruct *cvs);
void* uart(void *arg);
void finish_uart(CtrlStruct *cvs);
void free_uart(CtrlStruct *cvs);




#endif



