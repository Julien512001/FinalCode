#ifndef I2C_GR5_H
#define I2C_GR5_H

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/main/ctrl_main_gr5.h"
#include "../../include/Protocoles/I2C_gr5.h"

#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pigpio.h>


#define ADDR_REAR 0x01
#define ADDR_FRONT 0x02
#define BUFFER_SIZE 5

typedef struct I2C {
    int handle_front;
    int handle_rear;
} I2C;


void init_i2c(CtrlStruct *cvs);
void finish_i2c(CtrlStruct *cvs);
void send_i2c(CtrlStruct *cvs, int addr, double omega1, double omega2);
void receive_i2c(CtrlStruct *cvs, int addr);

void send_i2c_front(CtrlStruct *cvs, double omega1, double omega2);
void send_i2c_rear(CtrlStruct *cvs, double omega1, double omega2);



void sendData(double* speedArray);
void select_slave(int addr);
void open_bus();
void close_bus();

#endif