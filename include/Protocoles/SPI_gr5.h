#ifndef SPI_GR5_H
#define SPI_GR5_H


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "../../include/main/CtrlStruct_gr5.h"


#include <pigpio.h>


#define SPI_CHANNEL 0
#define SPI_SPEED   1000000
#define SPI_FLAGS   0




typedef struct SPI
{
    int spi_handle;
    int *rxData;
};


void init_spi(CtrlStruct *cvs);
void send_spi(CtrlStruct *cvs, uint32_t message, uint8_t addr);
void reveive_spi(CtrlStruct *cvs, int addr);
void free_spi(CtrlStruct *cvs);




#endif



