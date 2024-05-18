#ifndef GPIO_INITIALIZE_GR5_H
#define GPIO_INITIALIZE_GR5_H

#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>


#define START_UP_PIN 14
/*
#define SDA_PIN 2
#define SCL_PIN 3
*/

void init_GPIO();

void finish_GPIO();


#endif
