#ifndef STARTUP_GR5_H
#define STARTUP_GR5_H

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/main/GPIO_initialize_gr5.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pigpio.h>


void startUp(CtrlStruct *cvs);


#endif