#ifndef _ODOMETRY_GR5_H_
#define _ODOMETRY_GR5_H_ 

#include "../main/CtrlStruct_gr5.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

typedef struct Speed {
    double w1;
    double w2;
    double w3;
    double w4;
} Speed;

typedef struct Odometry
{
    double x;
    double y;
    double theta;
    double last_t;
    Speed *speed;
} Odometry;

// function prototype
void update_odometry(CtrlStruct *cvs);
void free_odometry(CtrlStruct *cvs);




#endif
