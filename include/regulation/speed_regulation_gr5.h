#ifndef _SPEED_REGULATION_GR5_H_
#define _SPEED_REGULATION_GR5_H_
 
#include "../main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/I2C_gr5.h"
#include "../../include/path/path_planning_gr5.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>



/// speed regulation
typedef struct SpeedRegulation
{
	double last_t; ///< last time the speed regulation was updated
	
	
} SpeedRegulation;

// function prototype
void speed_regulation(CtrlStruct *cvs, double vx, double vy, double omega);

void free_speedRegulation(CtrlStruct *cvs);


#endif
