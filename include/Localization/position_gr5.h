#ifndef _POSITION_GR5_H_
#define _POSITION_GR5_H_ 

#include "../main/CtrlStruct_gr5.h"



typedef struct Position
{
    double x;
    double y;
    double theta;
    double last_t;
    int flagUpdate;
    int flagRotation;

} Position;

// function prototype
void update_robotPosition(CtrlStruct *cvs);

void update_robotPosition_aruco(CtrlStruct *cvs);

void free_robotPosition(CtrlStruct *cvs);


#endif
