#ifndef _ARUCO_GR5_H_
#define _ARUCO_GR5_H_ 

#include "../main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/wifi_gr5.h"


typedef struct Aruco
{
    double xR;
    double yR;
    double thetaR;

    double xO;
    double yO;
    double thetaO;

    double theta1;
    double theta2;
} Aruco;

// function prototype

void *update_aruco(void *arg);
void free_aruco(CtrlStruct *cvs);



#endif
