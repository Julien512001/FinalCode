#include "../../include/Localization/aruco_gr5.h"


void *update_aruco(void *arg)
{   
    CtrlStruct* cvs = (CtrlStruct*)arg;
    while(1){
        printf("hello");
        wifi(cvs);

        // printf("%d, %d, %d\n", cvs->aruco->xR, cvs->aruco->yR, cvs->aruco->thetaR);
        printf("fin");
    }
}


void free_aruco(CtrlStruct *cvs)
{
    free(cvs->aruco);
}