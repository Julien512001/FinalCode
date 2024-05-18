#include "../../include/Localization/position_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/path/path_planning_gr5.h"
#include "../../include/Localization/aruco_gr5.h"
#include <math.h>




/*! \brief update the robot position using lidar and odometry
 * 
 * \param[in,out] cvs controller main structure
 */

void update_robotPosition(CtrlStruct *cvs)
{
    double t;
    double last_t;
    CtrlIn *inputs;

    inputs = cvs->inputs;

    t = inputs->t;
    last_t = cvs->position->last_t;

    update_odometry(cvs);

    switch (cvs->position->flagRotation)
    {
    case 0:
        if (cvs->position->flagUpdate) {
            double xR = cvs->lidar->x;
            double yR = cvs->lidar->y;
            double thetaR = cvs->lidar->theta;

            cvs->rob_pos->x = xR;
            cvs->rob_pos->y = yR;
            cvs->rob_pos->theta = thetaR;

            cvs->odometry->x = xR;
            cvs->odometry->y = yR;
            cvs->odometry->theta = thetaR;
        }
        
        cvs->rob_pos->x = cvs->odometry->x;
        cvs->rob_pos->y = cvs->odometry->y;
        cvs->rob_pos->theta = cvs->odometry->theta;
        break;
        
    case 1:
        cvs->rob_pos->x = cvs->odometry->x;
        cvs->rob_pos->y = cvs->odometry->y;
        cvs->rob_pos->theta = cvs->odometry->theta;
        break;
    
    default:
        break;
    }
}

void update_robotPosition_aruco(CtrlStruct *cvs)
{
    double t;
    double last_t;
    CtrlIn *inputs;

    inputs = cvs->inputs;

    t = inputs->t;
    last_t = cvs->position->last_t;
    
    if (cvs->position->flagUpdate) {
        double xR = cvs->lidar->x;
        double yR = cvs->lidar->y;
        double thetaR = cvs->lidar->theta;
        
        cvs->odometry->x = (abs(cvs->odometry->x - xR) > 0.05) ? cvs->odometry->x : xR;
        cvs->odometry->y = (abs(cvs->odometry->y - yR) > 0.05) ? cvs->odometry->y : yR;
        cvs->odometry->theta = (abs(cvs->odometry->theta - thetaR) > 10) ? cvs->odometry->theta : thetaR;
        
       cvs->position->flagUpdate = 0;
    }
    
    cvs->rob_pos->x = cvs->odometry->x;
    cvs->rob_pos->y = cvs->odometry->y;
    cvs->rob_pos->theta = cvs->odometry->theta;
}

void free_robotPosition(CtrlStruct *cvs)
{
	free(cvs->position);
}
