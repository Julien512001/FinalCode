#ifndef _INIT_POS_GR5_H_
#define _INIT_POS_GR5_H_ 
 

#include "../main/CtrlStruct_gr5.h"


/// robot position (to update with odometry)
typedef struct RobotPosition
{
	double x; ///< x position [m]
	double y; ///< y position [m]
	double theta; ///< robot orientation [rad]

	double last_t; ///< last time odometry was updated

} RobotPosition;

// function prototype
void set_init_position(int robot_id, RobotPosition *rob_pos);

void free_position(CtrlStruct *cvs);

#endif
