#include "../../include/Localization/calibration_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"

// calibration states
enum {CALIB_START, CALIB_STATE_1, CALIB_STATE_2, CALIB_STATE_3, CALIB_FINISH};

/*! \brief calibration of the robot to get its actual position
 * 
 * \param[in,out] cvs controller main structure
 */

void calibration(CtrlStruct *cvs)
{
	// variables declaration
	double t;

	CtrlIn *inputs; ///< controller inputs
	RobotCalibration *calib; ///< calibration structure
	RobotPosition *rob_pos;  ///< robot position (to calibrate)

	// variables initialization
	inputs  = cvs->inputs;
	calib   = cvs->calib;
	rob_pos = cvs->rob_pos;
	
	t = inputs->t;

	// finite state machine (FSM)
	switch (calib->flag)
	{	
		case CALIB_START: // start calibration
			speed_regulation(cvs, -1, -1, -1); // not moving

			calib->flag = CALIB_STATE_1; // directly go to state CALIB_STATE_1
			calib->t_flag = t; // save current time
			break;

		case CALIB_STATE_1: // first calibration state
			beaconsCalibration(cvs);					// Determine the position of each beacons w.r.t. the robot
			getPositionLidarCalib(cvs);					// Compare each precomputed beacons positions with actual measurement

			cvs->odometry->x = cvs->lidar->x;
			cvs->odometry->y = cvs->lidar->y;
			cvs->odometry->theta = cvs->lidar->theta;
		
			cvs->rob_pos->x = cvs->lidar->x;
			cvs->rob_pos->y = cvs->lidar->y;
			cvs->rob_pos->theta = cvs->lidar->theta;

			cvs->main_state = RUN_STATE;
		
			break;

		default:
			printf("Error: unknown calibration state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
	}
}

void free_calibration(CtrlStruct *cvs)
{
	free(cvs->calib);
} 