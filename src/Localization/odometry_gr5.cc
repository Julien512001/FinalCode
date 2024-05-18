#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/I2C_gr5.h"
#include <math.h>

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	CtrlIn *inputs;         ///< controller inputs
	RobotPosition *rob_pos; ///< robot position

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	double t = inputs->t;
	double delta_t = 0.0;

	double vx_odo;
	double vy_odo;
	double omega_odo;

	double vx_I;
	double vy_I;

    double w1, w2, w3, w4;

	w1 = cvs->inputs->wheel_speeds[W1];
	w2 = cvs->inputs->wheel_speeds[W2];
	w3 = cvs->inputs->wheel_speeds[W3];
	w4 = cvs->inputs->wheel_speeds[W4];

	vx_odo = 	(w1 + w2 + w3 + w4) * r/4.0;
	vy_odo = 	(w1 - w2 - w3 + w4) * r/4.0;
	omega_odo = (-w1 + w2 - w3 + w4) * r/0.7422;
	
	delta_t = t - cvs->odometry->last_t;

	double theta = M_PI/180.0*cvs->odometry->theta;

	vx_I = (- vx_odo*sin(omega_odo*delta_t) + vy_odo*cos(omega_odo*delta_t));
    vy_I = (vx_odo*cos(omega_odo*delta_t) + vy_odo*sin(omega_odo*delta_t));

 	cvs->odometry->x += vx_I*delta_t;
	cvs->odometry->y += vy_I*delta_t;
	cvs->odometry->theta +=  180.0/M_PI * omega_odo*delta_t;

	cvs->odometry->last_t = t;
}

void free_odometry(CtrlStruct *cvs)
{
	free(cvs->odometry);
}
