#include "../../include/regulation/speed_regulation_gr5.h"

/*! \brief wheels speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */

void speed_regulation(CtrlStruct *cvs, double vx, double vy, double omega)
{
	double omega1;
	double omega2;

	double omega3;
	double omega4;
	
	if (vx == NULL) {
		vx = cvs->path->pathSpeed->vx;
	}
	if (vy == NULL) {
		vy = cvs->path->pathSpeed->vy;
	} 
	if (omega == NULL) {
		omega = cvs->path->pathSpeed->omega;
	} 

	if (vx == -1) {
		vx = 0.0;
	}
	if (vy == -1) {
		vy = 0.0;
	} 
	if (omega == -1) {
		omega = 0.0;
	}

	omega1 = (vx - vy - 0.18555 * omega)/0.024;
	omega2 = (vx + vy + 0.18555 * omega)/0.024;
	omega3 = (vx + vy - 0.18555 * omega)/0.024;
	omega4 = (vx - vy + 0.18555 * omega)/0.024;
	
	cvs->outputs->wheel_commands[W1] = omega1;
	cvs->outputs->wheel_commands[W2] = omega2;
	cvs->outputs->wheel_commands[W3] = omega3;
	cvs->outputs->wheel_commands[W4] = omega4;
}

void free_speedRegulation(CtrlStruct *cvs)
{
	free(cvs->sp_reg);
}

