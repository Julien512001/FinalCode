#ifndef _CALIBRATION_GR5_H_
#define _CALIBRATION_GR5_H_ 
 
#include "../main/CtrlStruct_gr5.h"

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time to save

	int flag; ///< flag for calibration

} RobotCalibration;

// function prototype
void calibration(CtrlStruct *cvs);

void free_calibration(CtrlStruct *cvs);



#endif
