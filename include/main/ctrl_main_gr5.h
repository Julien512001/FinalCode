#ifndef _CTRL_MAIN_GR5_H_
#define _CTRL_MAIN_GR5_H_


#include <stdio.h>
#include <stdlib.h>

#include <cstdio>
#include <math.h>

#include "../main/CtrlStruct_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/path/path_planning_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/strategy/strategy_gr5.h"
#include "../../include/Localization/calibration_gr5.h"
#include "../../include/Localization/opp_pos_gr5.h"
#include "../../include/Localization/position_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"
#include "../../include/MicroSwitch/startUp_gr5.h"
#include "../../include/Actuators/Actuators_gr5.h"
#include "../../include/Sonars/Sonars_gr5.h"
#include "../../include/Protocoles/uart_gr5.h"

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

#endif
