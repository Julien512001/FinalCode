#ifndef _STRATEGY_GR5_H_
#define _STRATEGY_GR5_H_

#include "../main/CtrlStruct_gr5.h"
#include "../../include/strategy/strategy_gr5.h"
#include "../../include/path/path_planning_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/position_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/Actuators/Actuators_gr5.h"
#include "../../include/LCD/LCD_gr5.h"
#include <math.h>

// strategy main states
enum {STRAT_STATE_1, STRAT_STATE_2, STRAT_STATE_3, STRAT_STATE_4, STRAT_STATE_5, STRAT_STATE_6, STRAT_STATE_7, STRAT_STATE_8, STRAT_STATE_9, STRAT_STATE_10, STRAT_STATE_11, STRAT_STATE_12, STRAT_STATE_13, STRAT_STATE_14, STRAT_STATE_15, STRAT_STATE_16, STRAT_STATE_17, STRAT_STATE_18, STRAT_STATE_19, STRAT_STATE_20, STRAT_STATE_21};

/// strategy
typedef struct Strategy
{
	int state; ///< main state of the strategy
	double t_strat;
	
} Strategy;

// function prototype
void FSM_Def_init_pos_actuators(CtrlStruct *cvs);

void FSM_Do_Solar_panel(CtrlStruct *cvs);

void FSM_PrepInTake_Plant(CtrlStruct *cvs);

void FSM_StopPrepInTake_Plant(CtrlStruct *cvs);

void FSM_Take_Plant(CtrlStruct *cvs);

void takeOff_Plant(CtrlStruct *cvs);

void FSM_WheelSecurityAntiBlocking(CtrlStruct *cvs);

void FSM_PrepDropPlantGarden(CtrlStruct *cvs);

void FSM_DropPlantGarden(CtrlStruct *cvs);

void FSM_Take_Pot_v1(CtrlStruct *cvs);

void FSM_Take_Pot_v2(CtrlStruct *cvs);

// function prototype
void main_strategy_jaune(CtrlStruct *cvs);

void main_strategy_bleu(CtrlStruct *cvs);

void main_strategy_sonar(CtrlStruct *cvs);

void stratLidar(CtrlStruct *cvs);

void free_strategy(CtrlStruct *cvs);

void BlueStrat(CtrlStruct *cvs);

void YellowStrat(CtrlStruct *cvs);

void Opposite_base_Strat(CtrlStruct *cvs);

void Get_out_Strat(CtrlStruct *cvs);




#endif

