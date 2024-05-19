#include "../../include/strategy/strategy_gr5.h"

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */

void FSM_Def_init_pos_actuators(CtrlStruct *cvs){
	PanelWheelStop(cvs);
	DSSPanelDown(cvs);
	EntonnoirIN(cvs);
	EntonnoirBrasMiddle(cvs);
	DSSForkDown(cvs);
	FeetechIn(cvs);
	TapisStop(cvs);
	RunActuators(cvs);

	sleep(4);
}


void FSM_Do_Solar_panel(CtrlStruct *cvs) // Bleu = 0
{	
	PanelWheelStop(cvs);
	DSSPanelUp(cvs);
	RunActuators(cvs);

	sleep(1);

	DSSPanelDown(cvs);
	RunActuators(cvs);
	
	sleep(1);

	if (cvs->robot_id == ROBOT_Y)
	{
		PanelWheelYellow(cvs);
	}
	else 
	{
		PanelWheelBleu(cvs);
	}
	RunActuators(cvs);

	sleep(1);

	PanelWheelStop(cvs);
	DSSPanelUp(cvs);
	RunActuators(cvs);

	sleep(1);
}

void FSM_PrepInTake_Plant(CtrlStruct *cvs)
{	
	TapisIn(cvs);
	EntonnoirOUT(cvs);
	EntonnoirBrasSeq(cvs);
	DSSForkUp(cvs);
	RunActuators(cvs);

}

void FSM_StopPrepInTake_Plant(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	EntonnoirBrasMiddle(cvs);
	RunActuators(cvs);

	sleep(0.5);

	EntonnoirSTOP(cvs);
	RunActuators(cvs);

}

void FSM_Take_Plant(CtrlStruct *cvs){

	EntonnoirBrasMiddle(cvs);
	TapisStop(cvs);
	DSSForkUp(cvs);
	EntonnoirOUT(cvs);
	RunActuators(cvs);

	sleep(4);

	EntonnoirBrasSeq(cvs);
	TapisIn(cvs);
	RunActuators(cvs);

	sleep(7);

	EntonnoirBrasFerme(cvs);
	EntonnoirIN(cvs);
	TapisStop(cvs);
	RunActuators(cvs);

	sleep(1.8);
	
	EntonnoirSTOP(cvs);
	RunActuators(cvs);

	sleep(2);
}


void FSM_takeOff_Plant(CtrlStruct *cvs){
	DSSForkDown(cvs);
	EntonnoirBrasMiddle(cvs);
	EntonnoirOUT(cvs);
	RunActuators(cvs);

	sleep(0.5);

	ParralaxOut(cvs);
	FeetechOut(cvs);
	RunActuators(cvs);
}

void FSM_WheelSecurityAntiBlocking(CtrlStruct *cvs){
	ParralaxIn(cvs);
	RunActuators(cvs);

	sleep(0.5);

	DSSForkUp(cvs);
	ParralaxStop(cvs);
	RunActuators(cvs);
}

void FSM_PrepDropPlantGarden(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	EntonnoirBrasMiddle(cvs);
	RunActuators(cvs);

}

void FSM_DropPlantGarden(CtrlStruct *cvs){

	FeetechIn(cvs);
	RunActuators(cvs);

	sleep(2);

	ParralaxIn(cvs);
	RunActuators(cvs);
}

void FSM_Take_Pot_v1(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	TapisStop(cvs);
	RunActuators(cvs);

	sleep(5);

	EntonnoirOUT(cvs);
	RunActuators(cvs);

	sleep(5);

	EntonnoirIN(cvs);
	RunActuators(cvs);

	sleep(1);

	EntonnoirSTOP(cvs);
	RunActuators(cvs);

	sleep(5);
}


void FSM_Take_Pot_v2(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	TapisStop(cvs);
	RunActuators(cvs);

	sleep(5);


	EntonnoirOUT(cvs);
	RunActuators(cvs);

	sleep(5);

	TapisIn_Pot(cvs);
	RunActuators(cvs);

	sleep(15);

	EntonnoirIN(cvs);
	TapisStop(cvs);
	RunActuators(cvs);

	sleep(1);

	EntonnoirSTOP(cvs);
	RunActuators(cvs);
}

void main_strategy_bleu(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	int time = round(t);
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{

		case STRAT1:
			speed_regulation(cvs, -1, -1, -1);
			DSSPanelMid(cvs);
			DSSForkMid(cvs);
			RunActuators(cvs);
			sleep(0.5);
			PanelBrasDown(cvs);
			RunActuators(cvs);
			strat->state = STRAT2;
			break;

		case STRAT2:
			path_planning_update(cvs, 1.85, 1.80);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.01;
			if (cvs->path->target_reached) {
				speed_regulation(cvs, -1, -1, -1);
				PanelBrasUp(cvs);
				RunActuators(cvs);
				sleep(0.5);
				DSSPanelUp(cvs);
				RunActuators(cvs);
				strat->state = STRAT3;
				strat->t_strat = t;
			}
			break;

		case STRAT3:
			path_planning_update(cvs, 1.30, 1.80);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.05;
			if (cvs->path->target_reached){
				speed_regulation(cvs, -1, -1, -1);
				EntonnoirOUT(cvs);
				TapisIn(cvs);
				EntonnoirBrasSeq(cvs);
				RunActuators(cvs);
				strat->state = STRAT4;
				strat->t_strat = t;
			}
			break;
			

		case STRAT4:
			path_planning_update(cvs, 1.0, 2.80);
			speed_regulation(cvs, NULL, NULL, NULL);
			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;

		default:
			printf("Strategy error: unknown state: %d !\n", strat->state);
			exit(EXIT_FAILURE);
	}
}

void main_strategy_yellow(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	int time = round(t);
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{

		case STRAT1:
			speed_regulation(cvs, -1, -1, -1);
			DSSPanelMid(cvs);
			DSSForkMid(cvs);
			RunActuators(cvs);
			sleep(0.5);
			PanelBrasDown(cvs);
			RunActuators(cvs);
			strat->state = STRAT2;
			break;

		case STRAT2:
			path_planning_update(cvs, 0.15, 1.80);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.01;
			if (cvs->path->target_reached) {
				speed_regulation(cvs, -1, -1, -1);
				PanelBrasUp(cvs);
				RunActuators(cvs);
				sleep(0.5);
				DSSPanelUp(cvs);
				RunActuators(cvs);
				strat->state = STRAT3;
				strat->t_strat = t;
			}
			break;

		case STRAT3:
			path_planning_update(cvs, 0.70, 1.80);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.05;
			if (cvs->path->target_reached){
				speed_regulation(cvs, -1, -1, -1);
				EntonnoirOUT(cvs);
				TapisIn(cvs);
				EntonnoirBrasSeq(cvs);
				RunActuators(cvs);
				strat->state = STRAT4;
				strat->t_strat = t;
			}
			break;
			

		case STRAT4:
			path_planning_update(cvs, 1.0, 2.80);
			speed_regulation(cvs, NULL, NULL, NULL);
			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;

		default:
			printf("Strategy error: unknown state: %d !\n", strat->state);
			exit(EXIT_FAILURE);
	}
}




void free_strategy(CtrlStruct *cvs)
{
	free(cvs->strat);
}
