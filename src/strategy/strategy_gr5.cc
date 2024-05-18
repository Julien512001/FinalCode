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

void main_strategy_jaune(CtrlStruct *cvs)
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
		case STRAT_STATE_1: // On se met en position panneau
			DSSPanelMid(cvs);
			DSSForkMid(cvs);
			RunActuators(cvs);
			sleep(0.5);
			PanelBrasDown(cvs);
			RunActuators(cvs);
			if (t > 1.5){strat->state = STRAT_STATE_5;}
			break;

		case STRAT_STATE_2: // on utilise pas les état de 2 a 4
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73);
			FSM_Do_Solar_panel(cvs);
			strat->state = STRAT_STATE_3;

			break;

		case STRAT_STATE_3: 
			speed_regulation(cvs, 0.2, -1, -1);
			if (t > 15){strat->state = STRAT_STATE_4;}
			break;

		case STRAT_STATE_4: 
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73);
			FSM_Do_Solar_panel(cvs);
			break;
		case STRAT_STATE_5: // On avance pour faire les 6 1er panneau
			speed_regulation(cvs, -0.2, -1, -1);
			if (t>11){
				strat->state = STRAT_STATE_6;
			}
			break;

		case STRAT_STATE_6:
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73); // on attend la commande d'arret des roues
			PanelBrasUp(cvs);
			RunActuators(cvs);
			sleep(0.5);
			DSSPanelUp(cvs);
			RunActuators(cvs);
			strat->state = STRAT_STATE_7;
			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_7:
			speed_regulation(cvs, 0.2, -0.2, -1); // On va en diago en arrière
			if (t > 15){strat->state = STRAT_STATE_8;}
			break;

		case STRAT_STATE_8:
			speed_regulation(cvs, -1, -1, -1); // On s'arrete
			sleep(1.73);
			strat->state = STRAT_STATE_9;
			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_9: // On se prépare a prendre les plantes
			EntonnoirOUT(cvs);
			TapisIn(cvs);
			EntonnoirBrasSeq(cvs);
			RunActuators(cvs);
			speed_regulation(cvs, -0.2, -1, -1); // On fonce dans les plantes
			if (t>21){strat->state = STRAT_STATE_10;}
			break;

		case STRAT_STATE_10:
			speed_regulation(cvs, -1, -1, -1); // On s'arrete pcq on a les plantes normalement
			sleep(1.73);
			EntonnoirIN(cvs); // On referme unp eu le bras
			RunActuators(cvs);
			sleep(0.5);
			EntonnoirSTOP(cvs);
			DSSForkDown(cvs); // On descend les fourche et en enfourche les plantes
			ParralaxOut(cvs);
			RunActuators(cvs);
			sleep(10);
			DSSForkUp(cvs); // On les lèves pour prépare la mise en jardinière
			strat->state = STRAT_STATE_11;

			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_11:
			speed_regulation(cvs, -1, -1, -1); 
			TapisStop(cvs);
			EntonnoirBrasMiddle(cvs);
			EntonnoirIN(cvs);
			RunActuators(cvs);
			break;	
	}
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
		case STRAT_STATE_1: // On se met en position panneau
			DSSPanelMid(cvs);
			DSSForkMid(cvs);
			RunActuators(cvs);
			sleep(0.5);
			PanelBrasDown(cvs);
			RunActuators(cvs);
			if (t > 1.5){strat->state = STRAT_STATE_5;}
			break;

		case STRAT_STATE_2: // on utilise pas les état de 2 a 4
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73);
			FSM_Do_Solar_panel(cvs);
			strat->state = STRAT_STATE_3;

			break;

		case STRAT_STATE_3: 
			speed_regulation(cvs, 0.2, -1, -1);
			if (t > 15){strat->state = STRAT_STATE_4;}
			break;

		case STRAT_STATE_4: 
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73);
			FSM_Do_Solar_panel(cvs);
			break;
		case STRAT_STATE_5: // On avance pour faire les 6 1er panneau
			speed_regulation(cvs, 0.2, -1, -1);
			if (t>11){
				strat->state = STRAT_STATE_6;
			}
			break;

		case STRAT_STATE_6:
			speed_regulation(cvs, -1, -1, -1);
			sleep(1.73); // on attend la commande d'arret des roues
			PanelBrasUp(cvs);
			RunActuators(cvs);
			sleep(0.5);
			DSSPanelUp(cvs);
			RunActuators(cvs);
			strat->state = STRAT_STATE_7;
			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_7:
			speed_regulation(cvs, -0.2, 0.2, -1); // On va en diago en arrière
			if (t > 15){strat->state = STRAT_STATE_8;}
			break;

		case STRAT_STATE_8:
			speed_regulation(cvs, -1, -1, -1); // On s'arrete
			sleep(1.73);
			strat->state = STRAT_STATE_9;
			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_9: // On se prépare a prendre les plantes
			EntonnoirOUT(cvs);
			TapisIn(cvs);
			EntonnoirBrasSeq(cvs);
			RunActuators(cvs);
			speed_regulation(cvs, 0.2, -1, -1); // On fonce dans les plantes
			if (t>21){strat->state = STRAT_STATE_10;}
			break;

		case STRAT_STATE_10:
			speed_regulation(cvs, -1, -1, -1); // On s'arrete pcq on a les plantes normalement
			sleep(1.73);
			EntonnoirIN(cvs); // On referme unp eu le bras
			RunActuators(cvs);
			sleep(0.5);
			EntonnoirSTOP(cvs);
			DSSForkDown(cvs); // On descend les fourche et en enfourche les plantes
			ParralaxOut(cvs);
			RunActuators(cvs);
			sleep(10);
			DSSForkUp(cvs); // On les lèves pour prépare la mise en jardinière
			strat->state = STRAT_STATE_11;

			//FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_11:
			speed_regulation(cvs, -1, -1, -1); 
			TapisStop(cvs);
			EntonnoirBrasMiddle(cvs);
			EntonnoirIN(cvs);
			RunActuators(cvs);
			break;	
		
		case STRAT_STATE_12:
			speed_regulation(cvs, -1, -1, -1); // 6eme panneau
			FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_13:
			speed_regulation(cvs, 0.2, -1, -1);
			break;
		
		case STRAT_STATE_14:
			speed_regulation(cvs, -1, -1, -1); // 5eme panneau
			FSM_Do_Solar_panel(cvs);
			break;

		case STRAT_STATE_15:
			speed_regulation(cvs, 0.2, -1, -1);
			break;
	}
}

void main_strategy_sonar(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			// sonar(cvs);
			// if ((cvs->inputs->sonars[FRONT] <= 20) | (cvs->inputs->sonars[LEFT1] <= 20) | (cvs->inputs->sonars[RIGHT] <= 20)){
			// 	speed_regulation(cvs, -1, -1, -1);
			// }else {
			// 	path_planning_update(cvs, 0.0, 1.0); //1.82593, 1.0974 pour le 1er panneau () le plus a droite
			// 	speed_regulation(cvs, NULL, NULL, NULL);
			// }

			path_planning_update(cvs, 1.5, 2.0);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.1;
			if (cvs->path->target_reached) {
				cvs->position->flagUpdate = 1;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				cvs->main_state = STOP_END_STATE;
				FSM_Do_Solar_panel(cvs);
				printf("Panneau Fait 1\n");
				strat->state = STRAT_STATE_2;
				//cvs->main_state = STRAT_STATE_2;
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_2:
			path_planning_update(cvs, 1.0, 2.6);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.05;
			if (cvs->path->target_reached) {
				printf("State 2\n");
				//strat->state = STRAT_STATE_3;
				cvs->main_state = STOP_END_STATE;

				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_3:
			path_planning_update(cvs, 1.8, 1.5474);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 3\n");
				strat->state = STRAT_STATE_4;
				//cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_4:
			speed_regulation(cvs, -1, -1, -1);
			/*
			path_planning_update(cvs, 0.2, 0.2);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 4\n");
				cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			*/
			break;
	}
}

void stratLidar(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			
			
			speed_regulation(cvs, -1, -1, -1);
			strat->state = STRAT_STATE_2;
			strat->t_strat = t;

			break;

		case STRAT_STATE_2:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t-strat->t_strat >= 3.0) {
				strat->state = STRAT_STATE_3;
				//cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_3:
			speed_regulation(cvs, -1, -0.4, -1);
			if (t-strat->t_strat >= 3.0) {
				//strat->state = STRAT_STATE_4;
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;
		
		case STRAT_STATE_4:
			speed_regulation(cvs, -1, -1, -1);
			if (t-strat->t_strat >= 7.0) {
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;
	}
}

void BlueStrat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;



	switch (strat->state)
	{
		case STRAT_STATE_1:
			speed_regulation(cvs, -1 , -1, -1);
			
			strat->state = STRAT_STATE_2;
			strat->t_strat = t;
			break;

		case STRAT_STATE_2:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t - strat->t_strat > 1.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				strat->state = STRAT_STATE_3;
				strat->t_strat = t;
				printf("t_strat:%f\n", strat->t_strat);
				printf("t:%f\n", t);
			}
			break;

		case STRAT_STATE_3:
			speed_regulation(cvs, 0.2, -1, -1);
			printf("t_diff:%f\n", t - strat->t_strat);

			if (t - strat->t_strat > 1.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				sleep(2);
				strat->state = STRAT_STATE_4;
				strat->t_strat = t;
				printf("t_strat:%f\n", strat->t_strat);
				printf("t:%f\n", t);
				printf("t_diff:%f\n", t - strat->t_strat);
			}

			break;

		case STRAT_STATE_4:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t - strat->t_strat > 2.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				sleep(1);
				strat->state = STRAT_STATE_5;
				strat->t_strat = t;
			}
			break;
		
		case STRAT_STATE_5:
			path_planning_update(cvs, 0.3, 0.2);
			speed_regulation(cvs, NULL, NULL, NULL);
			
			cvs->path->sigma_th = 0.1;
			if (cvs->path->target_reached) {
				speed_regulation(cvs, -1, -1, -1);	
				cvs->main_state = STOP_END_STATE;
			}
			break;
			
	}
}

void YellowStrat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:

			break;

		case STRAT_STATE_2:

			break;

		case STRAT_STATE_3:

			break;

		case STRAT_STATE_4:
			break;
			
	}
}


void Opposite_base_Strat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			speed_regulation(cvs, 0.2, -1, -1);
			sleep(7.0);
			strat->state = STRAT_STATE_2;
			speed_regulation(cvs, -1, -1, -1);
			increment_score(11);
			break;

		case STRAT_STATE_2:
			FSM_Do_Solar_panel(cvs);
			cvs->main_state = STOP_END_STATE;
			break;
	}
}

void Get_out_Strat(CtrlStruct *cvs)
{
	speed_regulation(cvs, 0.2, -1, -1);
	sleep(2.5);
	cvs->main_state = STOP_END_STATE;
	increment_score(1);
}


void free_strategy(CtrlStruct *cvs)
{
	free(cvs->strat);
}
