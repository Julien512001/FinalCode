#ifndef _OPP_POS_GR5_H_
#define _OPP_POS_GR5_H_ 
 
#include "../main/CtrlStruct_gr5.h"


/// opponents position
typedef struct OpponentsPosition
{
	double x; ///< x position of opponents [m]
	double y; ///< y position of opponents [m]

	int nb_opp; ///< number of opponents

} OpponentsPosition;

// function prototype
void opponents_tower(CtrlStruct *cvs);

void free_Opponent(CtrlStruct *cvs);



#endif
