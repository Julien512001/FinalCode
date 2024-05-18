#include "../../include/Localization/opp_pos_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include <math.h>

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// Nothing to do, the opponent position is computed only in the opp_pos structure
}

void free_Opponent(CtrlStruct *cvs)
{
	free(cvs->opp_pos);
}

