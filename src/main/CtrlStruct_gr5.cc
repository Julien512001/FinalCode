#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/path/path_planning_gr5.h"
#include "../../include/strategy/strategy_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/Localization/calibration_gr5.h"
#include "../../include/Localization/opp_pos_gr5.h"
#include "../../include/Localization/position_gr5.h"
#include "../../include/Sonars/Sonars_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"
#include "../../include/Protocoles/uart_gr5.h"
#include "../../include/Localization/aruco_gr5.h"
#include "../../include/LCD/LCD_gr5.h"


/*! \brief initialize the controller structure
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */

CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
	CtrlStruct *cvs;

	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));

	cvs->inputs  = inputs;
	cvs->outputs = outputs;

	// Communication protocols
	cvs->spi = (SPI*) malloc(sizeof(SPI));
	cvs->spi->rxData = (int *) malloc(4*sizeof(int));

	cvs->spi->spi_handle = 0.0;


	cvs->i2c = (I2C*) malloc(sizeof(I2C));

	cvs->uart = (UART*) malloc(sizeof(UART));
 
	// init of Aruco Struct

	cvs->aruco = (Aruco*) malloc(sizeof(Aruco));

	cvs->aruco->xR = 0.0;
	cvs->aruco->yR = 0.0;
	cvs->aruco->thetaR = 0.0;

	// states
	cvs->main_state = 0;
	
	// IDs
	cvs->robot_id = ROBOT_B;
	cvs->team_id  = TEAM_A;

	// robot position
	cvs->rob_pos = (RobotPosition*) malloc(sizeof(RobotPosition));

	cvs->rob_pos->x = 0.0;
	cvs->rob_pos->y = 0.0;

	cvs->rob_pos->theta  = 0.0;
	cvs->rob_pos->last_t = 0.0;

	// Calibration
	cvs->calib = (RobotCalibration*) malloc(sizeof(RobotCalibration));

	cvs->calib->flag = 0;
	cvs->calib->t_flag = 0.0;

	// Position

	cvs->position = (Position*) malloc(sizeof(Position));

	cvs->position->x = 0.0;
	cvs->position->y = 0.0;
	cvs->position->theta = 0.0;
	cvs->position->last_t = 0.0;

	// speed regulation
	cvs->sp_reg = (SpeedRegulation*) malloc(sizeof(SpeedRegulation));

	cvs->sp_reg->last_t = 0.0;

	// strategy
	cvs->strat = (Strategy*) malloc(sizeof(Strategy));

	cvs->strat->state = STRAT_STATE_1;
	cvs->strat->t_strat = 0.0;

	// opponents position
	cvs->opp_pos = (OpponentsPosition*) malloc(sizeof(OpponentsPosition));

	cvs->opp_pos->x = 0.0;
	cvs->opp_pos->y = 0.0;

	// path-planning
	cvs->path = (PathPlanning*) malloc(sizeof(PathPlanning));
	cvs->path->pathSpeed = (PathSpeed*) malloc(sizeof(PathSpeed));

	cvs->path->F_tot_x = 0.0;
	cvs->path->F_tot_y = 0.0;

	cvs->path->pathSpeed->vx = 0.0;
	cvs->path->pathSpeed->vy = 0.0;
	cvs->path->pathSpeed->omega = 0.0;

	// Lidar

	cvs->lidar = (Lidar*) malloc(sizeof(Lidar));
	cvs->lidar->beacons = (Beacons*) malloc(sizeof(Beacons));
	cvs->lidar->initBeacons = (InitBeacons*) malloc(sizeof(InitBeacons));
	cvs->lidar->myLidar = (MyLidar*) malloc(sizeof(MyLidar));

	sl_lidar_response_measurement_node_hq_t nodes[8192];
	cvs->lidar->myLidar->nodes = nodes;
    size_t count = _countof(nodes);
	cvs->lidar->myLidar->count = count;

	cvs->lidar->nbrOpponents = 0;

	// Odometry

	cvs->odometry = (Odometry*) malloc(sizeof(Odometry));
	cvs->odometry->speed = (Speed*) malloc(sizeof(Speed));
	cvs->odometry->x = 0.0;
	cvs->odometry->y = 0.0;
	cvs->odometry->theta = 0.0;

	cvs->odometry->speed->w1 = 0.0;
	cvs->odometry->speed->w2 = 0.0;
	cvs->odometry->speed->w3 = 0.0;
	cvs->odometry->speed->w4 = 0.0;

	//Actuator
	cvs->actuator = (Actuator*) malloc(sizeof(Actuator));
	cvs->actuator->message_array = (uint32_t*) calloc(20,sizeof(uint32_t));

	return cvs;
}

/*! \brief release controller main structure memory
 * 
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	free(cvs);
}

