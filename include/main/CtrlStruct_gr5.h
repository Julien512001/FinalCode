#ifndef _CTRL_STRUCT_GR5_H_
#define _CTRL_STRUCT_GR5_H_


#include "../../include/ctrl_interface/ctrl_io.h"

#include <stdlib.h>
#include <stdio.h>


#define r 0.024



/// main states
enum {CALIB_STATE, WAIT_INIT_STATE, RUN_STATE, STOP_END_STATE, NB_MAIN_STATES, BACK_HOME_STATE};

/// robot IDs
enum {ROBOT_B, ROBOT_R, ROBOT_Y, ROBOT_W, NB_ROBOTS};

/// teams
enum {TEAM_A, TEAM_B, NB_TEAMS};

#define NUMBER_OF_REGISTER 4

// forward declaration
typedef struct RobotPosition RobotPosition;
typedef struct SpeedRegulation SpeedRegulation;
typedef struct RobotCalibration RobotCalibration;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct PathPlanning PathPlanning;
typedef struct Strategy Strategy;
typedef struct Lidar Lidar;
typedef struct Odometry Odometry;
typedef struct Position Position;
typedef struct SPI SPI;
typedef struct I2C I2C;
typedef struct UART UART;
typedef struct Aruco Aruco;
typedef struct Actuator Actuator;

/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs;

	RobotPosition *rob_pos; ///< robot position
	OpponentsPosition *opp_pos; ///< opponents position
	SpeedRegulation *sp_reg; ///< speed regulation
	RobotCalibration *calib; ///< calibration
	PathPlanning *path; ///< path-planning
	Strategy *strat; ///< strategy
	Lidar *lidar;
	Odometry *odometry;
	Position *position;
	SPI *spi;
	I2C *i2c;
	UART *uart;
	Aruco *aruco;
	Actuator *actuator;

	int main_state; ///< main state
	int robot_id;   ///< ID of the robot
	int startPosition;
	int team_id;    ///< ID of the team


} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);


#endif
