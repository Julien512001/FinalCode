#ifndef _CTRL_IO_H_
#define _CTRL_IO_H_

// number of micro-switches
#define NB_U_SWITCH 2

// maximum number of wheels
#define NB_WHEELS 4

// Number of sonar
#define NB_SONAR 4

// ID of the right,
// RF_ID : Right front
// LF_ID : Left front
// RR_ID : Right rear
// LR_ID : Left rear
enum{RF_ID, LF_ID, RR_ID, LR_ID};

// ID of the sonars
enum{RIGHT, LEFT1, FRONT, LEFT2}; // Changed to match the corresponding outputs of the sonars
enum{W1, W2, W3, W4};				// Names of the wheels

/// Controller inputs
typedef struct CtrlIn
{
	/*! \brief time reference
	 */
	double t; ///< time [s]

	double sonars[NB_SONAR];

	int StartSwitch;

	int U_Switch[6];

	double last_lidar_angle[1000] = {0};
	double last_lidar_radius[1000] = {0};

	int nb_lidar_data = 0;
	double last_lidar_update;

	double wheel_speeds[NB_WHEELS] = {0};


} CtrlIn;

typedef struct CtrlOut
{
	/*! \brief wheel commands
	*/
	double wheel_commands[NB_WHEELS];


} CtrlOut;





#endif
