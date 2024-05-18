#ifndef _LiDar_GR5_H_
#define _LiDar_GR5_H_ 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <thread>

 
#include "../../sdk/include/sl_lidar.h"
#include "../../sdk/include/sl_lidar_driver.h"
#include "../../sdk/include/rplidar.h"
#include "../../sdk/include/rplidar_driver.h"

#include "../main/CtrlStruct_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/Localization/opp_pos_gr5.h"
#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/calibration_gr5.h"

using namespace sl;

#ifndef _countof
	#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif



/// robot position (to update with odometry)

typedef struct Polar {
    double angle;
    double radius;
} Polar;

typedef struct Beacons
{
    Polar beacon1;
    Polar beacon2;
    Polar beacon3;
} Beacons;

typedef struct Cartesian
{
    double x;
    double y;
} Cartesian;

typedef struct InitBeacons
{
    Cartesian beacon1;
    Cartesian beacon2;
    Cartesian beacon3;
} InitBeacons;

typedef struct MyLidar {
    size_t count;
    sl_lidar_response_measurement_node_hq_t* nodes;
    ILidarDriver *lidar;
} MyLidar;

typedef struct Lidar {
    MyLidar *myLidar;
    InitBeacons *initBeacons;
    Beacons *beacons;
    Polar *opponents;
    Polar *myCentroids;
    int nbrCentroids;
    int nbrOpponents;
    double x;
    double y;
    double theta;
    double previous_x;
    double previous_y;
    double rad_opp;
    double ang_opp;
    double previous_theta;
} Lidar;

void* lidar(void* arg);


ILidarDriver* connectLidar();
void disconnectLidar(ILidarDriver* lidar);
void getData(CtrlStruct *cvs);
void getPositionLidarCalib(CtrlStruct *cvs);


void init_lidar(CtrlStruct *cvs);
void beaconsCalibration(CtrlStruct *cvs);


void get_position(CtrlStruct *cvs);
void guess(CtrlStruct *cvs, Cartesian* guessBeacons);
void updateBeacons(CtrlStruct *cvs, Cartesian *guessBeacons);

void makeCluster(CtrlStruct *cvs);
double getCentroid(double* myCluster, int countCluster);
bool getSTD(double* myCluster, int countCluster, double myCentroid, float std_th);
bool updateMemory(CtrlStruct *cvs);
void lostBeacons(CtrlStruct *cvs);
void ToTal(CtrlStruct *cvs);

void opponentTracking(CtrlStruct *cvs);

void free_lidar(CtrlStruct *cvs);


#endif