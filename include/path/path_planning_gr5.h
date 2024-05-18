#ifndef _PATH_PLANNING_GR5_H_
#define _PATH_PLANNING_GR5_H_

#include "../main/CtrlStruct_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/// path-planning main structure
typedef struct PathSpeed 
{
	double vx;
	double vy;
	double omega;
} PathSpeed;

typedef struct PathPlanning
{
	double F_tot_x; ///< dummy variable
	double F_tot_y;
	double U_att;
	double U_rep;
	double theta_force;
	double sigma_th;			// Distance from the target that turn target_reached to "true"
	bool target_reached;
	bool final_target_reached;
	PathSpeed *pathSpeed;
} PathPlanning;

typedef struct PredictedPath
{
	double x_predicted;
	double y_predicted;
} PredictedPath;

// function prototype

void predictive_path_planner(CtrlStruct *cvs, double x_target, double y_target);
double path_prediction(CtrlStruct *cvs, double x_target, double y_target, double x_pred, double y_pred);
double minimalDistanceBis(double x1, double y1, double x2, double y2, double xR, double yR);

void path_planning_update(CtrlStruct *cvs, double x_target, double y_target);
double euclidean(double xR, double yR, double x_target, double y_target);
double minimalDistance(double *point1, double *point2, double xR, double yR, double* xy_array);
void makeHeatmap(CtrlStruct *cvs);

void free_pathPlanning(CtrlStruct *cvs);



#endif
