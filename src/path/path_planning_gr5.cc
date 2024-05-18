#include "../../include/path/path_planning_gr5.h"



/*! \brief update the path-planning algorithm
 * 
 * \param[in,out] cvs controller main structure
 */

const double cartesianCirclesObs[6][2] = {{0.5,1.5},
                                 {0.7,2.0},
                                 {1.3,2.0},
                                 {1.50,1.50},
                                 {1.30,1.00},
                                 {0.70,1.00}};
const int circlesNumber = 6;
const double circlesRadius = 0.125;

double cartesianEdgeObs[5][2][2] = {
                                            {{0.0,0.0}, {0.0,1.05}},
                                            {{0.0,1.95}, {0.0,3.0}},
                                            {{0.0,3.0}, {2.0,3.0}},
                                            {{2.0,3.0}, {2.0,0.0}},
                                            {{2.0,0.0}, {0.0,0.0}}
                                        };
const int edgesNumber = 5;
double edgesRadius = 0.01;

double cartesianRectangleObs[1][2][2] = {{{0.0,1.05}, {0.0,1.95}}};
const int rectangleNumber = 1;
double rectangleRadius = 0.15;

void path_planning_update(CtrlStruct *cvs, double x_target, double y_target)
{
    // General variables
    double xR = cvs->rob_pos->x;
    double yR = cvs->rob_pos->y;                  // Robot cartesian position
    double radius_robot = 0.20;

    // Initilization of attractive variables
    double sigma0 = 0.3;                  // Threshold distance from the target position
    double xsi = 1.0;                     // Attractive gain constant
    double sigma_th = cvs->path->sigma_th;                // Threshold distance for the reached target

    // Initilization of repulsive variables
    double eta_0 = 0.07;                     // Repulsive gain constant
    double eta_w = 0.005;
    double eta_opp = 0.1;
    double rho_i;                   // Distance from the robot to the nearest point of each obstacle
    double rho_0 = circlesRadius + radius_robot + 0.05;                   // The radius of influence of the obstacle
    double rho_opp = 5*radius_robot;
    double rho_w = edgesRadius + radius_robot;
    double rho_r = rectangleRadius + radius_robot + 0.15;
    double rho_x;                   // Distance from the robot to the obstacle following x
    double rho_y;                   // Distance from the robot to the obstacle following y

    double F_att_x = 0.0;           // Attractive forces in x direction
    double F_att_y = 0.0;           // Attractive forces y direction
    double F_rep_x = 0.0;           // Repulsive forces in x direction
    double F_rep_y = 0.0;           // Repulsive forces in y direction
    double F_tot_x = 0.0;           // Resultant force in x direction
    double F_tot_y = 0.0;           // Resultant force in y direction
    double theta_force = 0.0;       // Orientation of the resultant force 

    double U_att = 0.0;
    double U_rep = 0.0;
    cvs->path->target_reached = false;

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    
    // Weights
    double kx_speed = 1.2; //1.2
    double ky_speed = 1.6; //1.6


    // Compute the distance between the robot and the target position
    double sigma = euclidean(xR, yR, x_target, y_target);

    // Verify if the target is reached
    if (abs(sigma <= sigma_th)) {
        F_tot_x = 0.0;
        F_tot_y = 0.0;
        cvs->path->target_reached = true;
    }

    // Attractive forces
    if (sigma <= sigma0) {
        F_att_x = - xsi * (xR - x_target);
        F_att_y = - xsi * (yR - y_target);
        U_att   = 0.5 * xsi * sigma*sigma;
    } else {
        F_att_x = - xsi * sigma0 * (xR - x_target) / sigma;
        F_att_y = - xsi * sigma0 * (yR - y_target) / sigma;
        U_att   = xsi * sigma*sigma0 - 1/2.0 * xsi*sigma0*sigma0;
    }


    // Repulsive forces for each opponents
    for (int i = 0; i < cvs->lidar->nbrOpponents; i++) {
        if (cvs->opp_pos->x != 0.0 && cvs->opp_pos->y != 0.0) {
            rho_i = euclidean(xR, yR, cvs->opp_pos->x, cvs->opp_pos->y);
            rho_x = xR - cvs->opp_pos->x;
            rho_y = yR - cvs->opp_pos->y;
            if (rho_i <= rho_opp) {
                F_rep_x = F_rep_x + eta_opp * (1/rho_i - 1/rho_opp) * 1/pow(rho_i,2) * (rho_x)/rho_i;
                F_rep_y = F_rep_y + eta_opp * (1/rho_i - 1/rho_opp) * 1/pow(rho_i,2) * (rho_y)/rho_i;
                U_rep   = U_rep   + 0.5 * eta_opp * pow((1/(rho_i) - 1/rho_0),2);
            }
        }   
    }

    // Repulsive forces for each edges
    double* xy_array = (double*) malloc(2*sizeof(double));
    for (int i = 0; i < edgesNumber; i++) {
        rho_i = minimalDistance(cartesianEdgeObs[i][0], cartesianEdgeObs[i][1], xR, yR, xy_array);
        if (rho_i <= rho_w) {
            F_rep_x = F_rep_x + eta_w * (1/rho_i - 1/rho_w) * 1/pow(rho_i,2) * (xR - xy_array[0])/rho_i;
            F_rep_y = F_rep_y + eta_w * (1/rho_i - 1/rho_w) * 1/pow(rho_i,2) * (yR - xy_array[1])/rho_i;
            U_rep   = U_rep   + 0.5 * eta_w * pow((1/(rho_i) - 1/rho_w),2);
        }
    }
    free(xy_array);

    // Resultant force applied on the robot
    F_tot_x = F_att_x + F_rep_x;
    F_tot_y = F_att_y + F_rep_y;
    theta_force = 180.0/M_PI * atan2(F_tot_y,F_tot_x);
    double F = sqrt(F_tot_x*F_tot_x + F_tot_y*F_tot_y);

    cvs->path->F_tot_x = F_tot_x;
    cvs->path->F_tot_y = F_tot_y;
    cvs->path->U_att = U_att;
    cvs->path->U_rep = U_rep;

    // Speed definitions
    vx = kx_speed*(cvs->path->F_tot_x*cos(M_PI/180.0 * cvs->rob_pos->theta) + cvs->path->F_tot_y*sin(M_PI/180.0 * cvs->rob_pos->theta));
    vy = ky_speed*(- cvs->path->F_tot_x*sin(M_PI/180.0 * cvs->rob_pos->theta) + cvs->path->F_tot_y*cos(M_PI/180.0 * cvs->rob_pos->theta));
    omega = 0.0;

    cvs->path->pathSpeed->vx = vx;
    cvs->path->pathSpeed->vy = vy;
    cvs->path->pathSpeed->omega = omega;
}

double euclidean(double xR, double yR, double x_target, double y_target) {
    return sqrt(pow(xR - x_target, 2) + pow(yR - y_target, 2));
}

double minimalDistanceBis(double x1, double y1, double x2, double y2, double xR, double yR) {

    double projection = ((x2-x1)*(xR-x1) + (y2-y1)*(yR-y1))/((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

    double x = projection*(x2-x1);
    double y = projection*(y2-y1);

    double distance = sqrt(pow(xR-x1-x, 2) + pow(yR-y1-y, 2));

    return distance;
    
}

double minimalDistance(double *point1, double *point2, double xR, double yR, double* xy_array) {
    double x1 = point1[0];
    double x2 = point2[0];
    double y1 = point1[1];
    double y2 = point2[1];

    double projection = ((x2-x1)*(xR-x1) + (y2-y1)*(yR-y1))/((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

    double x = projection*(x2-x1);
    double y = projection*(y2-y1);
    xy_array[0] = x + x1;
    xy_array[1] = y + y1;

    double distance = sqrt(pow(xR-x1-x, 2) + pow(yR-y1-y, 2));

    return distance;
}


void makeHeatmap(CtrlStruct *cvs){

    FILE *fp_heatmap;
    fp_heatmap = fopen("heatmap.txt", "w");

     for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 100; j++) {
            cvs->rob_pos->x = 2.0/100.0 * (float) i;
            cvs->rob_pos->y = 3.0/100.0 * (float) j;
            path_planning_update(cvs, 1.0, 1.5);
            double U_att = cvs->path->U_att;
            double U_rep = cvs->path->U_rep;
            double U_tot = U_att + U_rep;

            double F_tot_x = cvs->path->F_tot_x;
            double F_tot_y = cvs->path->F_tot_y;
            double F_tot = sqrt(F_tot_x*F_tot_x + F_tot_y*F_tot_y);
            fprintf(fp_heatmap, "%f\n", U_tot);
            
        }
    }
    fclose(fp_heatmap);
}

void free_pathPlanning(CtrlStruct *cvs)
{
    free(cvs->path);
}



