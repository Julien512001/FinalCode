#include "../../include/Localization/LiDar_gr5.h"
sl_result op_result;

double ERROR_THETA_CLUSTER  = 10.0;
double ERROR_DIST_CLUSTER   = 0.15;

double ERROR_THETA_STD      = 1000.0;
double ERROR_DIST_STD       = 0.25;

double ERROR_THETA_MEMORY   = 10.0;
double ERROR_DIST_MEMORY    = 0.10;

/*! \brief Compute the position of the robot with the LiDar
 * 
 * \param[in,out] cvs controller main structure
 */

void* lidar(void* arg)
{
    CtrlStruct * cvs = (CtrlStruct*) arg;
    while(true){
        if (cvs->main_state == RUN_STATE) {
            get_position(cvs);
        }
    }
    return nullptr;
}

void getPositionLidarCalib(CtrlStruct *cvs)
{
    getData(cvs);
    makeCluster(cvs);
    updateMemory(cvs);
    ToTal(cvs);
}

void get_position(CtrlStruct *cvs)
{
    getData(cvs);
    makeCluster(cvs);
    Cartesian *guessBeacons = (Cartesian*) malloc(3*sizeof(Cartesian));
    guess(cvs, guessBeacons);   // Guess the beacons position with odometric position
    updateBeacons(cvs, guessBeacons);   // Update the real beacons
    ToTal(cvs);                         // Triangulation algorithm
    opponentTracking(cvs);              // Tracking of the opponent position
}

ILidarDriver* connectLidar(){
    ILidarDriver* lidar;
    IChannel* _channel;
    lidar = *createLidarDriver();
    _channel = (*createSerialPortChannel("/dev/ttyUSB0", 115200));
    if (SL_IS_OK((lidar)->connect(_channel))){
        printf("Connected\n");

        lidar->setMotorSpeed();
        rp::standalone::rplidar::RplidarScanMode scanMode;
        lidar->startScan(false, true, 0, &scanMode);
        return lidar;
    }
    printf("Connection failed\n");
    return NULL;
}

void disconnectLidar(ILidarDriver* lidar){

    lidar->stop();
    lidar->setMotorSpeed(0);
    delete lidar;
    printf("Disconnected\n");
}

void getData(CtrlStruct *cvs)
{
    ILidarDriver *lidar = cvs->lidar->myLidar->lidar;
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    op_result = cvs->lidar->myLidar->lidar->grabScanDataHq(nodes, count);
    int countFilter = 0;
    if (SL_IS_OK(op_result)) {
        cvs->lidar->myLidar->lidar->ascendScanData(nodes, count);
        for (int pos = 0; pos < (int)count ; ++pos) {
            double posangle = (nodes[pos].angle_z_q14 * 90.f) / (1<<14);
            double posdistance = nodes[pos].dist_mm_q2/ (1<<2) * 1/1000.0;

            if ((posdistance < 3.8) & (posdistance > 0.0)){
                cvs->inputs->last_lidar_angle[countFilter] = posangle;
                cvs->inputs->last_lidar_radius[countFilter] = posdistance;
                countFilter ++;
            }
        }
    }
    else{
        printf("Error scan");
    }
    cvs->inputs->nb_lidar_data = countFilter;
}

void init_lidar(CtrlStruct *cvs)
{
    cvs->lidar->initBeacons->beacon1.x = 0.05;
    cvs->lidar->initBeacons->beacon1.y = -0.094;
    cvs->lidar->initBeacons->beacon2.x = 1.95;
    cvs->lidar->initBeacons->beacon2.y = -0.094;
    cvs->lidar->initBeacons->beacon3.x = 1.0;
    cvs->lidar->initBeacons->beacon3.y = 3.094;
}

void beaconsCalibration(CtrlStruct *cvs)
{
    if (cvs->robot_id == ROBOT_B) {
        switch(cvs->startPosition){
            case 1:
                cvs->lidar->beacons->beacon1.angle = 224.516083;
                cvs->lidar->beacons->beacon1.radius  = 0.384226;
                cvs->lidar->beacons->beacon2.angle = 99.851601;
                cvs->lidar->beacons->beacon2.radius  = 1.620000;
                cvs->lidar->beacons->beacon3.angle = 11.752625;
                cvs->lidar->beacons->beacon3.radius  = 2.980000;
                break;

            case 2:
                cvs->lidar->beacons->beacon1.angle = 260.062866;
                cvs->lidar->beacons->beacon1.radius  = 1.828857;
                cvs->lidar->beacons->beacon2.angle = 163.404938;
                cvs->lidar->beacons->beacon2.radius  = 0.291043;
                cvs->lidar->beacons->beacon3.angle = 342.617798;
                cvs->lidar->beacons->beacon3.radius  = 3.021333;
                break;

            case 3:
                cvs->lidar->beacons->beacon1.angle = 17.340088;
                cvs->lidar->beacons->beacon1.radius  = 3.033333;
                cvs->lidar->beacons->beacon2.angle = 341.096191;
                cvs->lidar->beacons->beacon2.radius  = 3.053333;
                cvs->lidar->beacons->beacon3.angle = 179.607178;
                cvs->lidar->beacons->beacon3.radius  = 0.266128;
                break;

            default:
                printf("positionRobot unknown \n");
                break;
        }
    } else {
        switch(cvs->startPosition){
            case 1:
                cvs->lidar->beacons->beacon1.angle = 17.157679;
                cvs->lidar->beacons->beacon1.radius  = 0.316460;
                cvs->lidar->beacons->beacon2.angle = 281.039429;
                cvs->lidar->beacons->beacon2.radius  = 1.825111;
                cvs->lidar->beacons->beacon3.angle = 197.459473;
                cvs->lidar->beacons->beacon3.radius  = 2.991200;
                break;

            case 2:
                cvs->lidar->beacons->beacon1.angle = 258.299561;
                cvs->lidar->beacons->beacon1.radius  = 1.659750;
                cvs->lidar->beacons->beacon2.angle = 134.610458;
                cvs->lidar->beacons->beacon2.radius  = 0.373588;
                cvs->lidar->beacons->beacon3.angle = 345.908203;
                cvs->lidar->beacons->beacon3.radius  = 2.962667;
                break;

            case 3:
                cvs->lidar->beacons->beacon1.angle = 17.340088;
                cvs->lidar->beacons->beacon1.radius  = 3.033333;
                cvs->lidar->beacons->beacon2.angle = 341.096191;
                cvs->lidar->beacons->beacon2.radius  = 3.053333;
                cvs->lidar->beacons->beacon3.angle = 179.607178;
                cvs->lidar->beacons->beacon3.radius  = 0.266128;
                break;

            default:
                printf("positionRobot unknown \n");
                break;
        }
    }

}

double getCentroid(double* myCluster, int countCluster) 
{
    float sum = 0.0;
    for (int i = 0; i < countCluster; i++) {
        sum += myCluster[i];
    }
    double myCentroid = sum/((float) (countCluster));
    return myCentroid;
}

bool getSTD(double* myCluster, int countCluster, double myCentroid, float std_th) 
{
    float std = 0.0;
    for (int i = 0; i < countCluster; i++) {
        std += pow(myCluster[i] - myCentroid, 2);
    }
    std = std/countCluster;
    if ( ((std) > (std_th))){
        return false;
    }else{
        return true;
    }
    
}

void makeCluster(CtrlStruct *cvs)
{
    CtrlIn *inputs;

    inputs = cvs->inputs;

    double *bufferTheta = (double*) calloc(inputs->nb_lidar_data, sizeof(double));
    if (bufferTheta == NULL) {
        printf("Allocation de mémoire échouée.\n");
    }
    double *bufferDist = (double*) calloc(inputs->nb_lidar_data, sizeof(double));
    if (bufferDist == NULL) {
        printf("Allocation de mémoire échouée.\n");
    }

    double *lstCentroidTheta = (double*) malloc(inputs->nb_lidar_data * sizeof(double));
    if (lstCentroidTheta == NULL) {
        printf("Allocation de mémoire échouée.\n");
    }
    double *lstCentroidDist = (double*) malloc(inputs->nb_lidar_data * sizeof(double));
    if (lstCentroidDist == NULL) {
        printf("Allocation de mémoire échouée.\n");
    }

    int countCentroid = 0;

    bufferTheta[0] = inputs->last_lidar_angle[0]; 
    bufferDist[0] = inputs->last_lidar_radius[0]; 

    int countBuffer = 1;
    
    //For the verif : firstBuffer = lastBuffer -> Same beacon
    double centroidTheta0 = 0.0;
    double centroidDist0 = 0.0;
    for(int i = 1; i < inputs->nb_lidar_data; i++){
        int j = i-1;
        if ((abs(inputs->last_lidar_radius[i] - inputs->last_lidar_radius[j]) < ERROR_DIST_CLUSTER) & (abs(inputs->last_lidar_angle[i] - inputs->last_lidar_angle[j]) < ERROR_THETA_CLUSTER)){
            bufferTheta[countBuffer] = inputs->last_lidar_angle[i]; 
            bufferDist[countBuffer] = inputs->last_lidar_radius[i]; 
            countBuffer++;
        }
        else{
            double centroidTheta = getCentroid(bufferTheta, countBuffer);
            double centroidDist = getCentroid(bufferDist, countBuffer);

            bool stateTheta = getSTD(bufferTheta, countBuffer, centroidTheta, ERROR_THETA_STD);
            bool stateDist = getSTD(bufferDist, countBuffer, centroidDist, ERROR_DIST_STD);

            bool state = (stateTheta & stateDist) ? true : false;

            if (state) {
                lstCentroidTheta[countCentroid] = centroidTheta;
                lstCentroidDist[countCentroid] = centroidDist;
                if (countCentroid == 0){
                    centroidTheta0 = centroidTheta;
                    centroidDist0 = centroidDist;
                }
                
                countCentroid ++;
            } 
            countBuffer = 0;
            bufferTheta[countBuffer] = inputs->last_lidar_angle[i]; 
            bufferDist[countBuffer] = inputs->last_lidar_radius[i]; 
            countBuffer = 1;
        }   
    }
    //For the last cluster
    double centroidTheta = getCentroid(bufferTheta, countBuffer);
    double centroidDist = getCentroid(bufferDist, countBuffer);
    
    bool state = false;
    if ((abs(lstCentroidDist[0]-centroidDist) < 0.1) && (abs(lstCentroidTheta[0]-centroidTheta) < 20.0)){
        state = true;
    }

    if (state) {
        //For the verif : firstBuffer = lastBuffer -> Same beacon
        double *bufferThetaFirstLast = (double*) malloc(2 * sizeof(double));
        double *bufferDistFirstLast = (double*) malloc(2 * sizeof(double));
        bufferThetaFirstLast[0] = centroidTheta0;
        bufferThetaFirstLast[1] = centroidTheta;
        bufferDistFirstLast[0] = centroidDist0;
        bufferDistFirstLast[1] = centroidDist;
        
        bufferThetaFirstLast[1] = bufferThetaFirstLast[1] - 360.0;

        double centroidThetaFirstLast= getCentroid(bufferThetaFirstLast, 2);
        double centroidDistFirstLast = getCentroid(bufferDistFirstLast, 2);
        if(centroidThetaFirstLast < 0){
            centroidThetaFirstLast = centroidThetaFirstLast + 360.0;
        }

        bool stateThetaFirstLast = getSTD(bufferThetaFirstLast, 2, centroidThetaFirstLast, ERROR_THETA_STD);
        bool stateDistFirstLast = getSTD(bufferDistFirstLast, 2, centroidDistFirstLast, ERROR_DIST_STD);

        bool stateFirstLast = (stateThetaFirstLast & stateDistFirstLast) ? true : false;
        if(stateFirstLast){
            lstCentroidTheta[0] = centroidThetaFirstLast;
            lstCentroidDist[0] = centroidDistFirstLast;

        }
        else{
            lstCentroidTheta[countCentroid] = centroidTheta;
            lstCentroidDist[countCentroid] = centroidDist;
            countCentroid ++;
        }
        free(bufferThetaFirstLast);
        free(bufferDistFirstLast);
    } 
    
    lstCentroidTheta = (double*) realloc(lstCentroidTheta, countCentroid * sizeof(double));
    lstCentroidDist = (double*) realloc(lstCentroidDist, countCentroid * sizeof(double));

    free(bufferTheta);
    free(bufferDist);

    cvs->lidar->myCentroids = (Polar*) malloc(countCentroid*sizeof(Polar));

    for (int i = 0; i < countCentroid; i++) {
        cvs->lidar->myCentroids[i].angle = lstCentroidTheta[i];
        cvs->lidar->myCentroids[i].radius = lstCentroidDist[i];
    }
    cvs->lidar->nbrCentroids = countCentroid;
}

void guess(CtrlStruct *cvs, Cartesian* guessBeacons)
{
    double theta = (cvs->odometry->theta);
    double xB3_R = (cvs->lidar->initBeacons->beacon3.x - cvs->odometry->x)*cos(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon3.y - cvs->odometry->y)*sin(M_PI/180.0*theta);
    double yB3_R = -(cvs->lidar->initBeacons->beacon3.x - cvs->odometry->x)*sin(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon3.y - cvs->odometry->y)*cos(M_PI/180.0*theta);

    double xB2_R = (cvs->lidar->initBeacons->beacon2.x - cvs->odometry->x)*cos(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon2.y - cvs->odometry->y)*sin(M_PI/180.0*theta);
    double yB2_R = -(cvs->lidar->initBeacons->beacon2.x - cvs->odometry->x)*sin(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon2.y - cvs->odometry->y)*cos(M_PI/180.0*theta);

    double xB1_R = (cvs->lidar->initBeacons->beacon1.x - cvs->odometry->x)*cos(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon1.y - cvs->odometry->y)*sin(M_PI/180.0*theta);
    double yB1_R = -(cvs->lidar->initBeacons->beacon1.x - cvs->odometry->x)*sin(M_PI/180.0*theta) + (cvs->lidar->initBeacons->beacon1.y - cvs->odometry->y)*cos(M_PI/180.0*theta);

    guessBeacons[0].x = xB1_R;
    guessBeacons[0].y = yB1_R;

    guessBeacons[1].x = xB2_R;
    guessBeacons[1].y = yB2_R;
    
    guessBeacons[2].x = xB3_R;
    guessBeacons[2].y = yB3_R;
}

void toCartesian(Polar polarCentroid, Cartesian *cartesianCentroid)
{
    cartesianCentroid->x = polarCentroid.radius*cos(M_PI/180.0 * polarCentroid.angle);
    cartesianCentroid->y = -polarCentroid.radius*sin(M_PI/180.0 * polarCentroid.angle);
}

void updateBeacons(CtrlStruct *cvs, Cartesian *guessBeacons)
{
    Cartesian cartesianCentroid;
    double min_val;
    int min_index = 0;
    double diff_rad;
    for (int i = 0; i < 3; i++) {
        min_val = 1000.0;
        for (int j = 0; j < cvs->lidar->nbrCentroids; j++) {
            toCartesian(cvs->lidar->myCentroids[j], &cartesianCentroid);
            //printf("c[%d] : %f, %f\n", j+1, cartesianCentroid.x, cartesianCentroid.y);
            diff_rad = sqrt(pow(cartesianCentroid.x - guessBeacons[i].x,2) + pow(cartesianCentroid.y - guessBeacons[i].y, 2)); 
            if (diff_rad < min_val) {
                min_val = diff_rad;
                min_index = j;
            }
        }
        
        switch (i)
        {
        case 0:
            cvs->lidar->beacons->beacon1 = cvs->lidar->myCentroids[min_index];
            break;
        case 1:
            cvs->lidar->beacons->beacon2 = cvs->lidar->myCentroids[min_index];
            break;
        case 2:
            cvs->lidar->beacons->beacon3 = cvs->lidar->myCentroids[min_index];
            break;
        default:
            break;
        }
    }
}

void ToTal(CtrlStruct *cvs) {

    double x1 = cvs->lidar->initBeacons->beacon1.x;
    double y1 = cvs->lidar->initBeacons->beacon1.y;
    double x2 = cvs->lidar->initBeacons->beacon2.x;
    double y2 = cvs->lidar->initBeacons->beacon2.y;
    double x3 = cvs->lidar->initBeacons->beacon3.x;
    double y3 = cvs->lidar->initBeacons->beacon3.y;

    x2 = x2 - x1;
    y2 = y2 - y1;
    x3 = x3 - x1;
    y3 = y3 - y1;

    double phi1;
    double phi2;
    double phi3;

    phi1 = 360.0 - cvs->lidar->beacons->beacon1.angle;
    phi2 = 360.0 - cvs->lidar->beacons->beacon2.angle;
    phi3 = 360.0 - cvs->lidar->beacons->beacon3.angle;

    double sinus12 = sin(M_PI/180.0 * (phi2 - phi1));
    double sinus23 = sin(M_PI/180.0 * (phi3 - phi2));

    double cosinus12 = cos(M_PI/180.0 * (phi2 - phi1));
    double cosinus23 = cos(M_PI/180.0 * (phi3 - phi2));

    double T12 = cosinus12/sinus12;
    double T23 = cosinus23/sinus23;
    double T31 = (1-T12*T23)/(T12 + T23);

    if (T12 > 1e8) T12 = 1e8;
    if (T23 > 1e8) T23 = 1e8;
    if (T31 > 1e8) T31 = 1e8;

    double x12 = x2  - T12*y2;
    double x23 = (x2 + x3) + T23*(y2 - y3);
    double x31 = x3  + T31*y3;

    double y12 = y2 + T12*x2;
    double y23 = (y2 + y3) - T23*(x2 - x3);
    double y31 = y3 - T31*x3;

    double k23 = x2*x3 + y2*y3 + T23*(x3*y1 - x2*y3);

    double D = (x12 - x23)*(y23 - y31) - (y12 - y23)*(x23 - x31);
    if (!D) {
        printf("D = 0 impossible");
    };

    
    double xR;
    double yR;
    double ThetaR;

    xR = (x1 + k23 * (y31 - y12)/D);
    yR = (y1 + k23 * (x12 - x31)/D);
    ThetaR = 180.0/M_PI * atan2(y2 - yR, x2 - xR) - phi2;

    cvs->lidar->x = xR;
    cvs->lidar->y = yR;
    cvs->lidar->theta = ThetaR;

    if (ThetaR >= 360.0){
        cvs->lidar->theta -= 360.0;
    }
    if (ThetaR < 0.0){
        cvs->lidar->theta += 360.0;
    }
}

void opponentTracking(CtrlStruct *cvs){
    
    for (int i = 0; i < cvs->lidar->nbrCentroids; i++) {
        if ((abs(cvs->lidar->myCentroids[i].radius - cvs->lidar->beacons->beacon1.radius) < 0.1 & abs(cvs->lidar->myCentroids[i].angle - cvs->lidar->beacons->beacon1.radius) < 15.0)) {
            continue;
        }
        if ((abs(cvs->lidar->myCentroids[i].radius - cvs->lidar->beacons->beacon2.radius) < 0.1 & abs(cvs->lidar->myCentroids[i].angle - cvs->lidar->beacons->beacon2.radius) < 15.0)) {
            continue;
        }
        if ((abs(cvs->lidar->myCentroids[i].radius - cvs->lidar->beacons->beacon3.radius) < 0.1 & abs(cvs->lidar->myCentroids[i].angle - cvs->lidar->beacons->beacon3.radius) < 15.0)) {
            continue;
        }
        else {
            
            double x = (cvs->lidar->myCentroids[i].radius) * cos(cvs->lidar->myCentroids[i].angle * M_PI/180.0);
            double y = - (cvs->lidar->myCentroids[i].radius) * sin(cvs->lidar->myCentroids[i].angle * M_PI/180.0);

            double xOpponent =   x * cos(cvs->rob_pos->theta * M_PI/180.0) - y * sin(cvs->rob_pos->theta * M_PI/180.0);
            double yOpponent =   x * sin(cvs->rob_pos->theta * M_PI/180.0) + y * cos(cvs->rob_pos->theta * M_PI/180.0);

            xOpponent += cvs->rob_pos->x;
            yOpponent += cvs->rob_pos->y;

            if (xOpponent >= 0.2 & xOpponent <= 1.8){
                if (yOpponent >= 0.2 & yOpponent <= 2.8){
                    cvs->opp_pos->x = xOpponent;
                    cvs->opp_pos->y = yOpponent;
                }
            }
        }
    }
}

void free_lidar(CtrlStruct *cvs) {
    free(cvs->lidar->beacons);
    free(cvs->lidar->myLidar);
    free(cvs->lidar->myCentroids);
    free(cvs->lidar->initBeacons);
    free(cvs->lidar->opponents);
    free(cvs->lidar);
}
