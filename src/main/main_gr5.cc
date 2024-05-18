#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/main/GPIO_initialize_gr5.h"
#include "../../include/main/ctrl_main_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/Protocoles/I2C_gr5.h"
#include "../../include/Sonars/Sonars_gr5.h"
#include "../../include/Localization/calibration_gr5.h"
#include "../../include/Localization/aruco_gr5.h"
#include "../../include/LCD/LCD_gr5.h"
#include "../../include/Protocoles/uart_gr5.h"

#define BILLION 1000000000L

int main(void) {

    CtrlIn *inputs;
    CtrlOut *outputs;
    CtrlStruct *cvs;

    //pthread_t threadArcuo;
    pthread_t threadLidar;
    pthread_t threadUart;


    // CtrlStruct init
    inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
    outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
    cvs = init_CtrlStruct(inputs, outputs);

    // GPIO init
    init_GPIO();

    // Communication protocole init
    init_spi(cvs);
    init_uart(cvs);

    // Controller init
    cvs->robot_id = ROBOT_B;
    cvs->startPosition = 1;
    controller_init(cvs);

    // Init LCD
    set_lcd();
    int i;

    // robot ID    
    // Connect Lidar
    cvs->lidar->myLidar->lidar = connectLidar();
    init_lidar(cvs);
    cvs->position->flagUpdate = 0;

    void* arg = (void*) cvs;

    pthread_create(&threadLidar, NULL, &lidar, arg);
    pthread_create(&threadUart, NULL, &uart, arg);

    cvs->calib->flag = 1;
    
    calibration(cvs);
    sleep(1.0);
    while (inputs->StartSwitch == 0)
    {
        startUp(cvs);
    }

    //TIME
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    cvs->lidar->nbrOpponents = 1;
    
    // control loop
    while (true)
    {
        clock_gettime(CLOCK_MONOTONIC, &end);
        cvs->inputs->t = (end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec) / BILLION;
        controller_loop(cvs);

        // time LCD
        if (i % 100 == 0){
            time(cvs->inputs->t);
            // printf("int i : %d\n", i);
        }i += 1;

        // Lidar position update
        double last_execution = 0.0;
        double timeUpdate = cvs->inputs->t - last_execution;
        if (timeUpdate >= 0.2){
            if (cvs->inputs->t > 2.0){
                cvs->position->flagUpdate = 1; //To modif for lidar
                last_execution = cvs->inputs->t;
            }
        }

        // Stop for the end of the match
        if (cvs->inputs->t >= 90.0) {
            speed_regulation(cvs,-1,-1,-1);
            cvs->main_state = STOP_END_STATE;
            printf("End of the match\n");
            break;
        }
    }

    pthread_join(threadLidar, NULL);
    pthread_join(threadUart, NULL);

    disconnectLidar(cvs->lidar->myLidar->lidar);

    controller_finish(cvs);

    finish_uart(cvs);
    finish_GPIO();
        
    return 0;
}