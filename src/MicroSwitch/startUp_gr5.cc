#include "../../include/MicroSwitch/startUp_gr5.h"




void startUp(CtrlStruct *cvs)
{
    CtrlIn *inputs;

    inputs = cvs->inputs;

    inputs->StartSwitch = !gpioRead(START_UP_PIN);
}