#include "../../include/Actuators/Actuators_gr5.h"
#include <math.h>


void RunActuators(CtrlStruct *cvs)
{
    CtrlIn *inputs;
    uint32_t *message = cvs->actuator->message_array;
    inputs = cvs->inputs;
    uint32_t result = 0;
    for (int i = 0; i<20; i++){
        result += message[i]*pow(2,i);
    }
    send_spi(cvs, result, 0);
}

void ResetActuator(CtrlStruct *cvs){
    for (int i = 0; i<20;i++){
        cvs->actuator->message_array[i] = 0;
    }
}

void ParralaxOut(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 1;
    cvs->actuator->message_array[1] = 0;
}

void ParralaxIn(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 0;
    cvs->actuator->message_array[1] = 1;
}

void ParralaxStop(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 0;
    cvs->actuator->message_array[1] = 0;
}

void DSSForkUp(CtrlStruct *cvs){
    cvs->actuator->message_array[2] = 1;
    cvs->actuator->message_array[3] = 0;
}

void DSSForkMid(CtrlStruct *cvs){
    cvs->actuator->message_array[2] = 0;
    cvs->actuator->message_array[3] = 1;
}

void DSSForkDown(CtrlStruct *cvs){
    cvs->actuator->message_array[2] = 0;
    cvs->actuator->message_array[3] = 0;
}

void FeetechOut(CtrlStruct *cvs){
    cvs->actuator->message_array[4] = 1;
    cvs->actuator->message_array[5] = 0;
}

void FeetechIn(CtrlStruct *cvs){
    cvs->actuator->message_array[4] = 0;
    cvs->actuator->message_array[5] = 1;
}

void FeetechStop(CtrlStruct *cvs){
    cvs->actuator->message_array[4] = 0;
    cvs->actuator->message_array[5] = 0;
}

void EntonnoirOUT(CtrlStruct *cvs){
    cvs->actuator->message_array[6] = 1;
    cvs->actuator->message_array[7] = 0;
}

void EntonnoirIN(CtrlStruct *cvs){
    cvs->actuator->message_array[6] = 0;
    cvs->actuator->message_array[7] = 1;
}

void EntonnoirSTOP(CtrlStruct *cvs){
    cvs->actuator->message_array[6] = 0;
    cvs->actuator->message_array[7] = 0;
}

void EntonnoirBrasFerme(CtrlStruct *cvs){
    cvs->actuator->message_array[8] = 1;
    cvs->actuator->message_array[9] = 0;
}

void EntonnoirBrasOuvert(CtrlStruct *cvs){
    cvs->actuator->message_array[8] = 0;
    cvs->actuator->message_array[9] = 1;
}

void EntonnoirBrasMiddle(CtrlStruct *cvs){
    cvs->actuator->message_array[8] = 0;
    cvs->actuator->message_array[9] = 0;
}

void EntonnoirBrasSeq(CtrlStruct *cvs){
    cvs->actuator->message_array[8] = 1;
    cvs->actuator->message_array[9] = 1;
}

void DSSPanelUp(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 0;
    cvs->actuator->message_array[11] = 0;
}

void DSSPanelDown(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 1;
    cvs->actuator->message_array[11] = 0;
}

void DSSPanelMid(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 0;
    cvs->actuator->message_array[11] = 1;
}

void PanelWheelYellow(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 1;
    cvs->actuator->message_array[13] = 0;
}

void PanelWheelBleu(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 0;
    cvs->actuator->message_array[13] = 1;
}

void PanelWheelStop(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 0;
    cvs->actuator->message_array[13] = 0;
}

void PanelBrasUp(CtrlStruct *cvs){
    cvs->actuator->message_array[14] = 0;
}

void PanelBrasDown(CtrlStruct *cvs){
    cvs->actuator->message_array[14] = 1;
}

void TapisIn(CtrlStruct *cvs){
    cvs->actuator->message_array[15] = 1;
    cvs->actuator->message_array[16] = 0;
}

void TapisIn_Pot(CtrlStruct *cvs){
    cvs->actuator->message_array[15] = 0;
    cvs->actuator->message_array[16] = 1;
}

void TapisOut(CtrlStruct *cvs){
    cvs->actuator->message_array[15] = 1;
    cvs->actuator->message_array[16] = 1;
}

void TapisStop(CtrlStruct *cvs){
    cvs->actuator->message_array[15] = 0;
    cvs->actuator->message_array[16] = 0;
}

void free_Actuator(CtrlStruct *cvs)
{
	free(cvs->actuator->message_array);
}

