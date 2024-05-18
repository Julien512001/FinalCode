#ifndef ACTUATOR_GR5_H
#define ACTUATOR_GR5_H

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"

typedef struct Actuator{
    uint32_t *message_array;
} Actuator;

void RunActuators(CtrlStruct *cvs);

void ResetActuator(CtrlStruct *cvs);

void ParralaxOut(CtrlStruct *cvs);

void ParralaxIn(CtrlStruct *cvs);

void ParralaxStop(CtrlStruct *cvs);

void DSSForkUp(CtrlStruct *cvs);

void DSSForkMid(CtrlStruct *cvs);

void DSSForkDown(CtrlStruct *cvs);

void FeetechOut(CtrlStruct *cvs);

void FeetechIn(CtrlStruct *cvs);

void FeetechStop(CtrlStruct *cvs);

void EntonnoirOUT(CtrlStruct *cvs);

void EntonnoirIN(CtrlStruct *cvs);

void EntonnoirSTOP(CtrlStruct *cvs);

void EntonnoirBrasFerme(CtrlStruct *cvs);

void EntonnoirBrasOuvert(CtrlStruct *cvs);

void EntonnoirBrasMiddle(CtrlStruct *cvs);

void EntonnoirBrasSeq(CtrlStruct *cvs);

void DSSPanelUp(CtrlStruct *cvs);

void DSSPanelMid(CtrlStruct *cvs);

void DSSPanelDown(CtrlStruct *cvs);

void PanelWheelYellow(CtrlStruct *cvs);

void PanelWheelBleu(CtrlStruct *cvs);

void PanelWheelStop(CtrlStruct *cvs);

void PanelBrasUp(CtrlStruct *cvs);

void PanelBrasDown(CtrlStruct *cvs);

void TapisIn(CtrlStruct *cvs);

void TapisIn_Pot(CtrlStruct *cvs);

void TapisOut(CtrlStruct *cvs);

void TapisStop(CtrlStruct *cvs);

void free_Actuator(CtrlStruct *cvs);


#endif