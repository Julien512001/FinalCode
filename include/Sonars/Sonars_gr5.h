#ifndef SONAR_GR5_H
#define SONAR_GR5_H

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"


#define ADDR_1 0x01
#define ADDR_2 0x02
#define ADDR_3 0x03
#define ADDR_4 0x04


void sonar(CtrlStruct *cvs);
double convertToDecimal(int *dataList, int dataSize);
double movingAverage(double *dataList, int dataSize);



#endif