#include "../../include/Sonars/Sonars_gr5.h"

#define T           1/50e6

void sonar(CtrlStruct *cvs)
{
    CtrlIn *inputs;

    inputs = cvs->inputs;

    reveive_spi(cvs, ADDR_1);
    cvs->inputs->sonars[ADDR_1 - 1] = convertToDecimal(cvs->spi->rxData, 4);

    reveive_spi(cvs, ADDR_2);
    cvs->inputs->sonars[ADDR_2 - 1] = convertToDecimal(cvs->spi->rxData, 4);

    reveive_spi(cvs, ADDR_3);
    cvs->inputs->sonars[ADDR_3 - 1] = convertToDecimal(cvs->spi->rxData, 4);

    reveive_spi(cvs, ADDR_4);
    cvs->inputs->sonars[ADDR_4 - 1] = convertToDecimal(cvs->spi->rxData, 4);
}

double convertToDecimal(int *dataList, int dataSize)
{
    int decimalValue = 0;

    for (int i = 0; i < dataSize; ++i) {
        decimalValue <<= 8;
        decimalValue += dataList[i];
    }

    double distance = (double) decimalValue*T/(340.0*2.0)*1e7;

    return distance;
}

double movingAverage(double *dataList, int dataSize)
{
    double avg = 0.0;
    for (int i = 0; i < dataSize; i++) {
        avg += dataList[i]/dataSize;
    }

    return avg;
}