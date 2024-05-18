#include "../../include/Protocoles/uart_gr5.h"

/*! \brief send and receive data with serial communication
 *  
 *  \param[in] cvs
*/

void init_uart(CtrlStruct *cvs)
{
    cvs->uart->handle_front = serOpen(ADDR_FRONT, 115200, 0);
    if (cvs->uart->handle_front < 0)
    {
        fprintf(stderr, "Impossible d'ouvrir le port pour FRONT\n");
        return;
    }
    
    cvs->uart->handle_rear = serOpen(ADDR_REAR, 9600, 0);
    if (cvs->uart->handle_rear < 0)
    {
        fprintf(stderr, "Impossible d'ouvrir le port pour REAR\n");
        return;
    }
}

void* uart(void *arg)
{
    CtrlStruct * cvs = (CtrlStruct*) arg;

    double dataTo_front1, dataTo_front2;
    double dataTo_rear1, dataTo_rear2;

    while (true) {
        dataTo_front1 = cvs->outputs->wheel_commands[W1]; 
        dataTo_front2 = cvs->outputs->wheel_commands[W2]; 
        dataTo_rear1  = cvs->outputs->wheel_commands[W3];
        dataTo_rear2  = cvs->outputs->wheel_commands[W4];
        
        char data_front[50];
        int ser;

        
        snprintf(data_front, sizeof(data_front), "%f %f\n", dataTo_front1, dataTo_front2);
        ser = serWrite(cvs->uart->handle_front, data_front, strlen(data_front));
        if (ser != strlen(data_front)) {
            //printf("erreur lors de l'envoie\n");
        }
        
        
        char data_rear[50];
        printf("%f\n", dataTo_rear1);
        snprintf(data_rear, sizeof(data_rear), "%f %f\n", dataTo_rear2, dataTo_rear1);
        ser = serWrite(cvs->uart->handle_rear, data_rear, strlen(data_rear));
        if (ser != strlen(data_front)) {
            //printf("erreur lors de l'envoie\n");
        }
        
       
        if (serDataAvailable(cvs->uart->handle_front) > 0)
        {
            char received_data_front[50];
            serRead(cvs->uart->handle_front, received_data_front, sizeof(received_data_front));
            sscanf(received_data_front, "%f %f", &cvs->inputs->wheel_speeds[W1], &cvs->inputs->wheel_speeds[W2]);
        }
        
        
        
        if (serDataAvailable(cvs->uart->handle_rear) > 0)
        {
            char received_data_rear[50];
            serRead(cvs->uart->handle_rear, received_data_rear, sizeof(received_data_rear));
            sscanf(received_data_rear, "%f %f", &cvs->inputs->wheel_speeds[W4], &cvs->inputs->wheel_speeds[W3]);
        }
    }
    
    
}

void finish_uart(CtrlStruct *cvs)
{
    serClose(cvs->uart->handle_front);
    serClose(cvs->uart->handle_rear);
}


void free_uart(CtrlStruct *cvs) 
{
    free(cvs->uart);
}