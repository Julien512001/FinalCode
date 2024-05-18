#include <stdio.h>
#include <pigpio.h> 
#include <string.h>

//gcc -o uart uart.c -lpigpio -lrt -lpthread

#define ARDUINO1_PORT "/dev/ttyACM0" 
#define ARDUINO2_PORT "/dev/ttyACM1"

int main()
{
    if (gpioInitialise() < 0) 
    {
        fprintf(stderr, "Erreur lors de l'initialisation de pigpio\n");
        return 1;
    }

    int serial_handle_arduino1 = serOpen(ARDUINO1_PORT, 9600, 0);
    if (serial_handle_arduino1 < 0)
    {
        fprintf(stderr, "Impossible d'ouvrir le port s�rie pour Arduino 1\n");
        gpioTerminate();
        return 1;
    }

    
    int serial_handle_arduino2 = serOpen(ARDUINO2_PORT, 9600, 0);
    if (serial_handle_arduino2 < 0)
    {
        fprintf(stderr, "Impossible d'ouvrir le port s�rie pour Arduino 2\n");
        serClose(serial_handle_arduino1);
        gpioTerminate();
        return 1;
    }
    
    printf("Ports s�rie ouverts avec succ�s\n");

    double speed1_arduino1 = 0.0; 
    double speed2_arduino1 = 0.0;
    double speed1_arduino2 = 0.0;
    double speed2_arduino2 = 0.0;
    
    double received_speed1_arduino1, received_speed2_arduino1;
    double received_speed1_arduino2, received_speed2_arduino2;

    while (1)
    {
        char data_arduino1[50];
        snprintf(data_arduino1, sizeof(data_arduino1), "%.2f %.2f\n", speed1_arduino1, speed2_arduino1);
        serWrite(serial_handle_arduino1, data_arduino1, strlen(data_arduino1));

        
        char data_arduino2[50];
        snprintf(data_arduino2, sizeof(data_arduino2), "%.2f %.2f\n", speed1_arduino2, speed2_arduino2);
        serWrite(serial_handle_arduino2, data_arduino2, strlen(data_arduino2));
        

        if (serDataAvailable(serial_handle_arduino1) > 0)
        {
            char received_data_arduino1[50];
            serRead(serial_handle_arduino1, received_data_arduino1, sizeof(received_data_arduino1));
            sscanf(received_data_arduino1, "%lf %lf", &received_speed1_arduino1, &received_speed2_arduino1);
        }

        
        if (serDataAvailable(serial_handle_arduino2) > 0)
        {
            char received_data_arduino2[50];
            serRead(serial_handle_arduino2, received_data_arduino2, sizeof(received_data_arduino2));
            sscanf(received_data_arduino2, "%lf %lf", &received_speed1_arduino2, &received_speed2_arduino2);
        }
        

        printf("%f, %f\n", received_speed1_arduino1, received_speed2_arduino1);
        printf("Vitesses Arduino 2: %.2f %.2f\n", received_speed1_arduino2, received_speed2_arduino2);

    }

    serClose(serial_handle_arduino1);
    serClose(serial_handle_arduino2);

    gpioTerminate(); // Termine pigpio
    return 0;
}
