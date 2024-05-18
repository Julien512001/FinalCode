#include "../../include/main/GPIO_initialize_gr5.h"


void init_GPIO()
{
    //init_GPIO();
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Échec de l'initialisation de pigpio.\n");
        exit(1);
    } else {
        printf("GPIO initialisé avec succès !\n");
    }

    gpioSetMode(START_UP_PIN, PI_INPUT);
}


void finish_GPIO()
{
    gpioTerminate();

}