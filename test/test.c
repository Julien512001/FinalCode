#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>
#include <math.h>
#include <unistd.h>

//gcc -o main test.c -lpigpio -lrt



void send_i2c(int addr, double omega1, double omega2) {
    
    int w1 = abs((int) ((omega1 * 100.0)));
    int w2 = abs((int) ((omega2 * 100.0)));
    // Séparer les valeurs en octets
    int a1 = (w1 >> 8);
    int a2 = w1 & 0xFF;
    int a3 = (w2 >> 8); 
    int a4 = w2 & 0xFF;
    // Déterminer les signes de w1 et w2
    int s1 = omega1 >= 0 ? 0 : 1;
    int s2 = omega2 >= 0 ? 0 : 1;
    printf("a1 = %d\n", a1);
    printf("a2 = %d\n", a2);
    printf("a3 = %d\n", a3);
    printf("a4 = %d\n", a4);

    
    // Combiner les signes pour former un octet
    int sign = (s1 << 4) | s2;

    // Créer le tableau de données à envoyer
    int data[] = {a1, a2, a3, a4, sign};

    // Ouvrir la communication I2C avec l'adresse TEENSY_ADDR
    int handle_teenzy = i2cOpen(1, addr, 0);
    if (handle_teenzy < 0) {
        printf("Erreur lors de l'ouverture de la communication I2C\n");
    }

    // Envoyer les données sur le bus I2C
    int send = i2cWriteBlockData(handle_teenzy, 0, (char *)data, 5);
    if (send != 0) {
        printf("Erreur 1 fois lors de l'envoi des données de %d\n", addr);
    }
    i2cClose(handle_teenzy);
}

int main() {

    if (gpioInitialise() < 0) {
        fprintf(stderr, "Échec de l'initialisation de pigpio.\n");
        exit(1);
    } else {
        printf("GPIO initialisé avec succès !\n");
    }
    int i = 1;
    while (1) {
        send_i2c(0x01, 15.0, 10.0);
        sleep(0.1);
        i++;
    }

    
    return 0;
}


