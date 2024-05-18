#include "../../include/Protocoles/I2C_gr5.h"

/*! \brief send data in the I2C channel (This communication protocol is no longer used because it was not functioning properly. We decided to switch to UART.)
 *  
 *  \param[in] addr I2C address of the device
*/

int file = -1;

void open_bus() {
    if ((file = open("/dev/i2c-1", O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        exit(1);
    }
}

void select_slave(int addr) {
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        printf("Failed to select slave.\n");
        exit(1);
    }
}

void sendData(double* speedArray) {
    uint8_t w1 = (uint8_t)speedArray[0];
    uint8_t w2 = (uint8_t)speedArray[1];
    uint8_t w3 = (uint8_t)speedArray[2];
    uint8_t w4 = (uint8_t)speedArray[3];

    // printf("%d\n%d\n%d\n%d\n", w1, w2, w3, w4);

    uint8_t data_front[2] = {w1, w2};
    uint8_t data_rear[2] = {w4, w3};

    // Send data to front address
    select_slave(ADDR_FRONT);
    if (write(file, data_front, 2) != 2) {
        printf("Failed to write to front address.\n");
        exit(1);
    }

    // Send data to rear address
    select_slave(ADDR_REAR);
    if (write(file, data_rear, 2) != 2) {
        printf("Failed to write to rear address.\n");
        exit(1);
    }
}

void close_bus() {
    close(file);
}


/*Ancienne fonction pour envoyer en i2c, on ne l'utilise plus maintenant ca peut être supprimer si vous voulez*/

void send_i2c(CtrlStruct *cvs, int addr, double omega1, double omega2) {
    
    int w1 = abs((int) round((omega1 * 100.0)));
    int w2 = abs((int) round((omega2 * 100.0)));
    // Séparer les valeurs en octets
    uint8_t a1 = (w1 >> 8);
    uint8_t a2 = w1 & 0xFF;
    uint8_t a3 = (w2 >> 8); 
    uint8_t a4 = w2 & 0xFF;
    // Déterminer les signes de w1 et w2
    uint8_t s1 = omega1 >= 0 ? 0 : 1;
    uint8_t s2 = omega2 >= 0 ? 0 : 1;

    
    // Combiner les signes pour former un octet
    uint8_t sign = (s1 << 4) | s2;

    // Créer le tableau de données à envoyer
    uint8_t data[] = {a1, a2, a3, a4, sign};

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

void receive_i2c(CtrlStruct *cvs, int addr)
{

    int handle_teenzy1 = i2cOpen(1, addr, 0);
    if (handle_teenzy1 < 0) {
        printf("Erreur lors de l'ouverture de la communication I2C\n");
    }
    
    char buffer[BUFFER_SIZE];

    // Lire les données depuis l'Arduino octet par octet
    int bytesRead = i2cReadDevice(handle_teenzy1, buffer, BUFFER_SIZE);
    if (bytesRead != BUFFER_SIZE) {
        printf("Erreur lors de la lecture des données depuis %d\n", addr);
    }

    // ------------------------------------------
    // Lecture et transformation de vitesse
    // ------------------------------------------

    int speed1, speed2;
    int sign1, sign2;
    double mes_speed1, mes_speed2;

    // Combiner les deux premiers octets pour former le premier nombre
    speed1 = (buffer[0] << 8) | buffer[1];
    // Combiner les deux derniers octets pour former le deuxième nombre
    speed2 = (buffer[2] << 8) | buffer[3];

    // Lire le signe
    sign1 = (buffer[4] >> 4);
    sign2 = (buffer[4] & 0x0f);

    speed1 = (sign1) ? - speed1 : speed1;
    speed2 = (sign2) ? - speed2 : speed2;

    mes_speed1 = (double) speed1/100.0;
    mes_speed2 = (double) speed2/100.0;

    switch (addr)
    {
        case ADDR_REAR:
            cvs->odometry->speed->w3 = mes_speed1;
            cvs->odometry->speed->w4 = mes_speed2;
            break;
        case ADDR_FRONT:
            cvs->odometry->speed->w1 = mes_speed1;
            cvs->odometry->speed->w2 = mes_speed2;
            break;
        default:
            break;
    }
    // Fermer la communication I2C
    i2cClose(handle_teenzy1);
}