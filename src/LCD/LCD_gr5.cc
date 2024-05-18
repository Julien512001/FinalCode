#include <stdio.h>
#include <pigpio.h>
#include <pigpiod_if2.h>
#include <string.h>
#include <string>
#include <iomanip>
// #include <cmath>

#include "../../include/LCD/LCD_gr5.h"

int LCDAddr = 0x27;
int BLEN = 1;
int fd;

int score_global;
int unit_time;

void write_word(int data){
    int temp = data;
    if ( BLEN == 1 )
        temp |= 0x08;
    else
        temp &= 0xF7;
    i2cWriteByte(fd, temp); // Utilisation de pigpio
}

void send_command(int comm){
    int buf;
    // Send bit7-4 firstly
    buf = comm & 0xF0;
    buf |= 0x04;            // RS = 0, RW = 0, EN = 1
    write_word(buf);
    gpioDelay(2000); // Utilisation de pigpio
    buf &= 0xFB;            // Make EN = 0
    write_word(buf);

    // Send bit3-0 secondly
    buf = (comm & 0x0F) << 4;
    buf |= 0x04;            // RS = 0, RW = 0, EN = 1
    write_word(buf);
    gpioDelay(2000); // Utilisation de pigpio
    buf &= 0xFB;            // Make EN = 0
    write_word(buf);
}

void send_data(int data){
    int buf;
    // Send bit7-4 firstly
    buf = data & 0xF0;
    buf |= 0x05;            // RS = 1, RW = 0, EN = 1
    write_word(buf);
    gpioDelay(2000); // Utilisation de pigpio
    buf &= 0xFB;            // Make EN = 0
    write_word(buf);

    // Send bit3-0 secondly
    buf = (data & 0x0F) << 4;
    buf |= 0x05;            // RS = 1, RW = 0, EN = 1
    write_word(buf);
    gpioDelay(2000); // Utilisation de pigpio
    buf &= 0xFB;            // Make EN = 0
    write_word(buf);
}

void init(){
    send_command(0x33);    // Must initialize to 8-line mode at first
    gpioDelay(5000); // Utilisation de pigpio
    send_command(0x32);    // Then initialize to 4-line mode
    gpioDelay(5000); // Utilisation de pigpio
    send_command(0x28);    // 2 Lines & 5*7 dots
    gpioDelay(5000); // Utilisation de pigpio
    send_command(0x0C);    // Enable display without cursor
    gpioDelay(5000); // Utilisation de pigpio
    send_command(0x01);    // Clear Screen
    i2cWriteByte(fd, 0x08); // Utilisation de pigpio
}

void clear(){
    send_command(0x01);    //clear Screen
}

void write(int x, int y, const char* data){
    int addr, i;
    int tmp;
    if (x < 0)  x = 0;
    if (x > 15) x = 15;
    if (y < 0)  y = 0;
    if (y > 1)  y = 1;

    // Move cursor
    addr = 0x80 + 0x40 * y + x;
    send_command(addr);
    
    tmp = strlen(data);
    for (i = 0; i < tmp; i++){
        send_data(data[i]);
    }
}

// void time(double time){
//     fd = i2cOpen(1, LCDAddr, 0); // Utilisation de pigpio
//     write(7, 0, std::to_string(round(t)).c_str());
// }
void time(double t){
    if (unit_time < t) {
        unit_time += 1;
        fd = i2cOpen(1, LCDAddr, 0); // Utilisation de pigpio
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << t;
        std::string rounded_time = stream.str();
        write(7, 0, rounded_time.c_str());
        // printf("unit_time: %f\n", unit_time);
    }
    if (t >= 100.0){
        write(7, 0, "100.00");
    }
}

void score(int score){
    write(8, 1, std::to_string(score).c_str());
}

void increment_score(int bonus){
    fd = i2cOpen(1, LCDAddr, 0); // Utilisation de pigpio
    score_global = score_global + bonus;
    score(score_global);
}

void set_lcd(){
    fd = i2cOpen(1, LCDAddr, 0); // Utilisation de pigpio
    init();
    score_global = 0;
    unit_time = 0;
    write(0, 0, "Time : ");
    write(7, 0, "0.0");
    write(0, 1, "Score : ");
    score(0);
}

// int main(){
//     if (gpioInitialise() < 0) {
//         fprintf(stderr, "pigpio initialisation failed\n");
//         return 1;
//     }
//     set_lcd();
//     increment_score(2);
// 	increment_score(2);
//     // gpioTerminate(); // Arrête pigpio avant de quitter le programme
//     return 0;
// }
