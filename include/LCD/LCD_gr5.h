#ifndef _LCD_GR5_H_
#define _LCD_GR5_H_ 


// function prototype

void write_word(int data);
void send_command(int comm);
void send_data(int data);
void init();
void clear();
void write(int x, int y, const char* data);
void time(double time);
void score(int score);
void increment_score(int bonus);
void set_lcd();

#endif
