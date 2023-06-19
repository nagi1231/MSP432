#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/*
p2.3->E2A
p2.4->E2B
p3.5->E1A
p3.7->E1B
*/

extern int encoder_left;
extern int encoder_right;


void init_encoder_left();
void init_encoder_right();
//side->0:×ó£»1£ºÓÒ
int read_decoder(uint8_t side);