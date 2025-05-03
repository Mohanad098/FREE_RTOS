#include "GPIO.h"
#ifndef HAL_H
#define HAL_H

void H_Ports_Init(void);
	
/*
Driver up: Port f pin 0
Driver down: Port f pin 4
Passenger Up: Port C pin 4
Passenger down Port C pin 5
JAM (IR/PB): Port C pin 6
Lock: Port C pin 7
All PB 1 leg ground 1 pin
*/
void H_PB_init(void);

/*
Limit switch up: PD6
Limit switch down: PD7
1 NC 1 pin
*/
void H_Limit_init(void);

/*
H_bridge
IN3 PB0
IN4 PB1
*/
void H_Motor_Init(void);

/*
ENA "Yellow" PD3
ENB "Green" PD2
*/
void H_Encoder_Init(void);

/*
Red: PF1
Blue: PF2
Green: PF3
*/
void H_Leds_Init(void);
#endif