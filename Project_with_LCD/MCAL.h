#include "HAL.h"

#ifndef MCAL_H
#define MCAL_H

/* Control vars */
#define JAM						(uint8) 0
#define DRIVER_UP			(uint8) 1
#define DRIVER_DOWN		(uint8) 2
#define PASS_UP				(uint8) 3
#define PASS_DOWN			(uint8) 4
#define LIMIT_UP			(uint8) 5
#define LIMIT_DOWN		(uint8) 6
#define LOCK					(uint8) 7
#define UNLOCK				(uint8) 8
#define WINDOW_STOP		(uint8) 9

#define STOP					0x0
#define CW						0x1
#define CCW						0x2

#define LED_STOP			0x0
#define LED_RED				0x1
#define LED_BLUE			0x2
#define LED_GREEN			0x3

void M_Init();
void M_MotorControl(uint8 direction);
uint8 M_Read(uint8 device);
void M_Led(uint8 control);

#endif