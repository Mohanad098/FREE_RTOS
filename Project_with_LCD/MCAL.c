#include "MCAL.h"

void M_Init()
{
		H_Ports_Init();
		H_PB_init();
		H_Limit_init();
    H_Motor_Init();
		H_Encoder_Init();
		H_Leds_Init();
}
void M_MotorControl(uint8 direction)
{
	switch (direction)
	{
		case CW:
			Port_SetPinValue(Gpio_PortB, Gpio_Pin0, PORT_PIN_LEVEL_HIGH); // IN1
      Port_SetPinValue(Gpio_PortB, Gpio_Pin1, PORT_PIN_LEVEL_LOW);  // IN2
      PWM0_0_CMPA_R = 31; // 25% duty cycle
			break;
		case CCW:
	    Port_SetPinValue(Gpio_PortB, Gpio_Pin0, PORT_PIN_LEVEL_LOW); // IN1
      Port_SetPinValue(Gpio_PortB, Gpio_Pin1, PORT_PIN_LEVEL_HIGH);  // IN2
      PWM0_0_CMPA_R = 31; // 25% duty cycle
			break;
		case STOP:
		default:
			Port_SetPinValue(Gpio_PortB, Gpio_Pin0, PORT_PIN_LEVEL_LOW); // IN1
      Port_SetPinValue(Gpio_PortB, Gpio_Pin1, PORT_PIN_LEVEL_LOW); // IN2
      PWM0_0_CMPA_R = 0; // No speed
	}
}
uint8 M_Read(uint8 device)
{
	switch (device)
	{
		case DRIVER_UP:
			return Port_ReadPin(Gpio_PortD,Gpio_Pin2);
			break;
		case DRIVER_DOWN:
			return Port_ReadPin(Gpio_PortD,Gpio_Pin1);
			break;
		case PASS_UP:
			return Port_ReadPin(Gpio_PortC,Gpio_Pin4);
			break;
		case PASS_DOWN:
			return Port_ReadPin(Gpio_PortC,Gpio_Pin5);
			break;
		case LIMIT_UP:
			return Port_ReadPin(Gpio_PortD,Gpio_Pin6);
			break;
		case LIMIT_DOWN:
			return Port_ReadPin(Gpio_PortD,Gpio_Pin7);
			break;
		case JAM:
			return Port_ReadPin(Gpio_PortC,Gpio_Pin6);
			break;
		case LOCK:
			return Port_ReadPin(Gpio_PortC,Gpio_Pin7);
			break;
		default: return -1;
	}
}

void M_Led(uint8 control)
{
	switch(control)
	{
		case LED_STOP:
			Port_SetPinValue(Gpio_PortF, Gpio_Pin1, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin2, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin3, PORT_PIN_LEVEL_LOW);
			break;
		case LED_RED:
			Port_SetPinValue(Gpio_PortF, Gpio_Pin1, PORT_PIN_LEVEL_HIGH);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin2, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin3, PORT_PIN_LEVEL_LOW);
			break;
		case LED_BLUE:
			Port_SetPinValue(Gpio_PortF, Gpio_Pin1, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin2, PORT_PIN_LEVEL_HIGH);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin3, PORT_PIN_LEVEL_LOW);
			break;
		case LED_GREEN:
			Port_SetPinValue(Gpio_PortF, Gpio_Pin1, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin2, PORT_PIN_LEVEL_LOW);
			Port_SetPinValue(Gpio_PortF, Gpio_Pin3, PORT_PIN_LEVEL_HIGH);
			break;
		default: //do nth
			break;			
	}
	
	
	
}