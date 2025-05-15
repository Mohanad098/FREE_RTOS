#include "HAL.h"

void H_Ports_Init(void)
{
	  Port_Init(Gpio_PortA);
	  Port_Init(Gpio_PortB);
	  Port_Init(Gpio_PortC);
	  Port_Init(Gpio_PortD);
	  Port_Init(Gpio_PortF);
}

void H_PB_init(void)
{
	/* Driver Up */
	
	Port_SetPinMode(Gpio_PortD, Gpio_Pin2, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortD, Gpio_Pin2, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortD, Gpio_Pin2, PULL_UP);
  Port_EnableInterrupt(Gpio_PortD, Gpio_Pin2, EDGE_FALLING);
	
	/* Driver down */
	
	Port_SetPinMode(Gpio_PortD, Gpio_Pin1, PORT_PIN_MODE_DIO); 
	Port_SetPinDirection(Gpio_PortD, Gpio_Pin1, PORT_PIN_IN);
	Port_SetInternalResistor(Gpio_PortD, Gpio_Pin1, PULL_UP);
	Port_EnableInterrupt(Gpio_PortD, Gpio_Pin1, EDGE_FALLING); 
	
	/* Passenger Up */
	Port_SetPinMode(Gpio_PortC, Gpio_Pin4, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortC, Gpio_Pin4, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortC, Gpio_Pin4, PULL_UP);
  Port_EnableInterrupt(Gpio_PortC, Gpio_Pin4, EDGE_FALLING);
	
	/* Passenger down */
	Port_SetPinMode(Gpio_PortC, Gpio_Pin5, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortC, Gpio_Pin5, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortC, Gpio_Pin5, PULL_UP);
  Port_EnableInterrupt(Gpio_PortC, Gpio_Pin5, EDGE_FALLING);
	
	/* Jamming IR sensor*/
	Port_SetPinMode(Gpio_PortC, Gpio_Pin6, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortC, Gpio_Pin6, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortC, Gpio_Pin6, PULL_UP);
  Port_EnableInterrupt(Gpio_PortC, Gpio_Pin6, EDGE_FALLING);
	
	/* Locking */
	Port_SetPinMode(Gpio_PortC, Gpio_Pin7, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortC, Gpio_Pin7, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortC, Gpio_Pin7, PULL_UP);	
	Port_EnableInterrupt(Gpio_PortC, Gpio_Pin7, EDGE_BOTH);

}

void H_Limit_init(void)
{
	/* Limit switch for upwards movement */
	Port_SetPinMode(Gpio_PortD, Gpio_Pin6, PORT_PIN_MODE_DIO);
	Port_SetPinDirection(Gpio_PortD, Gpio_Pin6, PORT_PIN_IN);
	Port_SetInternalResistor(Gpio_PortD, Gpio_Pin6, PULL_UP);
	Port_EnableInterrupt(Gpio_PortD, Gpio_Pin6, EDGE_RISING);
	
	/* Limit switch for downwards movement */
	Port_SetPinMode(Gpio_PortD, Gpio_Pin7, PORT_PIN_MODE_DIO);
	Port_SetPinDirection(Gpio_PortD, Gpio_Pin7, PORT_PIN_IN);
	Port_SetInternalResistor(Gpio_PortD, Gpio_Pin7, PULL_UP);
	Port_EnableInterrupt(Gpio_PortD, Gpio_Pin7, EDGE_RISING);
}

void H_Motor_Init(void) {

  // IN1 and IN2 as output
	Port_SetPinMode(Gpio_PortB, Gpio_Pin0, PORT_PIN_MODE_DIO);
  Port_SetPinMode(Gpio_PortB, Gpio_Pin1, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortB, Gpio_Pin0, PORT_PIN_OUT);
  Port_SetPinDirection(Gpio_PortB, Gpio_Pin1, PORT_PIN_OUT);

	PWM0_Init();
  // PWM init on PB6 can be added if needed (we'll just use direction for now)
}

void H_Encoder_Init(void) {

  // A (PA2) and B (PA3) as input
  Port_SetPinMode(Gpio_PortA, Gpio_Pin2, PORT_PIN_MODE_DIO);
  Port_SetPinMode(Gpio_PortA, Gpio_Pin3, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortA, Gpio_Pin2, PORT_PIN_IN);
  Port_SetPinDirection(Gpio_PortA, Gpio_Pin3, PORT_PIN_IN);
  Port_SetInternalResistor(Gpio_PortA, Gpio_Pin2, PULL_UP);
  Port_SetInternalResistor(Gpio_PortA, Gpio_Pin3, PULL_UP);

  Port_EnableInterrupt(Gpio_PortA, Gpio_Pin2, EDGE_RISING);
}

void H_Leds_Init(void)
{
	//Red Led
	Port_SetPinMode(Gpio_PortF, Gpio_Pin1, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortF, Gpio_Pin1, PORT_PIN_OUT);
	
	//Blue Led 
	Port_SetPinMode(Gpio_PortF, Gpio_Pin2, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortF, Gpio_Pin2, PORT_PIN_OUT);
	
	//Green Led
	Port_SetPinMode(Gpio_PortF, Gpio_Pin3, PORT_PIN_MODE_DIO);
  Port_SetPinDirection(Gpio_PortF, Gpio_Pin3, PORT_PIN_OUT);
}