 /******************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - GPIO Driver.
 *
 * Author: Mohanad Hany
 ******************************************************************************/

#include "Platform_Types.h"
#include "Common_Macros.h"
#include "tm4c123gh6pm.h"

#ifndef GPIO_H
#define GPIO_H

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirection;

/* Description: Enum to hold PIN initial value */
typedef enum{
  PORT_PIN_LEVEL_LOW,PORT_PIN_LEVEL_HIGH
}PortPinLevelValue;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Description: Enum to hold PIN initial mode */
typedef enum
{
  PORT_PIN_MODE_ADC,
  PORT_PIN_MODE_DIO,
  PORT_PIN_MODE_UART,
  PORT_PIN_MODE_UART_MODULE1,
  PORT_PIN_MODE_SPI,
  PORT_PIN_MODE_SPI_MODULE3,
  PORT_PIN_MODE_I2C,
  PORT_PIN_MODE_PWM_MODULE0,
  PORT_PIN_MODE_PWM_MODULE1,
  PORT_PIN_MODE_GPT,
  PORT_PIN_MODE_CAN,
  PORT_PIN_MODE_TIMER_PWM,
  PORT_PIN_MODE_USB
    
}PortPinMode;

typedef enum {
    EDGE_RISING,
    EDGE_FALLING,
		EDGE_BOTH
} Interrupt_Edge;

/* Description: Shall cover max available port pins (8) for TM4C */
typedef uint8 Port_PinType;

/* Port Index in the array of structures in Port_PBcfg.c */
#define Gpio_PortA                          (uint8)0x00
#define Gpio_PortB                          (uint8)0x01
#define Gpio_PortC                          (uint8)0x02
#define Gpio_PortD                          (uint8)0x03
#define Gpio_PortE                          (uint8)0x04
#define Gpio_PortF                          (uint8)0x05

/* Pin Index in the array of structures in Port_PBcfg.c */
#define Gpio_Pin0                           (uint8)0x00
#define Gpio_Pin1                           (uint8)0x01
#define Gpio_Pin2                           (uint8)0x02
#define Gpio_Pin3                           (uint8)0x03
#define Gpio_Pin4                           (uint8)0x04
#define Gpio_Pin5                           (uint8)0x05
#define Gpio_Pin6                           (uint8)0x06
#define Gpio_Pin7                           (uint8)0x07


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/*** Function Enables the clock for the port ***/
/* Arguments used:
1) uint8 port numnber:
Gpio_PortA
Gpio_PortB
Gpio_PortC
Gpio_PortD
Gpio_PortE
Gpio_PortF
******************************************************************************/
void Port_Init(uint8 Port_Num);


/*****************************************************************************/
/*** Function Sets the port pin direction ***/
/* Arguments used:
1) uint8 port numnber:
Gpio_PortA
Gpio_PortB
Gpio_PortC
Gpio_PortD
Gpio_PortE
Gpio_PortF

2) Port_PinType Pin_Num:
Gpio_Pin0
Gpio_Pin1
Gpio_Pin2
Gpio_Pin3
Gpio_Pin4
Gpio_Pin5
Gpio_Pin6
Gpio_Pin7

3) Port_PinDirection:
PORT_PIN_IN
PORT_PIN_OUT
******************************************************************************/
void Port_SetPinDirection(uint8 Port_Num, Port_PinType Pin_Num, Port_PinDirection Direction);


/*****************************************************************************/
/* Sets the port pin mode */
/* Arguments used:
1) uint8 port numnber
2) Port_PinType Pin_Num
3) PortPinMode:
  PORT_PIN_MODE_ADC,
  PORT_PIN_MODE_DIO,
  PORT_PIN_MODE_UART,
  PORT_PIN_MODE_UART_MODULE1,
  PORT_PIN_MODE_SPI,
  PORT_PIN_MODE_SPI_MODULE3,
  PORT_PIN_MODE_I2C,
  PORT_PIN_MODE_PWM_MODULE0,
  PORT_PIN_MODE_PWM_MODULE1,
  PORT_PIN_MODE_GPT,
  PORT_PIN_MODE_CAN,
  PORT_PIN_MODE_TIMER_PWM,
  PORT_PIN_MODE_USB
******************************************************************************/
void Port_SetPinMode(uint8 Port_Num, Port_PinType Pin_Num, PortPinMode Pin_Mode);


/*****************************************************************************/
/* Sets internal resistor */
/* Arguments used:
1) uint8 port numnber
2) Port_PinType Pin_Num
3) Port_InternalResistor:
OFF
PULL_UP
PULL_DOWN
******************************************************************************/
void Port_SetInternalResistor(uint8 Port_Num, Port_PinType Pin_Num, Port_InternalResistor resistor_direction);


/*****************************************************************************/
/* Set pin value if output */
/* Arguments used:
1) uint8 port numnber
2) Port_PinType Pin_Num
3) PortPinLevelValue:
PORT_PIN_LEVEL_LOW
PORT_PIN_LEVEL_HIGH
******************************************************************************/
void Port_SetPinValue(uint8 Port_Num, Port_PinType Pin_Num, PortPinLevelValue Pin_Value);


/*****************************************************************************/
/* Read pin value */
/* Arguments used:
1) uint8 port number
2) pin number
******************************************************************************/
uint8 Port_ReadPin(uint8 Port_Num, Port_PinType Pin_Num);


/*****************************************************************************/
/* Enable interrupt */
/* Arguments used:
1) uint8 port numnber
2) Port_PinType Pin_Num
3) Rising or falling edge
EDGE_RISING
EDGE_FALLING
*/
void Port_EnableInterrupt(uint8 Port_Num, Port_PinType Pin_Num, Interrupt_Edge edge_type);

void PWM0_Init(void);

#endif /* PORT_H */
