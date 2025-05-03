/******************************************************************************
*
* Module: GPIO
*
* File Name: GPIO.c
*
* Description: Source file for TM4C123GH6PM Microcontroller - GPIO Driver.
*
* Author: Mohanad Hany
******************************************************************************/

#include <assert.h>
#include "GPIO.h"

/******************************************************************************/ 
/**************************** PORT INIT FUNCTION ******************************/

/* Enables the clock for the port */
void Port_Init(uint8 Port_Num)
{
  volatile uint32 delay = 0;
  SYSCTL_RCGC2_R |= (1<<Port_Num);
  delay = SYSCTL_RCGC2_R;
}

/******************************************************************************/ 
/************************ PIN DIRECTION FUNCTION ******************************/

/* Sets the port pin direction */
void Port_SetPinDirection(uint8 Port_Num, Port_PinType Pin_Num, Port_PinDirection Direction)
{
  /* Checking JTAG and non existent pins are not chosen  */
  boolean error=FALSE;
  if ((Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) /* JTAG pins */
      || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)))
  {
    error=TRUE;
    assert(error);
  }
  else
  {
    /* Unlocking the JTAG pin in portD if chosen */
    if(Port_Num==Gpio_PortD && Pin_Num==Gpio_Pin7)
    {
      GPIO_PORTD_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTD_CR_R,Pin_Num);
    }
    /* Unlocking the JTAG pin in portF if chosen */
    else if(Port_Num==Gpio_PortF && Pin_Num==Gpio_Pin0)
    {
      GPIO_PORTF_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTF_CR_R,Pin_Num);    
    }
    /* Checking which direction chosen */
    switch (Direction)
    {
      
      /* In case INPUT direction was chosen */  
    case PORT_PIN_IN:
      /* Setting the direction for the chosen pin */
      switch (Port_Num)
      {
      case Gpio_PortA:  CLEAR_BIT(GPIO_PORTA_DIR_R,Pin_Num);
      break;
      case Gpio_PortB:  CLEAR_BIT(GPIO_PORTB_DIR_R,Pin_Num);
      break;
      case Gpio_PortC:  CLEAR_BIT(GPIO_PORTC_DIR_R,Pin_Num);
      break;
      case Gpio_PortD:  CLEAR_BIT(GPIO_PORTD_DIR_R,Pin_Num);
      break;
      case Gpio_PortE:  CLEAR_BIT(GPIO_PORTE_DIR_R,Pin_Num);
      break;
      case Gpio_PortF:  CLEAR_BIT(GPIO_PORTF_DIR_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }
      break;
      
      /* In case OUTPUT direction was chosen */   
    case PORT_PIN_OUT:
      /* Setting the direction for the chosen pin */
      switch (Port_Num)
      {
      case Gpio_PortA:  SET_BIT(GPIO_PORTA_DIR_R,Pin_Num);
      break;
      case Gpio_PortB:  SET_BIT(GPIO_PORTB_DIR_R,Pin_Num);
      break;
      case Gpio_PortC:  SET_BIT(GPIO_PORTC_DIR_R,Pin_Num);
      break;
      case Gpio_PortD:  SET_BIT(GPIO_PORTD_DIR_R,Pin_Num);
      break;
      case Gpio_PortE:  SET_BIT(GPIO_PORTE_DIR_R,Pin_Num);
      break;
      case Gpio_PortF:  SET_BIT(GPIO_PORTF_DIR_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }
      break;
    default: /* Do Nth */  break;
    }
  }
}

/******************************************************************************/ 
/************************** PIN MODE FUNCTION *********************************/

/* Sets the port pin mode */
void Port_SetPinMode(uint8 Port_Num, Port_PinType Pin_Num, PortPinMode Pin_Mode)
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  /* Checking JTAG and non existent pins are not chosen  */
  boolean error=FALSE;
  if ((Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) /* JTAG pins */
      || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)))
  {
    error=TRUE;
    assert(error);
  }
  else
  {
    /* Unlocking the JTAG pin in portD if chosen */
    if(Port_Num==Gpio_PortD && Pin_Num==Gpio_Pin7)
    {
      GPIO_PORTD_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTD_CR_R,Pin_Num);
    }
    /* Unlocking the JTAG pin in portF if chosen */
    else if(Port_Num==Gpio_PortF && Pin_Num==Gpio_Pin0)
    {
      GPIO_PORTF_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTF_CR_R,Pin_Num);    
    }
    
    /* Pointing a pointer towards the base address of the selected PORT */
    switch(Port_Num)
    {
    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    default: /* Do Nth */ break;
    }
    
    /********* Enable analog for ADC mode and disable it for the rest **********/
    if(Pin_Mode==PORT_PIN_MODE_ADC)
    {
      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Pin_Num);
      /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Pin_Num);
    }
    /********* Enabling Digital for all modes except ADC *************/
    else
    {
      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Pin_Num);
      /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Pin_Num);
    }
    
    
    /**************************** PORT MODES ****************************/
    
    switch(Pin_Mode)
    {
    case PORT_PIN_MODE_ADC:
      if((Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5)) \
        || (Port_Num==Gpio_PortC) || (Port_Num==Gpio_PortE) \
          || ((Port_Num==Gpio_PortD) && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)))
      {
        /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
      }
      else
      {
        /*Wrong pin chosen*/
      }      
      break;
      
    case PORT_PIN_MODE_DIO:
      /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
      /* Clear the PMCx bits for this pin */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));     
      break;
      
    case PORT_PIN_MODE_UART:
      if((Port_Num==Gpio_PortA && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)) \
        || (Port_Num==Gpio_PortC) \
          || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
            || (Port_Num==Gpio_PortE && (Pin_Num!=Gpio_Pin2 && Pin_Num!=Gpio_Pin3)) \
              || (Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Pin_Num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_UART_MODULE1:
      if (Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Pin_Num * 4));
      }
      else if (Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Pin_Num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_SPI:
      if((Port_Num==Gpio_PortA && (Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3 || Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5)) \
        || (Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
          || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) \
            || (Port_Num==Gpio_PortF && Pin_Num!=Gpio_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Pin_Num * 4));        
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_SPI_MODULE3:
      if (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Pin_Num * 4));
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_I2C:
      if((Port_Num==Gpio_PortA && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) \
          || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)) \
            || (Port_Num==Gpio_PortE && (Pin_Num!=Gpio_Pin4 && Pin_Num!=Gpio_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Pin_Num * 4));        
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_PWM_MODULE0:
      if((Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5)) \
          || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)) \
            || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin4 || Pin_Num!=Gpio_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Pin_Num * 4));              
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_PWM_MODULE1:
      if ((Port_Num==Gpio_PortA && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)) \
          || (Port_Num==Gpio_PortE && (Pin_Num!=Gpio_Pin4 && Pin_Num!=Gpio_Pin5)) \
            || (Port_Num==Gpio_PortF && Pin_Num!=Gpio_Pin4))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Pin_Num * 4));                    
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;      
      
    case PORT_PIN_MODE_GPT:
      if((Port_Num==Gpio_PortB) \
        || (Port_Num==Gpio_PortC) \
          || (Port_Num==Gpio_PortD) \
            || (Port_Num==Gpio_PortF))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Pin_Num * 4));                          
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_CAN:
      if((Port_Num==Gpio_PortA && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1)) \
        || (Port_Num==Gpio_PortB && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5)) \
          || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin4 || Pin_Num!=Gpio_Pin5)))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Pin_Num * 4));                                
      }
      else if(Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin3))      
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Pin_Num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_TIMER_PWM:
      if(Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3))
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000e << (Pin_Num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    case PORT_PIN_MODE_USB:
      if (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin4 || Pin_Num==Gpio_Pin5))
      {
        /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Pin_Num);
        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Pin_Num);
        /* Disable Alternative function for this pin by clearing the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Clear the PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));        
      }
      else if ((Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortD && (Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) \
          || (Port_Num==Gpio_PortF && Pin_Num==Gpio_Pin4))  
      {
        /* Enable Alternative function for this pin by Setting the corresponding bit in GPIOAFSEL register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Pin_Num);         
        /* Set the desired PMCx bits for this pin */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Pin_Num * 4));
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Pin_Num * 4));      
      }
      else
      {
        /* Wrong pin chosen for this mode */
      }
      break;
      
    default: /* Do nothing */ break;
    }
    
  }
}

/******************************************************************************/ 
/******************** PIN Internal Resistor FUNCTION **************************/

/* Sets internal resistor */
void Port_SetInternalResistor(uint8 Port_Num, Port_PinType Pin_Num, Port_InternalResistor resistor_direction)
{
  /* Checking JTAG and non existent pins are not chosen  */
  boolean error=FALSE;
  if ((Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) /* JTAG pins */
      || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)))
  {
    error=TRUE;
    assert(error);
  }
  else
  {
    /* Unlocking the JTAG pin in portD if chosen */
    if(Port_Num==Gpio_PortD && Pin_Num==Gpio_Pin7)
    {
      GPIO_PORTD_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTD_CR_R,Pin_Num);
    }
    /* Unlocking the JTAG pin in portF if chosen */
    else if(Port_Num==Gpio_PortF && Pin_Num==Gpio_Pin0)
    {
      GPIO_PORTF_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTF_CR_R,Pin_Num);    
    }
    /* Checking which internal resistor was chosen */
    switch (resistor_direction)
    {
      /* In case pull up */  
    case PULL_UP:
      /* Setting the chosen pin */
      switch (Port_Num)
      {
      case Gpio_PortA:    SET_BIT(GPIO_PORTA_PUR_R,Pin_Num);
      break;
      case Gpio_PortB:    SET_BIT(GPIO_PORTB_PUR_R,Pin_Num);
      break;
      case Gpio_PortC:    SET_BIT(GPIO_PORTC_PUR_R,Pin_Num);
      break;
      case Gpio_PortD:    SET_BIT(GPIO_PORTD_PUR_R,Pin_Num);
      break;
      case Gpio_PortE:    SET_BIT(GPIO_PORTE_PUR_R,Pin_Num);
      break;
      case Gpio_PortF:    SET_BIT(GPIO_PORTF_PUR_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }
      break;
      
      /* In case pull down */  
    case PULL_DOWN:
      /* Setting the chosen pin */    
      switch (Port_Num)
      {
      case Gpio_PortA:    SET_BIT(GPIO_PORTA_PDR_R,Pin_Num);
      break;
      case Gpio_PortB:    SET_BIT(GPIO_PORTB_PDR_R,Pin_Num);
      break;
      case Gpio_PortC:    SET_BIT(GPIO_PORTC_PDR_R,Pin_Num);
      break;
      case Gpio_PortD:    SET_BIT(GPIO_PORTD_PDR_R,Pin_Num);
      break;
      case Gpio_PortE:    SET_BIT(GPIO_PORTE_PDR_R,Pin_Num);
      break;
      case Gpio_PortF:    SET_BIT(GPIO_PORTF_PDR_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }
      break;
      
      /* In case no internal resistor needed */  
    case OFF:
      /* Clearing both pull up and down registers for the chosen pin */
      switch (Port_Num)
      {
        
      case Gpio_PortA:    
        CLEAR_BIT(GPIO_PORTA_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTA_PDR_R,Pin_Num);
        break;
        
      case Gpio_PortB:
        CLEAR_BIT(GPIO_PORTB_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTB_PDR_R,Pin_Num);
        break;
        
      case Gpio_PortC:
        CLEAR_BIT(GPIO_PORTC_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTC_PDR_R,Pin_Num);
        break;
        
      case Gpio_PortD:
        CLEAR_BIT(GPIO_PORTD_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTD_PDR_R,Pin_Num);      
        break;
        
      case Gpio_PortE:
        CLEAR_BIT(GPIO_PORTE_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTE_PDR_R,Pin_Num);
        break;
        
      case Gpio_PortF:
        CLEAR_BIT(GPIO_PORTF_PUR_R,Pin_Num);
        CLEAR_BIT(GPIO_PORTF_PDR_R,Pin_Num);
        break;
        
      default:/* Do Nth */ break;
      }    
      break;
    }
  }
}

/******************************************************************************/ 
/************************** PIN Value FUNCTION ********************************/

/* Set pin value if output */
void Port_SetPinValue(uint8 Port_Num, Port_PinType Pin_Num, PortPinLevelValue Pin_Value)
{
  /* Checking JTAG and non existent pins are not chosen  */
  boolean error=FALSE;
  if ((Port_Num==Gpio_PortC && (Pin_Num==Gpio_Pin0 || Pin_Num==Gpio_Pin1 || Pin_Num==Gpio_Pin2 || Pin_Num==Gpio_Pin3)) /* JTAG pins */
      || (Port_Num==Gpio_PortE && (Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)) \
        || (Port_Num==Gpio_PortF && (Pin_Num==Gpio_Pin5 || Pin_Num==Gpio_Pin6 || Pin_Num==Gpio_Pin7)))
  {
    error=TRUE;
    assert(error);
  }
  else
  {
    /* Unlocking the JTAG pin in portD if chosen */
    if(Port_Num==Gpio_PortD && Pin_Num==Gpio_Pin7)
    {
      GPIO_PORTD_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTD_CR_R,Pin_Num);
    }
    /* Unlocking the JTAG pin in portF if chosen */
    else if(Port_Num==Gpio_PortF && Pin_Num==Gpio_Pin0)
    {
      GPIO_PORTF_LOCK_R=0x4C4F434B;
      SET_BIT(GPIO_PORTF_CR_R,Pin_Num);    
    }
    switch(Pin_Value)
    {
      /* In case pin value = 0 */
    case PORT_PIN_LEVEL_LOW:
      /* Clearing the value of the chosen pin */
      switch (Port_Num)
      {
      case Gpio_PortA:    CLEAR_BIT(GPIO_PORTA_DATA_R,Pin_Num);
      break;
      case Gpio_PortB:    CLEAR_BIT(GPIO_PORTB_DATA_R,Pin_Num);
      break;
      case Gpio_PortC:    CLEAR_BIT(GPIO_PORTC_DATA_R,Pin_Num);
      break;
      case Gpio_PortD:    CLEAR_BIT(GPIO_PORTD_DATA_R,Pin_Num);
      break;
      case Gpio_PortE:    CLEAR_BIT(GPIO_PORTE_DATA_R,Pin_Num);
      break;
      case Gpio_PortF:    CLEAR_BIT(GPIO_PORTF_DATA_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }    
      break;
      /* In case pin value = 1 */  
    case PORT_PIN_LEVEL_HIGH:
      /* Setting the value of the chosen pin */
      switch (Port_Num)
      {
      case Gpio_PortA:    SET_BIT(GPIO_PORTA_DATA_R,Pin_Num);
      break;
      case Gpio_PortB:    SET_BIT(GPIO_PORTB_DATA_R,Pin_Num);
      break;
      case Gpio_PortC:    SET_BIT(GPIO_PORTC_DATA_R,Pin_Num);
      break;
      case Gpio_PortD:    SET_BIT(GPIO_PORTD_DATA_R,Pin_Num);
      break;
      case Gpio_PortE:    SET_BIT(GPIO_PORTE_DATA_R,Pin_Num);
      break;
      case Gpio_PortF:    SET_BIT(GPIO_PORTF_DATA_R,Pin_Num);
      break;
      default:/* Do Nth */ break;
      }    
      break;
      
    default: /* Do Nth */ break;   
    }
  }
}


uint8 Port_ReadPin(uint8 Port_Num, Port_PinType Pin_Num)
{
   switch(Port_Num)
  {
  case Gpio_PortA:
   return Get_Bit(GPIO_PORTA_DATA_R,Pin_Num); 
   break;
   
  case Gpio_PortB:
   return Get_Bit(GPIO_PORTB_DATA_R,Pin_Num);
   break;
   
  case Gpio_PortC:
   return Get_Bit(GPIO_PORTC_DATA_R,Pin_Num);
   break;
   
  case Gpio_PortD:
   return Get_Bit(GPIO_PORTD_DATA_R,Pin_Num);
   break;
   
  case Gpio_PortE:
   return Get_Bit(GPIO_PORTE_DATA_R,Pin_Num);
   break;
   
  case Gpio_PortF:
   return  Get_Bit(GPIO_PORTF_DATA_R,Pin_Num);
   break;
   
  default:
    return -1;
  }
}


void Port_EnableInterrupt(uint8 Port_Num, Port_PinType Pin_Num, Interrupt_Edge edge_type)
{
  switch(Port_Num)
  {
    case Gpio_PortA:
      GPIO_PORTA_IS_R  &= ~(1 << Pin_Num); // Edge-sensitive

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTA_IBE_R |= (1 << Pin_Num); // Both edges
      }
      else
      {
        GPIO_PORTA_IBE_R &= ~(1 << Pin_Num); // Single edge
        if (edge_type == EDGE_RISING)
          GPIO_PORTA_IEV_R |= (1 << Pin_Num);  // Rising edge
        else
          GPIO_PORTA_IEV_R &= ~(1 << Pin_Num); // Falling edge
      }

      GPIO_PORTA_ICR_R = (1 << Pin_Num); // Clear any prior interrupt
      GPIO_PORTA_IM_R  |= (1 << Pin_Num); // Enable interrupt
      NVIC_EN0_R |= (1 << 0); // IRQ0 = GPIOA
      break;

    case Gpio_PortB:
      GPIO_PORTB_IS_R  &= ~(1 << Pin_Num);

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTB_IBE_R |= (1 << Pin_Num);
      }
      else
      {
        GPIO_PORTB_IBE_R &= ~(1 << Pin_Num);
        if (edge_type == EDGE_RISING)
          GPIO_PORTB_IEV_R |= (1 << Pin_Num);
        else
          GPIO_PORTB_IEV_R &= ~(1 << Pin_Num);
      }

      GPIO_PORTB_ICR_R = (1 << Pin_Num);
      GPIO_PORTB_IM_R  |= (1 << Pin_Num);
      NVIC_EN0_R |= (1 << 1); // IRQ1 = GPIOB
      break;

    case Gpio_PortC:
      GPIO_PORTC_IS_R  &= ~(1 << Pin_Num);

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTC_IBE_R |= (1 << Pin_Num);
      }
      else
      {
        GPIO_PORTC_IBE_R &= ~(1 << Pin_Num);
        if (edge_type == EDGE_RISING)
          GPIO_PORTC_IEV_R |= (1 << Pin_Num);
        else
          GPIO_PORTC_IEV_R &= ~(1 << Pin_Num);
      }

      GPIO_PORTC_ICR_R = (1 << Pin_Num);
      GPIO_PORTC_IM_R  |= (1 << Pin_Num);
      NVIC_EN0_R |= (1 << 2); // IRQ2 = GPIOC
      break;

    case Gpio_PortD:
      GPIO_PORTD_IS_R  &= ~(1 << Pin_Num);

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTD_IBE_R |= (1 << Pin_Num);
      }
      else
      {
        GPIO_PORTD_IBE_R &= ~(1 << Pin_Num);
        if (edge_type == EDGE_RISING)
          GPIO_PORTD_IEV_R |= (1 << Pin_Num);
        else
          GPIO_PORTD_IEV_R &= ~(1 << Pin_Num);
      }

      GPIO_PORTD_ICR_R = (1 << Pin_Num);
      GPIO_PORTD_IM_R  |= (1 << Pin_Num);
      NVIC_EN0_R |= (1 << 3); // IRQ3 = GPIOD
      break;

    case Gpio_PortE:
      GPIO_PORTE_IS_R  &= ~(1 << Pin_Num);

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTE_IBE_R |= (1 << Pin_Num);
      }
      else
      {
        GPIO_PORTE_IBE_R &= ~(1 << Pin_Num);
        if (edge_type == EDGE_RISING)
          GPIO_PORTE_IEV_R |= (1 << Pin_Num);
        else
          GPIO_PORTE_IEV_R &= ~(1 << Pin_Num);
      }

      GPIO_PORTE_ICR_R = (1 << Pin_Num);
      GPIO_PORTE_IM_R  |= (1 << Pin_Num);
      NVIC_EN0_R |= (1 << 4); // IRQ4 = GPIOE
      break;

    case Gpio_PortF:
      GPIO_PORTF_IS_R  &= ~(1 << Pin_Num);

      if (edge_type == EDGE_BOTH)
      {
        GPIO_PORTF_IBE_R |= (1 << Pin_Num);
      }
      else
      {
        GPIO_PORTF_IBE_R &= ~(1 << Pin_Num);
        if (edge_type == EDGE_RISING)
          GPIO_PORTF_IEV_R |= (1 << Pin_Num);
        else
          GPIO_PORTF_IEV_R &= ~(1 << Pin_Num);
      }

      GPIO_PORTF_ICR_R = (1 << Pin_Num);
      GPIO_PORTF_IM_R  |= (1 << Pin_Num);
      NVIC_EN0_R |= (1 << 30); // IRQ30 = GPIOF
      break;

    default:
      // Invalid port number
      break;
  }
}


void PWM0_Init(void) {
  SYSCTL_RCGCPWM_R |= 0x01;       // Enable PWM0 clock
  SYSCTL_RCGCGPIO_R |= (1 << 1);  // Enable Port B clock
  volatile int delay = SYSCTL_RCGCGPIO_R; // dummy delay

  // Configure PB6 for PWM
  Port_SetPinDirection(Gpio_PortB, Gpio_Pin6, PORT_PIN_OUT);
  Port_SetPinMode(Gpio_PortB, Gpio_Pin6, PORT_PIN_MODE_PWM_MODULE0);

  SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
  SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000E0000) | (7 << 17); // PWM clock = system clock/64

  PWM0_0_CTL_R = 0;             // disable PWM while configuring
  PWM0_0_GENA_R = 0x0000008C;   // output high on load, low on comparator A down
  PWM0_0_LOAD_R = 124;    // Set period for 0.25kHz PWM if 10MHz clock
  PWM0_0_CMPA_R = 31;     // Set duty cycle (25% initially)
  PWM0_0_CTL_R = 1;             // enable PWM0
  PWM0_ENABLE_R |= 0x01;        // Enable PWM0 output on PB6
}

