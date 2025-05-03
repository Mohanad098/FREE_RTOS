/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Stellaris library includes. */
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"

/* Drivers */
#include "GPIO.h"
#include "MCAL.h"
#include "LCD_I2C.h"

/* Global variables */
volatile int32_t encoder_count = 0;
uint8 control = 0;
sint16 angle = 0;
uint8 motor_revs = 0;

/* Declaretions */
#define PPR 11
#define GEAR_RATIO 20  // Adjust to your motor's actual ratio
#define ULONG_MAX			0xFFFFFFFF

/* Task handle */
TaskHandle_t xControlTaskHandle;
TaskHandle_t xDriverUpHandle;
TaskHandle_t xPassengerUpHandle;

/* Semaphores */
xSemaphoreHandle xDriverUp;
xSemaphoreHandle xDriverDown;
xSemaphoreHandle xPassengerUp;
xSemaphoreHandle xPassengerDown;
xSemaphoreHandle xJamming;
xSemaphoreHandle xPassengerUpAuto;
xSemaphoreHandle xPassengerDownAuto;

/* Queues */
xQueueHandle xQueue;

/* Tasks prototype */
void vControlTask(void *pvParameters);
void vDriverUp(void *pvParameters);
void vDriverDown(void *pvParameters);
void vPassengerUp(void *pvParameters);
void vPassengerDown(void *pvParameters);
void vJamming (void *pvParameters);
void vLCD (void *pvParameters);

void u8_to_str(uint8_t num, char *str) {
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    int i = 0;
    while (num > 0) {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }

    str[i] = '\0';

    // Reverse the string
    for (int j = 0; j < i / 2; j++) {
        char temp = str[j];
        str[j] = str[i - j - 1];
        str[i - j - 1] = temp;
    }
}

int main(void)
{
		M_Init();

		/* Create semaphores */
		vSemaphoreCreateBinary(xDriverUp);
		vSemaphoreCreateBinary(xDriverDown);
		vSemaphoreCreateBinary(xPassengerUp);
		vSemaphoreCreateBinary(xPassengerDown);	
		vSemaphoreCreateBinary(xJamming);	
		vSemaphoreCreateBinary(xPassengerUpAuto);
		vSemaphoreCreateBinary(xPassengerDownAuto);
		xQueue = xQueueCreate( 1, sizeof( uint8 ) );
	
		if(( xDriverUp != NULL )&& (xDriverDown !=NULL ) && (xPassengerUp !=NULL) && (xPassengerDown !=NULL) && (xJamming !=NULL) && (xPassengerUpAuto !=NULL) && (xPassengerDownAuto !=NULL) && xQueue != NULL)
		{
			//Empty semaphores
			xSemaphoreTake(xDriverUp, 0);
			xSemaphoreTake(xDriverDown,0);
			xSemaphoreTake(xPassengerUp,0);
			xSemaphoreTake(xPassengerDown,0);	
			xSemaphoreTake(xJamming,0);	
			xSemaphoreTake(xPassengerUpAuto,0);
			xSemaphoreTake(xPassengerDownAuto,0);
			//Initializing Tasks
			xTaskCreate( vControlTask,"Control Task", configMINIMAL_STACK_SIZE, NULL,5,&xControlTaskHandle);
			xTaskCreate( vDriverUp, "Driver upwards", 250, NULL, 3, &xDriverUpHandle );
			xTaskCreate( vDriverDown, "Driver downwards", 250, NULL, 3, NULL );
			xTaskCreate( vPassengerUp, "Passenger Up", 200, NULL, 3, &xPassengerUpHandle );
			xTaskCreate( vPassengerDown, "Passenger Down", 200, NULL, 3, NULL );
		  xTaskCreate( vJamming, "Jamming", 200, NULL, 4, NULL );
			xTaskCreate( vLCD,	"Printing", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
			vTaskStartScheduler();
			
		}
		
    for(;;);
}

/***********************************************************************/
/* Control Task */
/*
Gets notified when a push button is pressed and tasks needs to change
*/
void vControlTask(void *pvParameters)
{
		uint32_t notificationValue;
    BaseType_t control_t;
		portBASE_TYPE xStatus;
		LCD_Init();
		LCD_Print("Window control");
		while (1) 
		{
			// Wait indefinitely for a notification
			control_t = xTaskNotifyWait(
					0,                      // Clear no bits on entry
					ULONG_MAX,               // Clear all bits on exit
					&notificationValue,      // Stores the received value
					portMAX_DELAY            // Wait indefinitely
			);
			if(control_t == pdTRUE)
			{
				switch(notificationValue)
				{
					case JAM: 				xSemaphoreGive(xJamming);
						break;
					case DRIVER_UP: 	xSemaphoreGive(xDriverUp);
						break;
					case DRIVER_DOWN: xSemaphoreGive(xDriverDown);
						break;
					case PASS_UP: 		xSemaphoreGive(xPassengerUp);
						break;
					case PASS_DOWN: 	xSemaphoreGive(xPassengerDown);
						break;
					case LIMIT_UP: 
						M_MotorControl(STOP);
						control = WINDOW_STOP;
						xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
						break;
					case LIMIT_DOWN: 
						M_MotorControl(STOP);
						control = WINDOW_STOP;
						xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
						break;
					default: //do nothing
						break;
				}
			}
    }
}

/********* Task 1 ***********/
/*
Description: 
*/
void vDriverUp(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;

  while(1)
  {
		xSemaphoreTake( xDriverUp, portMAX_DELAY );
		control = DRIVER_UP;
		M_MotorControl(CW);
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
		if(M_Read(DRIVER_UP) && control != JAM) //button not pressed after 50ms
		{
			control = DRIVER_UP;
			M_MotorControl(CW);
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send driver up to lcd
		}
		else if (!M_Read(DRIVER_UP))
		{//Driver up button pressed - Limit switch not pressed - Driver down PB not pressed - didn't come from Jam
			control = DRIVER_UP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send driver up to lcd
			while (!M_Read(DRIVER_UP) && !M_Read(LIMIT_UP) && M_Read(DRIVER_DOWN) && control != JAM && motor_revs!=10) // Button is pressed (because PULL-UP, pressed = 0)
			{       
				M_MotorControl(CW);
				//angle = ((float)encoder_count / (PPR * GEAR_RATIO)) * 360.0;
				//vPrintStringAndNumber("Motor angle = ",angle);
			}
			// Button not pressed
			M_MotorControl(STOP);
			control = WINDOW_STOP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
		}
    
  }
}

void vDriverDown(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;

  while(1)
  {
		xSemaphoreTake( xDriverDown, portMAX_DELAY );
    control = DRIVER_DOWN;
		M_MotorControl(CCW);
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
		if(M_Read(DRIVER_DOWN)) //button not pressed after 50ms
		{
			control = DRIVER_DOWN;
			M_MotorControl(CCW);
			xStatus = xQueueSendToBack( xQueue, &control, 0 );
		}
		else if (!M_Read(DRIVER_DOWN))
		{//Driver up button pressed - Limit switch not pressed - Driver down PB not pressed
			control = DRIVER_DOWN;
			xStatus = xQueueSendToBack( xQueue, &control, 0 );
			while (!M_Read(DRIVER_DOWN) && !M_Read(LIMIT_DOWN) && M_Read(DRIVER_UP) && motor_revs!=0) // Button is pressed (because PULL-UP, pressed = 0)
			{       
				M_MotorControl(CCW);
				//angle = ((float)encoder_count / (PPR * GEAR_RATIO)) * 360.0;
				//vPrintStringAndNumber("Motor angle = ",angle);
			}
			// Button not pressed
			M_MotorControl(STOP);
			control = WINDOW_STOP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
		}
    
  }
}

void vPassengerUp(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;

  while(1)
  {
		xSemaphoreTake( xPassengerUp, portMAX_DELAY );
		control = PASS_UP;
		M_MotorControl(CW);
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
		if(M_Read(PASS_UP) && control != JAM) //button not pressed after 50ms
		{
			control = PASS_UP;
			M_MotorControl(CW);
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send passenger up to LCD
		}
		else if (!M_Read(PASS_UP))
		{//Driver up button pressed - Limit switch not pressed - Driver down PB not pressed - Not locked - didn't come from jam
			control = PASS_UP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send passenger up to lcd
			while (!M_Read(PASS_UP) && !M_Read(LIMIT_UP) && M_Read(PASS_DOWN) && M_Read(LOCK) && control != JAM && motor_revs!=10) // Button is pressed (because PULL-UP, pressed = 0)
			{       
				M_MotorControl(CW);
				//angle = ((float)encoder_count / (PPR * GEAR_RATIO)) * 360.0;
				//vPrintStringAndNumber("Motor angle = ",angle);
			}
			// Button not pressed
			M_MotorControl(STOP);
			control = WINDOW_STOP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
		}
    
  }
}

void vPassengerDown (void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;
	
  while(1)
  {
		xSemaphoreTake( xPassengerDown, portMAX_DELAY );
		control = PASS_DOWN;
		M_MotorControl(CCW);
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
		if(M_Read(PASS_DOWN)) //button not pressed after 50ms
		{
			control = PASS_DOWN;
			M_MotorControl(CCW);
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send passenger down to lcd
		}
		else if (!M_Read(PASS_DOWN))
		{//Driver up button pressed - Limit switch not pressed - Driver down PB not pressed
			control = PASS_DOWN;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send passenger down to lcd
			while (!M_Read(PASS_DOWN) && !M_Read(LIMIT_DOWN) && M_Read(PASS_UP) && M_Read(LOCK) && motor_revs!=0) // Button is pressed (because PULL-UP, pressed = 0)
			{       
				M_MotorControl(CCW);
				//angle = ((float)encoder_count / (PPR * GEAR_RATIO)) * 360.0;
				//vPrintStringAndNumber("Motor angle = ",angle);
			}
			// Button not pressed
			M_MotorControl(STOP);
			control = WINDOW_STOP;
			xStatus = xQueueSendToBack( xQueue, &control, 0 ); //send window stop to lcd
		}
    
  }
}


void vJamming (void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	portBASE_TYPE xStatus;
	for(;;)
	{
		xSemaphoreTake(xJamming,portMAX_DELAY);
		control = JAM;
		M_MotorControl(STOP);
		vTaskDelay(pdMS_TO_TICKS(500));
		xStatus = xQueueSendToBack( xQueue, &control, 0 );
		M_MotorControl(CCW);
		vTaskDelay(pdMS_TO_TICKS(2000));
		M_MotorControl(STOP);
				
	}
}

void vLCD (void *pvParameters)
{
	portBASE_TYPE xStatus;
	uint8 uReceivedValue;
	const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	for(;;)
	{
		if( uxQueueMessagesWaiting( xQueue ) != 0 )
		{
			//vPrintString( "Queue should have been empty!\r\n" );
		}
		xStatus = xQueueReceive( xQueue, &uReceivedValue, portMAX_DELAY );

		if( xStatus == pdPASS )
		{
			/* Data was successfully received from the queue, print out the received
			value. */
			
			if(uReceivedValue <= 10)
			{
				uint8 str[4] = {0}; 
				u8_to_str(uReceivedValue*5, str); //convert to cm 1 rev -> 5 cm
				LCD_SetCursor(1,0);
				LCD_Print("Window at ");	
				if(uReceivedValue*5 == 5)
				{
					LCD_SetCursor(1,11);
					LCD_Print("05");
				}
				else
				{
					LCD_SetCursor(1,11);
					LCD_Print(str);
				}
			}
			
			else
			{
				switch(uReceivedValue)
				{
					case JAM: 				
						LCD_Clear();
						LCD_Print("Obstacle detect");
						break;
					case DRIVER_UP: 	
					case PASS_UP: 		
						LCD_Clear();	
						LCD_Print("Window up");
						break;
					case DRIVER_DOWN: 			
					case PASS_DOWN: 	
						LCD_Clear();
						LCD_Print("Window down");
						break;
					case LIMIT_UP:    
						LCD_Clear();
						LCD_Print("Limit upwards");
						break;
					case LIMIT_DOWN: 	
						LCD_Clear();
						LCD_Print("Limit downwards");
						break;
					case WINDOW_STOP:
						LCD_Clear();
						LCD_Print("Window stop");
						break;
					default: //do nothing
						break;
				}
			}
		}
		else
		{
			/* We did not receive anything from the queue even after waiting for 100ms.
			This must be an error as the sending tasks are free running and will be
			continuously writing to the queue. */
		}
	}
}



/*********************************************** ISR ***********************************************/



/* GPIO A handler, for encoder */

void GPIOA_Handler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_PORTA_RIS_R & (1 << Gpio_Pin2)) {
    uint8_t channelB = (GPIO_PORTA_DATA_R >> Gpio_Pin3) & 0x01;
    if (channelB)
      encoder_count++;
    else
      encoder_count--;
    GPIO_PORTA_ICR_R = (1 << Gpio_Pin2); // clear interrupt
		// 600 = 1 rev
		angle = (sint16) (((float)encoder_count / (PPR * GEAR_RATIO)) * 360.0);
		if(angle < 0)
		{
			angle = 0;
			motor_revs = 0;
			xTaskNotifyFromISR(xControlTaskHandle,LIMIT_DOWN,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
		}
		else if(angle > 6000)
		{
			motor_revs = 10;
			xTaskNotifyFromISR(xControlTaskHandle,LIMIT_UP,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
		}
		else
		{
			motor_revs = angle / 600 ;
		}
		if(angle % 300 == 0)
		{
			xQueueSendFromISR(xQueue, &motor_revs, &xHigherPriorityTaskWoken);
		}
		
  }
	// Yield if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIOC_Handler(void) {
	static TickType_t last_interrupt_time = 0;
	TickType_t current_time = xTaskGetTickCountFromISR();
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_PORTC_RIS_R & (1 << Gpio_Pin4)) 
	{	
    GPIO_PORTC_ICR_R = (1 << Gpio_Pin4); // clear interrupt
		if(M_Read(LOCK))
		{
			xTaskNotifyFromISR(xControlTaskHandle,PASS_UP,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
			//(Task to notify, Notification value, Overwrite previous value, )
		}
  }
	else if (GPIO_PORTC_RIS_R & (1 << Gpio_Pin5)) 
	{	
    GPIO_PORTC_ICR_R = (1 << Gpio_Pin5); // clear interrupt
		if(M_Read(LOCK))
		{
			xTaskNotifyFromISR(xControlTaskHandle,PASS_DOWN,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
			//(Task to notify, Notification value, Overwrite previous value, )
		}
  }	
	else if (GPIO_PORTC_RIS_R & (1 << Gpio_Pin6)) 
	{	
    GPIO_PORTC_ICR_R = (1 << Gpio_Pin6); // clear interrupt
		if(control == DRIVER_UP || control == PASS_UP)
		{
			if ((current_time - last_interrupt_time) > pdMS_TO_TICKS(200)) // 100ms debounce
			{ 
				xTaskNotifyFromISR(xControlTaskHandle,JAM,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
				//(Task to notify, Notification value, Overwrite previous value, )
			}
		}
	}
	
	else if (GPIO_PORTC_RIS_R & (1 << Gpio_Pin7)) 
	{	
    GPIO_PORTC_ICR_R = (1 << Gpio_Pin7); // clear interrupt
		//uint8_t uValueToSend;

    if ((GPIO_PORTC_DATA_R & (1 << Gpio_Pin7)) == 0)
        M_Led(LED_RED);
    else
        M_Led(LED_STOP);

    //xQueueSendFromISR(xQueue, &uValueToSend, &xHigherPriorityTaskWoken);	
	}
	
	
	
	// Yield if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIOD_Handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_PORTD_RIS_R & (1 << Gpio_Pin6)) 
	{	
    GPIO_PORTD_ICR_R = (1 << Gpio_Pin6); // clear interrupt
		if(control == DRIVER_UP || control == PASS_UP)
		{
			xTaskNotifyFromISR(xControlTaskHandle,LIMIT_UP,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
			//(Task to notify, Notification value, Overwrite previous value, )
		}
  }
	else if (GPIO_PORTD_RIS_R & (1 << Gpio_Pin7)) 
	{	
    GPIO_PORTD_ICR_R = (1 << Gpio_Pin7); // clear interrupt
		if(control == DRIVER_DOWN || control == PASS_DOWN)
		{
			xTaskNotifyFromISR(xControlTaskHandle,LIMIT_DOWN,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
			//(Task to notify, Notification value, Overwrite previous value, )
		}
	}
	// Yield if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIOF_Handler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_PORTF_RIS_R & (1 << Gpio_Pin0)) 
	{	
    GPIO_PORTF_ICR_R = (1 << Gpio_Pin0); // clear interrupt
		xTaskNotifyFromISR(xControlTaskHandle,DRIVER_UP,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
		//(Task to notify, Notification value, Overwrite previous value, )
  }
	else if (GPIO_PORTF_RIS_R & (1 << Gpio_Pin4)) 
	{	
    GPIO_PORTF_ICR_R = (1 << Gpio_Pin4); // clear interrupt
		xTaskNotifyFromISR(xControlTaskHandle,DRIVER_DOWN,eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
		//(Task to notify, Notification value, Overwrite previous value, )
  }
	// Yield if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}