/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define timeout_expired(start, len) ((HAL_GetTick() - (start)) >= (len))
#define TIMEOUT "TIMEOUT!"
uint32_t local_time, sensor_time;
const float speedOfSound = 0.0343/2;
uint8_t buff[10];
uint32_t distance, local_time;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char pcWriteBuffer[1024];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for myLedTask */
osThreadId_t myLedTaskHandle;
const osThreadAttr_t myLedTask_attributes = {
  .name = "myLedTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};
/* Definitions for myBtn */
osThreadId_t myBtnHandle;
const osThreadAttr_t myBtn_attributes = {
  .name = "myBtn",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 712
};
/* Definitions for rtosTimer1 */
osTimerId_t rtosTimer1Handle;
const osTimerAttr_t rtosTimer1_attributes = {
  .name = "rtosTimer1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint32_t hcsr04_read (void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedTask(void *argument);
void StartBtnTask(void *argument);
void callbackTimer1(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of rtosTimer1 */
  rtosTimer1Handle = osTimerNew(callbackTimer1, osTimerPeriodic, NULL, &rtosTimer1_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myLedTask */
  myLedTaskHandle = osThreadNew(StartLedTask, NULL, &myLedTask_attributes);

  /* creation of myBtn */
  myBtnHandle = osThreadNew(StartBtnTask, NULL, &myBtn_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the myLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
	   osDelay(1000);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  uint8_t buff[] = "Hello world\r\n";
//	 	  HAL_UART_Transmit_IT(&huart1, buff, strlen(buff));

  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartBtnTask */
/**
* @brief Function implementing the myBtn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtnTask */
void StartBtnTask(void *argument)
{
  /* USER CODE BEGIN StartBtnTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
	  uint32_t distance  = hcsr04_read();
	  memset(buff, 0, sizeof buff);
	  int_to_char_arr(distance, buff);

	  HAL_UART_Transmit_IT(&huart1, buff, strlen(buff));
	  CDC_Transmit_FS(buff, strlen(buff));
	  osDelay(1000);
  }
  /* USER CODE END StartBtnTask */
}

/* callbackTimer1 function */
void callbackTimer1(void *argument)
{
  /* USER CODE BEGIN callbackTimer1 */

	float val = 123.234;
	char buff1[] = "123.123\r\n";
	//HAL_UART_Transmit_IT(&huart1, buff1, strlen(buff1));
  /* USER CODE END callbackTimer1 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint32_t hcsr04_read (void)
{
	local_time=0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin HIGH
	delay(2);  // wait for 2 us

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin low

	// read the time for which the pin is high
	int32_t start = HAL_GetTick();
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)// wait for the ECHO pin to go high
	{
		if (timeout_expired(start, 2000)) {
			CDC_Transmit_FS(TIMEOUT, strlen(TIMEOUT));
			break;
		}
	}
	start = HAL_GetTick();
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)    // while the pin is high
	 {
		if (timeout_expired(start, 2000)) {
					CDC_Transmit_FS(TIMEOUT, strlen(TIMEOUT));
					break;
				}
		local_time++;   // measure time for which the pin is high
		delay(1);
	 }
	return 28*local_time* speedOfSound;
}


void delay (uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<us);
}

void int_to_char_arr(uint32_t source, uint8_t *buf)
{
    int i = 0;
    while(source > 0) {
        buf[i] = source % 10 + 48;
        source = source/10;
        i++;
    }
    if(source != 0)
    	reverse(buf, i);
    buff[i++] = 13;
}

void reverse(uint8_t* arr, uint16_t n)
{
	uint16_t temp = 0;
	for (uint16_t low = 0, high = n - 1; low < high; low++, high--) {
		temp = arr[high];
		arr[high] = arr[low];
		arr[low] = temp;
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
