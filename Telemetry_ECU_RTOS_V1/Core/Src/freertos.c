/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;

char message[100];
uint8_t	imu_readings[IMU_NUMBER_OF_BYTES];
int16_t 	accel_data[3];
float		acc_x, acc_y, acc_z;
int		acc_x_int, acc_y_int, acc_z_int;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RecepcionCANHandle;
osThreadId IMUprocessHandle;
osThreadId UARTcomHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartRecepcionCAN(void const * argument);
void StartIMUprocess(void const * argument);
void StartUARTcom(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RecepcionCAN */
  osThreadDef(RecepcionCAN, StartRecepcionCAN, osPriorityAboveNormal, 0, 128);
  RecepcionCANHandle = osThreadCreate(osThread(RecepcionCAN), NULL);

  /* definition and creation of IMUprocess */
  osThreadDef(IMUprocess, StartIMUprocess, osPriorityNormal, 0, 128);
  IMUprocessHandle = osThreadCreate(osThread(IMUprocess), NULL);

  /* definition and creation of UARTcom */
  osThreadDef(UARTcom, StartUARTcom, osPriorityBelowNormal, 0, 128);
  UARTcomHandle = osThreadCreate(osThread(UARTcom), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	/*int v = 0;
	char message[50];
	snprintf(message, sizeof(message), "Default: %u\r\n", v);
	//HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);*/
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRecepcionCAN */
/**
* @brief Function implementing the RecepcionCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecepcionCAN */
void StartRecepcionCAN(void const * argument)
{
  /* USER CODE BEGIN StartRecepcionCAN */
  /* Infinite loop */
  for(;;)
  {
	//int v = 1;
	char message[50];
	//snprintf(message, sizeof(message), "CAN: %u\r\n", v);
	//HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	strcpy(message, "Enviado\n");

	if(CANSPI_Receive(&rxMessage))
	 {
		txMessage.frame.idType = rxMessage.frame.idType;
		txMessage.frame.id = rxMessage.frame.id;
		txMessage.frame.dlc = rxMessage.frame.dlc;
		txMessage.frame.data0++;
		txMessage.frame.data1 = rxMessage.frame.data1;
		txMessage.frame.data2 = rxMessage.frame.data2;
		txMessage.frame.data3 = rxMessage.frame.data3;
		txMessage.frame.data4 = rxMessage.frame.data4;
		txMessage.frame.data5 = rxMessage.frame.data5;
		txMessage.frame.data6 = rxMessage.frame.data6;
		txMessage.frame.data7 = rxMessage.frame.data7;
		CANSPI_Transmit(&txMessage);

		snprintf(message, sizeof(message),
			 "CAN ID: %lu, DLC: %u, Data: %u %u %u %u %u %u %u %u\r\n",
			 rxMessage.frame.id, rxMessage.frame.dlc,
			 rxMessage.frame.data0, rxMessage.frame.data1, rxMessage.frame.data2,
			 rxMessage.frame.data3, rxMessage.frame.data4, rxMessage.frame.data5,
			 rxMessage.frame.data6, rxMessage.frame.data7);
		HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

	 }
	osDelay(100);
  }
  /* USER CODE END StartRecepcionCAN */
}

/* USER CODE BEGIN Header_StartIMUprocess */
/**
* @brief Function implementing the IMUprocess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUprocess */
void StartIMUprocess(void const * argument)
{
  /* USER CODE BEGIN StartIMUprocess */
  /* Infinite loop */
  for(;;)
  {

	 GetAccelData(&hi2c1, (uint8_t*)imu_readings);

	 accel_data[0] = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
	 accel_data[1] = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
	 accel_data[2] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);


	 acc_x = ((float)(accel_data[0]))/100.0f; //m/s2
	 acc_y = ((float)(accel_data[1]))/100.0f;
	 acc_z = ((float)(accel_data[2]))/100.0f;

	 acc_x_int = (int)acc_x;
	 acc_y_int = (int)acc_y;
	 acc_z_int = (int)acc_z;


	 sprintf(message, "X: %d Y: %d Z: %d\r\n", acc_x_int, acc_y_int, acc_z_int);
	 HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);


	 osDelay(100);
  }
  /* USER CODE END StartIMUprocess */
}

/* USER CODE BEGIN Header_StartUARTcom */
/**
* @brief Function implementing the UARTcom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTcom */
void StartUARTcom(void const * argument)
{
  /* USER CODE BEGIN StartUARTcom */
  /* Infinite loop */
  for(;;)
  {
	  char message[50];
	  int val = 50;
	  //snprintf(message, sizeof(message), "%u", rxMessage.frame.data0);
	  snprintf(message, sizeof(message), "%u", val);
	  HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	  osDelay(200);
  }
  /* USER CODE END StartUARTcom */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

