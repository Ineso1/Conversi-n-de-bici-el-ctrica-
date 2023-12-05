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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_TIME 0.01
#define NUM_TAPS 21
#define BLOCK_SIZE 32
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

const float32_t firCoeffs[NUM_TAPS] = {
    -0.0162988415008265,
    -0.0709271864972703,
    0.0225851291371347,
    0.0371278894358235,
    -0.0134217682863348,
    -0.0589846533854996,
    0.0145099553645701,
    0.103203330310147,
    -0.0148779327102912,
    -0.317339878494921,
    0.515006915991497,
    -0.317339878494921,
    -0.0148779327102912,
    0.103203330310147,
    0.0145099553645701,
    -0.0589846533854996,
    -0.0134217682863348,
    0.0371278894358235,
    0.0225851291371347,
    -0.0709271864972703,
    -0.0162988415008265
};
float32_t firState[NUM_TAPS + BLOCK_SIZE - 1];  // Filter state buffer
arm_fir_instance_f32 FIR_Filter;
float32_t inputBuffer[BLOCK_SIZE];
float32_t outputBuffer[BLOCK_SIZE];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId RecepcionCANHandle;
osThreadId IMUprocessHandle;
osThreadId UARTcomHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void IMU_acceleration(void);
void IMU_dsp(void);
void acceleration_data_conversion(float acc_x, float acc_y, float acc_z, char *message);

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
	// Initialize FIR filter
	arm_fir_init_f32(&FIR_Filter, NUM_TAPS, (float32_t*)firCoeffs, firState, BLOCK_SIZE);
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
	static size_t bufferIndex = 0; // Keeps track of the current position in the buffer
  /* Infinite loop */
  for(;;)
  {

	 IMU_acceleration();
	 inputBuffer[bufferIndex] = acc_z; // Store the current sample in the buffer
	 bufferIndex++; // Move to the next position in the buffer

	 if (bufferIndex >= BLOCK_SIZE) {
		 bufferIndex = 0; // Reset buffer index
		 IMU_dsp();
		 acceleration_data_conversion(acc_x, acc_y, outputBuffer[BLOCK_SIZE], message); //Sends the data
	 }

	 osDelay(2);
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
void IMU_acceleration() {
    uint8_t imu_readings[IMU_NUMBER_OF_BYTES];
    GetAccelData(&hi2c1, imu_readings);

    int16_t accel_data[3];
    accel_data[0] = ((int16_t)(imu_readings[1] << 8) | imu_readings[0]);
    accel_data[1] = ((int16_t)(imu_readings[3] << 8) | imu_readings[2]);
    accel_data[2] = ((int16_t)(imu_readings[5] << 8) | imu_readings[4]);

    acc_x = ((float)accel_data[0]) / 100.0f;
    acc_y = ((float)accel_data[1]) / 100.0f;
    acc_z = ((float)accel_data[2]) / 100.0f;
}

void IMU_dsp(void) {
	arm_fir_f32(&FIR_Filter, inputBuffer, outputBuffer, BLOCK_SIZE);
}

void acceleration_data_conversion(float acc_x, float acc_y, float acc_z, char *message) {
    // Calculate G-forces, roll, pitch, and velocity
    float g_force_x = acc_x / 9.81f;
    float g_force_y = acc_y / 9.81f;
    float g_force_z = acc_z / 9.81f;

    float roll = atan2(acc_y, acc_z) * 180.0f / PI;
    float pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0f / PI;

    static float velocity_x = 0.0f, velocity_y = 0.0f, velocity_z = 0.0f;
    velocity_x += acc_x * SAMPLE_TIME;
    velocity_y += acc_y * SAMPLE_TIME;
    velocity_z += acc_z * SAMPLE_TIME;

    sprintf(message, "GX: %.2f GY: %.2f GZ: %.2f Roll: %.2f Pitch: %.2f VX: %.2f VY: %.2f VZ: %.2f\r\n",
            g_force_x, g_force_y, g_force_z,
            roll, pitch,
            velocity_x, velocity_y, velocity_z);
}

/* USER CODE END Application */

