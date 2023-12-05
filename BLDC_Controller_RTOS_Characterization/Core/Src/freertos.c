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
#include <stdio.h>
#include <string.h>
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

// Proportions
int CURRENT_SCALING;
int VOLTAGE_SCALING;

// Parameter
unsigned int HALL_OVERSAMPLE;
int HALL_IDENTIFY_DUTY_CYCLE;
unsigned int F_PWM;
unsigned int DUTY_CYCLE_MAX;

// Current cutoff
int FULL_SCALE_CURRENT_MA;

// Throttle limits
unsigned int THROTTLE_LOW;
unsigned int THROTTLE_HIGH;

/*  Variables  */
int pwm_A;
int pwm_B;
int pwm_C;
int adc_bias;
int duty_cycle;
int voltage_mv;
int current_ma;
int throttle_pwm;

unsigned int hallToMotor[8];

/*Angular velocity*/

unsigned int previous_hall_state = 0;
uint16_t last_pwm_interrupt_time = 0;
uint16_t angularVelocity = 0;

//DMA
volatile uint16_t adc_dma_result[1];
int adc_channel_count;
uint8_t adc_conv_complete_flag;
char adc_dma_result_buffer[100];

uCAN_MSG txMessage;
uCAN_MSG rxMessage;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId pwmHandleHandle;
osThreadId adcHandleHandle;
osThreadId canHandlerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//Configuration setting
void init_bldc(void);

//Motor phase algorithm
unsigned int get_halls(void);
void process_halls(void);
void writePhases(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl);
void write_pd_table(unsigned int halls, unsigned int duty);

//ADC variables
void read_throttle(void);
void read_voltage(void);
void read_current(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartPwmHandle(void const * argument);
void StartAdcHandle(void const * argument);
void StartCanHandler(void const * argument);

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
     init_bldc();
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of pwmHandle */
  osThreadDef(pwmHandle, StartPwmHandle, osPriorityIdle, 0, 128);
  pwmHandleHandle = osThreadCreate(osThread(pwmHandle), NULL);

  /* definition and creation of adcHandle */
  osThreadDef(adcHandle, StartAdcHandle, osPriorityAboveNormal, 0, 128);
  adcHandleHandle = osThreadCreate(osThread(adcHandle), NULL);

  /* definition and creation of canHandler */
  osThreadDef(canHandler, StartCanHandler, osPriorityHigh, 0, 128);
  canHandlerHandle = osThreadCreate(osThread(canHandler), NULL);

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
	uint8_t nextState[] = {5, 3, 1, 6, 4, 2};
  /* Infinite loop */
  for(;;)
  {
	for(unsigned int j = 0; j<2; j++){
    unsigned int hall = get_halls();
    write_pd_table(hall, HALL_IDENTIFY_DUTY_CYCLE);
    osDelay(1);
    write_pd_table(nextState[hall - 1], HALL_IDENTIFY_DUTY_CYCLE);
	}
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartPwmHandle */
/**
* @brief Function implementing the pwmHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPwmHandle */
void StartPwmHandle(void const * argument)
{
  /* USER CODE BEGIN StartPwmHandle */
	uint8_t nextState[] = {5, 3, 1, 6, 4, 2};
  /* Infinite loop */
  for(;;)
  {
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	for(unsigned int j = 0; j < 160; j++)
	{
	 unsigned int hall = get_halls();
     write_pd_table(hall, throttle_pwm);
     osDelay(1);
     write_pd_table(nextState[hall - 1], HALL_IDENTIFY_DUTY_CYCLE);
     uint32_t current_time = HAL_GetTick();
     uint32_t time_elapsed = current_time - bldc->last_pwm_interrupt_time;

     // Calculate angular velocity
     angular_velocity = (60000 / (time_elapsed)) * (hall - previous_hall_state);
	}
  }
  /* USER CODE END StartPwmHandle */
}

/* USER CODE BEGIN Header_StartAdcHandle */
/**
* @brief Function implementing the adcHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcHandle */
void StartAdcHandle(void const * argument)
{
  /* USER CODE BEGIN StartAdcHandle */
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    read_throttle();
    read_current();
    unsigned int halls = get_halls();
	write_pd_table(halls, throttle_pwm);
    osDelay(7);
  }
  /* USER CODE END StartAdcHandle */
}

/* USER CODE BEGIN Header_StartCanHandler */
/**
* @brief Function implementing the canHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanHandler */
void StartCanHandler(void const * argument)
{
  /* USER CODE BEGIN StartCanHandler */
  /* Infinite loop */
  for(;;)
  {
	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		  txMessage.frame.id = 0x0A;
		  txMessage.frame.dlc = 8;
		  txMessage.frame.data0 = throttle_pwm;
		  txMessage.frame.data1 = current_ma;
		  txMessage.frame.data2 = 0;
		  txMessage.frame.data3 = 0;
		  txMessage.frame.data4 = 0;
		  txMessage.frame.data5 = 0;
		  txMessage.frame.data6 = 0;
		  txMessage.frame.data7 = 0;
    CANSPI_Transmit(&txMessage);
    osDelay(50);
  }
  /* USER CODE END StartCanHandler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void init_bldc(void){
    // Proportions
	CURRENT_SCALING = 3.3 / 0.001 / 20 / 4096 * 1000;
	VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;

    // Parameter
	HALL_OVERSAMPLE = 8;
	HALL_IDENTIFY_DUTY_CYCLE = 40;
	F_PWM = 16000;
	DUTY_CYCLE_MAX = 255;

    // Current cutoff
	FULL_SCALE_CURRENT_MA = 30000;

    // Throttle limits
	THROTTLE_LOW = 210;
	THROTTLE_HIGH = 1740;


	adc_channel_count = sizeof(adc_dma_result)/sizeof(adc_dma_result[0]);
	adc_conv_complete_flag = 0;


	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 255);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 255);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 255);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_ADC_Start(&hadc2);
}

unsigned int get_halls(void){
	unsigned int hallCounts[] = {0, 0, 0};

	// Read all the Hall pins repeatedly and tally the results
	for (unsigned int i = 0; i < HALL_OVERSAMPLE; i++) {
		hallCounts[0] += HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
		hallCounts[1] += HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
		hallCounts[2] += HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
	}

	unsigned int hall = 0;

	// If votes >= threshold, set the corresponding bit to 1
	if (hallCounts[0] > HALL_OVERSAMPLE / 2)
		hall |= (1 << 0);
	if (hallCounts[1] > HALL_OVERSAMPLE / 2)
		hall |= (1 << 1);
	if (hallCounts[2] > HALL_OVERSAMPLE / 2)
		hall |= (1 << 2);

	// Print the hall value
	//print_binary(hall);

	/*
	char message[50];
	snprintf(message, sizeof(message), "Hall Value: %u\r\n", hall);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	*/

	return hall;
}

void process_halls(void){


    for(unsigned int j = 0; j < 200; j++)
    {
	    unsigned int hall = get_halls();
	    write_pd_table(hall, throttle_pwm);
    }
    throttle_pwm = 0;

}

void writePhases(uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ah);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 255 - al);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, bh);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 255 - bl);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ch);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 255 - cl);
}

void write_pd_table(unsigned int halls, unsigned int duty){

	if(duty >= 257){
		duty = 0;
	}
	if(duty < 27){
		throttle_pwm = 0;
		duty = 0;
		halls = 8;
	}

	unsigned int complement = 255 - duty - 6;


	switch(halls){
        case 1: // Case 001
           writePhases(duty, 0, 0, complement, 0, 255);  //writePhases(0, 0, duty, 255, 0, 0);
           break;
        case 2: // Case 010
           writePhases(0, 0, duty, 0, 255, complement);  //writePhases(0, duty, 0, 0, 0, 255);
           break;
        case 3: // Case 011
           writePhases(duty, 0, 0, complement, 255, 0);  //writePhases(0, duty, 0, 255, 0, 0);
           break;
        case 4: // Case 100
           writePhases(0, duty, 0, 255, complement, 0);  //writePhases(duty, 0, 0, 0, 255, 0);
           break;
        case 5: // Case 101
           writePhases(0, duty, 0, 0, complement, 255);  //writePhases(0, 0, duty, 0, 255, 0);
           break;
        case 6: // Case 110
           writePhases(0, 0, duty, 255, 0, complement);  //writePhases(duty, 0, 0, 0, 0, 255);
           break;
        default: // Case 000 or error
           writePhases(0, 0, 0, 255, 255, 255);
	}

}

void read_throttle(void){
	unsigned int throttle_adc = HAL_ADC_GetValue(&hadc2);

	throttle_adc = (throttle_adc - THROTTLE_LOW) * 255;
	if(throttle_adc < 0){
		throttle_adc *= -1;
	}

	throttle_adc = throttle_adc / (THROTTLE_HIGH - THROTTLE_LOW);
	throttle_pwm = throttle_adc;

	/*
	char message[50];
	snprintf(message, sizeof(message), "Throttle Value: %u\r\n", throttle_adc);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	*/


	    if (throttle_adc >= 255) // Bound the output between 0 and 255
	    	throttle_pwm = 255;

	    if (throttle_adc <= 0)
	    	throttle_pwm = 0;

}

void read_voltage(void){
	int voltage_adc = HAL_ADC_GetValue(&hadc4);
	voltage_mv = voltage_adc * CURRENT_SCALING;
	/*Scale and limits*/
}

void read_current(void){
	int current_adc = HAL_ADC_GetValue(&hadc4);
	current_ma = current_adc * VOLTAGE_SCALING;
	/*Scale and limits*/
	/*Function to protect over current*/
}

/* USER CODE END Application */

