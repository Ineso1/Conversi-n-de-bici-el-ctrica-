#include "bldc_controller.h"
#include "usart.h"
#include <cstdio>  // Include this for snprintf
#include <cstring> // Include this for strlen
#include <string.h>

void init(struct Bldc* bldc){
    // Proportions
	bldc->CURRENT_SCALING = 3.3 / 0.001 / 20 / 4096 * 1000;
	bldc->VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;

    // Parameter
	bldc->HALL_OVERSAMPLE = 18;
	bldc->HALL_IDENTIFY_DUTY_CYCLE = 25;
	bldc->F_PWM = 16000;
	bldc->DUTY_CYCLE_MAX = 255;

    // Current cutoff
	bldc->FULL_SCALE_CURRENT_MA = 30000;

    // Throttle limits
	bldc->THROTTLE_LOW = 200;
	bldc->THROTTLE_HIGH = 2540;

	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_2, 255);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_2, 255);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_2, 255);

    HAL_TIM_PWM_Start(bldc->PWM_A, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(bldc->PWM_A, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(bldc->PWM_B, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(bldc->PWM_B, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(bldc->PWM_C, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(bldc->PWM_C, TIM_CHANNEL_2);

}

// Function to set up the Hall sensor pins
void setPins_Hall(struct Bldc* bldc, GPIO_TypeDef* hall_a_port, uint16_t hall_a_pin, GPIO_TypeDef* hall_b_port, uint16_t hall_b_pin, GPIO_TypeDef* hall_c_port, uint16_t hall_c_pin) {
    bldc->HALL_A_Port = hall_a_port;
    bldc->HALL_A_PIN = hall_a_pin;
    bldc->HALL_B_Port = hall_b_port;
    bldc->HALL_B_PIN = hall_b_pin;
    bldc->HALL_C_Port = hall_c_port;
    bldc->HALL_C_PIN = hall_c_pin;
}

// Function to set up the pre-driver pins
void setPins_Predriver(struct Bldc* bldc, GPIO_TypeDef* h_a_port, uint16_t h_a_pin, GPIO_TypeDef* h_b_port, uint16_t h_b_pin, GPIO_TypeDef* h_c_port, uint16_t h_c_pin, GPIO_TypeDef* l_a_port, uint16_t l_a_pin, GPIO_TypeDef* l_b_port, uint16_t l_b_pin, GPIO_TypeDef* l_c_port, uint16_t l_c_pin) {
    bldc->H_A_Port = h_a_port;
    bldc->H_A_PIN = h_a_pin;
    bldc->H_B_Port = h_b_port;
    bldc->H_B_PIN = h_b_pin;
    bldc->H_C_Port = h_c_port;
    bldc->H_C_PIN = h_c_pin;
    bldc->L_A_Port = l_a_port;
    bldc->L_A_PIN = l_a_pin;
    bldc->L_B_Port = l_b_port;
    bldc->L_B_PIN = l_b_pin;
    bldc->L_C_Port = l_c_port;
    bldc->L_C_PIN = l_c_pin;
}

// Function to set up PWM Timers
void setTimers(struct Bldc* bldc, TIM_HandleTypeDef* PWM_A, TIM_HandleTypeDef* PWM_B, TIM_HandleTypeDef* PWM_C) {
    bldc->PWM_A = PWM_A;
    bldc->PWM_B = PWM_B;
    bldc->PWM_C = PWM_C;
}

// Function to set up the throttle pin and ADC channel
void setThrottle(struct Bldc* bldc, GPIO_TypeDef* throttle_port, uint16_t throttle_pin, ADC_HandleTypeDef* throttle_adc) {
    bldc->THROTTLE_Port = throttle_port;
    bldc->THROTTLE_PIN = throttle_pin;
    bldc->THROTTLE_ADC = throttle_adc;
}

// Function to set up the current sensor pin and ADC channel
void setISense(struct Bldc* bldc, GPIO_TypeDef* iSense_port, uint16_t iSense_pin, ADC_HandleTypeDef* iSense_adc) {
    bldc->I_SENSE_Port = iSense_port;
    bldc->I_SENSE_PIN = iSense_pin;
    bldc->I_SENSE_ADC = iSense_adc;
}

// Function to set up the voltage sensor pin and ADC channel
void setVSense(struct Bldc* bldc, GPIO_TypeDef* vSense_port, uint16_t vSense_pin, ADC_HandleTypeDef* vSense_adc) {
    bldc->V_SENSE_Port = vSense_port;
    bldc->V_SENSE_PIN = vSense_pin;
    bldc->V_SENSE_ADC = vSense_adc;
}

void print_binary(uint32_t n) {
    char binary_str[34];  // Se aumenta en 1 para incluir el salto de línea
    binary_str[33] = '\0';

    for (int i = 31; i >= 0; --i) {
        binary_str[i] = (n & 1) ? '1' : '0';
        n >>= 1;
    }

    binary_str[32] = '\n';  // Agrega un salto de línea al final

    HAL_UART_Transmit(&huart2, (uint8_t *)binary_str, strlen(binary_str), HAL_MAX_DELAY);
}


unsigned int get_halls(struct Bldc* bldc){
	unsigned int hallCounts[] = {0, 0, 0};

	// Read all the Hall pins repeatedly and tally the results
	for (unsigned int i = 0; i < bldc->HALL_OVERSAMPLE; i++) {
		hallCounts[0] += HAL_GPIO_ReadPin(bldc->HALL_A_Port, bldc->HALL_A_PIN);
		hallCounts[1] += HAL_GPIO_ReadPin(bldc->HALL_B_Port, bldc->HALL_B_PIN);
		hallCounts[2] += HAL_GPIO_ReadPin(bldc->HALL_C_Port, bldc->HALL_C_PIN);
	}

	unsigned int hall = 0;

	// If votes >= threshold, set the corresponding bit to 1
	if (hallCounts[0] > bldc->HALL_OVERSAMPLE / 2)
		hall |= (1 << 0);
	if (hallCounts[1] > bldc->HALL_OVERSAMPLE / 2)
		hall |= (1 << 1);
	if (hallCounts[2] > bldc->HALL_OVERSAMPLE / 2)
		hall |= (1 << 2);

	// Print the hall value
	//print_binary(hall);

	/*char message[50];
	snprintf(message, sizeof(message), "Hall Value: %u\r\n", hall);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);*/

	return hall;
}

void process_halls(struct Bldc* bldc){


    for(unsigned int j = 0; j < 200; j++)
    {
	    unsigned int hall = get_halls(bldc);
	    write_pd_table(bldc, hall, bldc->throttle_pwm);
    }
    bldc->throttle_pwm = 0;
    HAL_ADC_Stop(bldc->THROTTLE_ADC);
    HAL_ADC_Start_IT(bldc->THROTTLE_ADC);

}

void writePhases(struct Bldc* bldc, uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_1, ah);
	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_2, 255 - al);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_1, bh);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_2, 255 - bl);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_1, ch);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_2, 255 - cl);
}

void write_pd_table(struct Bldc* bldc, unsigned int halls, unsigned int duty){

	if(duty > 255){
		duty = 0;
	}
	if(duty < 40){
		bldc->throttle_pwm = 0;
		duty = 0;
		halls = 8;
	}

	unsigned int complement = 255 - duty - 6;


	switch(halls){
        case 1: // Case 001
           writePhases(bldc, duty, 0, 0, complement, 0, 255);  //writePhases(0, 0, duty, 255, 0, 0);
           break;
        case 2: // Case 010
           writePhases(bldc, 0, 0, duty, 0, 255, complement);  //writePhases(0, duty, 0, 0, 0, 255);
           break;
        case 3: // Case 011
           writePhases(bldc, duty, 0, 0, complement, 255, 0);  //writePhases(0, duty, 0, 255, 0, 0);
           break;
        case 4: // Case 100
           writePhases(bldc, 0, duty, 0, 255, complement, 0);  //writePhases(duty, 0, 0, 0, 255, 0);
           break;
        case 5: // Case 101
           writePhases(bldc, 0, duty, 0, 0, complement, 255);  //writePhases(0, 0, duty, 0, 255, 0);
           break;
        case 6: // Case 110
           writePhases(bldc, 0, 0, duty, 255, 0, complement);  //writePhases(duty, 0, 0, 0, 0, 255);
           break;
        default: // Case 000 or error
           writePhases(bldc, 0, 0, 0, 255, 255, 255);
	}

}

void read_throttle(struct Bldc* bldc){
	unsigned int throttle_adc = HAL_ADC_GetValue(bldc->THROTTLE_ADC);

	throttle_adc = (throttle_adc - bldc->THROTTLE_LOW) * 255;
	if(throttle_adc < 0){
		throttle_adc *= -1;
	}

	throttle_adc = throttle_adc / (bldc->THROTTLE_HIGH - bldc->THROTTLE_LOW);
	bldc->throttle_pwm = throttle_adc;

	/*
	char message[50];
	snprintf(message, sizeof(message), "Throttle Value: %u\r\n", throttle_adc);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	*/

	    if (throttle_adc >= 255) // Bound the output between 0 and 255
	    	bldc->throttle_pwm = 255;

	    if (throttle_adc <= 0)
	    	bldc->throttle_pwm = 0;

}

void read_voltage(struct Bldc* bldc){
	int voltage_adc = HAL_ADC_GetValue(bldc->V_SENSE_ADC);
	bldc->voltage_mv = voltage_adc * bldc->CURRENT_SCALING;
	/*Scale and limits*/
}

void read_current(struct Bldc* bldc){
	int current_adc = HAL_ADC_GetValue(bldc->I_SENSE_ADC);
	bldc->current_ma = current_adc * bldc->VOLTAGE_SCALING;
	/*Scale and limits*/
	/*Function to protect over current*/
}


void pwm_irq(struct Bldc* bldc) {
	HAL_ADC_Start_IT(bldc->THROTTLE_ADC);
	HAL_ADC_Start(bldc->I_SENSE_ADC);
	//__HAL_TIM_CLEAR_FLAG(bldc->PWM_A, TIM_FLAG_UPDATE);

	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void adc_irq(struct Bldc* bldc) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	read_throttle(bldc);
	read_voltage(bldc);
	read_current(bldc);

	unsigned int halls = get_halls(bldc);

	write_pd_table(bldc, halls, bldc->throttle_pwm);
	//__HAL_ADC_RESET_HANDLE_STATE(bldc->THROTTLE_ADC);
	/*char message[50];
	snprintf(message, sizeof(message), "Throttle Value: %u\r\n", bldc->throttle_pwm);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);*/

	HAL_TIM_PWM_Stop(bldc->PWM_A, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(bldc->PWM_A, TIM_CHANNEL_2);
}
