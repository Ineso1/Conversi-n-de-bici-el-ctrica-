#include "bldc_controller.h"
#include "usart.h"
#include <cstdio>  // Include this for snprintf
#include <cstring> // Include this for strlen

void init(struct Bldc* bldc){
    // Proportions
	bldc->CURRENT_SCALING = 3.3 / 0.001 / 20 / 4096 * 1000;
	bldc->VOLTAGE_SCALING = 3.3 / 4096 * (47 + 2.2) / 2.2 * 1000;

    // Parameter
	bldc->HALL_OVERSAMPLE = 8;
	bldc->HALL_IDENTIFY_DUTY_CYCLE = 25;
	bldc->F_PWM = 16000;
	bldc->DUTY_CYCLE_MAX = 4294967295;

    // Current cutoff
	bldc->FULL_SCALE_CURRENT_MA = 30000;

    // Throttle limits
	bldc->THROTTLE_LOW = 600;
	bldc->THROTTLE_HIGH = 2650;

    HAL_TIM_PWM_Start_IT(bldc->PWM_A, TIM_CHANNEL_1);

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
	char message[50];
	snprintf(message, sizeof(message), "Hall Value: %u\r\n", hall);
	HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);

	return hall;
}

void process_halls(struct Bldc* bldc){

	for (unsigned int i = 0; i < 6; i++) {
	        uint8_t nextState = (i + 1) % 6;

	        // Switch between states for a while
	        for (unsigned int j = 0; j < 200; j++) {
	            HAL_Delay(1);
	            write_pd_table(bldc, i, bldc->HALL_IDENTIFY_DUTY_CYCLE);
	            HAL_Delay(1);
	            write_pd_table(bldc, nextState, bldc->HALL_IDENTIFY_DUTY_CYCLE);
	        }

	        bldc->hallToMotor[get_halls(bldc)] = (i + 2) % 6;
	    }

		write_pd_table(bldc, 0, 0);

}

void writePhases(struct Bldc* bldc, uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl){
	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_1, ah);
	__HAL_TIM_SET_COMPARE(bldc->PWM_A, TIM_CHANNEL_2, al);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_1, bh);
	__HAL_TIM_SET_COMPARE(bldc->PWM_B, TIM_CHANNEL_2, bl);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_1, ch);
	__HAL_TIM_SET_COMPARE(bldc->PWM_C, TIM_CHANNEL_2, cl);
}

void write_pd_table(struct Bldc* bldc, unsigned int motorState, unsigned int duty){
	if (duty == 0 || duty > 255)
	        motorState = 255;

	unsigned int complement = 255 - duty;

	if (motorState == 0)
		writePhases(bldc, 0, duty, 0, 255, complement, 0);
	else if (motorState == 1)
		writePhases(bldc, 0, 0, duty, 255, 0, complement);
	else if (motorState == 2)
		writePhases(bldc, 0, 0, duty, 0, 255, complement);
	else if (motorState == 3)
		writePhases(bldc, duty, 0, 0, complement, 255, 0);
	else if (motorState == 4)
		writePhases(bldc, duty, 0, 0, complement, 0, 255);
	else if (motorState == 5)
		writePhases(bldc, 0, duty, 0, 0, complement, 255);
	else
		writePhases(bldc, 0, 0, 0, 0, 0, 0);
}

void read_throttle(struct Bldc* bldc){
	unsigned int throttle_adc = HAL_ADC_GetValue(bldc->THROTTLE_ADC);
	throttle_adc = (throttle_adc - bldc->THROTTLE_LOW) * 256;
	throttle_adc = throttle_adc / (bldc->THROTTLE_HIGH - bldc->THROTTLE_LOW);
	throttle_adc = 150;
	bldc->throttle_pwm = throttle_adc;

	    if (throttle_adc > 255) // Bound the output between 0 and 255
	    	bldc->throttle_pwm = 255;

	    if (throttle_adc < 0)
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
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_ADC_Start_IT(bldc->THROTTLE_ADC);
	HAL_ADC_Start_IT(bldc->I_SENSE_ADC);
	__HAL_TIM_CLEAR_FLAG(bldc->PWM_A, TIM_FLAG_UPDATE);
	while (HAL_ADC_GetState(bldc->THROTTLE_ADC) != HAL_ADC_STATE_REG_EOC) {}
	while (HAL_ADC_GetState(bldc->I_SENSE_ADC) != HAL_ADC_STATE_REG_EOC) {}
}

void adc_irq(struct Bldc* bldc) {
	read_throttle(bldc);
	read_voltage(bldc);
	read_current(bldc);
}







