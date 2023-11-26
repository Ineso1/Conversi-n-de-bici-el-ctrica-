#ifndef BLDC_CONTROLLER_H
#define BLDC_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

struct Bldc {

	/* Port and Pin Prototypes */

	// Hall sensors
	GPIO_TypeDef* HALL_A_Port;
	GPIO_TypeDef* HALL_B_Port;
	GPIO_TypeDef* HALL_C_Port;
	uint16_t HALL_A_PIN;
	uint16_t HALL_B_PIN;
	uint16_t HALL_C_PIN;

	// Throttle
	GPIO_TypeDef* THROTTLE_Port;
	uint16_t THROTTLE_PIN;
	ADC_HandleTypeDef* THROTTLE_ADC;

	// Pre-driver outputs
	GPIO_TypeDef* H_A_Port;
	GPIO_TypeDef* H_B_Port;
	GPIO_TypeDef* H_C_Port;
	GPIO_TypeDef* L_A_Port;
	GPIO_TypeDef* L_B_Port;
	GPIO_TypeDef* L_C_Port;
	uint16_t H_A_PIN;
	uint16_t H_B_PIN;
	uint16_t H_C_PIN;
	uint16_t L_A_PIN;
	uint16_t L_B_PIN;
	uint16_t L_C_PIN;

	//PWM Timers
	TIM_HandleTypeDef* PWM_A;
	TIM_HandleTypeDef* PWM_B;
	TIM_HandleTypeDef* PWM_C;

	// Current and voltage sensors
    GPIO_TypeDef* I_SENSE_Port;
    GPIO_TypeDef* V_SENSE_Port;
    uint16_t I_SENSE_PIN;
    uint16_t V_SENSE_PIN;
    ADC_HandleTypeDef* I_SENSE_ADC;
    ADC_HandleTypeDef* V_SENSE_ADC;

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
};

struct Bldc* create_Bldc();

//Pin and Ports setters
void setPins_Hall(struct Bldc* bldc, GPIO_TypeDef* hall_a_port, uint16_t hall_a_pin, GPIO_TypeDef* hall_b_port, uint16_t hall_b_pin, GPIO_TypeDef* hall_c_port, uint16_t hall_c_pin);
void setPins_Predriver(struct Bldc* bldc, GPIO_TypeDef* h_a_port, uint16_t h_a_pin, GPIO_TypeDef* h_b_port, uint16_t h_b_pin, GPIO_TypeDef* h_c_port, uint16_t h_c_pin, GPIO_TypeDef* l_a_port, uint16_t l_a_pin, GPIO_TypeDef* l_b_port, uint16_t l_b_pin, GPIO_TypeDef* l_c_port, uint16_t l_c_pin);
void setTimers(struct Bldc* bldc, TIM_HandleTypeDef* PWM_A, TIM_HandleTypeDef* PWM_B, TIM_HandleTypeDef* PWM_C);
void setThrottle(struct Bldc* bldc, GPIO_TypeDef* throttle_port, uint16_t throttle_pin, ADC_HandleTypeDef* throttle_adc);
void setISense(struct Bldc* bldc, GPIO_TypeDef* iSense_port, uint16_t iSense_pin, ADC_HandleTypeDef* iSense_adc);
void setVSense(struct Bldc* bldc, GPIO_TypeDef* vSense_port, uint16_t vSense_pin, ADC_HandleTypeDef* vSense_adc);

//Configuration setting
void init(struct Bldc* bldc);

//Motor phase algorithm
unsigned int get_halls(struct Bldc* bldc);
void process_halls(struct Bldc* bldc);
void writePhases(struct Bldc* bldc, uint16_t ah, uint16_t bh, uint16_t ch, uint16_t al, uint16_t bl, uint16_t cl);
void write_pd_table(struct Bldc* bldc, unsigned int halls, unsigned int duty);

//ADC variables
void read_throttle(struct Bldc* bldc);
void read_voltage(struct Bldc* bldc);
void read_current(struct Bldc* bldc);

//IRQ functions
void pwm_irq(struct Bldc* bldc);
void adc_irq(struct Bldc* bldc);

#ifdef __cplusplus
}
#endif

#endif
