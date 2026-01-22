#ifndef _MOTOR_H
#define _MOTOR_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define MIN_POS_DELTA 10

typedef struct{
	GPIO_TypeDef *gpio_port;
	uint8_t gpio_pin;
}GPIO_motor;

typedef struct{
	double ctrl_signal;
	double kp, ki, kd;
	double p_err, i_err, d_err;
	uint32_t us_time;
	uint32_t d_time;
	uint32_t prev_time;
	int32_t setpoint;
	int32_t current_pos;
	int32_t prev_error;
}PID;

typedef struct{
	TIM_HandleTypeDef *htim_pwm;
	TIM_HandleTypeDef *htim_enc;
	PID pid_pos;
	uint32_t pwm_tim_channel1;
	uint32_t pwm_tim_channel2;
	uint32_t enc_tim_channel1;
	uint32_t enc_tim_channel2;
	int16_t last_pos;
	uint16_t max_speed;
	float current_draw;
	uint8_t pwm_signal;
}motor;

motor init_motor(TIM_HandleTypeDef *htim_pwm,
				 uint32_t pwm_tim_channel1,
				 uint32_t pwm_tim_channel2,
				 TIM_HandleTypeDef *htim_enc,
				 uint32_t enc_tim_channel1,
				 uint32_t enc_tim_channel2,
				 double kp, double ki, double kd);

int16_t encoder_delta(motor *motor);

void compute_pid(motor *motor, PID *pid);

void move_abs_motor(motor *motor,
				    int32_t setpoint);
#endif
