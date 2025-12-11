#ifndef _MOTOR_H
#define _MOTOR_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct{
	GPIO_TypeDef gpio_port;
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
	int32_t prev_error;
	int32_t *current_pos;
	int32_t last_pos;
}PID;

typedef struct Motor{
	PID motor_pid;
	uint8_t pwm_signal;
	GPIO_motor pwm_pin1;
	GPIO_motor pwm_pin2;
	GPIO_motor en_pin1;
	GPIO_motor en_pin2;
	float current_draw;
}motor;

motor init_motor(GPIO_motor pwm_pin1,
				 GPIO_motor pwm_pin2,
				 GPIO_motor en_pin1,
				 GPIO_motor en_pin2,
				 int32_t *encoder,
				 double kp, double ki, double kd);

#endif
