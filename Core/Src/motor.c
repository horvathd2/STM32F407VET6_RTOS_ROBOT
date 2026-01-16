
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

motor init_motor(TIM_HandleTypeDef *htim_pwm,
				 uint32_t pwm_tim_channel1,
				 uint32_t pwm_tim_channel2,
				 GPIO_motor pwm_pin1,
				 GPIO_motor pwm_pin2,
				 TIM_HandleTypeDef *htim_enc,
				 uint32_t enc_tim_channel1,
				 uint32_t enc_tim_channel2,
				 GPIO_motor en_pin1,
				 GPIO_motor en_pin2,
				 double kp, double ki, double kd)
{
	motor dc_motor;

	dc_motor.motor_pid.kp = kp;
	dc_motor.motor_pid.kd = kd;
	dc_motor.motor_pid.ki = ki;

	dc_motor.htim_pwm = htim_pwm;
	dc_motor.pwm_tim_channel1 = pwm_tim_channel1;
	dc_motor.pwm_tim_channel2 = pwm_tim_channel2;
	dc_motor.pwm_pin1 = pwm_pin1;
	dc_motor.pwm_pin2 = pwm_pin2;

	dc_motor.htim_enc = htim_enc;
	dc_motor.enc_tim_channel1 = enc_tim_channel1;
	dc_motor.enc_tim_channel2 = enc_tim_channel2;
	dc_motor.en_pin1 = en_pin1;
	dc_motor.en_pin2 = en_pin2;

	dc_motor.motor_pid.prev_time = 0;
	dc_motor.motor_pid.us_time = 0;
	dc_motor.motor_pid.d_time = 0;
	dc_motor.motor_pid.d_err = 0;
	dc_motor.motor_pid.prev_error = 0;
	dc_motor.motor_pid.current_pos = 0;
	dc_motor.motor_pid.setpoint = 0;
	dc_motor.motor_pid.last_pos = 0;

	HAL_TIM_PWM_Start(dc_motor.htim_pwm, dc_motor.pwm_tim_channel1);
	HAL_TIM_PWM_Start(dc_motor.htim_pwm, dc_motor.pwm_tim_channel2);

    HAL_TIM_Encoder_Start(dc_motor.htim_enc, dc_motor.enc_tim_channel1);
    HAL_TIM_Encoder_Start(dc_motor.htim_enc, dc_motor.enc_tim_channel2);

	return dc_motor;
}

float compute_pid(motor *motor)
{
	motor->motor_pid.us_time = HAL_GetTick();
	motor->motor_pid.d_time = (motor->motor_pid.us_time - motor->motor_pid.prev_time);

	if (motor->motor_pid.d_time < 1) motor->motor_pid.d_time = 1;

	motor->motor_pid.p_err	= motor->motor_pid.setpoint - motor->motor_pid.current_pos;
	motor->motor_pid.d_err	= (motor->motor_pid.p_err - motor->motor_pid.prev_error)/motor->motor_pid.d_time;
	motor->motor_pid.i_err	= motor->motor_pid.i_err + (motor->motor_pid.p_err * motor->motor_pid.d_time);

	motor->motor_pid.ctrl_signal = (motor->motor_pid.kp * motor->motor_pid.p_err) +
							 (motor->motor_pid.kd * motor->motor_pid.d_err) +
							 (motor->motor_pid.ki * motor->motor_pid.i_err);

	motor->motor_pid.prev_error	= motor->motor_pid.p_err;
	motor->motor_pid.prev_time	= motor->motor_pid.us_time;

	//if (motor->motor_pid.ctrl_signal > motor->max_speed) motor->motor_pid.ctrl_signal = motor->max_speed;
	//if (motor->motor_pid.ctrl_signal < -motor->max_speed) motor->motor_pid.ctrl_signal = -motor->max_speed;
}

void move_abs_motor(motor *motor,
				    int32_t setpoint)
{
	motor->motor_pid.current_pos = __HAL_TIM_GET_COUNTER(motor->htim_enc);
	motor->motor_pid.setpoint = setpoint;

	compute_pid(motor);

	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel1, motor->pwm_signal);
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel2, motor->pwm_signal);
}

