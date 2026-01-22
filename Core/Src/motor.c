
#include "motor.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stdlib.h"

motor init_motor(TIM_HandleTypeDef *htim_pwm,
				 uint32_t pwm_tim_channel1,
				 uint32_t pwm_tim_channel2,
				 TIM_HandleTypeDef *htim_enc,
				 uint32_t enc_tim_channel1,
				 uint32_t enc_tim_channel2,
				 double kp, double ki, double kd)
{
	motor dc_motor;

	dc_motor.pid_pos.kp = kp;
	dc_motor.pid_pos.kd = kd;
	dc_motor.pid_pos.ki = ki;

	dc_motor.htim_pwm = htim_pwm;
	dc_motor.pwm_tim_channel1 = pwm_tim_channel1;
	dc_motor.pwm_tim_channel2 = pwm_tim_channel2;

	dc_motor.htim_enc = htim_enc;
	dc_motor.enc_tim_channel1 = enc_tim_channel1;
	dc_motor.enc_tim_channel2 = enc_tim_channel2;

	dc_motor.last_pos = 0;
	dc_motor.max_speed = 32767;

	dc_motor.pid_pos.prev_time = 0;
	dc_motor.pid_pos.us_time = 0;
	dc_motor.pid_pos.d_time = 0;
	dc_motor.pid_pos.d_err = 0;
	dc_motor.pid_pos.prev_error = 0;
	dc_motor.pid_pos.current_pos = 0;
	dc_motor.pid_pos.setpoint = 0;

	HAL_TIM_PWM_Start(dc_motor.htim_pwm, dc_motor.pwm_tim_channel1);
	HAL_TIM_PWM_Start(dc_motor.htim_pwm, dc_motor.pwm_tim_channel2);

    HAL_TIM_Encoder_Start(dc_motor.htim_enc, dc_motor.enc_tim_channel1);
    HAL_TIM_Encoder_Start(dc_motor.htim_enc, dc_motor.enc_tim_channel2);

	return dc_motor;
}

int16_t encoder_delta(motor *motor)
{
    uint16_t now = __HAL_TIM_GET_COUNTER(motor->htim_enc);

    int16_t delta = (int16_t)(now - motor->last_pos);
    motor->last_pos = now;

    return delta;
}

void compute_pid(motor *motor, PID *pid)
{
	pid->us_time = HAL_GetTick();
	pid->d_time = (pid->us_time - pid->prev_time);

	if (motor->pid_pos.d_time < 1) motor->pid_pos.d_time = 1;

	pid->p_err	= pid->setpoint - pid->current_pos;
	pid->d_err	= (pid->p_err - pid->prev_error)/pid->d_time;
	pid->i_err	= pid->i_err + (pid->p_err * pid->d_time);

	pid->ctrl_signal = (pid->kp * pid->p_err) +
					   (pid->kd * pid->d_err) +
					   (pid->ki * pid->i_err);

	pid->prev_error	= pid->p_err;
	pid->prev_time	= pid->us_time;

	if (pid->ctrl_signal > motor->max_speed) pid->ctrl_signal = motor->max_speed;
	if (pid->ctrl_signal < -motor->max_speed) pid->ctrl_signal = -motor->max_speed;
}

static inline void fwd(motor *motor){
	//motor->moving_fwd = 1;
	//motor->moving_bwd = 0;
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel1, 0);
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel2, abs(motor->pwm_signal));
}

static inline void bwd(motor *motor){
	//motor->moving_fwd = 0;
	//motor->moving_bwd = 1;
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel1, abs(motor->pwm_signal));
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel2, 0);
}

static inline void stop(motor *motor){
	//motor->moving_fwd = 0;
	//motor->moving_bwd = 0;
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel1, 0);
	__HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_tim_channel2, 0);
}

void move_abs_motor(motor *motor,
				    int32_t setpoint)
{
	motor->pid_pos.current_pos = encoder_delta(motor);
	motor->pid_pos.setpoint = setpoint;

	compute_pid(motor, &motor->pid_pos);

	if(motor->pid_pos.ctrl_signal > MIN_POS_DELTA) fwd(motor);
	else if(motor->pid_pos.ctrl_signal < -MIN_POS_DELTA) bwd(motor);
	else stop(motor);
}

