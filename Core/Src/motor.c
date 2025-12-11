
#include "motor.h"

motor init_motor(GPIO_motor pwm_pin1,
				 GPIO_motor pwm_pin2,
				 GPIO_motor en_pin1,
				 GPIO_motor en_pin2,
				 int32_t *encoder,
				 double kp, double ki, double kd)
{
	motor dc_motor;

	dc_motor.motor_pid.kp = kp;
	dc_motor.motor_pid.kd = kd;
	dc_motor.motor_pid.ki = ki;
	dc_motor.motor_pid.current_pos = encoder;

	dc_motor.pwm_pin1.gpio_port = pwm_pin1;
	dc_motor.pwm_pin2.gpio_port = pwm_pin2;
	dc_motor.en_pin1.gpio_port = en_pin1;
	dc_motor.en_pin2.gpio_port = en_pin2;

	return dc_motor;
}

float compute_pid(motor *motor)
{

}

void move_abs_motor(motor *motor,
					int32_t setpoint)
{

}

