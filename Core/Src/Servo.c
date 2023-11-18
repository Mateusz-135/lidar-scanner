/*
 * Servo.c
 *
 *  Created on: Aug 22, 2023
 *      Author: Mateusz Gburek
 */

#include "Servo.h"

void init(Servo *servo, TIM_HandleTypeDef *timer, uint32_t timer_channel)
{
	servo->min_angle_possible = 0.0f;
	servo->max_angle_possible = 180.0f;

	servo->min_angle_allowed_by_user = 0.0f;
	servo->max_angle_allowed_by_user = 180.0f;

	servo->min_pulse_width_microseconds = 1000;
	servo->max_pulse_width_microseconds = 2000;

	servo->timer = timer;
	servo->timer_channel = timer_channel;

	servo->position = 0.0f;
}

void servoMoveTo(Servo *servo, float angle)
{
	if(angle < servo->min_angle_allowed_by_user)
		angle = servo->min_angle_allowed_by_user;
	else if(angle > servo->max_angle_allowed_by_user)
		angle = servo->max_angle_allowed_by_user;

	servo->position = angle;

	float range = (float)(servo->max_pulse_width_microseconds - servo->min_pulse_width_microseconds);

	float percent = (servo->position - servo->min_angle_possible) / (servo->max_angle_possible - servo->min_angle_possible);

	float pulse_microseconds = (percent * range) + (float)servo->min_pulse_width_microseconds;

	// 20000 microseconds as for period of 50Hz PWM signal
	uint32_t duty = (uint32_t)( (pulse_microseconds * (float)(servo->timer->Init.Period + 1)) / 20000.0);

	__HAL_TIM_SET_COMPARE(servo->timer, servo->timer_channel, duty);
}

void servoMoveBy(Servo *servo, float angle)
{
	servoMoveTo(servo, servo->position + angle);
}
