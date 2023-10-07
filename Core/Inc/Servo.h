/*
 * Servo.h
 *
 *  Created on: Aug 22, 2023
 *      Author: Mateusz Gburek
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	uint32_t min_pulse_width_microseconds;
	uint32_t max_pulse_width_microseconds;

	float min_angle_possible;
	float max_angle_possible;

	float min_angle_allowed_by_user;
	float max_angle_allowed_by_user;

	TIM_HandleTypeDef *timer;
	uint32_t timer_channel;

	float position;
	float previous_position;

} Servo;

void init(Servo *servo, TIM_HandleTypeDef *timer, uint32_t timer_channel);

void servoMoveTo(Servo *servo, float angle);
void servoMoveBy(Servo *servo, float angle);

#endif /* INC_SERVO_H_ */
