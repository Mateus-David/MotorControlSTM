/*
 * motor_encoder.h
 *
 *  Created on: 13 de ago. de 2026
 *      Author: padil
 */

#ifndef INC_MOTOR_ENCODER_H_
#define INC_MOTOR_ENCODER_H_
#include "stdint.h"
#include "main.h"

typedef struct{
	volatile int32_t velocity;
	volatile int64_t position;
	volatile uint32_t last_count_value;
	volatile uint8_t  initialized;
}encoder;

void update_encoder(volatile encoder  *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(volatile encoder *encoder_value);

#endif /* INC_MOTOR_ENCODER_H_ */
