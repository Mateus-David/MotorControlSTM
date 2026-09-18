/*
 * motor_encoder.c
 *
 *  Created on: 13 de ago. de 2026
 *      Author: padil
 */
#include "motor_encoder.h"



void update_encoder(volatile encoder  *encoder_value, TIM_HandleTypeDef *htim){ //Minha versão
    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
    static uint8_t first_time = 0;
    if(!first_time){
        encoder_value->velocity = 0;
        first_time = 1;
    }else{
        if(temp_counter == encoder_value->last_count_value){
            encoder_value->velocity = 0; // Motor Parado
        }else if(temp_counter > encoder_value->last_count_value){ //
            if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){ //Ocorreu Overflow
                encoder_value->velocity = -encoder_value->last_count_value - (__HAL_TIM_GET_AUTORELOAD(htim)- temp_counter);
            }else{
                encoder_value->velocity = temp_counter - encoder_value->last_count_value;
            }
        }else{
            if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
                encoder_value->velocity = temp_counter - encoder_value->last_count_value;
            }else{
                encoder_value->velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value->last_count_value);
            }
        }
    }
    encoder_value->position += encoder_value->velocity;
    encoder_value->last_count_value = temp_counter;
}


//void update_encoder(volatile encoder *encoder_value, TIM_HandleTypeDef *htim) //Versão gpt
//{
//    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
//
//    if (!encoder_value->initialized)
//    {
//        encoder_value->velocity = 0;
//        encoder_value->initialized = 1;
//    }
//    else
//    {
//        encoder_value->velocity = (int32_t)(temp_counter - encoder_value->last_count_value);
//    }
//
//    encoder_value->position += encoder_value->velocity;
//    encoder_value->last_count_value = temp_counter;
//}


void reset_encoder(volatile encoder *encoder_value){
	encoder_value->velocity = 0;
	encoder_value->position = 0;
	encoder_value->last_count_value =0;
}


