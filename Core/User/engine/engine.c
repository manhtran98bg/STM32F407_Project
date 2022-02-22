/*
 * engine.c
 *
 *  Created on: Feb 21, 2022
 *      Author: kylin
 */

#include "engine.h"
TIM_HandleTypeDef htim1;
void engine_init(Engine_t *Engine,GPIO_TypeDef* EN_PORT, uint16_t EN_PIN,
									GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN,
									GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, uint8_t speed){
	Engine->mode = OFF;
	Engine->speed = 0;
	Engine->EN_PORT = EN_PORT;
	Engine->EN_PIN = EN_PIN;
	Engine->IN1_PORT = IN1_PORT;
	Engine->IN1_PIN = IN1_PIN;
	Engine->IN2_PORT = IN2_PORT;
	Engine->IN2_PIN = IN2_PIN;
	HAL_GPIO_WritePin(Engine->IN1_PORT, Engine->IN1_PIN, RESET);
	HAL_GPIO_WritePin(Engine->IN2_PORT, Engine->IN2_PIN, RESET);
	engine_set_duty(Engine, 0);
}
void engine_start(Engine_t *Engine, Mode mode){
	Engine->mode = mode;
	HAL_GPIO_WritePin(Engine->IN1_PORT, Engine->IN1_PIN, RESET);
	HAL_GPIO_WritePin(Engine->IN2_PORT, Engine->IN2_PIN, SET);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void engine_stop(Engine_t *Engine){
	Engine->mode = OFF;
	HAL_GPIO_WritePin(Engine->IN1_PORT, Engine->IN1_PIN, RESET);
	HAL_GPIO_WritePin(Engine->IN2_PORT, Engine->IN2_PIN, RESET);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void engine_set_duty(Engine_t *Engine, uint8_t duty){
	uint16_t pulse = 0;
	if (duty == 0) {
		pulse = 0;
		Engine->mode = OFF;
	}
	else pulse = (uint16_t)((float)(duty*0.9) + 10);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, pulse);
	Engine->speed = duty;
	Engine->mode = FORWARD;
}
