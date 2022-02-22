/*
 * engine.h
 *
 *  Created on: Feb 21, 2022
 *      Author: kylin
 */

#ifndef USER_ENGINE_ENGINE_H_
#define USER_ENGINE_ENGINE_H_
#include "main.h"

typedef enum{
	OFF, FORWARD, BACKWARD
}Mode;
typedef struct {
	Mode mode;
	uint8_t speed;
	GPIO_TypeDef* EN_PORT;
	uint16_t EN_PIN;
	GPIO_TypeDef* IN1_PORT;
	uint16_t IN1_PIN;
	GPIO_TypeDef* IN2_PORT;
	uint16_t IN2_PIN;
}Engine_t;

void engine_init(Engine_t *Engine,GPIO_TypeDef* EN_PORT, uint16_t EN_PIN,
									GPIO_TypeDef* IN1_PORT, uint16_t IN1_PIN,
									GPIO_TypeDef* IN2_PORT, uint16_t IN2_PIN, uint8_t speed);
void engine_start(Engine_t *Engine, Mode mode);
void engine_stop(Engine_t *Engine);
void engine_set_duty(Engine_t *Engine, uint8_t duty);
#endif /* USER_ENGINE_ENGINE_H_ */
