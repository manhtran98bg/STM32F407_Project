/*
 * sr04.h
 *
 *  Created on: Feb 21, 2022
 *      Author: kylin
 */

#ifndef USER_SR04_SR04_H_
#define USER_SR04_SR04_H_

#include "main.h"
#include "delay/delay.h"
#define HCSR04_TIMEOUT			1000000
#define HCSR04_NUMBER			((float)0.0171821)

typedef struct {
	float Distance;              /*!< Distance measured from sensor in centimeters */
	uint16_t distance_int;
	uint16_t distance_dec;
	GPIO_TypeDef* ECHO_GPIOx;    /*!< Pointer to GPIOx PORT for ECHO pin. Meant for private use only */
	uint16_t ECHO_GPIO_Pin;      /*!< GPIO Pin for ECHO pin. Meant for private use only */
	GPIO_TypeDef* TRIGGER_GPIOx; /*!< Pointer to GPIOx PORT for TRIGGER pin. Meant for private use only */
	uint16_t TRIGGER_GPIO_Pin;   /*!< GPIO Pin for ECHO pin. Meant for private use only */
}HCSR04_t;

uint8_t HCSR04_Init(HCSR04_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIGGER_GPIOx, uint16_t TRIGGER_GPIO_Pin);
float HCSR04_Read(HCSR04_t* HCSR04);
#endif /* USER_SR04_SR04_H_ */
