/*
 * sr04.c
 *
 *  Created on: Feb 21, 2022
 *      Author: kylin
 */

#include "sr04.h"
extern TIM_HandleTypeDef htim4;
uint8_t HCSR04_Init(HCSR04_t* HCSR04, GPIO_TypeDef* ECHO_GPIOx, uint16_t ECHO_GPIO_Pin, GPIO_TypeDef* TRIGGER_GPIOx, uint16_t TRIGGER_GPIO_Pin){
	/* Init Delay functions */

	/* Save everything */
	HCSR04->ECHO_GPIOx = ECHO_GPIOx;
	HCSR04->ECHO_GPIO_Pin = ECHO_GPIO_Pin;
	HCSR04->TRIGGER_GPIOx = TRIGGER_GPIOx;
	HCSR04->TRIGGER_GPIO_Pin = TRIGGER_GPIO_Pin;
	HAL_GPIO_WritePin(TRIGGER_GPIOx, TRIGGER_GPIO_Pin, RESET);
	/* Start measurement, check if sensor is working */
	if (HCSR04_Read(HCSR04) >= 0) {
		/* Sensor OK */
		return 1;
	}

	/* Sensor error */
	return 0;
}

float HCSR04_Read(HCSR04_t* HCSR04) {
	uint32_t time, timeout;
	/* Trigger low */
	HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, RESET);
	/* Delay 2 us */
	dUS_tim4(2);
	/* Trigger high for 10us */
	HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, SET);
	/* Delay 10 us */
	dUS_tim4(10);
	/* Trigger low */
	HAL_GPIO_WritePin(HCSR04->TRIGGER_GPIOx, HCSR04->TRIGGER_GPIO_Pin, RESET);

	/* Give some time for response */

	timeout = HCSR04_TIMEOUT;
	while (!HAL_GPIO_ReadPin(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) {
		if (timeout-- == 0x00) {
			return -1;
		}
	}
	/* Start time */
	time = 0;
	/* Wait till signal is low */
	__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_TIM_Base_Start(&htim4);
	while (HAL_GPIO_ReadPin(HCSR04->ECHO_GPIOx, HCSR04->ECHO_GPIO_Pin)) {
//		/* Increase time */
//		time++;
//		/* Delay 1us */
//		dUS_tim4(1);
	}
	time = __HAL_TIM_GET_COUNTER(&htim4);
	HAL_TIM_Base_Stop(&htim4);
	/* Convert us to cm */
	HCSR04->Distance =  (float)(time/58.0);
	HCSR04->distance_int = (int)HCSR04->Distance;
	HCSR04->distance_dec = (HCSR04->Distance - HCSR04->distance_int)*10;
	/* Return distance */
	return HCSR04->Distance;
}
