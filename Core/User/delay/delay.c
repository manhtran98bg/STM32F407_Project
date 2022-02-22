/*
 * delay.c
 *
 *  Created on: Feb 21, 2022
 *      Author: kylin
 */

#include "delay.h"
extern TIM_HandleTypeDef htim4;
void dUS_tim4(uint32_t uS)
{
	__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_TIM_Base_Start(&htim4);
	while(__HAL_TIM_GET_COUNTER(&htim4) < uS);
	HAL_TIM_Base_Stop(&htim4);
}
