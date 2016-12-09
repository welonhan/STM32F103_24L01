
#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__


#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

//??????
/** ?????? */
enum
{
	CHANNEL1 = 1,		//?????
	CHANNEL2,
	CHANNEL3,
	CHANNEL4
};


void PWM_Config(void);
void PWM_Set(TIM_TypeDef* TIMx, uint8_t channel ,int8_t num );

#endif



