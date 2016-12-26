
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
void PWM_SetDutycycle(TIM_TypeDef* TIMx, uint8_t channel ,uint8_t num );

#endif



