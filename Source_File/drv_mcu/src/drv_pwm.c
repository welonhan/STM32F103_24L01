
#include "drv_pwm.h"

extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef  TIM_OCInitStructure;

/**
  * @brief  Configure the TIM4 Pins.
  * @param  None
  * @retval None
  */
void PWM_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/**
  * @brief  Configure the TIM4 Pins.
  * @param  None
  * @retval None
  */

void PWM_Config(void)
{
	
	 /* TIM1 Configuration ---------------------------------------------------
   Generate 7 PWM signals with 4 different duty cycles:
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   and Connectivity line devices and to 24 MHz for Low-Density Value line and
   Medium-Density Value line devices
   
   The objective is to generate 7 PWM signal at 17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   The channel 1 and channel 1N duty cycle is set to 50%
   The channel 2 and channel 2N duty cycle is set to 37.5%
   The channel 3 and channel 3N duty cycle is set to 25%
   The channel 4 duty cycle is set to 12.5%
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
  ----------------------------------------------------------------------- */
	uint16_t TimerPeriod = 0;
	uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
	
	PWM_GPIO_Config();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);		//TIM4 CLOCK ENABLE
	
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  TimerPeriod = 2000;//(72000000 / 50 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) 150* (TimerPeriod - 1)) / 2000);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
  Channel2Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 2000);
  /* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N */
  Channel3Pulse = (uint16_t) (((uint32_t) 250 * (TimerPeriod - 1)) / 2000);
  /* Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4 */
  Channel4Pulse = (uint16_t) (((uint32_t) 100* (TimerPeriod- 1)) / 2000);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 72000000/(2001*50);
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM4, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
		
}


/**
  * @brief  Configure the TIM4 Pins.
  * @param  None
  * @retval None
  */

void PWM_Set(TIM_TypeDef* TIMx, uint8_t channel ,int8_t num )
{
	uint16_t pwm_num;
	int8_t num1;
	if(num>100)
		num1=100;
	else if (num<-100)
		num1=-100;
	else
		num1=num;
	
	pwm_num=(uint16_t)(150+num1);
	
	switch(channel)
	{
		case CHANNEL1:
		{
			TIM_OCInitStructure.TIM_Pulse = pwm_num;
			TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		}
		case CHANNEL2:
		{
			TIM_OCInitStructure.TIM_Pulse = pwm_num;
			TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		}
		case CHANNEL3:
		{
			TIM_OCInitStructure.TIM_Pulse = pwm_num;
			TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		}
		case CHANNEL4:
		{
			TIM_OCInitStructure.TIM_Pulse = pwm_num;
			TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		}
	}
	 /* TIM4 counter enable */
  TIM_Cmd(TIMx, ENABLE);

  /* TIM4 Main Output Enable */
  TIM_CtrlPWMOutputs(TIMx, ENABLE);
		
}

