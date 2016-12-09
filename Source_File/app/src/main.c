/**
  ******************************************************************************
  * @author  泽耀科技 ASHINING
  * @version V3.0
  * @date    2016-10-08
  * @brief   主函数C文件
  ******************************************************************************
  * @attention
  *
  * 官网	:	http://www.ashining.com
  * 淘宝	:	https://shop105912646.taobao.com
  * 阿里巴巴:	https://cdzeyao.1688.com
  ******************************************************************************
  */



#include "main.h"				//main.h 中含有TX/RX、软件SPI/硬件SPI选择配置选项

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_TypeDef* TIMx;

/* Private function prototypes -----------------------------------------------*/
void EXTI0_Config(void);
void EXTI10_Config(void);

const char *g_Ashining = "ashining";
uint8_t g_TxMode = 0, g_UartRxFlag = 0;
uint8_t g_UartRxBuffer[ 100 ] = { 0 };
uint8_t g_RF24L01RxBuffer[ 32 ] = { 0 }; 

uint8_t pData[8]={88,00,99,99,00,00,66,55};

/**
  * @brief :主函数 
  * @param :无
  * @note  :无
  * @retval:无
  */ 
int main( void )
{	
	uint8_t i = 0;

	//串口初始化
	drv_uart_init( 9600 );
	
	//延时初始化
	drv_delay_init( );
	
	//LED初始化
	drv_led_init( );
	
	//SPI初始化
	drv_spi_init( );
	
	//RF24L01引脚初始化
	NRF24L01_Gpio_Init( );
	
	//检测nRF24L01
	NRF24L01_check( );	
	
	EXTI0_Config();					//按键中断
	EXTI10_Config();				//24L01 中断
	
	PWM_Config();
	
	while(1)
	{
		//led_red_flashing( );
		drv_delay_500Ms( 1 );
		NRF24L01_Write_Tx_Payload_InAck( (uint8_t *)pData, 8 );
	}
		
	
#ifdef	__RF24L01_TX_TEST__		
//=========================================================================================//	
//*****************************************************************************************//
//************************************* 发送 **********************************************//
//*****************************************************************************************//
//=========================================================================================//	
	
	//按键初始化
	drv_button_init( );
	
	RF24L01_Set_Mode( MODE_TX );		//发送模式
	while( 1 )	
	{
		//模式切换
		if( BUTOTN_PRESS_DOWN == drv_button_check( ))
		{
			g_TxMode = 1 - g_TxMode;		//模式会在 TX_MODE_1( 0 ),TX_MODE_2( 1 )之间切换
			
			//状态显示清零
			//led_green_off( );
			led_red_off( );
			
			if( TX_MODE_1 == g_TxMode )
			{
				for( i = 0; i < 6; i++ )		
				{
					led_red_flashing( );	//固定发送模式，红灯闪烁3次
					drv_delay_500Ms( 1 );		
				}
			}
			else
			{
				for( i = 0; i < 6; i++ )
				{
					//led_green_flashing( );	//串口发送模式，绿灯闪烁3次
					drv_delay_500Ms( 1 );
				}
			}
		}
		
		//发送
		if( TX_MODE_1 == g_TxMode )
		{
			NRF24L01_TxPacket( (uint8_t *)g_Ashining, 8 );		//模式1发送固定字符,1S一包
			drv_delay_500Ms( 1 );	
			drv_delay_500Ms( 1 );	
			led_red_flashing( );			
		}
		else
		{	
			//查询串口数据
			i = drv_uart_rx_bytes( g_UartRxBuffer );
			
			if( 0 != i )
			{
				NRF24L01_TxPacket( g_UartRxBuffer, i );
				led_red_flashing( );
			}
		}
	}
	
#else		
//=========================================================================================//	
//*****************************************************************************************//
//************************************* 接收 **********************************************//
//*****************************************************************************************//
//=========================================================================================//	
	
	RF24L01_Set_Mode( MODE_RX );		//接收模式
	while( 1 )
	{
		NRF24L01_RxPacket( g_RF24L01RxBuffer );		//接收字节
		i = NRF24L01_Read_Reg( R_RX_PL_WID );			//接收字节个数
		if( 0 != i )
		{
			led_red_flashing( );
			drv_uart_tx_bytes( g_RF24L01RxBuffer,i);	//输出接收到的字节
			
		}
	}
		

	
#endif
	
}

/**
  * @brief  Configure PA.00 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI0_Config(void)
{
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure PB.10 24L01 IRQ in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI10_Config(void)
{

  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  /* Configure PB.10 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  /* Connect EXTI9 Line to PB.09 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

  /* Configure EXTI10 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI10 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

}

