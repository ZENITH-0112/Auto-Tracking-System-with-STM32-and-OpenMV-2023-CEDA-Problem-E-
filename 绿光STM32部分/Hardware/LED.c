#include "stm32f10x.h"                  // Device header
#include "Delay.h"
extern uint16_t Num1;

/**
  * 函    数：LED和蜂鸣器初始化，PB6为LED，PB7为蜂鸣器
  * 参    数：无
  * 返 回 值：无
  */
void LED_And_Bee_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);		//开启GPIOB的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//将PB6和PB7引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);				//设置PB6引脚为低电平
	GPIO_SetBits(GPIOB, GPIO_Pin_7);				//设置PB7引脚为高电平
}

/**
  * 函    数：LED开启
  * 参    数：无
  * 返 回 值：无
  */
void LED_ON(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_6);		//设置PB6引脚为高电平
}

/**
  * 函    数：LED关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED_OFF(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);		//设置PB6引脚为低电平
}


/**
  * 函    数：蜂鸣器开启
  * 参    数：无
  * 返 回 值：无
  */
void Bee_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);		//设置PB7引脚为低电平
}

/**
  * 函    数：蜂鸣器关闭
  * 参    数：无
  * 返 回 值：无
  */
void Bee_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_7);		//设置PB7引脚为高电平
}



void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_InternalClockConfig(TIM4);//设置内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4,ENABLE);
}
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
	{
		if(Num1 == 1)
		{	
			LED_ON();
			Bee_ON();
			Delay_ms(100);

			LED_OFF();
			Bee_OFF();
			Num1 = 0;
		}

		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
	
}

