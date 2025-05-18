#include "stm32f10x.h" // Device header
#include "Delay.h"
#define SERVO_MIN_PULSE  500     // 最小PWM脉宽（0.5ms）
#define SERVO_MAX_PULSE  2500    // 最大PWM脉宽（2.5ms）
#define SERVO_ANGLE_RANGE 270.0f // 舵机总旋转角度
/**
  * @brief  初始化X轴舵机，占用TIM2,PA1口
  * @param  无
  * @retval 无
  */

void ServoX_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);//设置内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;   //ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;  //PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);

	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 0;      //CCR
	TIM_OC2Init(TIM2,&TIM_OCInitStruct);	
	
	TIM_Cmd(TIM2,ENABLE);

	
}

/**
  * @brief  初始化y轴舵机，占用TIM3，PA6口
  * @param  无
  * @retval 无
  */
void ServoY_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;          // PA6
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    TIM_InternalClockConfig(TIM3);  // 设置 TIM3 内部时钟
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;   // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;  // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0; // CCR
    // PA6 → TIM3_CH1
    TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	
    TIM_Cmd(TIM3, ENABLE);
}


/**
  * @brief  使X轴舵机归位
  * @param  无
  * @retval 无
  */
void ServoX_ResetAngle(void)
{
	TIM_SetCompare2(TIM2,1527);
}


/**
  * @brief  使y轴舵机归位
  * @param  无
  * @retval 无
  */
void ServoY_ResetAngle(void)
{
	TIM_SetCompare1(TIM3,1570);
}

/**
  * @brief  设置X轴舵机角度
  * @param  角度,逆时针为正，顺时针为负
  * @retval 无
  */
void ServoX_SetAngle(int16_t Compare)//1500
{
	uint16_t Angle = 1527 + Compare * 500 / 100;
	TIM_SetCompare2(TIM2,Angle);
}

/**
  * @brief  设置y轴舵机角度
  * @param  角度，往上为正，往下为负
  * @retval 无
  */
void ServoY_SetAngle(int16_t Compare)//1590
{
	
	uint16_t Angle = 1600 + Compare * 500 / 100;
	TIM_SetCompare1(TIM3,Angle);
	
}

void Servo_Tracking(void)//完成第一题的追踪目标，系数待定
{
	ServoX_SetAngle(24);
	Delay_ms(800);
	ServoY_SetAngle(10);
	Delay_ms(800);
	
	ServoX_SetAngle(-20);
	Delay_ms(800);
	ServoY_SetAngle(-30);
	Delay_ms(800);
	
	ServoX_SetAngle(23);
	Delay_ms(800);
	ServoY_SetAngle(9);

}
