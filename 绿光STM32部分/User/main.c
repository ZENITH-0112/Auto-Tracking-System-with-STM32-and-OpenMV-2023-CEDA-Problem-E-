#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "Servo.h"
#include "LED.h"
uint8_t KeyNum;	//定义用于接收按键键码的变量
uint16_t Num1 = 0;
uint8_t Key = 0;
static int16_t last_x = 0;
static int16_t last_y = 0;
int8_t step = 5;
int main(void)
{
	Serial_Init();		//串口初始化
	ServoX_Init();
	ServoY_Init();
	ServoX_ResetAngle();
	ServoY_ResetAngle();
	LED_And_Bee_Init();
	Key_Init();
	Timer_Init();
	while (1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum == 1)
		{
			Key = !Key;
		}
		if(!Key)
		{
			if((Serial_RxPacket[0] - 128)&&(Serial_RxPacket[1] - 128))
				Num1 = 1;
			ServoX_SetAngle(Serial_RxPacket[0] - 128);
			ServoY_SetAngle(Serial_RxPacket[1] - 128);
		}
	}
}
