#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"

#include "Key.h"
#include "Servo.h"
uint8_t KeyNum;			//定义用于接收按键键码的变量
int8_t step = 1;
uint8_t iskey1active = 0;
static int16_t last_x = 0;
static int16_t last_y = 0;
int main(void)
{
	Key_Init();//按键初始化，用于变化状态
	Serial_Init();		//串口初始化
	ServoX_Init();
	ServoY_Init();
	ServoX_ResetAngle();
	ServoY_ResetAngle();
    
	
	
	while (1)
	{
		KeyNum = Key_GetNum ();
	    if(KeyNum == 1)
		{
		iskey1active = !iskey1active;
		}
		if (iskey1active)
		{
				if(Serial_RxPacket[0] > last_x + step)
			{
				last_x += step;
			}
			else if(Serial_RxPacket[0] < last_x - step)
			{
				last_x -= step;
			}
			else
				last_x = Serial_RxPacket[0];
		
			if(Serial_RxPacket[1] > last_y + 1)
			{
				last_y += 1;
			}
			else if(Serial_RxPacket[1] < last_y - 1)
			{
				last_y -= 1;
			}
			else
			last_y = Serial_RxPacket[1];
		
		
			ServoX_SetAngle(Serial_RxPacket[0] - 128);
			ServoY_SetAngle(Serial_RxPacket[1] - 128);
		

		}
			
    
		if(KeyNum ==2)
		{
			 Servo_Tracking();
		}
		
		
		
	}
	
}
	
		

