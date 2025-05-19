#ifndef __SERVO_H__
#define __SERVO_H__
#define SERVO_MIN_PULSE  500     // 最小PWM脉宽（0.5ms）
#define SERVO_MAX_PULSE  2500    // 最大PWM脉宽（2.5ms）
#define SERVO_ANGLE_RANGE 270.0f // 舵机总旋转角度

void ServoX_Init(void);
void ServoY_Init(void);
void ServoX_ResetAngle(void);
void ServoY_ResetAngle(void);
void ServoX_SetAngle(int16_t Compare);
void ServoY_SetAngle(int16_t Compare);



#endif
