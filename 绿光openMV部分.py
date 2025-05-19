
#注意：
#该代码是绿色激光点追踪红色激光点



import sensor, image, time
from machine import UART
import math
#基本配置
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(True)            #镜头垂直反转
sensor.set_hmirror(True)          #镜头水平反转
sensor.set_auto_gain(False)       #关闭自动增益
sensor.set_auto_whitebal(False)   #关闭白平衡
sensor.set_contrast(1)            #提高对比度
sensor.set_brightness(-1) #设置相机亮度 取值-3到3
sensor.set_auto_exposure(False,exposure_us = 20000)
sensor.skip_frames(time = 1000)


#颜色阈值
greenpoint_threshold2 = (62, 94, -57, -29, 9, 53)#
greenpoint_threshold3 = (40, 86, -22, -2, 0, 13)
#红色阈值
redpoint_threshold = (23, 97, 21, 71, 66, -3)
#绿点坐标，滞留绿点坐标
greenpoint = ((0,0),(0,0))
sub_greenpoint = ((0,0),(0,0))
#配置串口，P4为Tx P5为Rx
uart = UART(3,115200)
#配置时钟
clock = time.clock()
#动态PID参数
Kp = 0.224
Ki = 0.0275
Kd = 0.108
eff_Kp = 0.234
eff_Kd = 0.108
#PID变量
error_x = 0
last_error_x = 0
integral_x = 0
error_y = 0
last_error_y = 0
integral_y = 0
output_x = 0
output_y = 0
#包头包尾
HEADER = b'\xFF'
FOOTER = b'\xFE'
#舵机配置
Servo_dx = 0
Servo_dy = 0
Servo_data = [0,0]
#状态标志
c_count = 0
p_count = 0
m_count = 0
n_count = 0
#角度转换参数,黑框实际宽度 / 屏幕中的像素
angle_conv_param1 = 0.47
#弧度转角度
ANGLE_FACTOR = 180.0 / math.pi
#用于稳定目标点切换
at_target_counter = 0


#发送数据包
def send_packet(data):
    packet = HEADER + bytes(data) + FOOTER
    uart.write(packet)


#根据顶点计算角度
def angle_cal(output_x , output_y):
    Servo_dx = int(math.atan2(output_x * angle_conv_param1,89) * ANGLE_FACTOR) + 128
    Servo_dy = int(math.atan2(output_y * angle_conv_param1,98) * ANGLE_FACTOR) + 128
    return Servo_dx , Servo_dy


#找绿点
def find_point(THRESHOLD, PIXELS_MIN = 1, AREA_MIN = 1):
    blobs = img.find_blobs([THRESHOLD],
                           pixels_threshold = PIXELS_MIN,
                           area_threshold = AREA_MIN,
                           merge=True)  # 合并相邻小片段
    if not blobs:
        print("未检测到任何绿点")
        return None

        # 选取像素最多的那个 blob 作为目标
    b = max(blobs, key=lambda x: x.pixels())
    #print("检测到 blobs 数量：", len(blobs))
    #for i, b in enumerate(blobs):
        #print(i, "→ cx:", b.cx(), "cy:", b.cy(), "w:", b.w(), "h:", b.h(), "pixels:", b.pixels())

    point0 = (b.cx(), b.cy())
    print(point0)
    return point0


#PID计算
def pid_compute(error, last_err, integral, Kp, Ki, Kd):

    if abs(integral) <= 6500:
        integral += error
    else:
        integral = 6500
    derivative = error - last_err
    output = Kp * error + Ki * integral + Kd * derivative
    print(output, error, integral)
    return output, error, integral




while(True):
    clock.tick()
    img = sensor.snapshot()
    img.gamma_corr(gamma=1.0, contrast=1.0)  #提高对比度

    #找到红绿激光位置
    redpoint = find_point(redpoint_threshold)
    greenpoint = find_point(greenpoint_threshold2)

    #如果同时识别到红绿点
    if redpoint and greenpoint:
        img.draw_cross(greenpoint[0], greenpoint[1], color=(0, 255, 0))
        img.draw_cross(redpoint[0], redpoint[1], color=(255, 0, 0))
        error_x = greenpoint[0] - redpoint[0]
        error_y = greenpoint[1] - redpoint[1]
        at_target_counter += 1 if abs(error_x) <= 3 and abs(error_y) <= 3 else 0      #当两点距离小于四个像素点时，认为已暂时成功追踪
        m_count = 1

        #用于初始位置直接缩短两点间距离，防止积分项有较大的初始值
        if n_count == 0:
            Servo_data = angle_cal(error_x,error_y)
            n_count = 1
            time.sleep_ms(40)

        #如果两点距离大于3个像素点则调用激进的PID参数
        if ((abs(error_x) > 3) or (abs(error_y) > 3)):
            if ((abs(error_x) >= 10) or (abs(error_y) >= 20)):
                output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,eff_Kp,Ki,eff_Kd)
                output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,eff_Kp,Ki,eff_Kd)
                Servo_data = angle_cal(output_x,output_y)
            else:
                output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,Kp,Ki,Kd)
                output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,Kp,Ki,Kd)
                Servo_data = angle_cal(output_x,output_y)
            send_packet(Servo_data)
        #保存上次识别到的红绿点位置，用于丢点时的即时追踪
        sub_redpoint = redpoint
        sub_greenpoint = greenpoint

    #如果绿点丢失，依照上次识别位置再次计算发包
    elif greenpoint is None:
        if m_count == 1:     #用于保证至少有识别到绿点才会进入程序，防报错
            error_x = sub_greenpoint[0] - sub_redpoint[0]
            error_y = sub_greenpoint[1] - sub_redpoint[1]
            if ((abs(error_x) > 3) or (abs(error_y) > 3)):
                if ((abs(error_x) >= 10) or (abs(error_y) >= 10)):
                    output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,eff_Kp,Ki,eff_Kd)
                    output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,eff_Kp,Ki,eff_Kd)
                    output_x +=3
                    output_y +=3
                    Servo_data = angle_cal(output_x,output_y)
                else:
                    output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,Kp,Ki,Kd)
                    output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,Kp,Ki,Kd)
                    output_x +=3
                    output_y +=3
                    Servo_data = angle_cal(output_x,output_y)
                send_packet(Servo_data)
        else:
            send_packet(Servo_data)

    if at_target_counter >= 2:  # 连续稳定2次后减少积分项，用于提升稳定性
        at_target_counter = 0
        if integral_x >= 0:
            integral_x -= 95
        else:
            integral_x += 95

        if integral_y >= 0:
            integral_y -= 95
        else:
            integral_y += 95

    #留出时间给舵机更新位置
    time.sleep_ms(15)







    print(clock.fps())
