# Untitled - By: Lenovo - Sun Apr 20 2025

import sensor, image, time
from machine import UART
from pyb import Pin
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
sensor.set_brightness(1)          #设置相机亮度 取值-3到3
sensor.skip_frames(time = 1000)


#颜色阈值
redpoint_threshold = (23, 97, 21, 71, 65, 1)
#红点坐标，缓存最后一次红点坐标
redpoint = ((0,0),(0,0))
sub_redpoint = ((0,0),(0,0))
#配置串口，P4为Tx P5为Rx
uart = UART(3,115200)
#配置时钟
clock = time.clock()
#动态PID参数
'''调参建议
   震荡频率过大：减小kp 增大kd
   速度过慢：增大kp 增大ki 或者提高积分上限
   '''
first_edge = True
# 常规PID参数（小误差时使用）
Kp = 0.18    # 原0.205
Ki = 0.023   # 原0.027
Kd = 0.13    # 原0.111

# 激进PID参数（误差较大时使用）
eff_Kp = 0.210  # 原0.210
eff_Kd = 0.13  # 原0.111
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
#黑框顶点坐标
blackframe_corners = ((0,0),(0,0),(0,0),(0,0))
#黑框中心点坐标
blackframe_centers = (0,0)
#状态标志
c_count = 0
p_count = 0
m_count = 0#标记是否曾经检测到红点
n_count = 0
#角度转换参数,黑框实际宽度 / 屏幕中的像素
angle_conv_param1 = 0.397
#黑框大致位置
ROI = (95,19,146,147)
#弧度转角度
ANGLE_FACTOR = 180.0 / math.pi
#用于稳定目标点切换
at_target_counter = 0

#p_in0 = Pin('P0', Pin.IN, Pin.PULL_UP)#设置p_in为输入引脚，并开启上拉电阻，复位用
p_in1 = Pin('P1', Pin.IN, Pin.PULL_UP)#停止光标移动
p_in2 = Pin('P2', Pin.IN, Pin.PULL_UP)#重启光标移动

#发送数据包
def send_packet(data):
    packet = HEADER + bytes(data) + FOOTER
    uart.write(packet)


#根据顶点计算角度
def angle_cal(output_x , output_y):
    Servo_dx = int(math.atan2(output_x * angle_conv_param1,99) * ANGLE_FACTOR) + 128
    Servo_dy = int(math.atan2(output_y * angle_conv_param1,99) * ANGLE_FACTOR) + 128
    return Servo_dx , Servo_dy


#找红点
def find_redpoint(THRESHOLD, PIXELS_MIN = 1, AREA_MIN = 1):
    blobs = img.find_blobs([THRESHOLD],
                           pixels_threshold = PIXELS_MIN,
                           area_threshold = AREA_MIN,
                           merge=True)  # 合并相邻小片段

    if not blobs:
        print("未检测到任何红点")
        return None
        # 选取像素最多的那个 blob 作为目标
    b = max(blobs, key=lambda x: x.pixels())
    #print("检测到 blobs 数量：", len(blobs))
    #for i, b in enumerate(blobs):
        #print(i, "→ cx:", b.cx(), "cy:", b.cy(), "w:", b.w(), "h:", b.h(), "pixels:", b.pixels())

    redpoint0 = (b.cx(), b.cy())
    print(redpoint0)
    return redpoint0


#PID计算
'''p项指的是当误差较大，产生较大的力去驱动减小误差
   i项指的是消除系统的稳态误差，当有误差一直存在就加大力度以消除
   d项指的是当误差快速减小时（接近目标），D项产生反向刹车防止冲过头'''


def pid_compute(error, last_err, integral, Kp, Ki, Kd):

    if abs(integral) <= 5000:#抗饱和，防止积分项无限大
        integral += error
    else:
        integral = 5000
    derivative = error - last_err
    output = Kp * error + Ki * integral + Kd * derivative
    print(output, error, integral)
    return output, error, integral


#生成路径点
def generate_path(corners,extras = 7):
    pts = []
    seqs = extras + 1
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i+1)%4]
        pts.append((x0,y0))
        for k in range(1, seqs):
            xi = x0 + (x1-x0)*k//seqs
            yi = y0 + (y1-y0)*k//seqs
            pts.append((xi, yi))
    return pts

#将黑框点按左上，右上，右下，左下排列
def sort_corners(corners):
    # corners: [(x0,y0), (x1,y1), (x2,y2), (x3,y3)]
    corners = sorted(corners, key=lambda p: p[1])  # 按 y 坐标排序
    top = sorted(corners[:2], key=lambda p: p[0])   # 上面两个点，按 x 排序：左上，右上
    bottom = sorted(corners[2:], key=lambda p: p[0]) # 下面两个点，按 x 排序：左下，右下
    return (top[0], top[1], bottom[1], bottom[0])  # 左上，右上，右下，左下


#标准化黑框顶点
def stdVertex(
    corners: Tuple[
        Tuple[int, int],  # top-left
        Tuple[int, int],  # top-right
        Tuple[int, int],  # bottom-right
        Tuple[int, int],  # bottom-left
    ],
    d: int
) -> Tuple[Tuple[int, int], Tuple[int, int], Tuple[int, int], Tuple[int, int]]:
    """
    计算内侧平移 d 后的矩形顶点坐标（整数）。

    - corners: 顺时针顺序的 4 个整型坐标。
    - d:       内侧平移距离，整型。
    """
    def perp_left(v: Tuple[int, int]) -> Tuple[int, int]:
        # 左侧法向量：(-y, x)
        return (-v[1], v[0])

    if len(corners) != 4:
        raise ValueError("`corners` 必须是 4 个点组成的元组。")

    new_pts = []
    for i in range(4):
        im1 = (i - 1) % 4
        ip1 = (i + 1) % 4

        xi, yi     = corners[i]
        xim1, yim1 = corners[im1]
        xip1, yip1 = corners[ip1]

        # 边向量
        E_last = (xi - xim1, yi - yim1)
        E_next = (xip1 - xi, yip1 - yi)

        # 内侧法向量
        N_last = perp_left(E_last)
        N_next = perp_left(E_next)

        # 边长（用 sqrt 代替 hypot）
        L_last = math.sqrt(E_last[0]*E_last[0] + E_last[1]*E_last[1])
        L_next = math.sqrt(E_next[0]*E_next[0] + E_next[1]*E_next[1])
        if L_last == 0 or L_next == 0:
            raise ValueError(f"第 {i} 个顶点相邻边长度为零！")

        # 归一化
        N_last = (N_last[0]/L_last, N_last[1]/L_last)
        N_next = (N_next[0]/L_next, N_next[1]/L_next)

        # 计算偏移后坐标并四舍五入为整数
        rx = xi + d * (N_last[0] + N_next[0])
        ry = yi + d * (N_last[1] + N_next[1])
        new_pts.append((int(round(rx)), int(round(ry))))

    return tuple(new_pts)






#计算出黑框外顶点，识别逻辑为累计五次所得的矩形框中心点偏移范围不大于9像素
while(c_count <= 3):
    img = sensor.snapshot()

    rects = img.find_rects(threshold = 12000,roi = ROI)

    max_area = 0
    max_rect = None
    #计算出黑框的中心点
    x0,y0 = blackframe_corners[0]#左上角点的坐标
    x1,y1 = blackframe_corners[2]#右下角点的坐标
    cx = (x0 + x1)//2
    cy = (y0 + y1)//2
    blackframe_centers = cx,cy

    for rect in rects:
        area = rect.w() * rect.h()
        ratio = max(rect.w() / rect.h(), rect.h() / rect.w())
        #识别矩形的长宽比限定范围为1-3（A4纸约为1.414），同时面积不大于————，以防识别到屏幕线
        if area > max_area and ratio > 1 and ratio < 2 and area < 7920 :
            max_area = area
            max_rect = rect

    if max_rect:
        #获取四个预备外顶点
        corners = max_rect.corners()
        #计算预备矩形的中心坐标
        x00,y00 = corners[0]
        x11,y11 = corners[2]
        cx1 = (x00 + x11)//2
        cy1 = (y00 + y11)//2
        corners_center = cx1,cy1
        delta_x = abs(blackframe_centers[0] - corners_center[0])
        delta_y = abs(blackframe_centers[1] - corners_center[1])
        #限定中心坐标的偏移范围，若在范围内则计数加一，否则重新计数
        if delta_x <= 3 and delta_y <= 3:
            c_count += 1
        else:
            c_count = 0
        #保存上次矩形位置
        blackframe_corners = corners#保留原始顶点
        blackframe_corners = sort_corners(blackframe_corners)#对其排序




blackframe_corners = stdVertex(blackframe_corners,2.5)
path_points = generate_path(blackframe_corners)#生成路径
path_len    = len(path_points)
path_idx    = 0    # 从第一个点开始



#仅供测试，预览识别矩形，中心坐标以及打印四个外顶点位置
'''for i in range(len(blackframe_corners)):
    start_point = blackframe_corners[i]
    end_point = blackframe_corners[(i+1) % 4]
    img.draw_line(start_point[0], start_point[1], end_point[0], end_point[1], color = 255)
    img.draw_cross(blackframe_centers,color = 255)
print(blackframe_corners)'''


sensor.set_brightness(0)          #设置相机亮度 取值-3到3
sensor.set_auto_exposure(False,exposure_us = 20000)
sensor.skip_frames(frames=5)

while(True):
    clock.tick()
    img = sensor.snapshot()
    #img.gamma_corr(gamma=0.5, contrast=1.0)
    tx, ty = path_points[path_idx]

    img.draw_cross(tx, ty, color=(0,0,255))

    for i in range(len(blackframe_corners)):#对矩形框进行画线
        start_point = blackframe_corners[i]
        end_point = blackframe_corners[(i+1) % 4]
        img.draw_line(start_point[0], start_point[1], end_point[0], end_point[1], color = 255)
        img.draw_cross(blackframe_centers,color = 255)
    print(blackframe_corners)



    redpoint = find_redpoint(redpoint_threshold)

    if path_idx == 0 and p_count == 0:
        # 前馈补偿（根据实际机械特性调整）
        Servo_data = angle_cal(5, 5)  # 预置初始偏移
        send_packet(Servo_data)
        time.sleep_ms(100)  # 等待机械响应
    if redpoint:
        img.draw_cross(redpoint[0], redpoint[1], color=(0, 255, 0))



        error_x = redpoint[0] - tx
        error_y = redpoint[1] - ty
        at_target_counter += 1 if abs(error_x) <= 3 and abs(error_y) <= 4 else 0
        m_count = 1
        if n_count == 0:
            error_x0 = redpoint[0] - tx
            error_y0 = redpoint[1] - ty
            Servo_data = angle_cal(error_x0,error_y0)
            n_count = 1
            time.sleep_ms(40)

        if ((abs(error_x) >= 2) or (abs(error_y) >= 2)):
            if first_edge and path_idx < 5:  # 前5个路径点使用软启动
               Ki_adj = Ki * (path_idx / 5)  # 线性渐增
            else:
               Ki_adj = Ki
            if ((abs(error_x) >= 8) or (abs(error_y) >= 8)):#用积进的pid
                output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,eff_Kp,Ki_adj,eff_Kd)
                output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,eff_Kp,Ki_adj,eff_Kd)
                print("调用激进参数")
                Servo_data = angle_cal(output_x * 0.94,output_y * 0.94)
            else:#用精确的pid
                output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,Kp,Ki_adj,Kd)
                output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,Kp,Ki_adj,Kd)
                Servo_data = angle_cal(output_x,output_y)
            send_packet(Servo_data)
        sub_redpoint = redpoint
    else:
        if m_count == 1:
            error_x = sub_redpoint[0] - tx
            error_y = sub_redpoint[1] - ty
            if ((abs(error_x) >= 2) or (abs(error_y) >= 2)):
                if ((abs(error_x) >= 10) or (abs(error_y) >= 10)):
                    output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,eff_Kp,Ki,eff_Kd)
                    output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,eff_Kp,Ki,eff_Kd)
                    print("调用激进参数")
                    Servo_data = angle_cal(output_x * 0.94,output_y * 0.94)
                else:
                    output_x, last_error_x,integral_x = pid_compute(error_x,last_error_x,integral_x,Kp,Ki,Kd)
                    output_y, last_error_y,integral_y = pid_compute(error_y,last_error_y,integral_y,Kp,Ki,Kd)
                    Servo_data = angle_cal(output_x * 0.94,output_y * 0.94)

                send_packet(Servo_data)
        else:
            send_packet(Servo_data)

    if at_target_counter >= 2:  # 连续稳定2帧
        path_idx = (path_idx + 1) % path_len
        at_target_counter = 0
        # 新增：第一条边特殊处理
        if path_idx == 1:  # 刚离开第一条边
            integral_x *= 0.4  # 保留40%积分
            integral_y *= 0.4
            first_edge = False  # 标记第一条边已完成
        if integral_x >= 0:
            integral_x -= 70
        else:
            integral_x += 70
        if integral_y >= 0:
            integral_y -= 70
        else:
            integral_y += 70

        p_count += 1

    if p_count == 33:
        while(True):
            pass




    time.sleep_ms(15)








    print(clock.fps())

