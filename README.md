# Auto-Tracking-System-with-STM32-and-OpenMV-2023-CEDA-Problem-E-
A real-time laser tracking system using STM32 and OpenMV, designed as a solution for Problem E of the 2023 National Electronic Design Contest. Features image-based target recognition, PID control, and dual-axis servo motion.
本项目是针对2023年国赛E题的一种思路，采用STM32和openMV开发。  

元件清单：  
18650锂电池3.7V 2800mAh 两个  
链接：https://e.tb.cn/h.6svCXN90n8wOaG8?tk=1mwNV8BMnKW  
18650锂电池盒，两节串联带开关带盖子（得小心，他家的正负线容易脱落）  
链接：https://e.tb.cn/h.6sz8uf2SPoKnfj2?tk=yJqxV8zdOSg  
二维云台，舵机采用TBSK280  
链接：https://e.tb.cn/h.6HcHJfXBmdJLPja?tk=i5SHV8BqgyY  
STM32最小系统板（可以买江协科技配套的）  
链接：https://e.tb.cn/h.6HcvIDDiLnIxs4Y?tk=AByMV8BsC2M  
3.3V稳压模块（建议使用STM32配套的ST-Link供电，防止PWM信号不稳定）  
降压模块  
链接：https://e.tb.cn/h.6HZEAewvUYP4ost?tk=GuADV8BBp03  
OpenMV4H7PLUS  
链接：https://e.tb.cn/h.6szUsRZsWVijkmf?tk=fh3PV8zZTBd  
激光模块（找商家要可持续工作的，防止直接烧坏，并且要可调光斑的）  
链接：https://e.tb.cn/h.6Hfxs1FyC2SQ4cj?tk=I5U6Vjcrlwr  

简要说明：  
1.由于舵机难调精度，故我们舍弃了速度，实测在1分钟内可跑完一圈（满分应在30秒内）  
2.由于时间有限，我们屏幕外框（0.5m x 0.5m）只能通过程序写死，并不使用openmv识别。如果想要识别的话可以设置状态机，也可以在识别外框后程序自动调整ROI区域  
3.由于作者现在大一水平有限，并不完全理解舵机有概率会突然跳步（无论怎样修改步幅都会突然移动相同距离）的原因，经初步排查有可能是电池供电不稳定的原因，但用适配器供电后任然会出现此情况。当然由于此舵机为开环控制，自然无法高精度。


