# 基于Arduino UNO R3开发板的平衡车设计
## 材料
- Arduino UNO R3
- Mpu6050陀螺仪
- HC-06蓝牙模块
- 晶门SSD 1306 0.96英寸OLED显示模块
- HC-SR04 超声波测距模块
- 红外对管模块
- WS2812全彩LED模块
- TB6612电机驱动模块
- 杜邦线若干（双公头，双母头，公母头）
- 电机两个（含霍尔测速编码器）

## 整车连线
1. 超声波 
    + GND -- GND
    + Echo -- A3
    + Trig -- A2
    + VCC -- 5V
2. 蓝牙(HC-06) 
    + RX -- 1（板子的发射端）
    + TX -- 0（板子的接收端）
    + GND -- GND
    + VCC -- VCC
3. 显示屏(HW-102)  
    + VCC -- 5V
    + GND -- GND
    + SCL -- SCL
    + SDA -- SDA
4. 电机驱动(TB6612)
    + PWMA -- 10
    + AIN2 -- 13
    + AIN1 -- 12
    + STBY -- 11
    + BIN1 -- 7
    + BIN2 -- 6
    + PWMB -- 9
    + GND -- GND
    + VM -- 12V
    + VCC -- 5V
    + GND -- GND
    + AO1 -- AO1
    + AO2 -- AO2
    + BO2 -- BO2
    + BO1 -- BO1
    + GND -- GND
5. 陀螺仪(MPU6050) 
    + VCC -- 5V
    + GND -- GND
    + SCL -- A5
    + SDA -- A4
    + INT -- 2
    + 最好在VCC和GND之间加个0.1uf的旁路电容
6. 电机A
    + AO2
    + 5V
    + 3
    + 8
    + GND(并联0.1uf的旁路电容)
    + AO1（正极）
7. 电机B
    + BO1（正极）
    + GND(并联0.1uf的旁路电容)
    + 4
    + 5
    + 5V
    + BO2
8. 全彩LED  
    + IN -- A1
    + VCC -- 5V
    + GND -- GND
9. 红外对管  
    + OUT -- A0
    + GND -- GND
    + VCC -- VCC

<div align="center"><img src="https://s2.ax1x.com/2019/07/09/ZyMTV1.png" alt="接线图" border="0" />

空余的为12V电池接口</div>

<div align="center"><img src="https://s2.ax1x.com/2019/07/11/ZRYGQA.jpg" alt="3维布局" />

*3维布局图*</div>

## 模块测试
以下是**单个模块**的测试程序。
- [蓝牙HC-06](https://github.com/JadenNeal/Balance-Car/blob/master/Bluetooth/Bluetooth.ino)
- [红外对管](https://github.com/JadenNeal/Balance-Car/blob/master/Infra/Infra.ino)
- [陀螺仪MPU6050](https://github.com/JadenNeal/Balance-Car/blob/master/MPU6050/MPU6050.ino)
- [晶门SSD 1306 OLED显示屏](https://github.com/JadenNeal/Balance-Car/blob/master/OLED/OLED.ino)
- [超声波SR04](https://github.com/JadenNeal/Balance-Car/blob/master/Ultrasonic/Ultrasonic.ino)
- [全彩LED](https://github.com/JadenNeal/Balance-Car/blob/master/fullLED/fullLED.ino)
- [电机](https://github.com/JadenNeal/Balance-Car/blob/master/motor/motor.ino)

**多模块**的联调存放在[others](https://github.com/JadenNeal/Balance-Car/tree/master/others)文件夹中。

## 模块认知
测试笔记记录在[这里](https://github.com/JadenNeal/Balance-Car/tree/master/TEST_NOTES)。
### UNO开发板
[参见这里](https://github.com/JadenNeal/Balance-Car/blob/master/TEST_NOTES/20190708/UNO_R3%E5%BC%80%E5%8F%91%E6%9D%BF%E7%AC%94%E8%AE%B0.md)
### Mpu6050模块
陀螺仪。  

[知乎](https://zhuanlan.zhihu.com/p/20082486)上的关于MPU6050的讲解。

[野火](http://www.luwl.net/wp-content/uploads/2017/03/MPU6050%E6%95%99%E7%A8%8B.pdf)上关于MPU6050的讲解。

[英文版手册下载地址](http://pdf1.alldatasheetcn.com/datasheet-pdf/view/517744/ETC1/MPU-6050.html)。

[中文版手册下载地址](https://wenku.baidu.com/view/a0c0f751a31614791711cc7931b765ce04087a13.html)。

### HC-06
蓝牙。  
HC-06是目前用的较多的蓝牙模块，HC-06只能作为从机工作，因此对于被控制的小车来说非常适合使用。

[这里](https://www.jianshu.com/p/5c220d0d3692)有非常清晰的接线图，其实不需要上位机测试，但更改名称或PIN需要用上位机进入AT进行更改。下载一个串口通信助手，打开后连接蓝牙（默认名称为HC-06），成功后即可发送数据，进行通信。

### SSD 1306
OLED显示模块。

显示小车信息。

### 红外对管模块
用来辅助测距。

### HC-SR04
超声波。

测距功能实现的主要模块。 

### WS2812
全彩LED。

显示小车状态。

## 问题汇总
在制作的过程中，遇到的问题有很多，包括硬件软件的问题，这里只列出几个比较典型的问题。

待补充。
