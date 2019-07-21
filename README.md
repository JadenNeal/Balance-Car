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
3. 显示屏(SSD 1306)  
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

<div align="center"><img src="https://s2.ax1x.com/2019/07/11/ZRYGQA.jpg" alt="3维布局" />

3维布局草图</div>

## 模块测试
### 单模块测试程序 
**注意：单模块测试时，接线按照**[原理图](https://github.com/JadenNeal/Balance-Car/blob/master/%E5%B9%B3%E8%A1%A1%E5%B0%8F%E8%BD%A6%E5%8E%9F%E7%90%86%E5%9B%BE.pdf)
- [蓝牙HC-06](https://github.com/JadenNeal/Balance-Car/blob/master/Bluetooth/Bluetooth.ino)
- [红外对管](https://github.com/JadenNeal/Balance-Car/blob/master/Infra/Infra.ino)
- [陀螺仪MPU6050](https://github.com/JadenNeal/Balance-Car/blob/master/MPU6050/MPU6050.ino)
- [晶门SSD 1306 OLED显示屏](https://github.com/JadenNeal/Balance-Car/blob/master/OLED/OLED.ino)
- [超声波SR04](https://github.com/JadenNeal/Balance-Car/blob/master/Ultrasonic/Ultrasonic.ino)
- [全彩LED](https://github.com/JadenNeal/Balance-Car/blob/master/fullLED/fullLED.ino)
- [电机](https://github.com/JadenNeal/Balance-Car/blob/master/motor/motor.ino)

### 多模块联调测试程序  
**注意：接线同上**
- [蓝牙控制电机](https://github.com/JadenNeal/Balance-Car/blob/master/others/BT_and_Motor/BT_and_Motor.ino)
- [串口控制电机](https://github.com/JadenNeal/Balance-Car/blob/master/others/Serial_and_Motor/Serial_and_Motor.ino)
- [测定小车机械中值](https://github.com/JadenNeal/Balance-Car/tree/master/others/final_v2)
- [红外线和LED](https://github.com/JadenNeal/Balance-Car/tree/master/others/infra_and_LED)

### 最终实现程序  
**注意：接线按照“整车连线”写的接线**
- [实现小车自平衡](https://github.com/JadenNeal/Balance-Car/blob/master/others/final_v4/final_v4.ino)
- [最终版程序](https://github.com/JadenNeal/Balance-Car/blob/master/others/final_v5/final_v5.ino)

## 模块认知
测试笔记记录在[这里](https://github.com/JadenNeal/Balance-Car/tree/master/TEST_NOTES)。
### UNO开发板
[参见这里](https://github.com/JadenNeal/Balance-Car/blob/master/TEST_NOTES/20190708/UNO_R3%E5%BC%80%E5%8F%91%E6%9D%BF%E7%AC%94%E8%AE%B0.md)
### Mpu6050模块
陀螺仪。  

[知乎](https://zhuanlan.zhihu.com/p/20082486)上的关于MPU6050的讲解。

[野火](http://www.luwl.net/wp-content/uploads/2017/03/MPU6050%E6%95%99%E7%A8%8B.pdf)上关于MPU6050的讲解。

[英文版手册下载地址](http://pdf1.alldatasheetcn.com/datasheet-pdf/view/517744/ETC1/MPU-6050.html)。

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

1. **Q**：明明板子单独测模块的时候没问题，但是后来小车调试的时候有时上传程序出错，两次上传隔的时间太短的话就更容易出现这种情况，为什么呢？  
    **A**：这是由于**在用USB上传程序时**，开发板的VCC接到了其他模块的缘故。如果不拔下来，多次往复就有很大概率导致板子烧毁。因此，在上传程序的时候，一定要把板子的VCC断开，这样就不会有问题了。  
    如果给空板子烧程序都失败，那**很有可能是板子坏了**。

2. **Q**：上传程序后，小车有反应，但是总有延迟。在转动小车的时候，轮胎总要过一段时间才会作出反应，这是为什么呢？
    **A**：**程序的问题**。检查代码中是否有遗留下来的`delay()`，是否有`Serial.println()`、`Serial.print()`等需要串口才工作的函数。把这些全部注释掉，再查看效果。

3. **Q**：loop()是空的，按理说不会有函数运行，但是给小车上传程序后，小车还是能够对陀螺仪的变化作出反应，为什么呢？
    **A**：对于本次程序来说，这是因为**中断**或者**串口**存在的原因。  
    来看arduino的程序结构：
    ```c
    #include <arduino.h> 

    int main(){ 
        init(); // 初始化库 
        initVariant(); // 初始化变量

        setup(); // 初始化 
        for(;;){ 
            loop(); // 主循环 
            if SerialEvent() 
                serialEvent(); // 串口函数 
        } 
    } 
    ```
    最后的死循环中，除了loop，还有一个串口事件判断函数——有事件则去执行，没有就只执行loop。而loop中虽然没有语句，**但setup()中还有中断的功能**
    ```c
    MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断，控制control函数
    MsTimer2::start();          //使用中断使能
    ```
    开启了中断使能后，程序就会每隔中断的时间去执行一次中断指向的函数。因此loop中没有语句也是可以的。

4. 关于速度环调试的问题整理，参见[调试笔记04](https://github.com/JadenNeal/Balance-Car/blob/master/TEST_NOTES/20190717/%E8%B0%83%E8%AF%95%E7%AC%94%E8%AE%B004.md)
5. 关于转向环调试的问题整理，参见[调试笔记04](https://github.com/JadenNeal/Balance-Car/blob/master/TEST_NOTES/20190717/%E8%B0%83%E8%AF%95%E7%AC%94%E8%AE%B004.md)
