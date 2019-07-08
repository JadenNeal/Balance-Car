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

## 流程
1. 开发平台及模组认知
2. 平衡小车设计案例分析
3. 小车功能开发
4. 成果展示

## 模块认知
测试笔记记录在[这里](https://github.com/JadenNeal/Balance-Car/tree/master/TEST_NOTES)。
### UNO开发板
[参见这里](https://github.com/JadenNeal/Balance-Car/blob/master/TEST_NOTES/20190708/UNO_R3%E5%BC%80%E5%8F%91%E6%9D%BF%E7%AC%94%E8%AE%B0.md)
### Mpu6050模块
陀螺仪。  

[知乎](https://zhuanlan.zhihu.com/p/20082486)上也有一篇关于MPU6050的讲解，代码缺少卡尔曼滤波，运行不起来，但原理部分基本讲清楚了。

[野火](http://www.luwl.net/wp-content/uploads/2017/03/MPU6050%E6%95%99%E7%A8%8B.pdf)上关于此模块的讲解。

[英文版手册下载地址](http://pdf1.alldatasheetcn.com/datasheet-pdf/view/517744/ETC1/MPU-6050.html)。

[中文版手册下载地址](https://wenku.baidu.com/view/a0c0f751a31614791711cc7931b765ce04087a13.html)。

### HC-06
蓝牙。  
HC-06是目前用的较多的蓝牙模块，HC-06只能作为从机工作，因此对于被控制的小车来说非常适合使用。

[这里](https://www.jianshu.com/p/5c220d0d3692)有非常清晰的接线图，其实不需要上位机测试，但更改名称或PIN需要用上位机进入AT进行更改。下载一个串口通信助手，打开后连接蓝牙（默认名称为HC-06），成功后即可发送数据，进行通信。

### SSD 1306
OLED显示模块。
待补充。

### 红外对管模块
待补充。

### HC-SR04
超声波。  

### WS2812
全彩LED。

### TB6612
电机驱动模块。
