# UNO R3开发板笔记
## 硬件接口介绍
Arduino是一款基于ATmega328P的微控制板。有14个数字输入/输出引脚（带`~`的可以输出调制模拟信号），6个模拟输入，16MHZ晶振时钟，


<div align="center"><img src="https://upload-images.jianshu.io/upload_images/12928010-3a6dce2da505b60f.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/838/format/webp" height="60%" width="60%"></div>

- 尺寸：W x H = 70 x 54 mm
- POWER引脚：开发板可提供3.3V和5V的电压输出，Vin引脚可用于从外部电源为开发板供电，推荐为7-12V。**电源输入、Vin和USB供电口三者只能接其中一个，否则容易导致板子烧毁。**
- Analog引脚：模拟输入，默认为INPUT，共有5个，为A0-A5。
- Digital引脚：数字引脚，输入输出都可以，其中标`~`的可作为调制模拟信号输出，其他的则使普通的数字引脚，只能输出0或1.共有14个。
    + TX和RX引脚：TX为发送，RX为接收，用于串口通讯。连接时应交叉连接（即板子的TX接模块的RX， 板子的RX接模块的TX）。
- AREF引脚：Reference voltage for the analog inputs (模拟输入的基准电压）。使用analogReference() 命令调用。

## 模拟接口/数字接口
### 数字接口
共14个，但0和1分别为RX和TX，一般不去占用，2-13是可以随意使用的。

**调制模拟信号**：说白了就是根据占空比来确定该时间段内的电平值。占空比越大，输出越接近高电平，具体的数值需要通过**等效电压**来计算（模电书上有讲）。特别的，50%的占空比的等效电压就是2.5V。

但UNO不能得到0-5V之间的任意一个电压，UNO输出的电压是**等分**的。UNO板有一个8bit的二进制空间来设置一个计数器，可以记的数就是2^0 + 2^1 + ... + 2^7 = 255， 即，将0-5V的占空比分成了255份。所以UNO可以模拟0-5V之间的255等分的电压值。

<div align="center"><img src="https://upload-images.jianshu.io/upload_images/12928010-23e5794a8c8f7ab9.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/602/format/webp" height="60%" width="60%"></div>

### 模拟接口
6个模拟接口。模拟接口只能读取不能输出模拟量（其实也可以输出，不过只能输出5V和0V，且输出的是CMOS信号而不是数字接口的TTL接口）。总之，一般模拟接口是用来接收传感器信号的。

和数字接口的计数器类似，模拟接口也有类似的功能，不同的是模拟接口采用10bit二进制的空间，能够计量2^0 + 2^1 + .. + 2^9 = 1023个数字，更为精确。然后，UNO将这些数字和电压值一一对应，比如1023对应5V， 0对应0V，中间的电压值也是按照比例来计算。