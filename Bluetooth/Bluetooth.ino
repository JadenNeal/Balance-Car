//#include <I2Cdev.h>
/*
连线方法：
RX -- TX
TX -- RX
VCC -- 5V
GND -- GND
下载程序时，先将VCC断开，然后上传程序，接着再接上VCC，
用手机连接蓝牙即可。
*/

char val;  //存储接收的变量

void setup() {
  Serial.begin(9600);   //与电脑的串口连接
  Serial.println("Hc-06 is ready!");
//  BT.begin(9600);  //设置波特率
}

void loop() {
  //如果串口接收到数据，就输出到屏幕
  if (Serial.available()) {
    val = Serial.read();

    Serial.print(val);
  }
}
