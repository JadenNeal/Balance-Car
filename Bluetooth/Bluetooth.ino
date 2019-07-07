#include <SoftwareSerial.h>
//使用软件串口，能将数字口模拟成串口
SoftwareSerial BT(8, 9);  //新建对象，板子的接收脚为8，发送脚为9
char val;  //存储接收的变量

void setup() {
  Serial.begin(9600);   //与电脑的串口连接
  Serial.println("Hc-06 is ready!");
  BT.begin(9600);  //设置波特率
}

void loop() {
  //如果串口接收到数据，就输出到蓝牙串口
  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
  }

  //如果接收到蓝牙模块的数据，输出到屏幕
  if (BT.available()) {
    val = BT.read();
    Serial.print(val);
  }
}
