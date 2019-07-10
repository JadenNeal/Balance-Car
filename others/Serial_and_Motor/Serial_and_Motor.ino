char val;  //存储接收的变量

int STBY = 11; //使能端
//Motor A
int PWMA = 10; //左电机PWM输出控制脚   
int AIN1 = 12; //左电机正极  
int AIN2 = 13; //左电机负极  
  
//Motor B  
int PWMB = 9; //右电机PWM输出控制脚
int BIN1 = 7; //右电机正极 
int BIN2 = 6; //右电机负极

void stop();  // 停转控制函数
void runset(int motor, int speed, int direction);  //转动控制函数

void setup() {
  Serial.begin(9600);   //与电脑的串口连接
  Serial.println("everything is ready!");

  pinMode(STBY, OUTPUT);  // 引脚设置
  
  pinMode(PWMA, OUTPUT);  
  pinMode(AIN1, OUTPUT);  
  pinMode(AIN2, OUTPUT);  
  
  pinMode(PWMB, OUTPUT);  
  pinMode(BIN1, OUTPUT);  
  pinMode(BIN2, OUTPUT);  
}

void loop() {
  //如果串口接收到数据，就可以控制电机了
  if (Serial.available()) {
    val = Serial.read();
    Serial.println(val);
   
    if (val == '0') 
    {
      stop();  // 0,停转
      Serial.println("val = 0: stop!");
      }
    else if (val == '1') 
    {
      runset(2, 255, 1);  // 右电机全速正转
      runset(1, 255, 1);  // 左电机全速正转
      Serial.println("val = 1: forward!");
      }
    else if (val == '2')
    {
      runset(2, 128, 0);  // 右电机半速反转
      runset(1, 128, 0);  // 左电机半速反转
      Serial.println("val = 2: backward!");
//      delay(2000);
      }
  }
}

void runset(int motor, int speed, int direction){  
  
  digitalWrite(STBY, HIGH); //使能驱动模块 
  
  boolean Pin1 = LOW;  
  boolean Pin2 = HIGH;  
  
  if(direction == 1){  
    Pin1 = HIGH;  
    Pin2 = LOW;  
  }  
  
  if(motor == 1){  
    digitalWrite(AIN1, Pin1);  
    digitalWrite(AIN2, Pin2);  
    analogWrite(PWMA, speed);  
  }
  else{  
    digitalWrite(BIN1, Pin1);  
    digitalWrite(BIN2, Pin2);  
    analogWrite(PWMB, speed);  
  }  
} 

void stop(){  
  digitalWrite(STBY, LOW);   
} 
