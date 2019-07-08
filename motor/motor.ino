/****************************************************************************8
   亚博智能科技有限公司
   程序：TB6612FNG测试程序
   程序直接烧录到平衡车上就可使用测试电机
*/
int STBY = 11; //使能端
  
//Motor A
int PWMA = 10; //左电机PWM输出控制脚   
int AIN1 = 12; //左电机正极  
int AIN2 = 13; //左电机负极  
  
//Motor B  
int PWMB = 9; //右电机PWM输出控制脚
int BIN1 = 7; //右电机正极 
int BIN2 = 6; //右电机负极  
  
void setup(){  
  pinMode(STBY, OUTPUT);  
  
  pinMode(PWMA, OUTPUT);  
  pinMode(AIN1, OUTPUT);  
  pinMode(AIN2, OUTPUT);  
  
  pinMode(PWMB, OUTPUT);  
  pinMode(BIN1, OUTPUT);  
  pinMode(BIN2, OUTPUT);  
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
  }else{  
    digitalWrite(BIN1, Pin1);  
    digitalWrite(BIN2, Pin2);  
    analogWrite(PWMB, speed);  
  }  
}  
  
void loop(){  
  runset(1, 255, 1); //左电机全速向前转  
  runset(2, 255, 1); //右电机全速向前转  
  
  delay(2000); //2秒 
  stop(); //停转
  delay(250); //两轮继续向前转  
  
  runset(1, 128, 0); //左电机半速向后转  
  runset(2, 128, 0); //右电机半速向后转  
  
  delay(1000);  
  stop();  
  delay(250);  
}  
   
void stop(){  
  digitalWrite(STBY, LOW);   
}  
