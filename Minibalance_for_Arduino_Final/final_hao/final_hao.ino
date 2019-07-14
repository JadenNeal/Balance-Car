#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <KalmanFilter.h>    //卡尔曼滤波
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include <Wire.h>
//#include "U8glib.h"
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
//#define KEY 3     //按键引脚
#define IN1 12   //TB6612FNG驱动模块控制信号 共6个
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9
#define ENCODER_L 2  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 5
#define ENCODER_R 4
#define DIRECTION_R 8
#define ZHONGZHI 1.5//小车的机械中值  DIFFERENCE
#define DIFFERENCE 5
#define PIN 3
#define MAX_LED 7
#define ADD true
#define SUB false



/****测试变量******/
int v;
int leftv, rightv;

/*******蓝牙控制*********/

int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量
int PID_Send;
unsigned char Flag_Stop = 1, Flash_Send;


KalmanFilter KalFilter;
int16_t ax, ay, az, gx, gy, gz;
Adafruit_NeoPixel strip = Adafruit_NeoPixel( MAX_LED, PIN, NEO_RGB + NEO_KHZ800 );
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
static int VelocityCount, TurnCount, EncoderCount, ledCount;

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
int Motor1, Motor2;      //电机叠加之后的PWM
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度


float Balance_Kp = 21, Balance_Kd = 0.5, Velocity_Kp = 2, Velocity_Ki = 0.01;


MPU6050 Mpu6050;

/************卡尔曼滤波相关变量***********/
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
int addr = 0;

float Angle, bias;






void setup() {
  Wire.begin();
  strip.begin();
  Serial.begin(9600);
  Mpu6050.initialize();
  MsTimer2::set(5, control);
  MsTimer2::start();
  attachInterrupt(0, READ_ENCODER_L, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, READ_ENCODER_R, CHANGE);  //开启外部中断 编码器接口2
}

/**************OLED显示屏***************/
//void draw() {
//
////  u8g.firstPage();
//  do {
//    u8g.setFont(u8g_font_unifont);//设置要显示字符的字体
//    u8g.drawStr(0, 20, "Hello Arduino");//显示字符
//    u8g.drawStr(0, 40, "Angle: ");
//    u8g.setPrintPos(60,40);
//    u8g.print(Angle);
//  } while( u8g.nextPage() );
//}

/*****************LED灯***************/
void light() {
  uint32_t color_Red = strip.Color(0, 255, 0);
  uint32_t color_Green = strip.Color(255, 0, 0);
  uint32_t color_Blue = strip.Color(0, 0, 255);
  uint32_t color_Yellow = strip.Color(255, 255, 0);
  int i = 0;
  for (i; i < MAX_LED; i++) {
    if (Flag_Qian) {
      strip.setPixelColor(i, color_Red);
    }
    else if (Flag_Hou) {
      strip.setPixelColor(i, color_Green);
    }
    else if (Flag_Left) {
      strip.setPixelColor(i, color_Blue);
    }
    else if (Flag_Right) {
      strip.setPixelColor(i, color_Yellow);
    }
  }
  strip.show();
}





void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;//根据另外一相电平判定方向
    else      Velocity_R--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--; //根据另外一相电平判定方向
    else     Velocity_R++;
  }
}

void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}

/**************蓝牙********************/
void serialEvent()
{
  static unsigned char Flag_PID, Receive[10], Receive_Data, i, j;
  static float Data;

  while (Serial.available()) {

    Receive_Data = Serial.read();
    if (Receive_Data == 0x7B) Flag_PID = 1; //参数指令起始位  {
    if (Receive_Data == 0x7D) Flag_PID = 2; //参数指令停止位   }
    if (Flag_PID == 1)
    {
      Receive[i] = Receive_Data;   //记录数据
      i++;
    }
    else  if (Flag_PID == 2) //执行指令
    {
      if (Receive[3] == 0x50)          PID_Send = 1; //获取PID参数  P
      else  if (Receive[3] == 0x57)    Flash_Send = 1; //掉电保存参数  W
      else  if (Receive[1] != 0x23) //更新PID参数  #
      {
        for (j = i; j >= 4; j--)
        {
          Data += (Receive[j - 1] - 48) * pow(10, i - j); //通讯协议
        }
        switch (Receive[1])
        {
          case 0x30:  Balance_Kp = Data / 100; break;
          case 0x31:  Balance_Kd = Data / 100; break;
          case 0x32:  Velocity_Kp = Data / 100; break;
          case 0x33:  Velocity_Ki = Data / 100; break;
          case 0x34:  break; //9个通道，预留5个
          case 0x35:  break;
          case 0x36:  break;
          case 0x37:  break;
          case 0x38:  break;
        }
      }
      Flag_PID = 0; //相关标志位清零
      i = 0;
      j = 0;
      Data = 0;
    }
    else  //蓝牙遥控指令
    {
      switch (Receive_Data)   {
        //这是MinibalanceV1.0的APP发送指令
        case 0x01: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
        case 0x02: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
        case 0x03: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
        case 0x04: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
        case 0x05: Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;              //后退
        case 0x06: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
        case 0x07: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
        case 0x08: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
        //这是MinibalanceV3.5的APP发送指令
        case 0x41: Flag_Qian = 1, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0;   break;              //前进
        case 0x42: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
        case 0x43: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;             //右转
        case 0x44: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 1;   break;              //右转
        case 0x45: Flag_Qian = 0, Flag_Hou = 1, Flag_Left = 0, Flag_Right = 0;   break;             //后退
        case 0x46: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;    break;               //左转
        case 0x47: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;               //左转
        case 0x48: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 1, Flag_Right = 0;   break;             //左转
        default: Flag_Qian = 0, Flag_Hou = 0, Flag_Left = 0, Flag_Right = 0 , Flag_Stop = 1;    break;               //停止
      }
    }
  }
}



/********************直立环*****************/
int balance(float Angle, float Gyro)
{
  float Bias;
  int balance;
  Bias = Angle - ZHONGZHI;   //===求出平衡的角度中值 和机械相关
  balance = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance;

}



/******************速度环******************/
int velocity(int Encoder_Left, int Encoder_Right)
{

  static float Velocity, Encoder_Least, Encoder , Movement;
  static float Encoder_Integral;


  if       ( Flag_Qian == 1) Movement = 600;
  else   if ( Flag_Hou == 1) Movement = -600;
  else    //这里是停止的时候反转，让小车尽快停下来
  {
    Movement = 0;
    if (Encoder_Integral > 300)   Encoder_Integral -= 200;
    if (Encoder_Integral < -300)  Encoder_Integral += 200;
  }

  //float kp=10,ki=0.4;
  Encoder_Least = (Encoder_Left + Encoder_Right) - 0;
  Encoder *= 0.7;
  Encoder += Encoder_Least * 0.3;
  Encoder_Integral += Encoder;
  Encoder_Integral =  Encoder_Integral - Movement;
  if (Encoder_Integral > 21000)    Encoder_Integral = 21000;        //===积分限幅
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;         //===积分限幅
  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;

  //  if (Angle < -40 || Angle > 40 || Flag_Stop == 1){
  //    analogWrite(PWMA, 0);  //PWM输出为0
  //    analogWrite(PWMB, 0); //PWM输出为0
  //    Encoder_Integral =0;
  //  }

  return Velocity;
}


/*******************转向环********************/
int turn(float gyro)//转向控制
{
  static float Turn_Target, Turn, Turn_Convert = 1;
  float Turn_Amplitude = 40, Kp = 2, Kd = 0.001;  //PD参数 Turn_Amplitude表示专项最大振幅
  if (1 == Flag_Left)             Turn_Target += Turn_Convert;  //根据遥控指令改变转向偏差
  else if (1 == Flag_Right)       Turn_Target -= Turn_Convert;//根据遥控指令改变转向偏差
  else Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude; //===转向速度限幅
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
  Turn = -Turn_Target * Kp + gyro * Kd;         //===结合Z轴陀螺仪进行PD控制
  return Turn;
}

/******************PWM赋值********************/
void Set_Pwm(int moto1, int moto2)
{
  if (moto1 > 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  //TB6612的电平控制
  else             digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); //TB6612的电平控制
  analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器
  if (moto2 < 0) digitalWrite(IN3, HIGH),     digitalWrite(IN4, LOW); //TB6612的电平控制
  else        digitalWrite(IN3, LOW),      digitalWrite(IN4, HIGH); //TB6612的电平控制
  analogWrite(PWMB, abs(moto2));//赋值给PWM寄存器
}



/****************PWM限幅**********************/
void Xianfu_Pwm(void)
{
  int Amplitude = 250;  //===PWM满幅是255 限制在250
  if (Flag_Qian == 1)  Motor1 += DIFFERENCE; //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
  if (Flag_Hou == 1)   Motor1 -= DIFFERENCE;
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude)  Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude)  Motor2 = Amplitude;
}

/********************Control函数***********************/
void control() {
  //static int VelocityCount, TurnCount,EncoderCount;
  int temp;
  sei();
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  Angle = KalFilter.angle;


  //serialEvent();
  //Serial.println(Angle);
  /*if (++ledCount >= 128){
    draw();
    ledCount = 0;
    }*/

  Balance_Pwm = balance(KalFilter.angle, KalFilter.Gyro_x);
  if (++VelocityCount >= 8) //速度控制，控制周期40ms
  {
    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);//速度PI控制，控制周期40ms
    VelocityCount = 0;
    //Serial.println(Velocity_Pwm);
  }

  if (++TurnCount >= 4)//转向控制，控制周期20ms
  {
    Turn_Pwm = turn(gz);
    TurnCount = 0;
  }
  /*Motor1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;  //直立速度转向环的叠加
    Motor2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm; //直立速度转向环的叠加*/

  light();

  Motor1 = Balance_Pwm + Velocity_Pwm + Turn_Pwm;
  Motor2 = Balance_Pwm + Velocity_Pwm - Turn_Pwm;



  Xianfu_Pwm();

  Set_Pwm(Motor1, Motor2);


  /*Serial.print("Angle: ");
    // Serial.print(KalFilter.angle);
    Serial.print("Gryo_x: ");
    //Serial.print(KalFilter.Gyro_x);
    Serial.print("\n");*/
}

void loop() {
  serialEvent();
}
