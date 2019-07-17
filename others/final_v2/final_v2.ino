/*
   实现功能：
   1. 读取陀螺仪数据用
   2. 陀螺仪控制电机
*/

#include <PinChangeInt.h>    // 外部中断
#include <MsTimer2.h>        // 定时中断
#include <KalmanFilter.h>    // 卡尔曼滤波
#include "I2Cdev.h"          // 陀螺仪用
#include "MPU6050_6Axis_MotionApps20.h"  //MPU6050库文件
#include "Wire.h"
#include <EEPROM.h>
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter

int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据

const int d_time = 50; //设定单位时间（ms）
int flagA = 0;//标志位设定
int flagB = 0;//标志位设定

#define STBY 11   // 使能端
#define AIN1 12   // AIN1
#define AIN2 13   // AIN2
#define AM1 3     // 左电机A相
#define BM1 8     // 左电机B相
#define PWMA 10   // PWMA
#define valA 0    // 左轮A相记录的脉冲数

#define BIN1 7    // BIN1
#define BIN2 6    // BIN2
#define AM2 4     // 右电机A相
#define BM2 5     // 右电机B相
#define PWMB 9    // PWMB
#define valB 0    // 右轮A相记录的脉冲数

double n;         //存储转速的变量
double m;         //存储转速的变量

#define ZHONGZHI 0     // 小车的机械中值
#define DIFFERENCE 2   // 未知

int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
int Motor1, Motor2;      //上面3个电机量叠加之后的PWM

float Battery_Voltage;   //电池电压 单位是V

volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;      //左右轮速度
int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量
int Angle, Show_Data, PID_Send;             //用于显示的角度和临时变量

float Balance_Kp = 25, Balance_Kd = 0.5;    // 直立Kp和Kd(0.5)
float Velocity_Kp = 3, Velocity_Ki = 0.01;  // 速度Kp和Ki 3, 0.01

//***************下面是卡尔曼滤波相关变量***************//
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
int addr = 0;
//*****************卡尔曼变量定义结束******************//

/**************************************************************************
  函数功能：检测小车是否被拿起
  入口参数：Z轴加速度 平衡倾角 左轮编码器 右轮编码器
  返回  值：0：无事件 1：小车被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int AM1eft, int AMright) {
  static unsigned int flag, count0, count1, count2;
  if (flag == 0) // 第一步
  {
    if (abs(AM1eft) + abs(AMright) < 15)   count0++;  // 条件1，小车接近静止
    else       count0 = 0;
    if (count0 > 10)      flag = 1, count0 = 0;
  }
  else if (flag == 1) // 进入第二步
  {
    if (++count1 > 400)       count1 = 0, flag = 0;                         // 超时不再等待2s
    if (Acceleration > 27000 && (Angle > (-14 + ZHONGZHI)) && (Angle < (14 + ZHONGZHI)))  flag = 2; //条件2，小车是在0度附近被拿起
  }
  else if (flag == 2)  // 第三步
  {
    if (++count2 > 200)       count2 = 0, flag = 0;       //超时不再等待1000ms
    if (abs(AM1eft + AMright) > 300)           //条件3，小车的轮胎因为正反馈达到最大的转速
    {
      flag = 0;  return 1;
    }
  }
  return 0;
}

/**************************************************************************
  函数功能：检测小车是否被放下 作者：平衡小车之家
  入口参数： 平衡倾角 左轮编码器 右轮编码器
  返回  值：0：无事件 1：小车放置并启动
**************************************************************************/
int Put_Down(float Angle, int AM1eft, int AMright) {
  static u16 flag, count;
  if (flag == 0)
  {
    if (Angle > (-10 + ZHONGZHI) && Angle < (10 + ZHONGZHI) && AM1eft == 0 && AMright == 0)      flag = 1; //条件1，小车是在0度附近的
  }
  else if (flag == 1)
  {
    if (++count > 100)       count = 0, flag = 0;  //超时不再等待 500ms
    //    if (AM1eft > 12 && AMright > 12 && AM1eft < 80 && AMright < 80) //条件2，小车的轮胎在未上电的时候被人为转动
    //    {
    //      flag = 0;
    //      flag = 0;
    //      return 1;    //检测到小车被放下
    //    }
  }
  return 0;
}

/**************************************************************************
  函数功能：异常关闭电机 作者：平衡小车之家
  入口参数：倾角和电池电压
  返回  值：1：异常  0：正常
**************************************************************************/
int Turn_Off(float angle, float voltage)
{
  int temp;
  if (angle < -30 || angle > 30 || voltage < 11.1) //电池电压低于11.1V关闭电机 // 倾角大于30度关闭电机
  {
    temp = 1;  // 异常标志位
    set_STBY(LOW);
  }
  else    temp = 0;   //不存在异常，返回0
  return temp;
}


/**************************************************************************
  函数功能：直立PD控制
  入口参数：角度、角速度
  返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
  float Bias;   // 偏置
  int balance;  // 直立
  Bias = Angle - 0 ;   // 求出平衡的角度中值和机械相关，与0比较
  balance = Balance_Kp * Bias + Gyro * Balance_Kd; //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数
  return balance;
}

/**************************************************************************
  函数功能：速度PI控制 作者：平衡小车之家
  入口参数：左轮编码器、右轮编码器
  返回  值：速度控制PWM
**************************************************************************/
int velocity(int AM1eft, int AMright)
{
  static float Velocity, AM1east, Encoder, Movement;
  static float Encoder_Integral, Target_Velocity;
  float kp = 2, ki = kp / 200;    //PI参数
  if       ( Flag_Qian == 1)Movement = 600;
  else   if ( Flag_Hou == 1)Movement = -600;
  else    //这里是停止的时候反转，让小车尽快停下来
  {
    Movement = 0;
    if (Encoder_Integral > 300)   Encoder_Integral -= 200;
    if (Encoder_Integral < -300)  Encoder_Integral += 200;
  }
  //=============速度PI控制器=======================//
  AM1east = (AM1eft + AMright) - 0;               // 获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
  Encoder *= 0.7;                                                   // 一阶低通滤波器
  Encoder += AM1east * 0.3;                                         // 一阶低通滤波器
  Encoder_Integral += Encoder;                                      // 积分出位移 积分时间：40ms
  Encoder_Integral = Encoder_Integral + Movement;                   // 接收遥控器数据，控制前进后退
  if (Encoder_Integral > 21000)    Encoder_Integral = 21000;        // 积分限幅
  if (Encoder_Integral < -21000) Encoder_Integral = -21000;         // 积分限幅
  Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki;                  // 速度PI控制
  if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 )    Encoder_Integral = 0;      // 小车停止的时候积分清零

  return Velocity;
}
/**************************************************************************
  函数功能：转向控制 作者：平衡小车之家
  入口参数：Z轴陀螺仪
  返回  值：转向控制PWM
**************************************************************************/
int turn(float gyro)  // 转向控制
{
  static float Turn_Target, Turn, Turn_Convert = 3;
  float Turn_Amplitude = 60, Kp = 2, Kd = 0.001;  // PD参数，转弯60
  if (Flag_Left == 1)             Turn_Target += Turn_Convert;  // 根据遥控指令改变转向偏差
  else if (Flag_Right == 1)       Turn_Target -= Turn_Convert;  // 根据遥控指令改变转向偏差
  else Turn_Target = 0;
  if (Turn_Target > Turn_Amplitude)  Turn_Target = Turn_Amplitude; // 转向速度限幅
  if (Turn_Target < -Turn_Amplitude) Turn_Target = -Turn_Amplitude;
  Turn = -Turn_Target * Kp + gyro * Kd;         // 结合Z轴陀螺仪进行PD控制

  return Turn;
}
/**************************************************************************
  函数功能：赋值给PWM寄存器 作者：平衡小车之家
  入口参数：左轮PWM、右轮PWM
  返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
  if (moto1 > 0)     digitalWrite(AIN1, LOW), digitalWrite(AIN2, HIGH);  //TB6612的电平控制
  else               digitalWrite(AIN1, HIGH),  digitalWrite(AIN2, LOW); //TB6612的电平控制
  analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器
  if (moto2 < 0) digitalWrite(BIN1, LOW),     digitalWrite(BIN2, HIGH); //TB6612的电平控制
  else        digitalWrite(BIN1, HIGH),      digitalWrite(BIN2, LOW); //TB6612的电平控制
  analogWrite(PWMB, abs(moto2));//赋值给PWM寄存器
}

/**************************************************************************
  函数功能：限制PWM赋值（死区限制）  作者：平衡小车之家
  入口参数：无
  返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
  int Amplitude = 100;  // PWM满幅是255 限制在100
  if (Flag_Qian == 1)  Motor2 -= DIFFERENCE; // DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
  if (Flag_Hou == 1)   Motor2 -= DIFFERENCE - 2;
  if (Motor1 < -Amplitude) Motor1 = -Amplitude;
  if (Motor1 > Amplitude)  Motor1 = Amplitude;
  if (Motor2 < -Amplitude) Motor2 = -Amplitude;
  if (Motor2 > Amplitude)  Motor2 = Amplitude;
}

/**************************************************************************
  函数功能：5ms控制函数 核心代码 作者：平衡小车之家
  入口参数：无
  返回  值：无
**************************************************************************/
void control()
{
  //  static int Velocity_Count, Turn_Count, Voltage_Count;  // 速度、转向和电压计数器
  //  int Voltage_All;  // 多次电压之和

  sei();  // 全局中断开启
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); // 通过卡尔曼滤波获取角度
  Angle = KalFilter.angle;   // Angle是一个用于显示的整形变量，这里隐藏了自动类型转换，左为int，右为float
  Balance_Pwm = balance(KalFilter.angle, KalFilter.Gyro_x);  // 直立PD控制 控制周期5ms

  Serial.print("角度：");
  Serial.println(KalFilter.angle); 
  delay(100);  // 延迟0.1秒
  //  Serial.print("加速度：");
  //  Serial.println(KalFilter.Gyro_x);
  //  Serial.println();  // 换行
  //  delay(500);       // 延迟0.5秒

  //  if (++Velocity_Count >= 8) //速度控制，控制周期40ms
  //  {
  //    Velocity_Left = Velocity_L;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  //    Velocity_Right = Velocity_R;    Velocity_R = 0; //读取右轮编码器数据，并清零
  //    Velocity_Pwm = velocity(Velocity_Left, Velocity_Right);//速度PI控制，控制周期40ms
  //    Velocity_Count = 0;  // 大于8就重新计数
  //  }
  //  if (++Turn_Count >= 4) //转向控制，控制周期20ms
  //  {
  //    Turn_Pwm = turn(gz);
  //    Turn_Count = 0;      // 大于4就重新计数
  //  }
  Motor1 = Balance_Pwm - Velocity_Pwm ;  //直立、速度、转向环的叠加
  Motor2 = Balance_Pwm - Velocity_Pwm ;  //直立、速度、转向环的叠加
  //  Serial.print("Motor1：");
  //  Serial.println(Motor1);
  //  Serial.print("Motor2：");
  //  Serial.println(Motor2);

  Xianfu_Pwm();  // 限幅

  //  float Temp = analogRead(0);  //采集一下电池电压
  //  Voltage_Count++;       //平均值计数器
  //  Voltage_All += Temp;   //多次采样累积
  //  采集200次电压就计算一次均值，然后去做一次处理
  //  if (Voltage_Count == 200) Battery_Voltage = Voltage_All * 0.05371 / 200, Voltage_All = 0, Voltage_Count = 0;

  if (Turn_Off(Angle, 11.8) == 0)
    Set_Pwm(Motor1, Motor2);  //如果不存在异常，赋值给PWM寄存器控制电机
}

/*
  设置STBY的状态
  HIGH为使能
  LOW为待机
*/
void set_STBY(boolean state) {
  digitalWrite(STBY, state);
}

/**************************************************************************
  函数功能：初始化 作者：平衡小车之家
  入口参数：无
  返回  值：无
**************************************************************************/
void setup() {
  pinMode(STBY, OUTPUT);    // 使能端

  pinMode(PWMA, OUTPUT);    // 电机PWMA
  pinMode(AIN1, OUTPUT);    // AIN1，控制电机1的方向，10为正转，01为反转
  pinMode(AIN2, OUTPUT);    // AIN2
  pinMode(AM1, INPUT);      // 3
  pinMode(BM1, INPUT);      // 8

  pinMode(PWMB, OUTPUT);    // 电机PWMB
  pinMode(BIN1, OUTPUT);    // BIN1，控制电机2的方向，10为正转，01为反转
  pinMode(BIN2, OUTPUT);    // BIN2
  pinMode(AM2, INPUT);      // 4
  pinMode(BM2, INPUT);      // 5

  Wire.begin();             //加入 IIC 总线
  Serial.begin(9600);       //开启串口，设置波特率为 9600
  Mpu6050.initialize();     //初始化MPU6050

  // supply your own gyro offsets here, scaled for min sensitivity
  Mpu6050.setXGyroOffset(81);  // 220
  Mpu6050.setYGyroOffset(-413);   // 76
  Mpu6050.setZGyroOffset(36);  // -85
  Mpu6050.setZAccelOffset(1788); // 1688 factory default for my test chip

  //  delay(1000);

  MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能


}

/**************************************************************************
  函数功能：主循环程序体
  入口参数：无
  返回  值：无
**************************************************************************/
void loop() {
  set_STBY(LOW);  // 初始设置STBY为LOW
  //  control();
}
