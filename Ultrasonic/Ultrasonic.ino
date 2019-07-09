#define TrigPin A2   // TriPin 的引脚接到A2
#define EchoPin A3   // EchoPin 的引脚接到A3

float Value_cm;   // 距离，cm

void setup()
{
	Serial.begin(9600);
	pinMode(TrigPin, OUTPUT);
	pinMode(EchoPin, INPUT);  // 模拟口默认是读取，所以这句可以不写。
}
void loop()
{
	digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin
	delayMicroseconds(2);
	digitalWrite(TrigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(TrigPin, LOW);
	Value_cm = float( pulseIn(EchoPin, HIGH) * 17 )/1000; //将回波时间换算成cm
	//读取一个引脚的脉冲（HIGH或LOW）。例如，如果value是HIGH，pulseIn()会等待引脚变为HIGH，开始计时，再等待引脚变为LOW并停止计时。
	//返回脉冲的长度，单位微秒。如果在指定的时间内无脉冲函数返回。
	//此函数的计时功能由经验决定，长时间的脉冲计时可能会出错。计时范围从10微秒至3分钟。（1秒=1000毫秒=1000000微秒）
	//接收到的高电平的时间（us）* 340m/s / 2 = 接收到高电平的时间（us） * 17000 cm / 1000000 us = 接收到高电平的时间 * 17 / 1000  (cm)

	Serial.print(Value_cm);
	Serial.println("cm");
	delay(1000);
}
