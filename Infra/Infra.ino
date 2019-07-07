#define infraOut 5  // 输出引脚号为5
int infra = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(infraOut, INPUT);  // 红外传感器
}

void loop() {
  // put your main code here, to run repeatedly:
  infra = digitalRead(infraOut);
  if (infra == 0)
  {
    Serial.println("Accept infra data!");
  }

}
