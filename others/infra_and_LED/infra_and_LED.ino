/*
 * 数字接法
 * 红外对管OUTPUT接5号引脚
 * 接收到信号infra为0，否则为1
*/
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
 
// 控制 WS2812 灯条的引脚编号
#define PIN        4
#define infraOut A0 // 输出引脚号
int infra = 0;      // 红外对管信号
 
//定义控制的 LED 数量
#define NUMPIXELS 7  // 共7个灯泡
 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
//相邻 LED 之间的延迟，单位毫秒
#define DELAYVAL 500
 
void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
//  Serial.begin(9600);  // 调试用
  
//  pinMode(infraOut, INPUT);  // 红外传感器，模拟接口默认为输入不需要此语句
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}
 
void loop() {
  /*
  无延迟，一次性全部点亮
  */
  pixels.clear(); // Set all pixel colors to 'off'
 
  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.

  infra = analogRead(infraOut);  // 尝试digitalRead(infraOut)
//  Serial.println(infra);  //  调试用
  
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
 
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    if (infra == 32){
    pixels.setPixelColor(i, pixels.Color(255, 0, 0));  // 红色
    }
    else
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));  // 绿色
 
    pixels.show();   // Send the updated pixel colors to the hardware.
 
//    delay(DELAYVAL); // 延迟DELAYVAL毫秒
  }
}
