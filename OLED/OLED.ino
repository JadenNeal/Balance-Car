#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define LOGO16_GLCD_HEIGHT 16 //定义显示高度
#define LOGO16_GLCD_WIDTH  16 //定义显示宽度

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup()   {                
  Serial.begin(9600);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  display.clearDisplay(); //清屏

  //英文字符显示，直接用display.println或print显示字符串就行
  display.setTextSize(1.5);             //设置字体大小
  display.setTextColor(WHITE);        //设置字体颜色白色
  display.setCursor(0,0);             //设置字体的起始位置
  display.println("Hello, this is our first led display: Balance Car");   //输出字符并换行
  
//  display.setTextColor(BLACK, WHITE); //设置字体黑色，字体背景白色 
//  display.println("3.1415926");          //输出数字并换行
  
  display.display();                  //显示以上

}

void loop() {

}
