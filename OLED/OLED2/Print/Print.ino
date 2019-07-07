//显示中英文字符程序

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define LOGO16_GLCD_HEIGHT 16 //定义显示高度
#define LOGO16_GLCD_WIDTH  16 //定义显示宽度

//中文：中  （存储点阵变量，用函数display.drawBitmap（）调用即可）
static const unsigned char PROGMEM str_1[] =
{ 
0x00,0x00,0x0F,0x08,0x08,0x08,0x08,0xFF,0x08,0x08,0x08,0x08,0x0F,0x00,0x00,0x00,
0x00,0x00,0xF0,0x20,0x20,0x20,0x20,0xFF,0x20,0x20,0x20,0x20,0xF0,0x00,0x00,0x00
};

//中文：国
static const unsigned char PROGMEM str_2[] =
{ 
0x00,0x7F,0x40,0x48,0x49,0x49,0x49,0x4F,0x49,0x49,0x49,0x48,0x40,0x7F,0x00,0x00,
0x00,0xFF,0x02,0x12,0x12,0x12,0x12,0xF2,0x12,0x52,0x32,0x12,0x02,0xFF,0x00,0x00
  };

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
  //println换行，print不换行
  display.setTextSize(1);             //设置字体大小
  display.setTextColor(WHITE);        //设置字体颜色白色
  display.setCursor(0,0);             //设置字体的起始位置
  display.println("Hello, world!");   //输出字符并换行
  
  display.setTextColor(BLACK, WHITE); //设置字体黑色，字体背景白色 
  display.println(3.141592);          //输出数字并换行
  
  display.setTextSize(2);             //设置字体大小
  display.setTextColor(WHITE);        //设置字体白色
  display.print("0x");                //输出字符
  display.println(0xDEADBEEF, HEX);   //输出为ASCII编码的十六进制
  //display.display();                  //显示以上
  
  //中文字符显示
  display.drawBitmap(26, 32, str_1, 16, 16, 1); //在坐标X:26  Y:16的位置显示中文字符凌
  display.drawBitmap(42, 32, str_2, 16, 16, 1); //在坐标X:42  Y:16的位置显示中文字符顺
  display.display();                  //把缓存的都显示
}

void loop() {

}
