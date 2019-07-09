// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69 地址
MPU6050 accelgyro;  //实例化mpu6050

int16_t ax, ay, az;   // 三维加速度
int16_t gx, gy, gz;   // 三维角速度
float gyro_y;         
float gy0;            // 陀螺仪零点
float rgyro=0.35;    

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);   //监视的波特率

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();  // 陀螺仪初始化

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_y=-(gy-10.58)*rgyro;
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);Serial.print("\t");
    Serial.print(gyro_y);Serial.print("\t");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
