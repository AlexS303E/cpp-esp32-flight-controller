#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  
  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("Не удалось найти MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 найден!");
  
  // Настройка MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Инициализация QMC5883L
  compass.init();
  Serial.println("QMC5883L инициализирован!");
}

void loop() {
  // Чтение данных с MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Вывод данных акселерометра
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(" Y: "); Serial.print(a.acceleration.y);
  Serial.print(" Z: "); Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  
  // Вывод данных гироскопа
  Serial.print("Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(" Y: "); Serial.print(g.gyro.y);
  Serial.print(" Z: "); Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  // Чтение данных с QMC5883L
  compass.read();
  
  // Вывод данных магнитометра
  Serial.print("Mag X: "); Serial.print(compass.getX());
  Serial.print(" Y: "); Serial.print(compass.getY());
  Serial.print(" Z: "); Serial.print(compass.getZ());
  Serial.println(" uT");
  
  // Задержка между измерениями
  delay(500);
}
