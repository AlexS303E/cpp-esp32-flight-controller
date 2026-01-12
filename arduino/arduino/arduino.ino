#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>
#include <ArduinoJson.h>

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

// Параметры для фильтрации
unsigned long lastTime = 0;
float dt = 0;
unsigned int counter = 0;

// Коэффициенты для комплементарного фильтра
const float alpha = 0.96;
float pitch = 0, roll = 0;

// Размер JSON-документа (рассчитывается через ArduinoJson Assistant)
const size_t JSON_CAPACITY = JSON_OBJECT_SIZE(6) + 
                            JSON_OBJECT_SIZE(3) +  // accel
                            JSON_OBJECT_SIZE(3) +  // gyro
                            JSON_OBJECT_SIZE(4) +  // mag
                            JSON_OBJECT_SIZE(4) +  // angles
                            200; // дополнительный буфер для строк

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Инициализация QMC5883L
  compass.init();
  
  // Калибровка компаса (раскомментировать для первой калибровки)
  // calibrateCompass();
  
  lastTime = micros();
  
  Serial.println("Система инициализирована");
  delay(1000);
}

void calibrateCompass() {
  Serial.println("Калибровка компаса - двигайте датчик по восьмерке");
  compass.calibrate();
  Serial.println("Калибровка завершена!");
}

String getDirection(float heading) {
  String directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int index = (int)((heading + 22.5) / 45) % 8;
  return directions[index];
}

float getYawFromMag(int mx, int my, float pitch, float roll) {
  // Компенсация наклона для магнитометра
  float pitchRad = pitch * PI / 180.0;
  float rollRad = roll * PI / 180.0;
  
  float Xh = mx * cos(pitchRad) + my * sin(rollRad) * sin(pitchRad);
  float Yh = my * cos(rollRad);
  
  float yaw = atan2(Yh, Xh) * 180.0 / PI;
  
  // Приведение к диапазону 0-360 градусов
  if (yaw < 0) yaw += 360;
  if (yaw > 360) yaw -= 360;
  
  return yaw;
}

void loop() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  
  // Чтение данных с датчиков
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  compass.read();
  
  // Расчет углов по акселерометру
  float accelPitch = atan2(-accel.acceleration.x, 
                          sqrt(accel.acceleration.y * accel.acceleration.y + 
                               accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  float accelRoll = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  
  // Комплементарный фильтр
  pitch = alpha * (pitch + gyro.gyro.x * dt * 180.0 / PI) + (1 - alpha) * accelPitch;
  roll = alpha * (roll - gyro.gyro.y * dt * 180.0 / PI) + (1 - alpha) * accelRoll;
  
  // Получение сырых данных магнитометра
  int mx = compass.getX();
  int my = compass.getY();
  int mz = compass.getZ();
  
  // Расчет курса (yaw) с компенсацией наклона
  float yaw = getYawFromMag(mx, my, pitch, roll);
  String direction = getDirection(yaw);
  
  // Проверка на переполнение магнитометра
  bool magOverflow = (abs(mx) > 30000 || abs(my) > 30000 || abs(mz) > 30000);
  
  // Создание JSON документа
  StaticJsonDocument<JSON_CAPACITY> doc;
  
  // Основные поля
  doc["counter"] = counter;
  doc["timestamp"] = millis();
  doc["temperature"] = round(temp.temperature * 10) / 10.0; // Округление до 0.1
  
  // Объект акселерометра
  JsonObject accelObj = doc.createNestedObject("accel");
  accelObj["x"] = round(accel.acceleration.x * 100) / 100.0; // Округление до 0.01
  accelObj["y"] = round(accel.acceleration.y * 100) / 100.0;
  accelObj["z"] = round(accel.acceleration.z * 100) / 100.0;
  
  // Объект гироскопа
  JsonObject gyroObj = doc.createNestedObject("gyro");
  gyroObj["x"] = round(gyro.gyro.x * 10) / 10.0; // Округление до 0.1
  gyroObj["y"] = round(gyro.gyro.y * 10) / 10.0;
  gyroObj["z"] = round(gyro.gyro.z * 10) / 10.0;
  
  // Объект магнитометра
  JsonObject magObj = doc.createNestedObject("mag");
  magObj["x"] = mx;
  magObj["y"] = my;
  magObj["z"] = mz;
  magObj["overflow"] = magOverflow;
  
  // Объект углов
  JsonObject anglesObj = doc.createNestedObject("angles");
  anglesObj["pitch"] = round(pitch * 10) / 10.0; // Округление до 0.1
  anglesObj["roll"] = round(roll * 10) / 10.0;
  anglesObj["yaw"] = round(yaw * 10) / 10.0;
  anglesObj["direction"] = direction;
  
  // Сериализация JSON в строку
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Вывод в Serial
  Serial.printf("I (%lu) FLIGHT_JSON: %s\n", millis(), jsonString.c_str());
  
  counter++;
  delay(100); // 10 Гц
}