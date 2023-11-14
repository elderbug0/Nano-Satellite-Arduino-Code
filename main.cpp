#include "dht.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SD.h>
#include <MPU6050.h>

#define dht_apin 3 // Analog Pin sensor is connected to

dht DHT;
Adafruit_BMP085 bmp;
MPU6050 mpu;

int chipSelect = 10;
File altitudeData;

float referencePressure = 0;
float maxAltitude = 0; // Initialize maximum altitude to 0

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second interval

void setup() {
  Serial.begin(9600);
  
  if (!bmp.begin()) {
    Serial.println("BMP180 sensor not found");
    while (1);
  }
  referencePressure = bmp.readPressure() / 100.0;

  Wire.begin();
  mpu.initialize();

  SD.begin(chipSelect);
  altitudeData = SD.open("altitude.txt", FILE_WRITE);
  if (altitudeData) {
    altitudeData.println("Time,Pressure(hPa),Altitude(m),Temperature(C),AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ");
    altitudeData.close();
  } else {
    Serial.println("Error opening altitude.txt");
  }
  
  delay(500); // Delay to let system boot
  Serial.println("DHT11 Humidity & Temperature Sensor\n\n");
  delay(1000); // Wait before accessing sensor
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  
    // Read DHT11 sensor data
    DHT.read11(dht_apin);
  
    // Read BMP180 sensor data
    float pressure = bmp.readPressure() / 100.0;
    float altitude = 44330.0 * (1.0 - pow(pressure / referencePressure, 0.1903));
    float temperature = bmp.readTemperature();

    // Read MPU6050 sensor data
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;

    mpu.getAcceleration(&accelX, &accelY, &accelZ);
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);

    // Print DHT11 data
    Serial.print("DHT11 - Current humidity: ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");

    // Print BMP180 data
    Serial.print("BMP180 - Current pressure: ");
    Serial.print(pressure);
    Serial.print(" hPa, altitude: ");
    Serial.print(altitude);
    Serial.print(" meters, temperature: ");
    Serial.print(temperature);
    Serial.print(" degrees Celsius, ");

    // Update maximum altitude
    if (altitude > maxAltitude) {
      maxAltitude = altitude;
    }

    // Print MPU6050 data
    Serial.print("MPU6050 - AccelX: ");
    Serial.print(accelX);
    Serial.print(" | AccelY: ");
    Serial.print(accelY);
    Serial.print(" | AccelZ: ");
    Serial.print(accelZ);
    Serial.print(" | GyroX: ");
    Serial.print(gyroX);
    Serial.print(" | GyroY: ");
    Serial.print(gyroY);
    Serial.print(" | GyroZ: ");
    Serial.println(gyroZ);

    // Get the current time in minutes and seconds
    int minutes = currentMillis / 60000;
    int seconds = (currentMillis / 1000) % 60;
    Serial.print("Time: ");
    Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {
      Serial.print("0"); // Add a leading zero if seconds is less than 10
    }
    Serial.print(seconds);

    // Create a data string to save to the SD card
    String dataString = String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds) + "," + String(pressure) + "," + String(altitude) + "," + String(temperature) + "," + String(accelX) + "," + String(accelY) + "," + String(accelZ) + "," + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);

    // Save data to the SD card
    altitudeData = SD.open("altitude.txt", FILE_WRITE);
    if (altitudeData) {
      altitudeData.println(dataString);
      altitudeData.close();
      Serial.println(", data saved to altitude.txt");
    } else {
      Serial.println(", error opening altitude.txt");
    }
  }
}
