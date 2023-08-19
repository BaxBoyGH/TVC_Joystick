#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <SPIFFS.h>
#include "FS.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <servo.h>
#include <ArduinoEigenDense.h>

#include <Wire.h>
#include <VL53L0X.h>

//initalize VL53L0X sensor
VL53L0X tofSensor;

Servo servo_a;
Servo servo_b;

#define VRX_PIN 34 // ESP32 pin GIOP36 (ADC0) connected to VRX pin
#define VRY_PIN 35 // ESP32 pin GIOP39 (ADC0) connected to VRY pin

#define CONTROL_PIN 33 //Pin to activate the servos

int neutralX = 0; // to store the X-axis neutral position
int neutralY = 0; // to store the Y-axis neutral position
int minX = 4095;  // min X position during calibration
int maxX = 0;     // max X position during calibration
int minY = 4095;  // min Y position during calibration
int maxY = 0;     // max Y position during calibration

struct Point {
    double x;
    double y;
    double mappedX;
    double mappedY;
};

void calibrateJoystick() {
  Serial.println("Please keep the joystick still for 5 seconds");

  unsigned long startTime = millis();
  long sumX = 0, sumY = 0;
  int count = 0;

  while (millis() - startTime < 5000) {
    sumX += analogRead(VRX_PIN);
    sumY += analogRead(VRY_PIN);
    count++;
    delay(10);
  }

  neutralX = sumX / count;
  neutralY = sumY / count;

  Serial.println("Please move the joystick around for 5 seconds");

  minX = 4095;  // min X position during calibration
  maxX = 0;     // max X position during calibration
  minY = 4095;  // min Y position during calibration
  maxY = 0;     // max Y position during calibration

  startTime = millis();
  while (millis() - startTime < 5000) {
    int valueX = analogRead(VRX_PIN);
    int valueY = analogRead(VRY_PIN);
    minX = min(minX, valueX);
    maxX = max(maxX, valueX);
    minY = min(minY, valueY);
    maxY = max(maxY, valueY);
    delay(10);
  }
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Angles calculated using parameters and alpha beta func

// Adjusted parameters for alpha function
Eigen::Vector3f a_0(-23, 0, 45);
Eigen::Vector3f g_a(10, -7.7, 75);
float l_a = 12.5;
float c_a = 45.32;
bool sign_a = true;

// Adjusted parameters for beta function
Eigen::Vector3f b_0(-5, -23, 45);
Eigen::Vector3f g_b(-12.7, 10, 110);
float l_b = 12.5;
float c_b = 79.9;
bool sign_b = true;

Eigen::Matrix3f calculate_R(float psi, float phi) {
    psi = psi * M_PI / 180.0; // convert to radians
    phi = phi * M_PI / 180.0; // convert to radians

    Eigen::Matrix3f R;
    R << cos(psi), 0, sin(psi),
         sin(phi) * sin(psi), cos(phi), -sin(phi) * cos(psi),
         -sin(psi) * cos(phi), sin(phi), cos(psi) * cos(phi);
    return R;
}

float alpha(float psi, float phi, const Eigen::Matrix3f& R) {
    Eigen::Vector3f x = (R * a_0 - g_a) / l_a;
    float z = (x.norm() * x.norm() + 1 - (c_a / l_a) * (c_a / l_a)) / 2;
    int pm = sign_a ? 1 : -1;
    float denominator = z + x[2];
    if (denominator == 0) { // avoid division by zero
        std::cout << "Denominator in alpha calculation is zero!" << std::endl;
        return std::nan("");
    }
    float sqrt_val = x[0] * x[0] + x[2] * x[2] - z * z;
    if (sqrt_val < 0) { // check if value inside sqrt is negative
        std::cout << "Negative value inside sqrt in alpha calculation!" << std::endl;
        return std::nan("");
    }
    return -2 * atan((x[0] + pm * sqrt(sqrt_val)) / denominator) * 180.0 / M_PI; // convert result from radians to degrees
}

float beta(float psi, float phi, const Eigen::Matrix3f& R) {
    Eigen::Vector3f x = (R * b_0 - g_b) / l_b;
    float z = (x.norm() * x.norm() + 1 - (c_b / l_b) * (c_b / l_b)) / 2;
    int pm = sign_b ? 1 : -1;
    float denominator = z + x[2];
    if (denominator == 0) { // avoid division by zero
        std::cout << "Denominator in beta calculation is zero!" << std::endl;
        return std::nan("");
    }
    float sqrt_val = x[1] * x[1] + x[2] * x[2] - z * z;
    if (sqrt_val < 0) { // check if value inside sqrt is negative
        std::cout << "Negative value inside sqrt in beta calculation!" << std::endl;
        return std::nan("");
    }
    return 2 * atan((x[1] + pm * sqrt(sqrt_val)) / denominator) * 180.0 / M_PI; // convert result from radians to degrees
}


void setup() {
    Serial.begin(9600);
    while (!Serial); // Warten Sie, bis die serielle Verbindung hergestellt ist

    Wire.begin(); 
    delay(10);
    pinMode(CONTROL_PIN, INPUT); // Setzen Sie den Pin als Eingang
    delay(1500);
    
    if (!tofSensor.init()) {
      Serial.println("Failed to initialize the TOF200C sensor!");
      while (1); // Endlos-Schleife, um das Programm anzuhalten
    } else {
      Serial.println("TOF200C sensor initialized successfully!");
    }
    tofSensor.setTimeout(500); // Setze Timeout auf 500ms
    tofSensor.startContinuous(50); // start continuous mode with 20ms delay between measurements


    if (false) { // Überprüfen Sie, ob der Pin HIGH ist
      servo_a.attach(12);
      servo_b.attach(13);
      delay(1000);
      servo_a.write(0); // Setzen Sie die Servos auf 0
      servo_b.write(0);
      Serial.println("System stopped by control pin");
      delay(10000);
      setup(); // Starten Sie das Setup erneut
    }

    calibrateJoystick();

    servo_a.attach(12);
    servo_b.attach(13);
    delay(1000);
    Serial.println("System started");

    //Serial print Alpha and Beta angles for testing
    Eigen::Matrix3f R = calculate_R(0, 0);
    Serial.println(alpha(0, 0, R));
    Serial.println(beta(0, 0, R));

    servo_a.write(alpha(0, 0, R));
    servo_b.write(180 + beta(0, 0, R));
} 

void loop() {
  
  //offset controlls
  float offset_a = -12;
  float offset_b = -12;

  // read X and Y analog values
  int valueX = analogRead(VRX_PIN);
  int valueY = analogRead(VRY_PIN);

  // Messen Sie die Entfernung:
  uint16_t distance = tofSensor.readRangeContinuousMillimeters();
  if (tofSensor.timeoutOccurred()) {
    Serial.println("Sensor timeout!");
  }


  // calibrate the values
  float calibratedX, calibratedY;

  if (valueX > neutralX) {
    calibratedX = mapFloat(valueX, neutralX, maxX, 0, 7.5);
  } else {
    calibratedX = mapFloat(valueX, minX, neutralX, -7.5, 0);
  }

  if (valueY > neutralY) {
    calibratedY = mapFloat(valueY, neutralY, maxY, 0, 7.5);
  } else {
    calibratedY = mapFloat(valueY, minY, neutralY, -7.5, 0);
  }

  // calculate rotation matrix
  Eigen::Matrix3f R = calculate_R(calibratedX, calibratedY);
  float alpha_value = alpha(calibratedX, calibratedY, R);
  float beta_value = beta(calibratedX, calibratedY, R);
  servo_a.write(alpha_value - offset_a);
  servo_b.write(135 + beta_value - offset_b);

  // print values
  Serial.print("Joystick value X:");
  Serial.print(calibratedX, 2);  // print with 2 decimal places
  Serial.print(", Y: ");
  Serial.print(calibratedY, 2);  // print with 2 decimal places
  Serial.print(", Alpha:");
  Serial.print(alpha_value, 2);  // print with 2 decimal places
  Serial.print(", Beta:");
  Serial.print(-1 * beta_value, 2);  // print with 2 decimal places
  Serial.print(", Distance: ");
  Serial.print(tofSensor.readRangeContinuousMillimeters());
  Serial.println("mm");

  delay(10);
}
