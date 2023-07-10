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

Servo servo_a;
Servo servo_b;

#define VRX_PIN 34 // ESP32 pin GIOP36 (ADC0) connected to VRX pin
#define VRY_PIN 35 // ESP32 pin GIOP39 (ADC0) connected to VRY pin

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


float alpha(float x, float y){

  return 35.001549 + 0.3863021839 * y + -4.4780694041 * x + 0.0649899492* y*y + 0.0111596275* x * y + -0.0397865873* x*x + -0.0002537119* y*y*y + 0.0032481368* x * y*y + 0.0018342949* x*x* y + -0.0073124944* x*x*x
              + -0.0000181577* y*y*y*y + -0.0000579877* x * y*y*y + 0.0003515851* x*x* y*y + 0.0001120064* x*x*x* y + -0.0003305037* x*x*x*x;
}

float beta(float x, float y){
  return -44.99245505923736 + -3.8951056064 * y + 0.9343299164 * x + 0.0261465955* y*y + -0.0372285176* x * y + -0.0563875585* x*x
        + -0.0055439795* y*y*y + 0.0046142001* x * y*y + 0.0014686303* x*x* y + -0.0006782020* x*x*x
        + 0.0003026989* y*y*y*y + -0.0003695765* x * y*y*y + -0.0001385496* x*x* y*y + 0.0001411509* x*x*x* y + -0.0000004344* x*x*x*x;
}


void setup() {
    Serial.begin(9600);
    delay(10);
    //setupSPIFFS();

    calibrateJoystick();

    servo_a.attach(13);
    servo_b.attach(12);
    delay(1000);
    servo_a.write(35);
    servo_b.write(180-45);
    Serial.println("Servos set to  \n");
    
    Serial.println("System started");

    //grid = readLookupTable("/output1.csv");

    //Serial.printf("Read %d points from lookup table\n", grid.points.size() * grid.points[0].size());


} 

void loop() {
  
  // read X and Y analog values
  int valueX = analogRead(VRX_PIN);
  int valueY = analogRead(VRY_PIN);

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

  servo_a.write(alpha(calibratedX, calibratedY));   
  servo_b.write(180+beta(calibratedX, calibratedY));

   // print data to Serial Monitor on Arduino IDE
  Serial.print(calibratedX, 2);  // print with 2 decimal places
  Serial.print(", ");
  Serial.print(calibratedY, 2);  // print with 2 decimal places
  Serial.print(",");
  Serial.print(alpha(calibratedX, calibratedY), 2);  // print with 2 decimal places
  Serial.print(", ");
  Serial.println(-1*beta(calibratedX, calibratedY), 2);  // print with 2 decimal places
  delay(50);
}
