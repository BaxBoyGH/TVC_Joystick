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


void listDir(fs::FS &fs, const char *dirname, uint8_t levels = 0);

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

struct Grid {
    double minX, minY, xSpacing, ySpacing;
    std::vector<std::vector<Point>> points;
};

Grid readLookupTable(const char* filename) {
    Grid grid;
    if (!SPIFFS.begin()) {
        Serial.println("Failed to mount file system");
        return grid;
    }

    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Unable to open file");
        return grid;
    }

    double previousX, previousY;
    int rowIndex = 0, colIndex = 0;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        Point point;
        sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &point.x, &point.y, &point.mappedX, &point.mappedY);

        // determine grid properties
        if (grid.points.empty()) {
            grid.minX = point.x;
            grid.minY = point.y;
            grid.points.push_back(std::vector<Point>(1, point));
        } else {
            if (point.y == previousY) {
                grid.points.back().push_back(point);
                if (colIndex != 0) {
                    grid.xSpacing = point.x - previousX;
                }
            } else {
                grid.points.push_back(std::vector<Point>(1, point));
                if (rowIndex != 0) {
                    grid.ySpacing = point.y - previousY;
                }
                rowIndex++;
                colIndex = 0;
            }
        }

        previousX = point.x;
        previousY = point.y;
        colIndex++;
    }

    file.close();
    return grid;
}

Point findMapping(const Grid& grid, double x, double y) {
    double xFraction = (x - grid.minX) / grid.xSpacing;
    double yFraction = (y - grid.minY) / grid.ySpacing;

    //Serial.printf("xFraction: %f, yFraction: %f, xSpacing: %f, ySpacing: %f\n", 
    //              xFraction, yFraction, grid.xSpacing, grid.ySpacing);

    int xIndex = round(xFraction);
    int yIndex = round(yFraction);

    if (yIndex >= 0 && yIndex < grid.points.size() && 
        xIndex >= 0 && xIndex < grid.points[yIndex].size()) {
        return grid.points[yIndex][xIndex];
    } else {
        //Serial.printf("Index out of range. xIndex: %d, yIndex: %d\n", xIndex, yIndex);
        return {0.0, 0.0, 0.0, 0.0}; // Default return value
    }
}

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
void setupSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error occurred while mounting SPIFFS");
    while (1)
      ;
  }
}

Grid grid;

void setup() {
    Serial.begin(9600);
    delay(10);
    setupSPIFFS();

    calibrateJoystick();

    servo_a.attach(13);
    servo_b.attach(12);
    delay(1000);
    servo_a.write(90);
    servo_b.write(90);
    Serial.println("Servos set to  \n");
    
    Serial.println("System started");

    grid = readLookupTable("/output1.csv");

    Serial.printf("Read %d points from lookup table\n", grid.points.size() * grid.points[0].size());

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

  Point mappedPoint = findMapping(grid, calibratedX, calibratedY);
  //Serial.printf("Mapped coordinates: %.6f, %.6f\n", mappedPoint.mappedX, mappedPoint.mappedY);

  servo_a.write(-1*mappedPoint.mappedY+15);   
  servo_b.write(mappedPoint.mappedX-25);

   // print data to Serial Monitor on Arduino IDE
  Serial.print(calibratedX, 2);  // print with 2 decimal places
  Serial.print(", ");
  Serial.print(calibratedY, 2);  // print with 2 decimal places
  Serial.print(",");
  Serial.print(mappedPoint.mappedX, 2);  // print with 2 decimal places
  Serial.print(", ");
  Serial.println(-1*mappedPoint.mappedY, 2);  // print with 2 decimal places
  delay(20);
}
