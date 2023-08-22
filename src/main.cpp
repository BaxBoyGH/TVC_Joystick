#include <Arduino.h>
#include <Servo.h>
#include <ArduinoEigenDense.h>
#include <wire.h>

Servo servo_a;
Servo servo_b;

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
        Serial.println("Denominator in alpha calculation is zero!");
        return NULL;
    }
    float sqrt_val = x[0] * x[0] + x[2] * x[2] - z * z;
    if (sqrt_val < 0) { // check if value inside sqrt is negative
        Serial.println("Negative value inside sqrt in alpha calculation!");
        return NULL;
    }
    return -2 * atan((x[0] + pm * sqrt(sqrt_val)) / denominator) * 180.0 / M_PI; // convert result from radians to degrees
}

float beta(float psi, float phi, const Eigen::Matrix3f& R) {
    Eigen::Vector3f x = (R * b_0 - g_b) / l_b;
    float z = (x.norm() * x.norm() + 1 - (c_b / l_b) * (c_b / l_b)) / 2;
    int pm = sign_b ? 1 : -1;
    float denominator = z + x[2];
    if (denominator == 0) { // avoid division by zero
        Serial.println("Denominator in beta calculation is zero!");
        return NULL;
    }
    float sqrt_val = x[1] * x[1] + x[2] * x[2] - z * z;
    if (sqrt_val < 0) { // check if value inside sqrt is negative
        Serial.println("Negative value inside sqrt in beta calculation!");
        return NULL;
    }
    return 2 * atan((x[1] + pm * sqrt(sqrt_val)) / denominator) * 180.0 / M_PI; // convert result from radians to degrees
}

//kalibration of Servos
const int CONTROL_PIN = 5; // G5 pin on the ESP32
float stepSize = 0.25;      // Global variable for step size
int delayTime = 250;      // Global variable for delay time

void calibrateServos(Servo& servoA, Servo& servoB) {
    
    while (digitalRead(CONTROL_PIN) == LOW) { // Wait until G5 is set back to high
        Serial.println("Please set G5 to high again! Waiting.");
        delay(250);
    }
    delay(1000); // Wait for 1 second to make sure the pin is not shorted to ground
    Serial.println("Starting calibration for Sero Alpha...");
    // Calibration for servo A (alpha)
    servoA.write(0);
    delay(delayTime);
    for(float angle = 0; angle <= 180; angle += stepSize) {
        Serial.print("Alpha-Angle: ");
        Serial.println(angle);
        if (digitalRead(CONTROL_PIN) == LOW) { // Check if the pin is shorted to ground
            Serial.println("Servo A collision detected!");
            break;
        }
        //Map the Angles 0-160 to 900-2100us for the Servo.writeMicroseconds function
        angle = map(angle, 0, 160, 900, 2100);
        servo_a.writeMicroseconds(angle);
        delay(delayTime);
    }
    float alphaCollisionAngle = servoA.read(); // Store the angle at which collision was detected
    Serial.println("Waiting for G5 to be set back to high...");
    while (digitalRead(CONTROL_PIN) == LOW) { // Wait until G5 is set back to high
        delay(100);
    }
    delay(1000); // Wait for 1 second to make sure the pin is not shorted to ground

    // Calibration for servo B (beta)
    Serial.println("Starting calibration for Sero Beta...");
    servoB.write(160);
    delay(delayTime);
    for(float angle = 160; angle >= 0; angle -= stepSize) {
        Serial.print("Beta-Angle: ");
        Serial.println(angle);
        if (digitalRead(CONTROL_PIN) == LOW) { // Check if the pin is shorted to ground
            Serial.println("Servo B collision detected!");
            break;
        }
        //Map the Angles 0-160 to 900-2100us for the Servo.writeMicroseconds function
        angle = map(angle, 0, 160, 900, 2100);
        servo_b.writeMicroseconds(angle);
        delay(delayTime);
    }
    float betaCollisionAngle = servoB.read(); // Store the angle at which collision was detected

    // Compute the offsets based on known collision angles
    float alphaOffset = alphaCollisionAngle - 50;
    float betaOffset = betaCollisionAngle - 125;

    // Print the angles at which collisions were detected and the computed offsets
    Serial.print("Alpha collision angle: ");
    Serial.println(alphaCollisionAngle);
    Serial.print("Alpha offset: ");
    Serial.println(alphaOffset);
    Serial.print("Beta collision angle: ");
    Serial.println(betaCollisionAngle);
    Serial.print("Beta offset: ");
    Serial.println(betaOffset);
}


// Global offset variables
float offset_alpha = 22; // Adjust this value based on your needs
float offset_beta = -18;  // Adjust this value based on your needs

void setup() {
    Serial.begin(115200);
    delay(1000);  // Give a delay for the serial connection to establish
    Serial.println("Starting system setup...");
    
    servo_a.attach(12);
    servo_b.attach(13);
    pinMode(CONTROL_PIN, INPUT_PULLUP); // Set CONTROL_PIN as input with pullup resistor
    delay(1000);
    Serial.println("Servos attached.");
    //Servo to zero Postion 
    servo_a.write(0);
    servo_b.write(160);
    delay(2000);

    // Initial angle setting
    Eigen::Matrix3f R = calculate_R(0, 0);
    Serial.println("Setting initial angles...");
    servo_a.write(alpha(0, 0, R) + offset_alpha);
    servo_b.write(160 + beta(0, 0, R) + offset_beta);
    //print alpha(0,0T and Beta(0,0,R values and the offset values and the final Value
    Serial.print("Alpha(0,0,R): ");
    Serial.println(alpha(0, 0, R));
    Serial.print("Beta(0,0,R): ");
    Serial.println(beta(0, 0, R));
    Serial.print("Alpha offset: ");
    Serial.println(offset_alpha);
    Serial.print("Beta offset: ");
    Serial.println(offset_beta);
    Serial.print("Alpha(0,0,R) + offset_alpha: ");
    Serial.println(alpha(0, 0, R) + offset_alpha);
    Serial.print("Beta(0,0,R) + offset_beta: ");
    Serial.println(160 + beta(0, 0, R) + offset_beta);

    Serial.println("Waiting, set G5 to Low for calibration mode (short to ground)");

    while(digitalRead(CONTROL_PIN) == HIGH) { // Wait until G5 is set back to high
        delay(100);
    }

    Serial.println("Calibration Mode activated!");
    calibrateServos(servo_a, servo_b);
}

void loop() {
    // Nothing to do here
}
