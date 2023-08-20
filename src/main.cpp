#include <Arduino.h>
#include <Servo.h>
#include <ArduinoEigenDense.h>


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

// Global offset variables
float offset_alpha = -25; // Adjust this value based on your needs
float offset_beta = -35;  // Adjust this value based on your needs


void setup() {
    Serial.begin(9600);
    delay(1000);  // Give a delay for the serial connection to establish
    Serial.println("Starting system setup...");
    
    servo_a.attach(12);
    servo_b.attach(13);
    delay(1000);
    Serial.println("Servos attached.");

    // Initial angle setting
    Eigen::Matrix3f R = calculate_R(0, 0);
    Serial.println("Setting initial angles...");
    servo_a.write(alpha(0, 0, R) + offset_alpha);
    servo_b.write(160 + beta(0, 0, R) + offset_beta);
    Serial.println("Initial angles set. Waiting for Serial..");

}

void loop() {
    // Nothing to do here
}
