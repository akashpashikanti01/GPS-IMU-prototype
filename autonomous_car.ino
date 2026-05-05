# Arduino Code for Autonomous Car

## Required Libraries
#include <Wire.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

## Global Variables
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055();
Servo motorServo;

double targetLatitude = 0.0;
double targetLongitude = 0.0;
double currentLatitude;
double currentLongitude;
double heading;

// PID controller variables
double kp = 1.0;
double ki = 0.1;
double kd = 0.05;
double previousError = 0;
double integral = 0;

// Function to calculate Haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 = lat1 * PI / 180.0;
    lon1 = lon1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    lon2 = lon2 * PI / 180.0;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double R = 6371e3; // Mean radius of Earth in meters
    return R * c; // Distance in meters
}

// Function to normalize angle
double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// Function to perform PID control
double PIDController(double setpoint, double measuredValue) {
    double error = setpoint - measuredValue;
    integral += error;
    double derivative = error - previousError;
    previousError = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}

// Function to read GPS data
void readGPS() {
    while (Serial.available() > 0) {
        gps.encode(Serial.read());
        if (gps.location.isUpdated()) {
            currentLatitude = gps.location.lat();
            currentLongitude = gps.location.lng();
        }
    }
}

// Function to control motors
void moveMotors(double speed) {
    // Implement motor control logic based on speed
    motorServo.write(speed);
}

// Setup function
void setup() {
    Serial.begin(9600);
    Serial1.begin(9600); // For GPS
    bno.begin();
    motorServo.attach(9); // Attach servo on pin 9
}

// Main loop
void loop() {
    readGPS();
    heading = bno.getVector(Adafruit_BNO055::VECTOR_EULER)[0]; // Get heading
    double distance = haversine(currentLatitude, currentLongitude, targetLatitude, targetLongitude);
    double targetHeading = atan2(targetLongitude - currentLongitude, targetLatitude - currentLatitude) * 180 / PI;
    targetHeading = normalizeAngle(targetHeading);

    double pidOutput = PIDController(targetHeading, heading);
    moveMotors(pidOutput);
    delay(100); // Delay for stability
}
