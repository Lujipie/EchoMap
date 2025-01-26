//ronaldo end of day 1
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_MPU6050.h>
#include <Servo.h> // Include Servo library

// Define pins for the ultrasonic sensor
#define TRIG_PIN 9 // Trigger pin is 9
#define ECHO_PIN 8 // Echo pin is 8

// Create sensor objects
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_MPU6050 mpu;

// Servo motor setup
Servo myServo;
int servo_angle = 0; // Initial angle of the servo motor
const int SERVO_PIN = 10; // Servo connected to pin 10

void setup() {
  Serial.begin(9600);

  // Set up ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Attach the servo motor
  myServo.attach(SERVO_PIN);
  myServo.write(servo_angle); // Start at 0 degrees

  // Initialize the MMA8451
  if (!mma.begin()) {
    Serial.println("MMA8451 not detected. Halting.");
    while (1); // Halt the program if MMA8451 is not detected
  }
  mma.setRange(MMA8451_RANGE_2_G); // Set MMA8451 range to ±2G
  Serial.println("MMA8451 initialized.");

  // Initialize the MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected. Halting.");
    while (1); // Halt the program if MPU6050 is not detected
  }
  Serial.println("MPU6050 initialized.");

  Serial.println("Ultrasonic, MPU6050, MMA8451, and Servo motor initialized.");
}

void loop() {
  // Read data from MMA8451
  mma.read();
  float xacc_mma = (mma.x / 4096.0) * 9.80665 * 1000; // MMA8451 X-axis in mm/s²
  float yacc_mma = (mma.y / 4096.0) * 9.80665 * 1000; // MMA8451 Y-axis in mm/s²
  float zacc_mma = (mma.z / 4096.0) * 9.80665 * 1000; // MMA8451 Z-axis in mm/s²

  // Read distance data from the ultrasonic sensor
  float distance_mm = getUltrasonicDistance(); // Get distance in mm

  // Move the servo by 1 degree
  servo_angle += 1; // Increment the angle
  if (servo_angle > 180) {
    servo_angle = 0; // Reset to 0 after reaching 180 degrees
  }
  myServo.write(servo_angle); // Set the servo angle

  // Print combined data as comma-separated values (without labels)
  Serial.print(xacc_mma, 2); Serial.print(","); // MMA8451 X
  Serial.print(yacc_mma, 2); Serial.print(","); // MMA8451 Y
  Serial.print(zacc_mma, 2); Serial.print(","); // MMA8451 Z
  Serial.print(distance_mm, 2); Serial.print(","); // Ultrasonic distance
  Serial.println(servo_angle); // Servo angle

  delay(100); // Small delay for stability
}

// Function to calculate distance using the ultrasonic sensor in millimeters
float getUltrasonicDistance() {
  // Send a 10-microsecond pulse to the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in millimeters (sound speed = 343 m/s)
  float distance = duration * 0.343 / 2; // Convert to mm and divide by 2 for round trip
  return distance;
}
