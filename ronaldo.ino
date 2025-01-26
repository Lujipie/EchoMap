#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_MPU6050.h>

// Define pins for the ultrasonic sensor
#define TRIG_PIN 9 // Trigger pin is 9
#define ECHO_PIN 8 // Echo pin is 8

// Create sensor objects
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud

  // Set up ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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

  Serial.println("Ultrasonic, MPU6050, and MMA8451 sensors initialized.");
}

void loop() {
  // Read data from MMA8451
  mma.read();
  float xacc_mma = (mma.x / 4096.0) * 9.80665 * 1000; // MMA8451 X-axis in mm/s²
  float yacc_mma = (mma.y / 4096.0) * 9.80665 * 1000; // MMA8451 Y-axis in mm/s²
  float zacc_mma = (mma.z / 4096.0) * 9.80665 * 1000; // MMA8451 Z-axis in mm/s²

  // Read gyroscope data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Gyroscope data in degrees per second (°/s)
  float xgyro_mpu = g.gyro.x; // X-axis angular velocity
  float ygyro_mpu = g.gyro.y; // Y-axis angular velocity
  float zgyro_mpu = g.gyro.z; // Z-axis angular velocity

  // Read distance data from the ultrasonic sensor
  float distance_mm = getUltrasonicDistance(); // Get distance in mm

  // Send data as a single line of CSV
  Serial.print(xacc_mma, 2); Serial.print(",");
  Serial.print(yacc_mma, 2); Serial.print(",");
  Serial.print(zacc_mma, 2); Serial.print(",");
  Serial.print(xgyro_mpu, 2); Serial.print(",");
  Serial.print(ygyro_mpu, 2); Serial.print(",");
  Serial.print(zgyro_mpu, 2); Serial.print(",");
  Serial.println(distance_mm, 2); // Ultrasonic distance in mm

  delay(500); // Delay for stability
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
