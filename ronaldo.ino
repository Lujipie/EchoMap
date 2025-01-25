#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>  // Include the Servo library

// Create an instance of the MMA8451 sensor
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// MPU6050 I2C address
const int MPU_ADDR = 0x68;

// Variables to store sensor data
float mmaAccelX = 0.0, mmaAccelY = 0.0, mmaAccelZ = 0.0;
float mpuAccelX = 0.0, mpuAccelY = 0.0, mpuAccelZ = 0.0;
float distance = 0.0;
float avgAccelX = 0.0, avgAccelY = 0.0, avgAccelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

// Servo motor setup
Servo myServo;  // Create a Servo object
int currentAngle = 0; // Variable to track the current angle of the servo
int angleIncrement = 10; // Angle increment for each movement (10 degrees)

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("Initializing sensors...");

  // Initialize the MMA8451 accelerometer
  if (!mma.begin()) {
    Serial.println("Could not find MMA8451 sensor. Check wiring!");
    while (1); // Stop execution if sensor is not found
  }
  Serial.println("MMA8451 found!");

  // Set accelerometer range (choose from ±2g, ±4g, ±8g)
  mma.setRange(MMA8451_RANGE_2_G);
  Serial.print("Range set to: ");
  switch (mma.getRange()) {
    case MMA8451_RANGE_2_G: Serial.println("±2g"); break;
    case MMA8451_RANGE_4_G: Serial.println("±4g"); break;
    case MMA8451_RANGE_8_G: Serial.println("±8g"); break;
  }

  // Initialize MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up MPU6050
  Wire.endTransmission(true);

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servo to pin D10
  myServo.attach(10); // Pin D10 for servo control

  // Initialize servo position
  myServo.write(currentAngle); // Set initial position
}

void loop() {
  // --- Move the Servo in Small Increments ---
  for (currentAngle = 0; currentAngle <= 180; currentAngle += angleIncrement) {
    myServo.write(currentAngle); // Rotate the servo to the current angle
    delay(500); // Wait for the servo to complete its movement

    // --- Read Acceleration Data ---
    readMMA8451();
    readMPU6050();

    // Calculate average acceleration
    avgAccelX = (mmaAccelX + mpuAccelX) / 2.0;
    avgAccelY = (mmaAccelY + mpuAccelY) / 2.0;
    avgAccelZ = (mmaAccelZ + mpuAccelZ) / 2.0;

    // --- Read Distance from Ultrasonic Sensor ---
    distance = getUltrasonicDistance();

    // --- Send Data to Serial ---
    sendDataToSerial(avgAccelX, avgAccelY, avgAccelZ, distance, gyroX, gyroY, gyroZ);

    // Short delay before moving to the next angle
    delay(200);
  }
}

// Function to read data from MMA8451
void readMMA8451() {
  sensors_event_t event;
  mma.getEvent(&event);

  mmaAccelX = event.acceleration.x;
  mmaAccelY = event.acceleration.y;
  mmaAccelZ = event.acceleration.z;
}

// Function to read data from MPU6050
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // Request 14 bytes of data

  // Read accelerometer data
  mpuAccelX = (Wire.read() << 8 | Wire.read()) / 16384.0; // Convert to g
  mpuAccelY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Convert to g
  mpuAccelZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Convert to g

  // Skip temperature data
  Wire.read(); Wire.read();

  // Read gyroscope data
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // Convert to deg/s
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // Convert to deg/s
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // Convert to deg/s
}

// Function to calculate distance from ultrasonic sensor
float getUltrasonicDistance() {
  // Send a 10-microsecond pulse to the TRIG pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the ECHO pulse
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm (speed of sound = 343 m/s or 0.0343 cm/µs)
  float distance = (duration * 0.0343) / 2.0;

  // Return the distance
  return distance;
}

// Function to send data over serial
void sendDataToSerial(float avgAccelX, float avgAccelY, float avgAccelZ, float distance, float gyroX, float gyroY, float gyroZ) {
  Serial.print(currentAngle); // Servo angle
  Serial.print(",");
  Serial.print(avgAccelX); // Averaged Acceleration X
  Serial.print(",");
  Serial.print(avgAccelY); // Averaged Acceleration Y
  Serial.print(",");
  Serial.print(avgAccelZ); // Averaged Acceleration Z
  Serial.print(",");
  Serial.print(distance); // Distance
  Serial.print(",");
  Serial.print(gyroX); // Gyro X
  Serial.print(",");
  Serial.print(gyroY); // Gyro Y
  Serial.print(",");
  Serial.print(gyroZ); // Gyro Z
  Serial.println(); // Newline to separate data entries
}
