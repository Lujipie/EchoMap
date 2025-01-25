#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>  // Include the Servo library

// Create an instance of the MMA8451 sensor
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 8;

// Variables to store sensor data
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float distance = 0.0;

// Servo motor setup
Servo myServo;  // Create a Servo object
int currentAngle = 0; // Variable to track the current angle of the servo
int angleIncrement = 2.5; // Angle increment for each movement (10 degrees)

// Calibration offsets for accelerometer
float accelX_offset = 0.0;
float accelY_offset = 0.0;
float accelZ_offset = 0.0;

// Filtering variables
#define NUM_SAMPLES 5
float accelX_samples[NUM_SAMPLES];
float accelY_samples[NUM_SAMPLES];
float accelZ_samples[NUM_SAMPLES];
int sampleIndex = 0;

// Low-pass filter constants
float accelX_filtered = 0.0;
float accelY_filtered = 0.0;
float accelZ_filtered = 0.0;
float alpha = 0.1; // Low-pass filter constant (adjust for more/less smoothing)

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

  // Set up ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attach the servo to pin D10
  myServo.attach(10); // Pin D10 for servo control

  // Initialize servo position
  myServo.write(currentAngle); // Set initial position

  // Initialize accelerometer offset and filtering arrays
  for (int i = 0; i < NUM_SAMPLES; i++) {
    accelX_samples[i] = 0;
    accelY_samples[i] = 0;
    accelZ_samples[i] = 0;
  }
}

void loop() {
  // --- Move the Servo in Small Increments ---
  for (currentAngle = 0; currentAngle <= 180; currentAngle += angleIncrement) {
    myServo.write(currentAngle); // Rotate the servo to the current angle
    delay(500); // Wait for the servo to complete its movement

    // --- Read Acceleration Data ---
    sensors_event_t event;
    mma.getEvent(&event);

    // Store current readings in the sample arrays (for moving average)
    accelX_samples[sampleIndex] = event.acceleration.x - accelX_offset;
    accelY_samples[sampleIndex] = event.acceleration.y - accelY_offset;
    accelZ_samples[sampleIndex] = event.acceleration.z - accelZ_offset;

    // Apply low-pass filter to the accelerometer data
    accelX_filtered = (alpha * event.acceleration.x) + ((1 - alpha) * accelX_filtered);
    accelY_filtered = (alpha * event.acceleration.y) + ((1 - alpha) * accelY_filtered);
    accelZ_filtered = (alpha * event.acceleration.z) + ((1 - alpha) * accelZ_filtered);

    // Move to the next sample index, looping back to 0 after NUM_SAMPLES
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

    // Calculate the average of the readings (moving average)
    float avgAccelX = 0, avgAccelY = 0, avgAccelZ = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
      avgAccelX += accelX_samples[i];
      avgAccelY += accelY_samples[i];
      avgAccelZ += accelZ_samples[i];
    }
    avgAccelX /= NUM_SAMPLES;
    avgAccelY /= NUM_SAMPLES;
    avgAccelZ /= NUM_SAMPLES;

    // --- Read Distance from Ultrasonic Sensor ---
    distance = getUltrasonicDistance();

    // --- Send Data to Serial ---
    sendDataToSerial(avgAccelX, avgAccelY, avgAccelZ, distance, accelX_filtered, accelY_filtered, accelZ_filtered);

    // Short delay before moving to the next angle
    delay(200);
  }
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
void sendDataToSerial(float avgAccelX, float avgAccelY, float avgAccelZ, float distance, float accelX_filtered, float accelY_filtered, float accelZ_filtered) {
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
  Serial.print(accelX_filtered); // Low-pass filtered Accel X
  Serial.print(",");
  Serial.print(accelY_filtered); // Low-pass filtered Accel Y
  Serial.print(",");
  Serial.print(accelZ_filtered); // Low-pass filtered Accel Z
  Serial.println(); // Newline to separate data entries
}
