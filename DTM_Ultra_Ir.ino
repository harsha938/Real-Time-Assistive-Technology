#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX
static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

const int PIR = 7;
int input_val = 0;

// Define pins for ultrasonic sensor
#define TRIG_PIN 4
#define ECHO_PIN 5

// Define the maximum distance (in centimeters) to consider an object detected
#define MAX_DISTANCE 100

// Create the Player object
DFRobotDFPlayerMini player;

void setup() {
  // Init USB serial port for debugging
  Serial.begin(9600);
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
    Serial.println("OK");

    // Set volume to maximum (0 to 30).
    player.volume(30);

  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
  delay(1000);

  // Ultrasonic sensor pin configurations
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the response from the sensor
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Convert the duration to distance (in centimeters)
  long distance_cm = duration * 0.034 / 2;

  // Print the distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  // Check if an object is within the maximum distance
  if (distance_cm <= MAX_DISTANCE) {
    // Play sound for object detection
    player.play(1);
    Serial.println("Object detected. Sound played.");
    delay(2000);
  } else {
    // Check if motion is detected by the flame sensor
    input_val = digitalRead(PIR);

    // Print PIR sensor value for debugging
    Serial.println(input_val);

    // Check if motion is detected
    if (input_val == 0) {
      // Play sound for flame detection
      player.play(2);
      Serial.println("Motion detected. Sound played.");
      delay(2000);
    }
  }
}
