# Real-Time Assistive Technology with GSM and GPS Integration

This project combines assistive technology with GSM and GPS modules using Arduino. The system plays audio feedback for object and motion detection and can make emergency calls with GPS coordinates when a button is pressed.

## Components

- Arduino Uno or any compatible microcontroller
- DFPlayer Mini MP3 Player Module
- Ultrasonic Sensor (HC-SR04)
- PIR Motion Sensor
- GSM Module (SIM800L or similar)
- GPS Module (NEO-6M or similar)
- Push Button
- Breadboard and connecting wires

## Connections

### DFPlayer Mini Module
- TX (DFPlayer) -> Pin 2 (Arduino)
- RX (DFPlayer) -> Pin 3 (Arduino)

### Ultrasonic Sensor
- VCC -> 5V (Arduino)
- GND -> GND (Arduino)
- Trig -> Pin 4 (Arduino)
- Echo -> Pin 5 (Arduino)

### PIR Motion Sensor
- VCC -> 5V (Arduino)
- GND -> GND (Arduino)
- OUT -> Pin 7 (Arduino)

### GSM Module
- TX (GSM) -> Pin 9 (Arduino)
- RX (GSM) -> Pin 10 (Arduino)

### GPS Module
- TX (GPS) -> Pin 4 (Arduino)
- RX (GPS) -> Pin 5 (Arduino)

### Push Button
- One leg -> Pin 2 (Arduino)
- Other leg -> GND (Arduino)

## Code Overview

The Arduino code integrates the DFPlayer Mini for audio feedback, an ultrasonic sensor for distance measurement, a PIR motion sensor for motion detection, a GSM module for making emergency calls, and a GPS module for obtaining the device's location.

## Libraries

Ensure you have the following libraries installed in your Arduino IDE:

- SoftwareSerial
- DFRobotDFPlayerMini

## Configuration

The following pins are defined for the connections:

```cpp
#define PIN_MP3_TX 2
#define PIN_MP3_RX 3
#define PIR 7
#define TRIG_PIN 4
#define ECHO_PIN 5
#define GSM_TX_PIN 9
#define GSM_RX_PIN 10
#define GPS_TX_PIN 4
#define GPS_RX_PIN 5
#define BUTTON_PIN 2
#define PHONE_NUMBER "+918919428737"
#define MAX_DISTANCE 100
```
## Initialization

The `setup` function initializes the serial communication for debugging, DFPlayer Mini, GSM, and GPS modules:

```cpp
void setup() {
  Serial.begin(9600);
  softwareSerial.begin(9600);
  gsmSerial.begin(9600);
  gpsSerial.begin(9600);
  
  if (player.begin(softwareSerial)) {
    Serial.println("OK");
    player.volume(30);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
  
  delay(2000);
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
}
```
## Main Loop

The `loop` function continuously checks for objects within the maximum distance using the ultrasonic sensor, motion using the PIR sensor, and handles button press for emergency calls:

```cpp
void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance_cm = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  if (distance_cm <= MAX_DISTANCE) {
    player.play(1);
    Serial.println("Object detected. Sound played.");
    delay(2000);
  } else {
    int input_val = digitalRead(PIR);
    Serial.println(input_val);

    if (input_val == 0) {
      player.play(2);
      Serial.println("Motion detected. Sound played.");
      delay(2000);
    }
  }

  if (digitalRead(BUTTON_PIN) == HIGH) {
    getGPSData();
    makeCall();
    delay(1000);
  }
}

void getGPSData() {
  gpsSerial.println("AT+CGNSINF");
  delay(500);

  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == ',') {
      latitude = gpsSerial.readStringUntil(',');
      for (int i = 0; i < 2; i++) {
        gpsSerial.readStringUntil(',');
      }
      longitude = gpsSerial.readStringUntil(',');
      break;
    }
  }
  
  Serial.print("Latitude: ");
  Serial.println(latitude);
  Serial.print("Longitude: ");
  Serial.println(longitude);
}

void makeCall() {
  gsmSerial.print("ATD");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println(";");

  Serial.println("Calling...");
}
```

## Usage

1. Connect the components as per the connections described above.
2. Upload the provided code to the Arduino.
3. Open the Serial Monitor to view the distance measurements, sensor status, and GPS data.
4. Place objects or create motion to trigger the respective audio feedback.
5. Press the button to make an emergency call with the current GPS coordinates.
