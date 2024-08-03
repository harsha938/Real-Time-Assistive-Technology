#include <SoftwareSerial.h>

// Define the TX and RX pins for the SoftwareSerial communication with the GSM module
#define GSM_TX_PIN 9
#define GSM_RX_PIN 10

// Define the TX and RX pins for the SoftwareSerial communication with the GPS module
#define GPS_TX_PIN 4
#define GPS_RX_PIN 5

// Define the pin for the push button
#define BUTTON_PIN 2

// Define the phone number to call
#define PHONE_NUMBER "+918919428737" // Replace with the phone number you want to call

// Create a SoftwareSerial objects for GSM and GPS modules
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);

// Variables to store latitude and longitude
String latitude = "";
String longitude = "";

void setup() {
  // Start serial communication with a baud rate of 9600
  Serial.begin(9600);

  // Start serial communication with the GSM module with the same baud rate
  gsmSerial.begin(9600);

  // Start serial communication with the GPS module with a baud rate of 9600
  gpsSerial.begin(9600);

  // Set button pin as input
  pinMode(BUTTON_PIN, INPUT);

  // Wait for the GSM module to initialize
  delay(2000);
  
  // Set GSM module to text mode
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
}

void loop() {
  // Check if the button is pressed
  if (digitalRead(BUTTON_PIN) == HIGH) {
    // Get GPS coordinates
    getGPSData();
    
    // Make a call
    makeCall();
    
    // Wait for some time to avoid multiple calls from a single press
    delay(1000);
  }
}

void getGPSData() {
  // Send command to GPS module to request data
  gpsSerial.println("AT+CGNSINF");

  // Wait for GPS module to respond
  delay(500);

  // Read GPS data
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == ',') {
      // Latitude
      latitude = gpsSerial.readStringUntil(',');
      
      // Skip other data
      for (int i = 0; i < 2; i++) {
        gpsSerial.readStringUntil(',');
      }
      
      // Longitude
      longitude = gpsSerial.readStringUntil(',');
      
      // Exit loop
      break;
    }
  }
  Serial.println("https://www.google.com/maps/?q=");
  Serial.println("Latitude:");
  Serial.println(latitude);
  Serial.println(",");
  Serial.println("Longitude:");
  Serial.println(longitude);
}

void makeCall() {
  // Dial the phone number
  gsmSerial.print("ATD");
  gsmSerial.print(PHONE_NUMBER);
  gsmSerial.println(";");

  Serial.println("Calling...");
}
