/*
  Teensy 4.0 Force Sensor Test Script
  Reads analog values from 5 force sensors mapped to specific fingers.
*/

// Define the Teensy analog pins based on your table
const int PIN_PINKY  = 23;  // A4
const int PIN_RING   = 22;  // A3
const int PIN_MIDDLE = 21;  // A13
const int PIN_INDEX  = 20;  // A1
const int PIN_THUMB  = 19;  // A0

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for serial port to connect (max 4 seconds)
  }

  Serial.println("=========================================");
  Serial.println("Teensy 4.0 Force Sensor Test Initialized");
  Serial.println("Format: Pinky | Ring | Middle | Index | Thumb");
  Serial.println("=========================================");
  
  // Set the analog read resolution to 10-bit (0-1023) for standard compatibility,
  // or change to 12-bit (0-4095) for higher precision on Teensy 4.0.
  analogReadResolution(10); 
}

void loop() {
  // Read raw analog values from each sensor
  int pinkyVal  = analogRead(PIN_PINKY);
  int ringVal   = analogRead(PIN_RING);
  int middleVal = analogRead(PIN_MIDDLE);
  int indexVal  = analogRead(PIN_INDEX);
  int thumbVal  = analogRead(PIN_THUMB);

  // Print formatted values to the Serial Monitor
  Serial.print("P: "); Serial.print(pinkyVal);   Serial.print("\t| ");
  Serial.print("R: "); Serial.print(ringVal);    Serial.print("\t| ");
  Serial.print("M: "); Serial.print(middleVal);  Serial.print("\t| ");
  Serial.print("I: "); Serial.print(indexVal);   Serial.print("\t| ");
  Serial.print("T: "); Serial.println(thumbVal);

  // Small delay to stabilize readings and prevent spamming the serial monitor
  delay(100);
}