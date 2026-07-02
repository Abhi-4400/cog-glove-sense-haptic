/*
 * Cog Glove - Force Sensor Serial Output
 * Outputs analog readings as comma-separated values in the order:
 * Thumb, Index, Middle, Ring, Pinky
 */

// Define the analog pins based on the wiring table
const int forceThumb  = A0;
const int forceIndex  = A1;
const int forceMiddle = A12;
const int forceRing   = A13;
const int forcePinky  = A4;

// Array for easier iteration
const int forcePins[] = {forceThumb, forceIndex, forceMiddle, forceRing, forcePinky};
const int numFingers = 5;

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  
  // Wait for the serial port to connect (required for some Teensy boards)
  while (!Serial) {
    ; 
  }
  
  Serial.println("Force Sensor Data Started...");
  Serial.println("Format: Thumb, Index, Middle, Ring, Pinky");
}

void loop() {
  // Loop through each pin, read the value, and print it
  for (int i = 0; i < numFingers; i++) {
    int sensorValue = analogRead(forcePins[i]);
    Serial.print(sensorValue);
    
    // Add a comma after every value except the last one
    if (i < numFingers - 1) {
      Serial.print(", ");
    }
  }
  
  // Print a newline character at the end of the data stream
  Serial.println();
  
  // A short delay to prevent flooding the serial monitor
  delay(50); 
}