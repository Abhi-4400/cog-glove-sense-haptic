/*
 * Cog Glove - Flex Sensor Serial Output
 * Outputs analog readings as comma-separated values in the order:
 * Thumb, Index, Middle, Ring, Pinky
 */

// Define the analog pins based on the wiring table
const int flexThumb  = A5;
const int flexIndex  = A6;
const int flexMiddle = A7;
const int flexRing   = A8;
const int flexPinky  = A9;

// Array for easier iteration
const int flexPins[] = {flexThumb, flexIndex, flexMiddle, flexRing, flexPinky};
const int numFingers = 5;

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  
  // Wait for the serial port to connect (required for some Teensy boards)
  while (!Serial) {
    ; 
  }
  
  Serial.println("Flex Sensor Data Started...");
  Serial.println("Format: Thumb, Index, Middle, Ring, Pinky");
}

void loop() {
  // Loop through each pin, read the value, and print it
  for (int i = 0; i < numFingers; i++) {
    int sensorValue = analogRead(flexPins[i]);
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