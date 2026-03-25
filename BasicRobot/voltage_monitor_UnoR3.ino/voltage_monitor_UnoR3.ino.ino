/*
  Arduino Uno 5V Voltage Monitor
  Reads a voltage on A0 and prints it to the Serial Monitor.
*/

const int analogPin = A0;  // Pin connected to voltage source
float vInput = 0.0;        // Variable to store measured voltage
float vRef = 5.0;          // Reference voltage (5V)

void setup() {
  Serial.begin(9600);      // Start serial communication
}

void loop() {
  // Read the analog value (0-1023)
  int sensorValue = analogRead(analogPin);

  // Convert to voltage: (value / max_steps) * ref_voltage) * voltage factor (5)
  vInput = ((sensorValue * vRef) / 1023.0) * 5;

  // Print results
  Serial.print("Voltage: ");
  Serial.print(vInput);
  Serial.println("V");

  delay(5000); // Wait 5000ms before next reading
}
