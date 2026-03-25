/*
  Arduino Uno 5V Voltage Monitor
  Reads a voltage on A0 and prints it to the Serial Monitor.
*/

#include <avr/sleep.h>

const int analogPin = A0;  // Pin connected to voltage source
float vInput = 0.0;        // Variable to store measured voltage
float vRef = 5.0;          // Reference voltage (5V)
const float voltageScaleFactor = 5.0;
const float reachedVoltageThreshold = 16.0;
const float shutdownVoltageThreshold = 16.1;

unsigned long firstReached16Timestamp = 0;
bool reached16V = false;

void shutdownArduino() {
  Serial.println("Voltage dropped below 16.1V. Entering shutdown mode.");
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void setup() {
  Serial.begin(9600);      // Start serial communication
}

void loop() {
  // Read the analog value (0-1023)
  int sensorValue = analogRead(analogPin);

  // Convert to voltage: (value / max_steps) * ref_voltage) * voltage factor (5)
  vInput = ((sensorValue * vRef) / 1023.0) * voltageScaleFactor;

  if (!reached16V && vInput >= reachedVoltageThreshold) {
    reached16V = true;
    firstReached16Timestamp = millis();
  }

  // Print results
  Serial.print("Voltage: ");
  Serial.print(vInput);
  Serial.println("V");
  if (reached16V) {
    Serial.print("First >= 16.0V timestamp (ms): ");
    Serial.println(firstReached16Timestamp);
  }

  if (reached16V && vInput < shutdownVoltageThreshold) {
    shutdownArduino();
  }

  delay(5000); // Wait 5000ms before next reading
}
