// basic_motors.ino
// Stripped-down motor test: drives both DC motors forward at max speed for 5 seconds on startup.
// Based on the chem-bot driver_code.ino pin assignments.

// Motor 1 direction pins
int M1_1 = D4;
int M1_2 = D5;

// Motor 2 direction pins
int M2_1 = D2;
int M2_2 = D3;

// Motor speed control pins
int m1_s_pin = A5;
int m2_s_pin = A4;

void setup() {
  // Set all motor pins to output
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  pinMode(m1_s_pin, OUTPUT);
  pinMode(m2_s_pin, OUTPUT);

  // Set direction to forward (HIGH/LOW pattern per motor)
  digitalWrite(M1_1, HIGH);
  digitalWrite(M1_2, LOW);
  digitalWrite(M2_1, HIGH);
  digitalWrite(M2_2, LOW);

  // Drive both motors at maximum speed (255)
  analogWrite(m1_s_pin, 255);
  analogWrite(m2_s_pin, 255);

  // Run for 5 seconds
  delay(5000);

  // Stop both motors
  analogWrite(m1_s_pin, 0);
  analogWrite(m2_s_pin, 0);
}

void loop() {
  // Nothing to do — single run on startup only
}
