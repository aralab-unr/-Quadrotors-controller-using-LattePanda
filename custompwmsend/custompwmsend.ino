void setup() {
  analogWrite(5, 125);  // 490 Hz
  analogWrite(6, 125);  // 490 Hz
  analogWrite(9, 125);  // 490 Hz
  analogWrite(10, 125);  // 490 Hz
  Serial.begin(115200);
}

void loop() {
    if (Serial.available() > 0) {
    // Read the incoming PWM values
    int pwmValue1 = Serial.parseInt();  // First PWM value
    Serial.flush();  // Clear any remaining data

    if (Serial.peek() == ',') {
      Serial.read(); // Skip the comma
    }
    int pwmValue2 = Serial.parseInt();  // Second PWM value
    if (Serial.peek() == ',') {
      Serial.read(); // Skip the comma
    }
    int pwmValue3 = Serial.parseInt();  // Third PWM value
    if (Serial.peek() == ',') {
      Serial.read(); // Skip the comma
    }
    int pwmValue4 = Serial.parseInt();  // Fourth PWM value
    Serial.flush();  // Clear any remaining data

    // Check if valid values are received
    if (pwmValue1 == 0 || pwmValue2 == 0 || pwmValue3 == 0 || pwmValue4 == 0) {
      return; // No valid integer received or incomplete data
    }

    // Convert the microseconds (1000-2000) to a value between 0 and 255
    int pwmOut1 = map(pwmValue1, 1000, 2000, 125, 255);
    int pwmOut2 = map(pwmValue2, 1000, 2000, 125, 255);
    int pwmOut3 = map(pwmValue3, 1000, 2000, 125, 255);
    int pwmOut4 = map(pwmValue4, 1000, 2000, 125, 255);

    // Send the PWM values to the respective pins
    analogWrite(5, pwmOut1);  // 490 Hz
    analogWrite(6, pwmOut2);  // 490 Hz
    analogWrite(9, pwmOut3);  // 490 Hz
    analogWrite(10, pwmOut4);  // 490 Hz
  } else {
    // If no data is available, keep sending 1000 µs to all servos (50% duty cycle)
    analogWrite(5, 125);  // 490 Hz
    analogWrite(6, 125);  // 490 Hz
    analogWrite(9, 125);  // 490 Hz
    analogWrite(10, 125);  // 490 Hz
  }
}
