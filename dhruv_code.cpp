#include <Arduino.h>

// PID Constants
float Kp = 25.0;
float Ki = 0.1;
float Kd = 3.0;
float setpoint = 370;  // Desired position
float integral = 0;
float previous_error = 0;
float output_limits_min = 0;
float output_limits_max = 1255;

// PID Compute Function
float computePID(float setpoint, float measured_value, float dt) {
  float error = setpoint - measured_value;
  integral += error * dt;
  float derivative = (error - previous_error) / dt;

  float output = Kp * error + Ki * integral + Kd * derivative;

  // Clamp the output to the defined limits
  if (output < output_limits_min) {
    output = output_limits_min;
  } else if (output > output_limits_max) {
    output = output_limits_max;
  }

  previous_error = error;  // Update previous error
  return output;
}

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  static unsigned long previous_time = 0;
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;  // Time difference in seconds
  previous_time = current_time;

  // Read the sensor (assuming a sensor is connected to A0)
  float sensorValue = analogRead(A0);
  float measured_value = sensorValue;  // Adjust range as needed

  // Compute PID control signal
  float control_signal = computePID(setpoint, measured_value, dt);

  // Send control signal to a motor or actuator (adjust pin as necessary)
  analogWrite(9, control_signal);  // Example: PWM output on pin 9

  // Output data for debugging
  Serial.print("Measured Value: ");
  Serial.print(measured_value);
  Serial.print("\t Control Signal: ");
  Serial.println(control_signal);

  delay(50);  // Delay to give the system time to process
}
