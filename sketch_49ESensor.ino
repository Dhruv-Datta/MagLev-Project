const int pinHall = A0;
bool isRunning = true;  // Control flag to start/stop the loop

void setup() {
  pinMode(pinHall, INPUT);
  Serial.begin(2000000);
  Serial.println("Time(ms) Voltage(mV) MagneticFlux(mT)");
  Serial.println("Type 'stop' to halt the program.");
}

void loop() {
  // Check for 'stop' command from Serial Monitor
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "stop") {
      isRunning = false;
      Serial.println("Program stopped. Type 'start' to resume.");
    }
    if (command == "start") {
      isRunning = true;
      Serial.println("Program resumed.");
    }
  }

  if (isRunning) {
    static unsigned long startTime = millis();
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    long measure = 0;
    for (int i = 0; i < 10; i++) {
      measure += analogRead(pinHall);
    }
    measure /= 10;

    float outputV = measure * 5000.0 / 1023.0;
    float magneticFlux = outputV * 53.33 - 133.3;

    Serial.print("Time(ms): ");
    Serial.print(elapsedTime);
    Serial.print("  ");
    Serial.print("Voltage(mV): ");
    Serial.print(outputV);
    Serial.print("  ");
    Serial.print("MagneticFlux(mT): ");
    Serial.println(magneticFlux);

    delay(100);
  }
}