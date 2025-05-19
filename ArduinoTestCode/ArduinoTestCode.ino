void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command == "PING") {
      Serial.println("PONG from Arduino");
    }
    else if (command == "LED_ON") {
      digitalWrite(13, HIGH);
      Serial.println("LED is ON from Arduino");
    }
    else if (command == "LED_OFF") {
      digitalWrite(13, LOW);
      Serial.println("LED is OFF from Arduino");
    }
    else {
      Serial.println("UNKNOWN COMMAND");
    }
  }
}
