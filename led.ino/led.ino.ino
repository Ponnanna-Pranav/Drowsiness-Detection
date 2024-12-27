#include <Wire.h>

const int ledPin = 13;    // LED connected to digital pin 13
const int buzzerPin = 9;  // Buzzer connected to digital pin 9

bool alarmStatus = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);  // Start serial communication at 9600 bps
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'D') {
      // Drowsiness detected
      digitalWrite(ledPin, HIGH);  // Turn on LED
      tone(buzzerPin, 1000);        // Turn on buzzer
      alarmStatus = true;
    } else if (command == 'N') {
      // No drowsiness
      digitalWrite(ledPin, LOW);   // Turn off LED
      noTone(buzzerPin);            // Turn off buzzer
      alarmStatus = false;
    } else if (command == 'Y') {
      // Yawning detected
      digitalWrite(ledPin, HIGH);  // Turn on LED
      alarmStatus = true;
    } else if (command == 'P') {
      // Not paying attention
      digitalWrite(ledPin, LOW);   // Turn off LED
      tone(buzzerPin, 1000);        // Turn on buzzer
      alarmStatus = true;
    }
  }
}
