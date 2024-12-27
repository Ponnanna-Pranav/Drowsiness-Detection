#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows
Servo myServo;

const int ledPin = 13;  // LED connected to digital pin 13
const int buzzerPin = 9;  // Buzzer connected to digital pin 9
const int servoPin = 10;

bool alarmStatus = false;

void setup() {
  lcd.init();  // initialize the lcd
  lcd.backlight();  // turn on backlight
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);  // Start serial communication at 9600 bps
  myServo.attach(servoPin); 
  
  // Initialize LCD with default message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ALL GOOD");
  lcd.setCursor(0, 1);
  lcd.print("DRIVE SAFE");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'D') {
      // Drowsiness detected
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wake up");
      digitalWrite(ledPin, HIGH);  // Turn on LED
      tone(buzzerPin, 1000);  // Turn on buzzer
      myServo.write(180); // Set servo to 180 degrees
      delay(50); // Small delay
      myServo.write(0); // Return servo to 0 degrees
      alarmStatus = true;
    } else if (command == 'N') {
      // No drowsiness
      lcd.clear();
      digitalWrite(ledPin, LOW);  // Turn off LED
      noTone(buzzerPin);  // Turn off buzzer
      myServo.write(90); // Neutral position
      alarmStatus = false;
      
      // Restore default LCD message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ALL GOOD");
      lcd.setCursor(0, 1);
      lcd.print("DRIVE SAFE");
    } else if (command == 'Y') {
      // Yawning detected
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Take a break");
      digitalWrite(ledPin, HIGH);  // Turn on LED
      alarmStatus = true;
    } else if (command == 'P') {
      // Not paying attention
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PAY ATTENTION");
      digitalWrite(ledPin, LOW);  // Turn off LED
      tone(buzzerPin, 1000);  // Turn on buzzer
      alarmStatus = true;
    }
  }
}
