#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include<Wire.h>
// LiquidCrystal_I2C lcd(0x27, 16, 2);
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Define LED pins (replace with actual LED connection pins)
const int ledPin1 = 10;
const int ledPin2 = 11;

void setup() {
  // Set LED pins as outputs
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  lcd.begin(16,2);
  // turn on the backlight
  // lcd.backlight();
  lcd.clear();
  //wait for a second
  delay(1000);
  // tell the screen to write on the top row
  lcd.setCursor(4,0);
  lcd.print("RESULT");
  // Start serial communication (adjust baud rate if needed)
  Serial.begin(9600);
  
}

void loop() {
  // Check for incoming serial data
  
  if (Serial.available()) {
    // char incomingData = Serial.read();
    String Data = Serial.readStringUntil('\n');
    // Serial.println(incomingData);
    Serial.println(Data);
    // printf(incomingData);

    // Process the received data
    if (Data == "1") {
      Serial.println(Data); 
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print("HI");

      // digitalWrite(ledPin1, HIGH);
      // digitalWrite(ledPin2, LOW);  // Turn on LED connected to pin 10
      delay(1000);
    } 
    else if (Data == "2") {
      // Serial.println(incomingData); 
      Serial.println(Data); 
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print("ME");
      // digitalWrite(ledPin1, LOW);
      // digitalWrite(ledPin2, HIGH);  // Turn on LED connected to pin 11
    } 

    else if (Data == "3") {
      // Serial.println(incomingData); 
      Serial.println(Data); 
      lcd.clear();
      lcd.setCursor(4,0);
      lcd.print("SEE YOU");
      // digitalWrite(ledPin1, LOW);
      // digitalWrite(ledPin2, HIGH);  // Turn on LED connected to pin 11
    } 

    else if (Data == "4") {
      // Serial.println(incomingData); 
      Serial.println(Data); 
      lcd.clear();
      lcd.setCursor(6,0);
      lcd.print("LOVE");
      // digitalWrite(ledPin1, LOW);
      // digitalWrite(ledPin2, HIGH);  // Turn on LED connected to pin 11
    } 
    else {
      // Handle invalid data (e.g., turn off LEDs, display error message)
      // digitalWrite(ledPin1, LOW);
      // digitalWrite(ledPin2, LOW);
      Serial.println("Detecting");
    }
    // delay(1000);
  // } else {
  //   // No data received, keep LEDs off
  //   digitalWrite(ledPin1, HIGH);
  //   digitalWrite(ledPin2, LOW);
  // }
}
}