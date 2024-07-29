#include <Arduino.h>
#include <Wire.h>


#define TRIG_PIN 2
#define ECHO_PIN 3
#define BUZZER_PIN 9


int redLedPin = 13;
int greenLedPin = 12;
int blueLedPin = 11;

int time = 0;
int distance = 0;

#define COMMON_ANODE

void setColor(byte red, byte green, byte blue) {

  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif

  analogWrite(redLedPin, red);
  analogWrite(greenLedPin, green);
  analogWrite(blueLedPin, blue);
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {

  setColor(0, 255, 0);
  // Generating impulse for TRIG_PIN
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Reading duration time if ECHO_PIN impulse
  time = pulseIn(ECHO_PIN, HIGH);
  
  // Calculating distance
  distance = (time / 2) / 29.1;

  if (distance < 10) {
    tone(BUZZER_PIN, 2000, 20);
    setColor(255, 0, 0);
    delay(100);
  } else if (distance < 20) {
    tone(BUZZER_PIN, 2000, 20);
    setColor(255, 255, 0);
    delay(200);
  } else if (distance < 40) {
    tone(BUZZER_PIN, 2000, 20);
    setColor(0, 255, 0);
    delay(400);
  }

  noTone(BUZZER_PIN);
  delay(100);
}


