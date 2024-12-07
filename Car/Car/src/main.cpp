#include <Arduino.h>
#include <IRremote.hpp>
#include <Wire.h>
#include <avr/wdt.h>

#define BATTERY_PIN A0

// Definicja pinów dla silników
#define MOTOR1_PIN1 8  // In1 dla pierwszego silnika
#define MOTOR1_PIN2 7  // In2 dla pierwszego silnika
#define MOTOR1_EN    9 // PWM dla pierwszego silnika

#define ECHO_PIN 2
#define MOTOR2_EN    3 // PWM dla drugiego silnika
#define MOTOR2_PIN1 4  // In3 dla drugiego silnika
#define MOTOR2_PIN2 5  // In4 dla drugiego silnika
#define TRIGGER_PIN 6

#define IR_PIN 10
#define BUZZER_PIN 11
#define DIODE_YELLOW 12
#define DIODE_RED 13

// Zmienne dla odbiornika IR
unsigned long timestamp = 0; // Przechowuje czas ostatniego sygnału pilota
#define TIMEOUT 100          // Czas w milisekundach, po którym silniki się zatrzymają
uint32_t currentCode = 0;      // Ostatnio odebrany kod

// Zmienne dla czujnika odległości
int time = 0;
int distance = 0;

unsigned long lastDistanceMeasurement = 0;
const unsigned long distanceInterval = 100; // Czas pomiędzy kolejnymi pomiarami (100ms)

// Próg napięcia baterii - 732 w zakresie 0-1024 daje około 6V (3.55 po dzielniku napięcia)
const int batteryThreshold = 727;

// Kierunki ruchu
#define FORWARD 1
#define BACKWARD 0
#define MAX_SPEED 255



// Funkcje sterujące ruchem silników
void Forward() {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR1_EN, MAX_SPEED);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR2_EN, MAX_SPEED);
}

void Backward() {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    analogWrite(MOTOR1_EN, MAX_SPEED);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    analogWrite(MOTOR2_EN, MAX_SPEED);
}

void Left() {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR1_EN, MAX_SPEED);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR2_EN, 0);
}

void Right() {
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR1_EN, 0);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR2_EN, MAX_SPEED);
}

void Stop() {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR1_EN, 0);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR2_EN, 0);
}

void Ultrasonic() {
    // Pomiar odległości co 'distanceInterval' ms
    if (millis() - lastDistanceMeasurement >= distanceInterval) {
        lastDistanceMeasurement = millis();

        // Odczyt z czujnika ultradźwiękowego
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);

        time = pulseIn(ECHO_PIN, HIGH);
        distance = time * 0.034 / 2;

        // Wykrywanie przeszkody
        if (distance < 15) {
            digitalWrite(DIODE_RED, HIGH);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(10);
            digitalWrite(BUZZER_PIN, LOW);
        } else {
            digitalWrite(DIODE_RED, LOW);
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}

void setup() {
    // Inicjalizacja komunikacji szeregowej
    Serial.begin(9600);

    // Inicjalizacja odbiornika IR
    IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
    Serial.println("Odbiornik IR włączony");

    // Konfiguracja pinów jako wyjścia
    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(MOTOR1_EN, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);
    pinMode(MOTOR2_EN, OUTPUT);
    pinMode(DIODE_RED, OUTPUT);
    pinMode(DIODE_YELLOW, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(DIODE_RED, LOW);
    digitalWrite(DIODE_YELLOW, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    wdt_enable(WDTO_2S);
}

void loop() { 
    wdt_reset(); // Resetowanie Watchdog Timer

    // Odczytanie kodu z pilota
    if (IrReceiver.decode()) {
        // Wyświetlenie odebranego kodu
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT){
            //Serial.println("Repeat signal received");
        } else {
            currentCode = IrReceiver.decodedIRData.decodedRawData;
        }

        // Wykonanie akcji w zależności od kodu
        switch (currentCode) {
            case 0xE718FF00: // Przycisk 'Forward'
                Serial.println("Forward");
                Forward();
                break;
            case 0xAD52FF00: // Przycisk 'Backward'
                Serial.println("Backward");
                Backward();
                break;
            case 0xF708FF00: // Przycisk 'Left'
                Serial.println("Left");
                Left();
                break;
            case 0xA55AFF00: // Przycisk 'Right'
                Serial.println("Right");
                Right();
                break;
            default:
                Serial.println("Unknown code or stop signal");
                Stop();
                break;
        }
        timestamp = millis(); // Zapisz czas ostatniego sygnału
        IrReceiver.resume(); // Wznowienie odbioru sygnałów
    }
    if (millis() - timestamp > TIMEOUT) {
        // Zatrzymanie silników po upływie TIMEOUT
        Serial.println("Forced stop");
        Stop();
    }

    int batteryValue = analogRead(BATTERY_PIN);
    //Serial.println(batteryValue);

    if (batteryValue < batteryThreshold) {
        digitalWrite(DIODE_YELLOW, HIGH);
    } else {
        digitalWrite(DIODE_YELLOW, LOW);
    }

    Ultrasonic();
}
