#include <Arduino.h>
#include <IRremote.hpp>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define BATTERY_PIN A0
const float REFERENCE_VOLTAGE = 4.97;  // Napięcie odniesienia ADC (zazwyczaj 5V)
const float R1 = 6900.0;  // Rezystor górny dzielnika (w ohmach)
const float R2 = 10000.0;   // Rezystor dolny dzielnika (w ohmach)
const float THRESHOLD_VOLTAGE = 6.0; // Próg napięcia w Voltach

// Definicja pinów dla silników
#define MOTOR1_PIN1 8  // In1 dla pierwszego silnika
#define MOTOR1_PIN2 7  // In2 dla pierwszego silnika
#define MOTOR1_EN    9 // PWM dla pierwszego silnika
#define MOTOR2_EN    10 // PWM dla drugiego silnika
#define MOTOR2_PIN1 4  // In3 dla drugiego silnika
#define MOTOR2_PIN2 5  // In4 dla drugiego silnika

#define ECHO_PIN 2
#define TRIGGER_PIN 6

#define IR_PIN 3
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

const int MAX_SPEED = 255;
const int CURVE_SPEED = 100;

int obstacleDistance = 15; // Domyślna odległość wykrywania przeszkody (w cm)
#define EEPROM_ADDRESS 0 // Adres w pamięci EEPROM

void Forward();
void Backward();
void Left();
void Right();
void Stop();
void BlinkYellow();
void Ultrasonic();
void saveDistanceToEEPROM(int distance);
int readDistanceFromEEPROM();

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

    analogWrite(MOTOR1_EN, 0);
    analogWrite(MOTOR2_EN, 0);

    digitalWrite(DIODE_RED, LOW);
    digitalWrite(DIODE_YELLOW, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    pinMode(BATTERY_PIN, INPUT);

    wdt_enable(WDTO_2S);

    // Odczytanie zapisanej odległości z EEPROM
    obstacleDistance = readDistanceFromEEPROM();
    Serial.println("Wczytana odległość z EEPROM: " + String(obstacleDistance));
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

        Serial.println("Received code: " + String(currentCode, HEX));

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
            case 0xea15ff00: // Przycisk 'Increase Distance'
                if (obstacleDistance < 40) {
                    obstacleDistance += 5;
                    saveDistanceToEEPROM(obstacleDistance);
                    BlinkYellow();
                    Serial.println("Increased distance to: " + String(obstacleDistance));
                }
                break;
            case 0xf807ff00: // Przycisk 'Decrease Distance'
                if (obstacleDistance > 5) {
                    obstacleDistance -= 5;
                    saveDistanceToEEPROM(obstacleDistance);
                    BlinkYellow();
                    Serial.println("Decreased distance to: " + String(obstacleDistance));
                }
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
        //Serial.println("Forced stop");
        Stop();
    }

    int adcValue = analogRead(BATTERY_PIN);  // Odczyt ADC
    float voltageADC = (adcValue / 1023.0) * REFERENCE_VOLTAGE;  // Napięcie na dzielniku
    float batteryVoltage = voltageADC * (R1 + R2) / R2;         // Przeliczenie na rzeczywiste napięcie

    // Sprawdzenie progu
    if (batteryVoltage < THRESHOLD_VOLTAGE) {
        digitalWrite(DIODE_YELLOW, HIGH);
        Serial.println("Low battery voltage " + String(batteryVoltage));
        
    } 
    else {
        digitalWrite(DIODE_YELLOW, LOW);
        Serial.println("Normal battery voltage " + String(batteryVoltage));
    }

    Ultrasonic();
}

// Funkcje sterujące ruchem silników
void Forward() {
    analogWrite(MOTOR1_EN, MAX_SPEED);
    analogWrite(MOTOR2_EN, MAX_SPEED);

    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW); 
}

void Backward() {
    analogWrite(MOTOR1_EN, CURVE_SPEED);
    analogWrite(MOTOR2_EN, CURVE_SPEED);

    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
}

void Left() {
    analogWrite(MOTOR1_EN, CURVE_SPEED);
    analogWrite(MOTOR2_EN, CURVE_SPEED);

    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
}

void Right() {
    analogWrite(MOTOR1_EN, CURVE_SPEED);
    analogWrite(MOTOR2_EN, CURVE_SPEED);

    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);  
}

void Stop() {
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW); 
}

void BlinkYellow(){
    digitalWrite(DIODE_YELLOW, HIGH);
    delay(50);
    digitalWrite(DIODE_YELLOW, LOW);
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
        distance = time / (29 * 2);

        // Wykrywanie przeszkody
        if (distance < obstacleDistance) {
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

void saveDistanceToEEPROM(int distance) {
    EEPROM.write(EEPROM_ADDRESS, distance);
}

int readDistanceFromEEPROM() {
    int storedDistance = EEPROM.read(EEPROM_ADDRESS);
    if (storedDistance < 5 || storedDistance > 40) {
        storedDistance = 15; // Domyślna odległość
        saveDistanceToEEPROM(storedDistance);
    }
    return storedDistance;
}
