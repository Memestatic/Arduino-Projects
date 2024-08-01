#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <DHT.h>

#define DHT11PIN 8
#define DHTTYPE DHT11

DHT dht11(DHT11PIN, DHTTYPE);

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

 
void setup(){
  lcd.begin(16 ,2);
  dht11.begin();
}
 
void loop(){       
  float temperature = dht11.readTemperature();
  float humidity = dht11.readHumidity();

  if(isnan(temperature) || isnan(humidity)){
    lcd.setCursor(0, 0);
    lcd.print("Read Error)");
  }                     
  else{
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");
  }

  delay(2000);
}