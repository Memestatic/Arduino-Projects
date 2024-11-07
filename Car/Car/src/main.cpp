#include <IRremote.h>

const int RemotePin = 8;

void setup() {
  Serial.begin(9600);
  IrReceiver.begin(RemotePin, ENABLE_LED_FEEDBACK);  // Initialize receiver
}

void loop() {
  if (IrReceiver.decode()) {  // Check if a signal is received
    Serial.println(IrReceiver.decodedIRData.command, HEX);  // Print received command in HEX
    IrReceiver.resume();  // Receive the next value
  }
}
