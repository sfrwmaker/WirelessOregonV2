#include <WlessOregonV2.h>

/* Receive Temperature and humidity data using oregon protocol V2.1
 * To turn on dump of received packet, set last parameter of receiveData method to true. 
 * Ensure you have anabled serial port.
 */

OregonDecoderV2 orscV2;

void reportSerial(const uint8_t *raw_data, const uint8_t count) {
    Serial.print(F("OregonDecoderV2: "));
    for (uint8_t i = 0; i < count; ++i) {
        Serial.print(raw_data[i] >> 4, HEX);
        Serial.print(raw_data[i] & 0x0F, HEX);
    }
    Serial.println();
}

void setup () {
    Serial.begin(115200);                         // Enable Serial for packet DUMP output
    Serial.println("\n[ookDecoder]");
    orscV2.begin(digitalPinToInterrupt(4));
    orscV2.attachDebugCallback(reportSerial);
}

void loop () {
  uint8_t ch   = 0;
  uint8_t id   = 0;
  int16_t temp  = 0;
  uint8_t humm = 0;
  bool battOK = false;
  if (orscV2.receiveData(ch, id, temp, humm, battOK)) {
    Serial.print("Channel = "); Serial.print(id, HEX);
    Serial.print(", ID = ");    Serial.print(id, HEX);
    char minus = ' ';
    if (temp < 0) {
      minus = '-';
      temp *= -1;
    }
    Serial.print(", Temp = ");  Serial.print(minus); Serial.print(temp/10); Serial.print("."); Serial.print(temp%10);
    Serial.print(", Humm = ");  Serial.print(humm);
    if (battOK) Serial.println(", battery OK");
    else        Serial.println(", replace battery");
  }
}
