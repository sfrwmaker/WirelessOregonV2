#include <WlessOregonV2.h>

/* Receive Temperature and humidity data using oregon protocol V2.1
 * To turn on dump of received packet, set last parameter of receiveData method to true. 
 * Ensure you have anabled serial port.
 */

OregonDecoderV2 orscV2;
 
void setup () {
    Serial.begin(115200);                         // Enable Serial for packet DUMP output
    Serial.println("\n[ookDecoder]");
    orscV2.begin(0);
}
 
void loop () {

  int temp = 0;
  byte humm = 0;
  byte id = 0;
  if (orscV2.receiveData(id, temp, humm, true)) {
    Serial.print("ID = ");     Serial.print(id, HEX);
    Serial.print(", Temp = "); Serial.print(temp/10); Serial.print("."); Serial.print(temp%10);
    Serial.print(", Humm = "); Serial.println(humm);
  }
}
