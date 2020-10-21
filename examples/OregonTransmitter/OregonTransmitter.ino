#include <WlessOregonV2.h>

const byte TX_PIN       = 11;               // Tha transtitter data PIN number
const byte LED_PIN      = 7;                // LED pin number, LED is on during the transmission
const byte channel      = 0x20;             // Not sure it is good idea to change it
const byte sensorID     = 0xBB;             // The ID of the sensor (byte) can be changed if there are several sensors exist
const byte sendHumidity = true;             // Whether the sensor can send humidity

//#define THN132N

OregonSensor os(TX_PIN, channel, sensorID, sendHumidity);
 
void setup() {
Serial.begin(115200);
  os.init();
  pinMode(LED_PIN, OUTPUT);

}
 
void loop() {
  static int temp = 50;                     // Degree of Celsius times 10
  static byte humidity = 0;                 // The relative himidity 0 - 100%
  static bool battOK = true;                // The battery status
  digitalWrite(LED_PIN, HIGH);
  Serial.print("Sending data of sensor "); Serial.print(sensorID, HEX);
  Serial.print(": temp = "); Serial.print(temp);
  Serial.print(", humidity = "); Serial.println(humidity);
  os.sendTempHumidity(temp, humidity, battOK);
  digitalWrite(LED_PIN, LOW);

  delay(10000);
  temp += 5; if (temp > 350) temp = 50;
  humidity += 1; if (humidity >= 100) humidity = 0;
  
}

