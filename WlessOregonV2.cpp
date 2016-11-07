#include <Arduino.h>
#include "WlessOregonV2.h"

//--------------------------------------- digitalWrite using direct port manipulation ---------------------
void directPortB::sendOne(void) {
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TWOTIME);
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
}

void directPortB::sendZero(void) {
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TWOTIME);
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
}

void directPortD::sendOne(void) {
  PORTD &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
  PORTD |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TWOTIME);
  PORTD &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
}

void directPortD::sendZero(void) {
  PORTD |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
  PORTD &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TWOTIME);
  PORTD |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
}

//----------------------------------- Send the temp & humidity using oregon v2.1 protocol -------------
void OregonSensor::init(void) {
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, LOW);
}

void OregonSensor::sendData(const byte *data, byte size) {
  for(byte i = 0; i < size; ++i) {
    byte m = 1;
    byte d = data[i];
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero();; m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero();
  }
}

void OregonSensor::sendOregon(void) {
  sendPreamble();
  byte size = 8;
  if (existsHumidity) size = 9;
  sendData(buffer, size);
  sendPostamble();
  digitalWrite(tx_pin, LOW);
}

void OregonSensor::sendPreamble(void) {
  byte PREAMBLE[] = {0xFF,0xFF};
  sendData(PREAMBLE, 2);
}

void OregonSensor::sendPostamble(void) {
  sendZero();
  sendZero();
  sendZero();
  sendZero();
  if (!existsHumidity) return;                      // TNHN132N
  sendZero();
  sendZero();
  sendZero();
  sendZero();
}

void OregonSensor::sendSync(void) {
  byte data = 0xA;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero();
}

int OregonSensor::sum(byte count) {
  int s = 0;
 
  for(byte i = 0; i < count; i++) {
    s += (buffer[i]&0xF0) >> 4;
    s += (buffer[i]&0xF);
  }
 
  if(int(count) != count)
    s += (buffer[count]&0xF0) >> 4;
 
  return s;
}

void OregonSensor::calculateAndSetChecksum(void) {
  if (!existsHumidity) {
    int s = ((sum(6) + (buffer[6]&0xF) - 0xa) & 0xff);
    buffer[6] |=  (s&0x0F) << 4;     buffer[7] =  (s&0xF0) >> 4;
  } else {
    buffer[8] = ((sum(8) - 0xa) & 0xFF);
  }
}

void OregonSensor::sendTempHumidity(int temp, byte humm, bool battery) {  // temperature centegrees * 10
  if(!battery) buffer[4] = 0x0C; else buffer[4] = 0x00;

  if(temp < 0) {
    buffer[6] = 0x08;
    temp *= -1; 
  } else {
    buffer[6] = 0x00;
  }
  byte d3 = temp % 10;                              // Set temperature decimal part
  buffer[4] |= d3 << 4;
  temp /= 10;
  byte d1 = temp / 10;                              // 1st decimal digit of the temperature
  byte d2 = temp % 10;                              // 2nd deciaml digit of the temperature
  buffer[5] = d1 << 4;
  buffer[5] |= d2;

  if (existsHumidity) {                             // THGR2228N
    buffer[7] = humm / 10;
    buffer[6] |= (humm % 10) << 4;
  }
  calculateAndSetChecksum();

for (byte i = 0; i < 9; ++i) {
  Serial.print(buffer[i], HEX);
}
Serial.println(".");
  sendOregon();                                     // The v2.1 protocol send the message two times
  delayMicroseconds(TWOTIME*8);
  sendOregon();
}

//------------------------------------------ class DecodeOOK ------------------------------------------------------
volatile word wl_pulse;

void DecodeOOK::begin(byte int_num) {
  attachInterrupt(int_num, DecodeOOK::interuptHandler, CHANGE);
}

void DecodeOOK::interuptHandler(void) {
    static word last;
    // determine the pulse length in microseconds, for either polarity
    wl_pulse = micros() - last;
    last += wl_pulse;
}

bool DecodeOOK::nextPulse(word width) {
  if (state != DONE)
    switch (decode(width)) {
      case -1:
        resetDecoder();
        break;
      case 1:
        done();
        break;
    }
  return isDone();
}

void DecodeOOK::resetDecoder(void) {
  total_bits = bits = pos = flip = 0;
  state = UNKNOWN;
}

void DecodeOOK::gotBit(char value) {
  total_bits++;
  byte *ptr = data + pos;
  *ptr = (*ptr >> 1) | (value << 7);
 
  if (++bits >= 8) {
    bits = 0;
    if (++pos >= sizeof data) {
      resetDecoder();
      return;
    }
  }
  state = OK;
}

void DecodeOOK::manchester(char value) {
  flip ^= value;                                // manchester code, long pulse flips the bit
  gotBit(flip);
}

void DecodeOOK::alignTail(byte max) {
  // align bits
  if (bits != 0) {
    data[pos] >>= 8 - bits;
    for (byte i = 0; i < pos; ++i)
      data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
    bits = 0;
  }
  // optionally shift bytes down if there are too many of 'em
  if (max > 0 && pos > max) {
    byte n = pos - max;
    pos = max;
    for (byte i = 0; i < pos; ++i)
      data[i] = data[i+n];
  }
}

void DecodeOOK::reverseBits () {
  for (byte i = 0; i < pos; ++i) {
    byte b = data[i];
    for (byte j = 0; j < 8; ++j) {
      data[i] = (data[i] << 1) | (b & 1);
      b >>= 1;
    }
  }
}

void DecodeOOK::reverseNibbles () {
  for (byte i = 0; i < pos; ++i)
    data[i] = (data[i] << 4) | (data[i] >> 4);
}

void DecodeOOK::done () {
  while (bits)
    gotBit(0);                                  // padding
  state = DONE;
}

void DecodeOOK::reportSerial(void) {
  for (byte i = 0; i < pos; ++i) {
    Serial.print(data[i] >> 4, HEX);
    Serial.print(data[i] & 0x0F, HEX);
  }
  Serial.println();
}

//------------------------------------------ class OregonDecoderV2, decode Oregon station signals ----------------
void OregonDecoderV2::gotBit (char value) {
  if(!(total_bits & 0x01)) {
    data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
  }
  total_bits++;
  pos = total_bits >> 4;
  if (pos >= sizeof data) {
    resetDecoder();
    return;
  }
  state = OK;
}

char OregonDecoderV2::decode(word width) {
  if (200 <= width && width < 1200) {
    //Serial.println(width);
    byte w = width >= 700;
 
    switch (state) {
      case UNKNOWN:
        if (w != 0) {                           // Long pulse
          ++flip;
        } else if (w == 0 && 24 <= flip) {      // Short pulse, start bit
          flip = 0;
          state = T0;
        } else {                                // Reset decoder
          return -1;
        }
        break;
      case OK:
        if (w == 0) {                           // Short pulse
          state = T0;
        } else {                                // Long pulse
          manchester(1);
        }
        break;
      case T0:
        if (w == 0) {                           // Second short pulse
          manchester(0);
        } else {                                // Reset decoder
          return -1;
        }
        break;
    }

  } else if (width >= 2500  && pos >= 8) {
    return 1;
  } else {
    return -1;
  }
  return 0;
}
bool OregonDecoderV2::receiveData(byte& channel, byte& sensorID, int& temp, byte& hum, bool& battOK, bool debug) {
  cli();
  word p = wl_pulse;
  wl_pulse = 0;
  sei();
 
  if (p != 0) {
    if (nextPulse(p)) {
      if (debug) {
        Serial.print(F("OregonDecoderV2: "));
        reportSerial();
      }
      decodeTempHumidity(temp, hum, battOK);
      channel  = data[2];
      sensorID = data[3];
      return true;
    }
  }
  return false;  
}

void OregonDecoderV2::decodeTempHumidity(int& temp, byte& hum, bool& battOK) {
  if (pos >= 8) {
    int Type = (data[0] << 8) | data[1];
    if (isSummOK(Type)) {
      int t = data[5] >> 4;                                   // 1st decimal digit
      t *= 10;
      t += data[5] & 0x0F;                                    // 2nd decimal digit
      t *= 10;
      t += data[4] >> 4;                                      // 3rd decimal digit
      if (data[6] & 0x08) t *= -1;
      temp = t;
      hum = 0;
      battOK = !(data[4] & 0x0C);
      if (Type == 0x1A2D) {                                   // THGR2228N
        hum  = data[7] & 0xF;
        hum *= 10;
        hum += data[6] >> 4;
      }
    }
  }
  resetDecoder();
}

int OregonDecoderV2::sum(byte count, const byte* data) {
  int s = 0;
 
  for(byte i = 0; i<count;i++) {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}

bool OregonDecoderV2::isSummOK(int sensorType) {
  byte s1 = 0;
  byte s2 = 0;

  switch (sensorType) {
    case 0x1A2D:                                // THGR2228N
      s1 = (sum(8, data) - 0xa) & 0xFF;
      return (data[8] == s1);
    case 0xEA4C:                                // TNHN132N
      s1 = (sum(6, data) + (data[6]&0xF) - 0xa) & 0xff;
      s2 = (s1 & 0xF0) >> 4;
      s1 = (s1 & 0x0F) << 4;
      return ((s1 == data[6]) && (s2 == data[7]));
    default:
      break;
  }
  return false;
}
 