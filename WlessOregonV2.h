#ifndef WlessOregonV2_h
#define WlessOregonV2_h

// Oregon V2 decoder modfied - Olivier Lebrun
// Oregon V2 decoder added - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $

//--------------------------------------- digitalWrite using direct port manipulation ---------------------
class directPort {
  public:
    directPort() { }
    virtual void sendOne(void);
    virtual void sendZero(void);
    virtual void selectPin(byte pin) {
      set_mask = 0;
      if (pin <= 7) {
        set_mask = 1 << pin;
        clr_mask = ~set_mask;
      }
    }
  protected:
    byte set_mask;
    byte clr_mask;
  public:
    const unsigned long TIME    = 512;
    const unsigned long TWOTIME = TIME*2;
};

class directPortB : public directPort {
  public:
    directPortB() { }
    virtual void sendOne(void);
    virtual void sendZero(void);
};

class directPortD : public directPort {
  public:
    directPortD() { }
    virtual void sendOne(void);
    virtual void sendZero(void);
};

//----------------------------------- Send the temp & humidity using oregon v2.1 protocol -------------
class OregonSensor {
  public:
    OregonSensor(byte txPin, byte channel, byte sensorID, bool Humidity = false) {
      tx_pin = txPin;
      if (tx_pin <= 7) {                        // PORTD
        dpD.selectPin(tx_pin);
        dp = &dpD;
      } else {
        if (tx_pin <= 13) {
          dpB.selectPin(tx_pin - 8);
          dp = &dpB;
        }
      }
      existsHumidity = Humidity;
      int type = 0xEA4C;                        // by default emulate TNHN132N
      if (existsHumidity)  type = 0x1A2D;       // emulate THGR2228N
        
      buffer[0] = type >> 8;
      buffer[1] = type & 0xFF;
      buffer[2] = channel;
      buffer[3] = sensorID;
    }
    void init(void);
    void sendTempHumidity(int temp, byte humm, bool battery);
  private:
    inline void sendOne(void)                   { dp->sendOne(); }
    inline void sendZero(void)                  { dp->sendZero(); }
    void sendData(const byte *data, byte size); // Send data buffer
    void sendPreamble(void);                    // Send preamble
    void sendPostamble(void);                   // Send postamble
    void sendOregon(void);                      // Send preamble, data, postamble
    void sendSync(void);
    int  sum(byte count);                       // Count the buffer summ
    void calculateAndSetChecksum(void);
    bool existsHumidity;                        // Weither THGR2228N (send Humidity)
    byte buffer[9];
    byte tx_pin;
    directPortB dpB;
    directPortD dpD;
    directPort* dp;
    const unsigned long TIME    = 512;
    const unsigned long TWOTIME = TIME*2;
};

//------------------------------------------ class DecodeOOK ------------------------------------------------------
class DecodeOOK {
  public:
    enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };
 
    DecodeOOK(void)                             { resetDecoder(); }
    void         begin(byte int_num);
    virtual void gotBit(char value);            // add one bit to the packet data buffer
    bool         isDone (void) const            { return state == DONE; }
    const byte*  getData(byte& count) const     { count = pos;  return data; }
    bool         nextPulse(word width);
    void         resetDecoder(void);
    void         manchester(char value);        // store a bit using Manchester encoding
    void         alignTail(byte max = 0);       // move bits to the front so that all the bits are aligned to the end
    void         reverseBits(void);
    void         reverseNibbles (void);
    void         done(void);
    void         reportSerial(void);
  protected:
    virtual char decode (word width) = 0;
    static void interuptHandler(void);
    byte total_bits, bits, flip, state, pos, data[25];
};

//------------------------------------------ class OregonDecoderV2, decode Oregon station signals ----------------
class OregonDecoderV2 : public DecodeOOK {
  public:   
    OregonDecoderV2()                           {}
    virtual void gotBit(char value);            // add one bit to the packet data buffer
    virtual char decode(word width);
    // Check and retrieve the useful data
    bool         receiveData(byte& cnannel, byte& sensorID, int& temp, byte& hum, bool& battOK, bool debug = false);
  private:
    void         decodeTempHumidity(int& temp, byte& hum, bool& battOK);
    int          sum(byte count, const byte* data);
    bool         isSummOK(int sensorType);
};

#endif