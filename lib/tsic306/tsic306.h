#ifndef TSIC306_H
#define TSIC306_H

#include <Arduino.h>

class TSIC306
{
public:
  explicit TSIC306(int pin);

  void begin();

  // STEP1: read one packet (8-bit data + parity)
  bool readPacket(uint8_t &dataByte,
                  uint8_t &parityBit,
                  uint32_t &tStrobe_us,
                  uint32_t firstEdgeTimeoutUs);

  // STEP2: read two packets and build 11-bit raw value
  bool readRaw11(uint16_t &raw11, uint32_t &tStrobe_us);

  // parity helper
  bool checkEvenParity(uint8_t dataByte, uint8_t parityBit);

  // STEP3: convert raw11 to temperature
  static float raw11ToC(uint16_t raw11);

  // Convenience: read and convert in one call
  bool readTempC(float &tempC, uint16_t &raw11, uint32_t &tStrobe_us);

private:
  bool waitForLevel(bool level, uint32_t timeoutUs);
  bool waitForFallingEdge(uint32_t timeoutUs);

  static uint8_t parityOddOf8(uint8_t v); // returns 1 if odd number of ones

private:
  int m_pin;
};

#endif