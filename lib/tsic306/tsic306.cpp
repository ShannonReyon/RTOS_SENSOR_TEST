#include "tsic306.h"

static constexpr uint32_t EDGE_TIMEOUT_US_SHORT = 10000;   // 10ms within a frame
static constexpr uint32_t EDGE_TIMEOUT_US_LONG  = 200000;  // 200ms between frames

TSIC306::TSIC306(int pin) : m_pin(pin) {}

void TSIC306::begin()
{
  pinMode(m_pin, INPUT_PULLUP); // idle is HIGH
}

bool TSIC306::waitForLevel(bool level, uint32_t timeoutUs)
{
  uint32_t t0 = micros();
  int want = level ? HIGH : LOW;

  while (digitalRead(m_pin) != want) {
    if ((micros() - t0) > timeoutUs) return false;
  }
  return true;
}

bool TSIC306::waitForFallingEdge(uint32_t timeoutUs)
{
  uint32_t t0 = micros();

  // ensure we're HIGH first
  while (digitalRead(m_pin) == LOW) {
    if ((micros() - t0) > timeoutUs) return false;
  }

  // wait for falling edge
  while (digitalRead(m_pin) == HIGH) {
    if ((micros() - t0) > timeoutUs) return false;
  }

  return true; // now LOW => falling edge happened
}

uint8_t TSIC306::parityOddOf8(uint8_t v)
{
  // returns 1 if number of 1-bits is odd
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;
  return v & 1;
}

bool TSIC306::checkEvenParity(uint8_t dataByte, uint8_t parityBit)
{
  // even parity => total ones (data + parity) is even
  // if data has odd ones => parity must be 1
  // if data has even ones => parity must be 0
  return parityBit == parityOddOf8(dataByte);
}

bool TSIC306::readPacket(uint8_t &dataByte,
                         uint8_t &parityBit,
                         uint32_t &tStrobe_us,
                         uint32_t firstEdgeTimeoutUs)
{
  dataByte = 0;
  parityBit = 0;

  // Wait for start bit falling edge
  if (!waitForFallingEdge(firstEdgeTimeoutUs)) return false;

  // Measure Tstrobe (falling -> rising)
  uint32_t t0 = micros();
  if (!waitForLevel(true, EDGE_TIMEOUT_US_SHORT)) return false;
  tStrobe_us = micros() - t0;

  // Read 8 data bits (MSB first)
  for (int i = 0; i < 8; i++) {
    if (!waitForFallingEdge(EDGE_TIMEOUT_US_SHORT)) return false;

    // sample middle of bit window
    delayMicroseconds(tStrobe_us);

    int bit = (digitalRead(m_pin) == HIGH) ? 1 : 0;
    dataByte = (dataByte << 1) | (uint8_t)bit;

    if (!waitForLevel(true, EDGE_TIMEOUT_US_SHORT)) return false;
  }

  // Parity bit
  if (!waitForFallingEdge(EDGE_TIMEOUT_US_SHORT)) return false;
  delayMicroseconds(tStrobe_us);
  parityBit = (digitalRead(m_pin) == HIGH) ? 1 : 0;
  if (!waitForLevel(true, EDGE_TIMEOUT_US_SHORT)) return false;

  // Verify parity before success
  if (!checkEvenParity(dataByte, parityBit)) return false;

  return true;
}

bool TSIC306::readRaw11(uint16_t &raw11, uint32_t &tStrobe_us)
{
  uint8_t b_hi = 0, b_lo = 0;
  uint8_t p1 = 0, p2 = 0;
  uint32_t t1 = 0, t2 = 0;

  // Packet 1: allow long wait (new frame)
  if (!readPacket(b_hi, p1, t1, EDGE_TIMEOUT_US_LONG)) return false;

  // Packet 2: should come shortly after
  if (!readPacket(b_lo, p2, t2, EDGE_TIMEOUT_US_SHORT)) return false;

  // Save Tstrobe from first packet
  tStrobe_us = t1;

  // Build 11-bit value:
  // b_hi lower 3 bits => bits [10:8]
  // b_lo all 8 bits   => bits [7:0]
  raw11 = (uint16_t(b_hi & 0x07) << 8) | uint16_t(b_lo);

  return true;
}

float TSIC306::raw11ToC(uint16_t raw11)
{
  // TSic306: -50..+150C mapped to 0..2047
  return (float(raw11) * 200.0f / 2047.0f) - 50.0f;
}

bool TSIC306::readTempC(float &tempC, uint16_t &raw11, uint32_t &tStrobe_us)
{
  if (!readRaw11(raw11, tStrobe_us)) return false;
  tempC = raw11ToC(raw11);
  return true;
}