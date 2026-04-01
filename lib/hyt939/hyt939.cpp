#include "hyt939.h"

HYT939::HYT939() : m_wire(nullptr), m_addr(0x28) {}

bool HYT939::begin(TwoWire &wire, uint8_t address)
{
    m_wire = &wire;
    m_addr = address;
    return true;
}

bool HYT939::trigger()
{
    if (!m_wire)
        return false;

    m_wire->beginTransmission(m_addr);
    m_wire->write(0x00);  // trigger measurement
    return (m_wire->endTransmission() == 0);
}

bool HYT939::readResult(uint8_t &status, float &rh_percent, float &temp_c)
{
    if (!m_wire)
        return false;

    uint8_t d[4];

    int received = m_wire->requestFrom((int)m_addr, 4);
    if (received < 4 || m_wire->available() < 4)
        return false;

    for (int i = 0; i < 4; i++)
        d[i] = m_wire->read();

    status = parseStatus(d);

    uint16_t rh14   = parseHumidityRaw14(d);
    uint16_t temp14 = parseTempRaw14(d);

    rh_percent = humidityFromRaw(rh14);
    temp_c     = tempFromRaw(temp14);

    return true;
}

uint8_t HYT939::parseStatus(const uint8_t d[4])
{
    return (d[0] >> 6) & 0x03;
}

uint16_t HYT939::parseHumidityRaw14(const uint8_t d[4])
{
    return ((uint16_t)(d[0] & 0x3F) << 8) | d[1];
}

uint16_t HYT939::parseTempRaw14(const uint8_t d[4])
{
    return ((uint16_t)d[2] << 6) | ((d[3] >> 2) & 0x3F);
}

float HYT939::humidityFromRaw(uint16_t raw14)
{
    return (100.0f * raw14) / 16383.0f;
}

float HYT939::tempFromRaw(uint16_t raw14)
{
    return (165.0f * raw14) / 16383.0f - 40.0f;
}