#ifndef HYT939_H
#define HYT939_H

#include <Arduino.h>
#include <Wire.h>

class HYT939
{
public:
    HYT939();
    /*

    // default HYT address is 0x28
    bool begin(TwoWire &wire = Wire, uint8_t address = 0x28);

    // Read full measurement (status + humidity + temperature)
    // status: 0=normal, 1=stale/old data, 2=command mode, 3=diagnostic
    bool read(uint8_t &status, float &rh_percent, float &temp_c);

    // Low-level read (raw 4 bytes)
    bool readRaw(uint8_t data[4]);

    // Convenience functions
    float readHumidityPercent(uint8_t *outStatus = nullptr);
    float readTemperatureC(uint8_t *outStatus = nullptr);

    */
    bool begin(TwoWire &wire, uint8_t address = 0x28);
    bool trigger();
    bool readResult(uint8_t &status, float &rh_percent, float &temp_c);

private:
    TwoWire *m_wire;
    uint8_t m_addr;

    // Convert 4 bytes into values
    static uint8_t  parseStatus(const uint8_t d[4]);
    static uint16_t parseHumidityRaw14(const uint8_t d[4]);
    static uint16_t parseTempRaw14(const uint8_t d[4]);

    static float humidityFromRaw(uint16_t raw14);
    static float tempFromRaw(uint16_t raw14);
};

#endif