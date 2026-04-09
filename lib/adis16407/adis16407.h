#ifndef ADIS16407_H
#define ADIS16407_H

#include <Arduino.h>
#include <SPI.h>

class ADIS16407
{
public:
    ADIS16407();

    // Initialize driver (SPI already begun in main)
    // Default: 100 kHz, MODE3 (stable with long wires)
    bool begin(SPIClass &spi, int csPin, uint32_t freqHz = 100000);

    // Prime pipelined SPI read after boot/reset
    void primePipeline(uint8_t dummyReads = 3);

    // --- Identification ---
    uint16_t readProdId();
    uint16_t readSerialNum();
    uint16_t readLotId1();
    uint16_t readLotId2();

    // --- Temperature ---
    int16_t  readTempRaw();
    float    readTempC();

    // --- Power supply ---
    uint16_t readSupplyRaw();
    float    readSupplyV();

    // --- Gyroscope ---
    bool readGyroRaw(int16_t &x, int16_t &y, int16_t &z);
    bool readGyroDps(float &x_dps, float &y_dps, float &z_dps);

    // Write hardware gyro offsets (in dps) into XGYRO_OFF/YGYRO_OFF/ZGYRO_OFF
    bool setGyroOffsetDps(float x_off_dps, float y_off_dps, float z_off_dps);

    // Convenience: measure bias while still, then write offsets into the sensor
    bool calibrateGyroOffsets(uint16_t samples = 300, uint16_t sampleDelayMs = 5);

    // read back raw offset registers (debug)
    bool readGyroOffsetRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb);


    // --- Accelerometer ---
    bool readAccelRaw(int16_t &x, int16_t &y, int16_t &z);
    bool readAccelG(float &x_g, float &y_g, float &z_g);

    // Hardware accel offsets (in g) into XACCL_OFF/YACCL_OFF/ZACCL_OFF
    bool setAccelOffsetG(float x_off_g, float y_off_g, float z_off_g);

    // Convenience: measure bias while still (assumes Z should be +1g), then write offsets
    bool calibrateAccelOffsets(uint16_t samples = 300, uint16_t sampleDelayMs = 5);

    // read back raw offset registers (debug)
    bool readAccelOffsetRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb);


    // --- Magnetometer ---
    bool readMagRaw(int16_t &x, int16_t &y, int16_t &z);
    bool readMagGauss(float &x_gs, float &y_gs, float &z_gs);

    // Hard-iron correction offsets (in gauss) -> XMAGN_HIC/YMAGN_HIC/ZMAGN_HIC
    bool setMagHardIronGauss(float x_off_gs, float y_off_gs, float z_off_gs);

    // Soft-iron correction scale factors (unitless; 1.0 = 100%) -> XMAGN_SIC/YMAGN_SIC/ZMAGN_SIC
    bool setMagSoftIronScale(float x_scale, float y_scale, float z_scale);

    // Read back correction registers (debug)
    bool readMagHardIronRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb);
    bool readMagSoftIronRegs(uint16_t &x_scale_raw, uint16_t &y_scale_raw, uint16_t &z_scale_raw);


    // --- Barometer ---
    uint16_t readBaroOutRaw14();     // BARO_OUT [13:0]
    uint8_t  readBaroOutLRaw8();     // BARO_OUTL [7:0]
    uint32_t readBaroRaw24();        // (BARO_OUT<<8) | BARO_OUTL
    float    readBaroMbar();         // high-resolution mbar
    float    readBaroMbarCoarse();   // BARO_OUT only (0.08 mbar/LSB)

private:
    // Pipelined register read (2 frames with CS held low)
    uint16_t readRegister(uint8_t reg);

    // 16-bit register write (two byte writes)
    bool writeRegister(uint8_t reg, uint16_t value);

    // Helpers for signed 14-bit fields used by gyro outputs and offset regs
    static int16_t signExtend14(uint16_t v14);
    static uint16_t pack14(int16_t s14);

    // Convert bias/offset between dps and register units
    static int16_t dpsToGyroOffsetLsb(float dps); // 0.0125 dps/LSB

    static int16_t gToAccelOffsetLsb(float g); // 3.333 mg/LSB

    static int16_t gaussToMagHardIronLsb(float gauss);   // 0.0005 gauss/LSB
    static uint16_t scaleToMagSoftIronReg(float scale);  // reg ≈ scale*2048 (12-bit)

    inline void csLow()  { digitalWrite(m_csPin, LOW); }
    inline void csHigh() { digitalWrite(m_csPin, HIGH); }

private:
    SPIClass *m_spi;
    int m_csPin;
    SPISettings m_settings;

    // Register map (base addresses)
    static constexpr uint8_t REG_LOT_ID1    = 0x52;
    static constexpr uint8_t REG_LOT_ID2    = 0x54;
    static constexpr uint8_t REG_PROD_ID    = 0x56;
    static constexpr uint8_t REG_SERIAL_NUM = 0x58;

    static constexpr uint8_t REG_TEMP_OUT   = 0x1A;

    static constexpr uint8_t REG_SUPPLY_OUT = 0x02;

    // Gyro outputs
    static constexpr uint8_t REG_XGYRO_OUT  = 0x04;
    static constexpr uint8_t REG_YGYRO_OUT  = 0x06;
    static constexpr uint8_t REG_ZGYRO_OUT  = 0x08;

    // Gyro offset registers (user programmable)
    static constexpr uint8_t REG_XGYRO_OFF  = 0x1E;
    static constexpr uint8_t REG_YGYRO_OFF  = 0x20;
    static constexpr uint8_t REG_ZGYRO_OFF  = 0x22;

    // Accel outputs
    static constexpr uint8_t REG_XACCL_OUT  = 0x0A;
    static constexpr uint8_t REG_YACCL_OUT  = 0x0C;
    static constexpr uint8_t REG_ZACCL_OUT  = 0x0E;

    // Accel offset registers (user programmable)
    static constexpr uint8_t REG_XACCL_OFF  = 0x24;
    static constexpr uint8_t REG_YACCL_OFF  = 0x26;
    static constexpr uint8_t REG_ZACCL_OFF  = 0x28;

    // Mag outputs
    static constexpr uint8_t REG_XMAGN_OUT = 0x10;
    static constexpr uint8_t REG_YMAGN_OUT = 0x12;
    static constexpr uint8_t REG_ZMAGN_OUT = 0x14;

    // Mag hard-iron correction (offset) registers
    static constexpr uint8_t REG_XMAGN_HIC = 0x2A;
    static constexpr uint8_t REG_YMAGN_HIC = 0x2C;
    static constexpr uint8_t REG_ZMAGN_HIC = 0x2E;

    // Mag soft-iron correction (scale) registers
    static constexpr uint8_t REG_XMAGN_SIC = 0x30;
    static constexpr uint8_t REG_YMAGN_SIC = 0x32;
    static constexpr uint8_t REG_ZMAGN_SIC = 0x34;

    // Baro outputs
    static constexpr uint8_t REG_BARO_OUT  = 0x16;
    static constexpr uint8_t REG_BARO_OUTL = 0x18;



    // Scale factors Gyro
    static constexpr float GYRO_OUT_DPS_PER_LSB   = 0.05f;    // X/Y/ZGYRO_OUT
    static constexpr float GYRO_OFF_DPS_PER_LSB   = 0.0125f;  // X/Y/ZGYRO_OFF

    // Scale factors Accel
    static constexpr float ACCL_MG_PER_LSB = 3.333f;  // mg/LSB
    static constexpr float ACCL_G_PER_LSB  = 0.003333f; // g/LSB (mg/1000)

    // Scale factors Magnetometer
    static constexpr float MAGN_GAUSS_PER_LSB = 0.0005f; // 0.5 mGauss/LSB

    // Scale factors Barometer
    static constexpr float BARO_MBAR_PER_LSB_COARSE = 0.08f;        // using BARO_OUT only
    static constexpr float BARO_MBAR_PER_LSB_FINE   = 0.0003125f;   // using BARO_OUT + BARO_OUTL (24-bit)
};

#endif