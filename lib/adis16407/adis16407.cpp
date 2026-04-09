#include "adis16407.h"

ADIS16407::ADIS16407()
    : m_spi(nullptr),
      m_csPin(-1),
      m_settings(100000, MSBFIRST, SPI_MODE3)
{
}

bool ADIS16407::begin(SPIClass &spi, int csPin, uint32_t freqHz)
{
    m_spi = &spi;
    m_csPin = csPin;

    pinMode(m_csPin, OUTPUT);
    digitalWrite(m_csPin, HIGH);

    // ADIS uses MODE3; 100 kHz is safe for long wires
    m_settings = SPISettings(freqHz, MSBFIRST, SPI_MODE3);

    return true;
}

void ADIS16407::primePipeline(uint8_t dummyReads)
{
    // ADIS reads are pipelined; discard first replies after reset
    for (uint8_t i = 0; i < dummyReads; i++) {
        (void)readRegister(REG_PROD_ID);
        delay(2);
    }
}

uint16_t ADIS16407::readRegister(uint8_t reg)
{
    // Pipelined read:
    // frame1: send address, ignore reply
    // frame2: send address again, reply contains requested register
    const uint16_t cmd = (uint16_t(reg) << 8);

    m_spi->beginTransaction(m_settings);
    csLow();
    delayMicroseconds(2);

    (void)m_spi->transfer16(cmd);
    uint16_t val = m_spi->transfer16(cmd);

    delayMicroseconds(2);
    csHigh();
    m_spi->endTransaction();

    return val;
}


int16_t ADIS16407::signExtend14(uint16_t v)
{
    uint16_t d = v & 0x3FFF;          // keep bits [13:0]
    if (d & 0x2000) d |= 0xC000;      // sign extend bit13
    return (int16_t)d;
}

uint16_t ADIS16407::pack14(int16_t s14)
{
    // clamp to 14-bit signed range
    if (s14 >  8191) s14 =  8191;
    if (s14 < -8192) s14 = -8192;
    return (uint16_t)s14 & 0x3FFF;
}

// --- Signed 12 ---
static int16_t to_signed12(uint16_t v)
{
    v &= 0x0FFF;                 // keep bits [11:0]
    if (v & 0x0800) v |= 0xF000; // sign extend 12->16
    return (int16_t)v;
}

// --- Unsigned 12 ---
static uint16_t to_u12(uint16_t v)
{
    return (v & 0x0FFF);
}


// --- Identification ---
uint16_t ADIS16407::readProdId()     { return readRegister(REG_PROD_ID); }
uint16_t ADIS16407::readSerialNum()  { return readRegister(REG_SERIAL_NUM); }
uint16_t ADIS16407::readLotId1()     { return readRegister(REG_LOT_ID1); }
uint16_t ADIS16407::readLotId2()     { return readRegister(REG_LOT_ID2); }


// --- Temperature ---
int16_t ADIS16407::readTempRaw()
{
    uint16_t reg = readRegister(REG_TEMP_OUT);
    return to_signed12(reg);
}

float ADIS16407::readTempC()
{
    // Datasheet: 25 C -> 0x000, scale = 0.136 C/LSB
    int16_t raw = readTempRaw();
    return 25.0f + (raw * 0.136f);
}


// --- Power supply ---
uint16_t ADIS16407::readSupplyRaw()
{
    uint16_t reg = readRegister(REG_SUPPLY_OUT);
    return to_u12(reg);
}

float ADIS16407::readSupplyV()
{
    // Datasheet: 2.418 mV/LSB
    uint16_t raw = readSupplyRaw();
    return raw * 0.002418f; // volts
}


// --- Gyroscope --- 
int16_t ADIS16407::dpsToGyroOffsetLsb(float dps)
{
    // Offset regs are 0.0125 dps/LSB
    float lsb_f = dps / GYRO_OFF_DPS_PER_LSB;
    long lsb = lroundf(lsb_f);
    if (lsb >  8191) lsb =  8191;
    if (lsb < -8192) lsb = -8192;
    return (int16_t)lsb;
}

bool ADIS16407::writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t hi = value >> 8;
    uint8_t lo = value & 0xFF;

    uint16_t cmd_hi = 0x8000 | ((reg + 1) << 8) | hi;
    uint16_t cmd_lo = 0x8000 | ( reg      << 8) | lo;

    m_spi->beginTransaction(m_settings);
    csLow();

    m_spi->transfer16(cmd_hi);
    m_spi->transfer16(cmd_lo);

    csHigh();
    m_spi->endTransaction();

    return true;
}

bool ADIS16407::readGyroRaw(int16_t &x, int16_t &y, int16_t &z)
{
    uint16_t xr = readRegister(REG_XGYRO_OUT);
    if (!(xr & 0x8000)== 0x8000){
      Serial.print(" X ND Error");
    }
    uint16_t yr = readRegister(REG_YGYRO_OUT);
        if (!(yr & 0x8000)== 0x8000){
      Serial.print(" Y ND Error");
    }
    uint16_t zr = readRegister(REG_ZGYRO_OUT);
        if (!(zr & 0x8000)== 0x8000){
      Serial.print(" Z ND Error");
    }

    x = signExtend14(xr);
    y = signExtend14(yr);
    z = signExtend14(zr);
    return true;
}

bool ADIS16407::readGyroDps(float &x_dps, float &y_dps, float &z_dps)
{
    int16_t x, y, z;
    if (!readGyroRaw(x, y, z)) return false;

    x_dps = x * GYRO_OUT_DPS_PER_LSB;
    y_dps = y * GYRO_OUT_DPS_PER_LSB;
    z_dps = z * GYRO_OUT_DPS_PER_LSB;

    return true;
}


bool ADIS16407::setGyroOffsetDps(float x_off_dps, float y_off_dps, float z_off_dps)
{
    // Convert desired offsets (dps) into offset-register LSBs
    int16_t x_lsb = dpsToGyroOffsetLsb(x_off_dps);
    int16_t y_lsb = dpsToGyroOffsetLsb(y_off_dps);
    int16_t z_lsb = dpsToGyroOffsetLsb(z_off_dps);

    uint16_t x_reg = pack14(x_lsb);
    uint16_t y_reg = pack14(y_lsb);
    uint16_t z_reg = pack14(z_lsb);

    // Write into sensor

    writeRegister(REG_XGYRO_OFF, x_reg);
    delay(2);
    writeRegister(REG_YGYRO_OFF, y_reg);
    delay(2);
    writeRegister(REG_ZGYRO_OFF, z_reg);
    delay(2);

/*     writeRegister(REG_XGYRO_OFF, 0x0);
    delay(2);
    writeRegister(REG_YGYRO_OFF, 0x0);
    delay(2);
    writeRegister(REG_ZGYRO_OFF, 0x0);
    delay(2); */

/*     Serial.print("Write data X:"); Serial.print(x_reg,HEX);
    Serial.print(" Y:"); Serial.print(y_reg,HEX);
    Serial.print(" Z:"); Serial.println(z_reg,HEX); */



    return true;
}


bool ADIS16407::calibrateGyroOffsets(uint16_t samples, uint16_t sampleDelayMs)
{

    // Clean the REG first (tested)
    writeRegister(REG_XGYRO_OFF, 0x0000);
    delay(2);
    writeRegister(REG_YGYRO_OFF, 0x0000);
    delay(2);
    writeRegister(REG_ZGYRO_OFF, 0x0000);
    delay(2);
    // Measure bias in DPS while sensor is stationary
    double sx = 0, sy = 0, sz = 0;

/*     for (int i = 0; i < 100; i++) {
      (void)readRegister(REG_XGYRO_OUT);
      (void)readRegister(REG_YGYRO_OUT);
      (void)readRegister(REG_ZGYRO_OUT);
      delay(2);
    } */

    for (uint16_t i = 0; i < samples; i++) {
        float gx, gy, gz;
        readGyroDps(gx, gy, gz);
        sx += gx; sy += gy; sz += gz;
        delay(sampleDelayMs);
    }

    float bx = 0;
    float by = 0;
    float bz = 0;

    bx = (float)(sx / samples);
    by = (float)(sy / samples);
    bz = (float)(sz / samples);


/*     Serial.print(" X:");          Serial.print(bx,2);
    Serial.print(" Y:");          Serial.print(by, 2);
    Serial.print(" Z:");          Serial.println(bz, 2); */

    // Write opposite polarity into offset regs
    return setGyroOffsetDps(-bx, -by, -bz);
}

bool ADIS16407::readGyroOffsetRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb)
{
    uint16_t xr = readRegister(REG_XGYRO_OFF);
    uint16_t yr = readRegister(REG_YGYRO_OFF);
    uint16_t zr = readRegister(REG_ZGYRO_OFF);

/*     Serial.print("Read data X:"); Serial.print(xr,HEX);
    Serial.print(" Y:"); Serial.print(yr,HEX);
    Serial.print(" Z:"); Serial.println(zr,HEX); */

    x_off_lsb = signExtend14(xr);
    y_off_lsb = signExtend14(yr);
    z_off_lsb = signExtend14(zr);
    return true;
}



// --- Accelerometer --- 
int16_t ADIS16407::gToAccelOffsetLsb(float g)
{
    // 3.333 mg/LSB = 0.003333 g/LSB
    float lsb_f = g / ACCL_G_PER_LSB;
    long lsb = lroundf(lsb_f);

    if (lsb >  8191) lsb =  8191;
    if (lsb < -8192) lsb = -8192;

    return (int16_t)lsb;
}

bool ADIS16407::readAccelRaw(int16_t &x, int16_t &y, int16_t &z)
{
    uint16_t xr = readRegister(REG_XACCL_OUT);
    uint16_t yr = readRegister(REG_YACCL_OUT);
    uint16_t zr = readRegister(REG_ZACCL_OUT);

    x = signExtend14(xr);
    y = signExtend14(yr);
    z = signExtend14(zr);

    return true;
}

bool ADIS16407::readAccelG(float &x_g, float &y_g, float &z_g)
{
    int16_t x, y, z;
    if (!readAccelRaw(x, y, z)) return false;

    x_g = x * ACCL_G_PER_LSB;
    y_g = y * ACCL_G_PER_LSB;
    z_g = z * ACCL_G_PER_LSB;

    return true;
}

bool ADIS16407::setAccelOffsetG(float x_off_g, float y_off_g, float z_off_g)
{
    int16_t x_lsb = gToAccelOffsetLsb(x_off_g);
    int16_t y_lsb = gToAccelOffsetLsb(y_off_g);
    int16_t z_lsb = gToAccelOffsetLsb(z_off_g);

    uint16_t x_reg = pack14(x_lsb);
    uint16_t y_reg = pack14(y_lsb);
    uint16_t z_reg = pack14(z_lsb);

    writeRegister(REG_XACCL_OFF, x_reg);
    delay(2);
    writeRegister(REG_YACCL_OFF, y_reg);
    delay(2);
    writeRegister(REG_ZACCL_OFF, z_reg);
    delay(2);

    return true;
}

bool ADIS16407::calibrateAccelOffsets(uint16_t samples, uint16_t sampleDelayMs)
{
    // Clear offsets first
    writeRegister(REG_XACCL_OFF, 0x0000);
    delay(2);
    writeRegister(REG_YACCL_OFF, 0x0000);
    delay(2);
    writeRegister(REG_ZACCL_OFF, 0x0000);
    delay(2);

    double sx = 0, sy = 0, sz = 0;

    for (uint16_t i = 0; i < samples; i++) {
        float ax, ay, az;
        readAccelG(ax, ay, az);
        sx += ax; sy += ay; sz += az;
        delay(sampleDelayMs);
    }

    float bx = (float)(sx / samples);
    float by = (float)(sy / samples);
    float bz = (float)(sz / samples);

    // Target is (0, 0, +1g)
    float ex = bx - 0.0f;
    float ey = by - 0.0f;
    float ez = bz - 1.0f;

    // Write opposite polarity to cancel it
    return setAccelOffsetG(-ex, -ey, -ez);
}

bool ADIS16407::readAccelOffsetRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb)
{
    uint16_t xr = readRegister(REG_XACCL_OFF);
    uint16_t yr = readRegister(REG_YACCL_OFF);
    uint16_t zr = readRegister(REG_ZACCL_OFF);

    x_off_lsb = signExtend14(xr);
    y_off_lsb = signExtend14(yr);
    z_off_lsb = signExtend14(zr);

    return true;
}



// --- Magnetometer ---
int16_t ADIS16407::gaussToMagHardIronLsb(float gauss)
{
    // 0.5 mGauss/LSB = 0.0005 gauss/LSB
    float lsb_f = gauss / MAGN_GAUSS_PER_LSB;
    long lsb = lroundf(lsb_f);

    if (lsb >  8191) lsb =  8191;
    if (lsb < -8192) lsb = -8192;

    return (int16_t)lsb;
}

uint16_t ADIS16407::scaleToMagSoftIronReg(float scale)
{
    // Datasheet: 0x0800 = 100% (1.0). So reg ~= scale*2048
    long reg = lroundf(scale * 2048.0f);

    if (reg < 0) reg = 0;
    if (reg > 0x0FFF) reg = 0x0FFF; // 12-bit max

    return (uint16_t)reg & 0x0FFF;
}

bool ADIS16407::readMagRaw(int16_t &x, int16_t &y, int16_t &z)
{
    uint16_t xr = readRegister(REG_XMAGN_OUT);
    uint16_t yr = readRegister(REG_YMAGN_OUT);
    uint16_t zr = readRegister(REG_ZMAGN_OUT);

    x = signExtend14(xr);
    y = signExtend14(yr);
    z = signExtend14(zr);

    return true;
}

bool ADIS16407::readMagGauss(float &x_gs, float &y_gs, float &z_gs)
{
    int16_t x, y, z;
    if (!readMagRaw(x, y, z)) return false;

    x_gs = x * MAGN_GAUSS_PER_LSB;
    y_gs = y * MAGN_GAUSS_PER_LSB;
    z_gs = z * MAGN_GAUSS_PER_LSB;

    return true;
}

bool ADIS16407::setMagHardIronGauss(float x_off_gs, float y_off_gs, float z_off_gs)
{
    int16_t x_lsb = gaussToMagHardIronLsb(x_off_gs);
    int16_t y_lsb = gaussToMagHardIronLsb(y_off_gs);
    int16_t z_lsb = gaussToMagHardIronLsb(z_off_gs);

    uint16_t x_reg = pack14(x_lsb);
    uint16_t y_reg = pack14(y_lsb);
    uint16_t z_reg = pack14(z_lsb);

    // Clear offsets first
    writeRegister(REG_XMAGN_HIC, 0x0000);
    delay(2);
    writeRegister(REG_YMAGN_HIC, 0x0000);
    delay(2);
    writeRegister(REG_ZMAGN_HIC, 0x0000);
    delay(2);

    writeRegister(REG_XMAGN_HIC, x_reg);
    delay(2);
    writeRegister(REG_YMAGN_HIC, y_reg);
    delay(2);
    writeRegister(REG_ZMAGN_HIC, z_reg);
    delay(2);

    return true;
}

bool ADIS16407::setMagSoftIronScale(float x_scale, float y_scale, float z_scale)
{
    uint16_t x_reg = scaleToMagSoftIronReg(x_scale);
    uint16_t y_reg = scaleToMagSoftIronReg(y_scale);
    uint16_t z_reg = scaleToMagSoftIronReg(z_scale);

    // Clear offsets first
    writeRegister(REG_XMAGN_SIC, 0x0000);
    delay(2);
    writeRegister(REG_YMAGN_SIC, 0x0000);
    delay(2);
    writeRegister(REG_ZMAGN_SIC, 0x0000);
    delay(2);

    // Stored in bits [11:0] (binary format). Upper bits unused -> write as-is.
    writeRegister(REG_XMAGN_SIC, x_reg);
    delay(2);
    writeRegister(REG_YMAGN_SIC, y_reg);
    delay(2);
    writeRegister(REG_ZMAGN_SIC, z_reg);
    delay(2);

    return true;
}

bool ADIS16407::readMagHardIronRegs(int16_t &x_off_lsb, int16_t &y_off_lsb, int16_t &z_off_lsb)
{
    uint16_t xr = readRegister(REG_XMAGN_HIC);
    uint16_t yr = readRegister(REG_YMAGN_HIC);
    uint16_t zr = readRegister(REG_ZMAGN_HIC);

    x_off_lsb = signExtend14(xr);
    y_off_lsb = signExtend14(yr);
    z_off_lsb = signExtend14(zr);

    return true;
}

bool ADIS16407::readMagSoftIronRegs(uint16_t &x_scale_raw, uint16_t &y_scale_raw, uint16_t &z_scale_raw)
{
    x_scale_raw = readRegister(REG_XMAGN_SIC) & 0x0FFF;
    y_scale_raw = readRegister(REG_YMAGN_SIC) & 0x0FFF;
    z_scale_raw = readRegister(REG_ZMAGN_SIC) & 0x0FFF;
    return true;
}


// --- Barometer ---
uint16_t ADIS16407::readBaroOutRaw14()
{
    uint16_t reg = readRegister(REG_BARO_OUT);
    return (reg & 0x3FFF); // bits [13:0] unsigned
}

uint8_t ADIS16407::readBaroOutLRaw8()
{
    uint16_t reg = readRegister(REG_BARO_OUTL);
    return (uint8_t)(reg & 0x00FF); // bits [7:0]
}

uint32_t ADIS16407::readBaroRaw24()
{
    uint16_t baro14 = readBaroOutRaw14();
    uint8_t  baroL  = readBaroOutLRaw8();

    // 24-bit result: (BARO_OUT << 8) + BARO_OUTL
    return ((uint32_t)baro14 << 8) | (uint32_t)baroL;
}

float ADIS16407::readBaroMbar()
{
    uint32_t raw24 = readBaroRaw24();
    return raw24 * BARO_MBAR_PER_LSB_FINE;
}

float ADIS16407::readBaroMbarCoarse()
{
    uint16_t raw14 = readBaroOutRaw14();
    return raw14 * BARO_MBAR_PER_LSB_COARSE;
}