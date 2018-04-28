/*
 * File description here
 */
#include "VEML6075.h"

/* VEML6075 SLAVE ADDRESS AND FUNCTION DESCRIPTION */
#define REG_UV_CONF  0x00
#define REG_Reserved01  0x01
#define REG_Reserved02  0x02
#define REG_Reserved03  0x03
#define REG_Reserved04  0x04
#define REG_Reserved05  0x05
#define REG_Reserved06  0x06
#define REG_UVA_Data    0x07
#define REG_UVD_Data    0x08
#define REG_UVB_Data    0x09
#define REG_UVCOMP1_Data  0x0A
#define REG_UVCOMP2_Data  0x0B
#define REG_ID  0x0C

// Following magic numbers are from 
// VISHAY veml6075 Application Note 84339
// Page 6 
#define UVA_A_COEF  (2.22)
#define UVA_B_COEF  (1.33)
#define UVB_C_COEF  (2.95)
#define UVB_D_COEF  (1.74)
#define UVA_sensitivity (0.93)
#define UVA_CIE_sensitivity (0.093)
#define UVB_sensitivity (2.1)
#define UVB_CIE_sensitivity (0.21)
// from page 15
#define UVA_RESPONSIVITY (0.001461)
#define UVB_RESPONSIVITY (0.002591)

VEML6075::VEML6075(PinName sda, PinName scl, int addr) : m_i2c(sda, scl), m_addr(addr<<1) {
    // activate the peripheral
}

VEML6075::~VEML6075() { }

#if 0
//
// numdata is supposed to be 2
//
void VEML6075::cmdWrite(uint8_t cmd, uint8_t *data, uint8_t numdata)
{
   uint8_t buf[3] ;

    buf[0] = cmd ;
    buf[1] = data[0] ;
    buf[2] = data[1] ;
    writeRegs(buf, 3) ;
}

//
// numdata is supposed to be 2
//

void VEML6075::cmdRead(uint8_t cmd, uint8_t *data, uint8_t numdata)
{
    uint8_t buf[1] ;
    buf[0] = cmd ;
    m_i2c.write(m_addr, buf, 1, true) ; // writing command 
    m_i2c.read(m_addr, (char*)data, numdata) ;
}
#endif 

void VEML6075::getUVConf(uint8_t *uvconf)
{
    uint8_t data[2] ;
    readRegs(REG_UV_CONF, data, 2) ;
    *uvconf = data[0] ;
}

void VEML6075::setUVConf(uint8_t uvconf)
{
     uint8_t data[3] ;
     data[0] = REG_UV_CONF ;
     data[1] = uvconf ;
     data[2] = 0 ;
     writeRegs(data, 3) ;
}


void VEML6075::getUVAData(uint16_t *uvadata)
{
    uint8_t data[2] ;
    readRegs(REG_UVA_Data, data, 2) ;
    *uvadata = (data[1]<<8) | data[0] ;
}

void VEML6075::getUVBData(uint16_t *uvbdata)
{
    uint8_t data[2] ;
    readRegs(REG_UVB_Data, data, 2) ;
    *uvbdata = (data[1]<<8) | data[0] ;
}

void VEML6075::getUVDData(uint16_t *uvddata)
{
    uint8_t data[2] ;
    readRegs(REG_UVD_Data, data, 2) ;
    *uvddata = (data[1]<<8) | data[0] ;
}

float VEML6075::UVI(void)
{
    float uvi = 0.0 ;
    uint16_t uvacomp = 0.0 ;
    uint16_t uvbcomp = 0.0 ;
    uint16_t uva, uvb, uvd, uvcomp1, uvcomp2 ;
    getUVAData(&uva) ;
    getUVBData(&uvb) ;
    getUVDData(&uvd) ;
    getUVCOMP1Data(&uvcomp1) ;
    getUVCOMP2Data(&uvcomp2) ;
    uvacomp = (uva - uvd) - UVA_A_COEF * (uvcomp1 - uvd) - UVA_B_COEF * (uvcomp2 - uvd) ;
    uvbcomp = (uvb - uvd) - UVB_C_COEF * (uvcomp1 - uvd) - UVB_D_COEF * (uvcomp2 - uvd) ;
//    uvi = (((float)uvbcomp * (float)UVB_RESPONSIVITY) + ((float)uvacomp * (float)UVA_RESPONSIVITY)) / 2.0 * 15 ; // For demo
    uvi = (((float)uvbcomp * (float)UVB_RESPONSIVITY) + ((float)uvacomp * (float)UVA_RESPONSIVITY)) / 2.0 ;
    return( uvi ) ;
}

// usage
// fvalue = veml->getUVA() ;
// printf("%f", fvalue) ;
float VEML6075::getUVA(void) 
{
    uint16_t data ;
    float value ;
    getUVAData(&data) ;
    value = (float)data / (float)UVA_sensitivity  ;
    return( value ) ;
}

float VEML6075::getUVA_CIE(void) 
{
    uint16_t data ;
    float value ;
    getUVAData(&data) ;
    value = (float)data / (float)UVA_CIE_sensitivity  ;
    return( value ) ;
}

float VEML6075::getUVB(void)
{
    uint16_t data ;
    float value ;
    getUVBData(&data) ;
    value = (float)data / (float)UVB_sensitivity  ;
    return( value ) ;
}

float VEML6075::getUVB_CIE(void)
{
    uint16_t data ;
    float value ;
    getUVBData(&data) ;
    value = (float)data / (float)UVB_CIE_sensitivity  ;
    return( value ) ;
}

void VEML6075::getUVCOMP1Data(uint16_t *uvcomp1data)
{
    uint8_t data[2] ;
    readRegs(REG_UVCOMP1_Data, data, 2) ;
    *uvcomp1data = (data[1]<<8) | data[0] ;
}

void VEML6075::getUVCOMP2Data(uint16_t *uvcomp2data)
{
    uint8_t data[2] ;
    readRegs(REG_UVCOMP2_Data, data, 2) ;
    *uvcomp2data = (data[1]<<8) | data[0] ;
}

void VEML6075::getID(uint16_t *id)
{
    uint8_t data[2] ;
    readRegs(REG_ID, data, 2) ;
    *id = (data[1]<<8) | data[0] ;
}

void VEML6075::readRegs(int addr, uint8_t * data, int len) {
    char t[1] = {addr};
    m_i2c.write(m_addr, t, 1, true);
    m_i2c.read(m_addr, (char *)data, len);
}

void VEML6075::writeRegs(uint8_t * data, int len) {
    m_i2c.write(m_addr, (char *)data, len);
}