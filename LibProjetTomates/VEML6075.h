/*
 * File description 
 *
 */

#ifndef _VEML6075_H_
#define _VEML6075_H_

#include "mbed.h"

/**
 * UVA and UVB Light Sensor with I2C Interface
 * I2C 7bit address: 0x10
 *
 */

class VEML6075 
{
public:
 /**
 *  constructor
 *
 * @param sda SDA pin
 * @param scl SCL pin
 * @param addr address of the I2C peripheral
 */
 VEML6075(PinName sda, PinName scl, int addr) ;
 
 ~VEML6075() ;

 /*
  * some member functions here (yet to be written)
  */
/**
 * get UVA 
 * @param none
 * @returns float UVA data
 */
float getUVA(void) ;

/**
 * get UVA_CIE 
 * @param none
 * @returns float UVA_CIE data
 */
float getUVA_CIE(void) ; 

/** 
 * get UVB
 * @param none
 * @returns float UVB data
 */
float getUVB(void) ; 

/**
 * get UVB_CIE
 * @param none
 * @returns float UVB_CIE data
 */
float getUVB_CIE(void) ;

/**
 * get UVConf
 * @param uint8_t *uvconf
 * @reurns none
 */
void getUVConf(uint8_t *uvconf) ;

/**
 * set UVConf
 * @param uint8_t uvconf
 * @returns none
 */
void setUVConf(uint8_t uvconf) ;

/**
 * get raw UVA data
 * @param uint16_t *uvadata
 * @returns none
 */
void getUVAData(uint16_t *uvadata) ;

/**
 * get raw UVB data
 * @param uint16_t *rvbdata
 * @returns none
 */
void getUVBData(uint16_t *uvbdata) ;

/**
 * get raw UVD data
 * @param uint16_t *uvddata 
 * @returns none
 *
 * @note UVD was defined in the sample device datasheet
 * @note but in the released device datasheet, it's gone
 * @note so if you are using production version device
 * @note make this function just return 0 to possible side effect.
 */
void getUVDData(uint16_t *uvddata) ;

/**
 * get UVCOMP1 data
 * @param *uint16_t *uvcomp1data
 * @returns none
 */
void getUVCOMP1Data(uint16_t *uvcomp1data) ;

/**
 * get UVCOMP2 data
 * @param *uint16_t *uvcomp2data
 * @returns none
 */
void getUVCOMP2Data(uint16_t *uvcomp2data) ;

/**
 * get UVI UV Index
 * @param none
 * @returns float UVI
 */
float UVI(void) ;

/**
 * get device ID
 * @param uint16_t *id device ID
 * @returns none
 */
void getID(uint16_t *id) ;

//  void cmdRead(uint8_t cmd, uint8_t *data, uint8_t numdata = 2) ;
//  void cmdWrite(uint8_t cmd, uint8_t *data, uint8_t numdata = 2) ;
  
private:
  I2C m_i2c;
  int m_addr;
  void readRegs(int addr, uint8_t * data, int len);
  void writeRegs(uint8_t * data, int len);
} ;
#endif /* _VEML6075_H_ */
