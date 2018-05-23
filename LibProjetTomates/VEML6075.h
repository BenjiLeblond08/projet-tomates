/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * VEML6075 library
 * 
 * @author  Benjamin LEBLOND <benjamin.leblond@orange.fr>
 * @version 1.0
 * @date    23-May-2018
 * 
 */

#ifndef _VEML6075_H
#define _VEML6075_H

#include "mbed.h"

#define _DEBUG

#ifndef DEBUG_PRINT
#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf("\r\n"__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

#define VEML6075_DEFAULT_ADDRESS   0x10<<1

#define REG_UV_CONF       0x00
#define REG_Reserved01    0x01
#define REG_Reserved02    0x02
#define REG_Reserved03    0x03
#define REG_Reserved04    0x04
#define REG_Reserved05    0x05
#define REG_Reserved06    0x06
#define REG_UVA_Data      0x07
#define REG_Reserved08    0x08
#define REG_UVB_Data      0x09
#define REG_UVCOMP1_Data  0x0A
#define REG_UVCOMP2_Data  0x0B
#define REG_ID            0x0C

// Following magic numbers are from 
// VISHAY veml6075 Application Note 84339
// Page 15 
#define UV_COEF_A        (2.22)
#define UV_COEF_B        (1.33)
#define UV_COEF_C        (2.95)
#define UV_COEF_D        (1.74)
#define UVA_RESPONSIVITY (0.001461)
#define UVB_RESPONSIVITY (0.002591)

/* Register value define : CONF */
// #define CONF_SD           0b00000001 // 0x01
// #define CONF_UV_AF_AUTO   0b00000000 // 0x00
// #define CONF_UV_AF_FORCE  0b00000010 // 0x02
// #define CONF_UV_TRIG_NO   0b00000000 // 0x00
// #define CONF_UV_TRIG_ONCE 0b00000100 // 0x04
// #define CONF_HD           0b00001000 // 0x08
// #define CONF_UV_IT_MASK   0b01110000 // 0x70
// #define CONF_UV_IT_50MS   0b00000000 // 0x00
// #define CONF_UV_IT_100MS  0b00010000 // 0x10
// #define CONF_UV_IT_200MS  0b00100000 // 0x20
// #define CONF_UV_IT_400MS  0b00110000 // 0x30
// #define CONF_UV_IT_800MS  0b01000000 // 0x40
// #define CONF_DEFAULT (CONF_UV_AF_AUTO | CONF_UV_TRIG_NO | CONF_UV_IT_100MS) 

/**
 * UVA and UVB Light Sensor with I2C Interface
 * I2C 7bit address: 0x10
 */
class VEML6075 
{
public:

	enum Conf
	{
		CONF_SD           = 0b00000001,
		CONF_UV_AF_AUTO   = 0b00000000,
		CONF_UV_AF_FORCE  = 0b00000010,
		CONF_UV_TRIG_NO   = 0b00000000,
		CONF_UV_TRIG_ONCE = 0b00000100,
		CONF_HD           = 0b00001000,
		CONF_UV_IT_MASK   = 0b01110000,
		CONF_UV_IT_50MS   = 0b00000000,
		CONF_UV_IT_100MS  = 0b00010000,
		CONF_UV_IT_200MS  = 0b00100000,
		CONF_UV_IT_400MS  = 0b00110000,
		CONF_UV_IT_800MS  = 0b01000000,
		CONF_DEFAULT      = (VEML6075::CONF_UV_AF_AUTO | VEML6075::CONF_UV_TRIG_NO | VEML6075::CONF_UV_IT_100MS) 
	};

	/**
	 * Class constructor
	 *
	 * @param sda SDA pin
	 * @param scl SCL pin
	 * @param addr address of the I2C peripheral
	 */
	VEML6075(PinName sda, PinName scl, char addr = VEML6075_DEFAULT_ADDRESS);

	/**
	 * Class constructor
	 *
	 * @param I2C SDA pin
	 * @param addr address of the I2C peripheral
	 */
	VEML6075(I2C* i2c_obj, char addr = VEML6075_DEFAULT_ADDRESS);

	~VEML6075();

	/**
	 * get UVA 
	 * @param none
	 * @returns float UVA data
	 */
	float getUVA(void);

	/** 
	 * get UVB
	 * @param none
	 * @returns float UVB data
	 */
	float getUVB(void);

	/**
	 * get UVI UV Index
	 * @param none
	 * @returns float UVI
	 */
	float getUVI(void);

	/**
	 * calc UVI UV Index
	 * @param none
	 * @returns float UVI
	 */
	float calcUVI(void);

	/**
	 * calc UVI UV Index
	 * @param float UVA
	 * @param float UVB
	 * @returns float UVI
	 */
	float calcUVI(float UVA, float UVB);

	/**
	 * get UVConf
	 * @param none
	 * @reurns char uvconf
	 */
	char getUVConf(void);

	/**
	 * set UVConf
	 * @param char uvconf
	 * @returns none
	 */
	void setUVConf(char uvconf);

	/**
	 * get device ID
	 * @param none
	 * @returns short id device ID
	 */
	short getID(void);

private:
	/**
	 * get raw UVA data
	 * @param none
	 * @returns short uva
	 */
	short getRawUVA(void);

	/**
	 * get raw UVB data
	 * @param none
	 * @returns short uvb
	 */
	short getRawUVB(void);

	/**
	 * get UVCOMP1 data
	 * @param none
	 * @returns short uvcomp1
	 */
	short getUVCOMP1(void);

	/**
	 * get UVCOMP2 data
	 * @param none
	 * @returns short uvcomp2
	 */
	short getUVCOMP2(void);

	void readReg(char reg_addr, char* data, int len);
	void writeReg(char* data, int len);

private:
	I2C* m_i2c;
	char m_address;
	float m_UVA, m_UVB;
};
#endif // _VEML6075_H_
