/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * HDC1080 library
 * 
 * @author  Benjamin LEBLOND <benjamin.leblond@orange.fr> ; Joseph Ellsworth CTO of A2WH
 * @version 1.0
 * @date    21-May-2018
 * 
 */

#ifndef _HDC1080_H
#define _HDC1080_H

#include "mbed.h"

// #define _DEBUG

#ifndef DEBUG_PRINT
#ifdef _DEBUG
extern Serial FTDI;
#define DEBUG_PRINT(...) FTDI.printf("\r\n"__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
#endif

#define HDC1080_DEFAULT_ADDRESS   0x40<<1
#define HDC1080_REG_TEMP          0x00
#define HDC1080_REG_HUMID         0x01
#define HDC1080_REG_CONFIG        0x02
#define HDC1080_REG_MAN_ID        0xFE
#define HDC1080_REG_SERIAL_FIRST  0xFB
#define HDC1080_REG_SERIAL_MID    0xFC
#define HDC1080_REG_SERIAL_LAST   0xFD
#define HDC1080_CHIP_ERROR        255
#define HDC1080_CHIP_ERR_L        0

/** 
 * HDC1080 class
 */
class HDC1080
{
public:

	/** Create a HDC1080 instance
	 *  which is connected to specified I2C pins with specified address
	 *
	 * @param sda I2C-bus SDA pin
	 * @param scl I2C-bus SCL pin
	 * @param addr (option) I2C-bus address (default: 0x80)
	 */
	HDC1080(PinName sda, PinName sck, char addr = HDC1080_DEFAULT_ADDRESS);

	/** Create a HDC1080 instance
	 *  which is connected to specified I2C pins with specified address
	 *
	 * @param i2c_obj pointer to I2C object
	 * @param addr (option) I2C-bus address (default: 0x80)
	 */
	HDC1080(I2C* i2c_obj, char addr = HDC1080_DEFAULT_ADDRESS);
	
	/**
	 * HDC1080 Destructor
	 */
	~HDC1080();

	void begin(void);

	uint16_t getData16(int chip_addr, int offset);

	/* Read temperature from hdc_1080 chip. Returns float
	containing the Celcius temperature or hdc_chip_error if
	error occurs reading the sensor */
	float getTemp(void);

	/* Read humidity from hdc_1080 chip. Returns a float
	containing the humidity or hdc_chip_error if error 
	occurs reading the sensor */
	float getHumid(void);

	int getManufactId(void);
	unsigned long getSerial(void);
	
private:
	I2C* m_i2c;
	char m_address;
	char m_buff[5];
};

#endif
