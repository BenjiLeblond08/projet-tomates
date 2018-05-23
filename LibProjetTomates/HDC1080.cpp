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

#include "HDC1080.h"

HDC1080::HDC1080(PinName sda, PinName scl, char addr)
	:
	m_i2c(new I2C(sda, scl)),
	m_address(addr)
{}

HDC1080::HDC1080(I2C* i2c_obj, char addr)
	:
	m_i2c(i2c_obj),
	m_address(addr)
{}

HDC1080::~HDC1080()
{}

void HDC1080::begin(void)
{
	DEBUG_PRINT("hdc_begin");
	memset(m_buff,0,3);
	m_buff[0] = HDC1080_REG_CONFIG;
	int res = m_i2c->write(m_address, m_buff, 2);
	DEBUG_PRINT("hdc_begin write res=%d", res);
}
 
uint16_t HDC1080::getData16(int chip_addr, int offset)
{
	memset(m_buff,0,3);
	// send chip address onto buss
	m_buff[0] = offset;
	int res = m_i2c->write(chip_addr, m_buff, 1);
	if (res != 0) {
		DEBUG_PRINT("error talking to chip %d offst=%d", chip_addr, offset);
		return 0;
	}
	// read data from chip
	wait(0.015);
	memset(m_buff,0,3);
	res = m_i2c->read(m_address, m_buff,2);
	if (res != 0) {
		DEBUG_PRINT("error reading chip %d offst=%d", chip_addr, offset);
		return 0;
	}
	return m_buff[0] << 8 | m_buff[1];
}
 
float HDC1080::getTemp(void)
{
	uint16_t rawT = HDC1080::getData16(m_address, HDC1080_REG_TEMP);
	if (rawT == 0) {
		DEBUG_PRINT("error reading hdc chip temp");
		return HDC1080_CHIP_ERROR;
	} else {
		float temp = ((float) rawT / pow(2.0f, 16.0f)) * 165.0f - 40.0f;
		DEBUG_PRINT("temp=%0.3f", temp);
		return temp;
	}
}

float HDC1080::getHumid(void)
{
	uint16_t rawH = HDC1080::getData16(m_address, HDC1080_REG_HUMID);
	if (rawH == 0) {
		DEBUG_PRINT("error reading hdc chip humid");
		return HDC1080_CHIP_ERROR;
	} else {
		float humid = ((float) rawH / pow(2.0f, 16.0f)) * 100.0f;
		DEBUG_PRINT("humid=%0.3f", humid);
		return humid;
	}
}
 
int HDC1080::getManufactId(void)
{
	uint16_t rawid = HDC1080::getData16(m_address, HDC1080_REG_MAN_ID);
	if (rawid == 0) {
		DEBUG_PRINT("error reading hdc chip manId");
		return (int) HDC1080_CHIP_ERROR;
	} else { 
		DEBUG_PRINT("man id=%x", (int) rawid);
		return rawid;
	}
}
 
unsigned long HDC1080::getSerial(void)
{
	wait(0.015);
	memset(m_buff,0,4);
	m_buff[0] = HDC1080_REG_MAN_ID;
	int res = m_i2c->write(m_address, m_buff, 1);
	if (res != 0) {
		DEBUG_PRINT("Error writing chip addr res=%d", res);
		return (unsigned long) HDC1080_CHIP_ERR_L;
	}
 
	wait(0.015);
	memset(m_buff,0,4);
	res = m_i2c->read(m_address, m_buff,4);
	if (res != 0) {
		DEBUG_PRINT("Errot reading chip serial res=%d#", res);
		return (unsigned long) HDC1080_CHIP_ERR_L;
	}
	 
	unsigned long rawser = m_buff[0] << 16 | m_buff[1] << 8 | m_buff[0];
	DEBUG_PRINT("ser=%lu", rawser);
	return rawser;
}
