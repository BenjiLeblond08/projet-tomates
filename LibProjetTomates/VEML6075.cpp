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

#include "VEML6075.h"

VEML6075::VEML6075(PinName sda, PinName scl, char addr)
	:
	m_i2c(new I2C(sda, scl)),
	m_address(addr),
	m_UVA(0),
	m_UVB(0)
{}

VEML6075::VEML6075(I2C* i2c_obj, char addr)
	:
	m_i2c(i2c_obj),
	m_address(addr),
	m_UVA(0),
	m_UVB(0)
{}

VEML6075::~VEML6075() { }

float VEML6075::getUVA(void) 
{
	short RawUVA, UVCOMP1, UVCOMP2;
	RawUVA = getRawUVA();
	UVCOMP1 = getUVCOMP1();
	UVCOMP2 = getUVCOMP2();
	if ((m_UVA = (float)RawUVA - (UV_COEF_A * (float)UVCOMP1) - (UV_COEF_B * (float)UVCOMP2)) < 0)
		m_UVA = 0;
	return(m_UVA);
}

float VEML6075::getUVB(void)
{
	short RawUVB, UVCOMP1, UVCOMP2;
	RawUVB = getRawUVB();
	UVCOMP1 = getUVCOMP1();
	UVCOMP2 = getUVCOMP2();
	if ((m_UVB = (float)RawUVB - (UV_COEF_C * (float)UVCOMP1) - (UV_COEF_D * (float)UVCOMP2)) < 0)
		m_UVB = 0;
	return(m_UVB);
}

float VEML6075::getUVI(void)
{
	getUVA();
	getUVB();
	return calcUVI();
}

float VEML6075::calcUVI(void)
{
	return calcUVI(m_UVA, m_UVB);
}

float VEML6075::calcUVI(float UVA, float UVB)
{
	float UVIA, UVIB, UVI;
	UVIA = UVA * (float)UVA_RESPONSIVITY;
	UVIB = UVB * (float)UVB_RESPONSIVITY;
	UVI = (UVIA + UVIB) / 2.0;
	DEBUG_PRINT("UVA = %f", UVA);
	DEBUG_PRINT("UVB = %f", UVB);
	DEBUG_PRINT("UVIA = %f", UVIA);
	DEBUG_PRINT("UVIB = %f", UVIB);
	DEBUG_PRINT("UVI = %f", UVI);
	return UVI;
}

char VEML6075::getUVConf(void)
{
	char data[2];
	readReg(REG_UV_CONF, data, 2);
	DEBUG_PRINT("uvconf = %#x", data[0]);
	return data[0];
}

void VEML6075::setUVConf(char uvconf)
{
	char data[3];
	data[0] = REG_UV_CONF;
	data[1] = uvconf;
	data[2] = 0;
	// writeReg(data, 3);
	m_i2c->write(m_address, data, 3);
}

short VEML6075::getID(void)
{
	char data[2];
	short id;
	readReg(REG_ID, data, 2);
	id = (data[1]<<8) | data[0];
	DEBUG_PRINT("id = %#x", id);
	return id;
}

short VEML6075::getRawUVA(void)
{
	char data[2];
	short UVAraw;
	readReg(REG_UVA_Data, data, 2);
	UVAraw = (data[1]<<8) | data[0];
	DEBUG_PRINT("UVAraw = %#x", UVAraw);
	return UVAraw;
}

short VEML6075::getRawUVB(void)
{
	char data[2];
	short UVBraw;
	readReg(REG_UVB_Data, data, 2);
	UVBraw = (data[1]<<8) | data[0];
	DEBUG_PRINT("UVBraw = %#x", UVBraw);
	return UVBraw;
}

short VEML6075::getUVCOMP1(void)
{
	char data[2];
	short UVcomp1;
	readReg(REG_UVCOMP1_Data, data, 2);
	UVcomp1 = (data[1]<<8) | data[0];
	DEBUG_PRINT("UVcomp1 = %#x", UVcomp1);
	return UVcomp1;
}

short VEML6075::getUVCOMP2(void)
{
	char data[2];
	short UVcomp2;
	readReg(REG_UVCOMP2_Data, data, 2);
	UVcomp2 = (data[1]<<8) | data[0];
	DEBUG_PRINT("UVcomp2 = %#x", UVcomp2);
	return UVcomp2;
}

void VEML6075::readReg(char reg_addr, char* data, int len) {
	char t[1] = {reg_addr};
	m_i2c->write(m_address, t, 1, true);
	m_i2c->read(m_address, (char*)data, len);
}

void VEML6075::writeReg(char* data, int len) {
	m_i2c->write(m_address, (char*)data, len);
}
