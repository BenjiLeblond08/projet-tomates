/**
 * Teensy 2.2
 * 
 */

#include "OSV2.h"

OSV2::OSV2(PinName out)
{
	m_data_out = new DigitalOut(out, 0);
}

OSV2::OSV2(DigitalOut *out)
	:
	m_data_out(out)
{
}

OSV2::~OSV2() { }


/////////////////////////////////////////////////////////////////////////////
// Codage des fonctions pour OSV2
/////////////////////////////////////////////////////////////////////////////

void OSV2::sendZero(void)
{
	SEND_HIGH();
	wait_us(OSV2_TIME);
	SEND_LOW();
	wait_us(OSV2_TWOTIME);
	SEND_HIGH();
	wait_us(OSV2_TIME);
}
void OSV2::sendOne(void)
{
	SEND_LOW();
	wait_us(OSV2_TIME);
	SEND_HIGH();
	wait_us(OSV2_TWOTIME);
	SEND_LOW();
	wait_us(OSV2_TIME);
}
void OSV2::sendQuarterMSB(unsigned char data)
{
	(bitRead(data, 4)) ? sendOne() : sendZero();
	(bitRead(data, 5)) ? sendOne() : sendZero();
	(bitRead(data, 6)) ? sendOne() : sendZero();
	(bitRead(data, 7)) ? sendOne() : sendZero();
}
void OSV2::sendQuarterLSB(unsigned char data)
{
	(bitRead(data, 0)) ? sendOne() : sendZero();
	(bitRead(data, 1)) ? sendOne() : sendZero();
	(bitRead(data, 2)) ? sendOne() : sendZero();
	(bitRead(data, 3)) ? sendOne() : sendZero();
}
void OSV2::sendData(unsigned char *data, unsigned char size)
{
	for(int i = 0; i < size; ++i)
	{
		sendQuarterLSB(data[i]);
		sendQuarterMSB(data[i]);
	}
}
void OSV2::sendOregon(unsigned char *data, unsigned char size)
{
	sendPreamble();
	sendData(data, size);
	sendPostamble();
}
void OSV2::sendPreamble(void)
{
	unsigned char PREAMBLE[]={0xFF,0xFF};
	sendData(PREAMBLE, 2);
}

void OSV2::sendPostamble(void)
{
	unsigned char POSTAMBLE[]={0x00};
	sendData(POSTAMBLE, 1);
}

void OSV2::sendSync(void)
{
	sendQuarterLSB(0xA);
}

void OSV2::setType(unsigned char *data, unsigned char *type)
{
	data[0] = type[0];
	data[1] = type[1];
}

void OSV2::setChannel(unsigned char *data, unsigned char channel)
{
	data[2] = channel;
}

void OSV2::setID(unsigned char *data, unsigned char ID)
{
	data[3] = ID;
}

void OSV2::setBatteryLevel(unsigned char *data, unsigned char level)
{
	if(!level) data[4] = 0x0C;
	else data[4] = 0x00;
}

void OSV2::setTemperature(unsigned char *data, float Temp)
{
	data[6] = (Temp <0) ? 0x08 : 0x00;
	
	int temp=(int)(abs(Temp)*10);
	data[5]=(temp/100) << 4;
	data[5]=data[5] | (((temp)/10) % 10);
	data[4]=(temp % 10) << 4;
}

void OSV2::setHumidity(unsigned char *data,unsigned char Hum)
{
	data[7] = (Hum / 10);
	data[6] |= (Hum % 10) << 4;
}

void OSV2::setPressure(unsigned char *data, float pres) 
{
	if ((pres > 850) && (pres < 1100)) 
		{
			data[8] = (int)(pres) - 856;
			data[9] = 0xC0;	
		}
}

unsigned char OSV2::sum(unsigned char count, unsigned char *data)
{
	int s = 0;
 
	for(int i = 0; i<count ; i++)
	{
		s += (data[i]&0xF0) >> 4;
		s += (data[i]&0xF);
	}
 
	if(int(count) != count)
		s += (data[count]&0xF0) >> 4;
 
	return s;
}

void OSV2::calculateAndSetChecksumTHR(unsigned char *data)
{
	data[8] = ((sum(8, data) - 0xa) & 0xFF);
}

void OSV2::calculateAndSetChecksumTHRP(unsigned char *data)
{
	data[10] = ((sum(10, data) - 0xa) & 0xFF);
}

void OSV2::calculateAndSetChecksumT(unsigned char *data)
{
	int s = ((sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
	data[6] |= (s&0x0F) << 4;
	data[7] = (s&0xF0) >> 4;
}
