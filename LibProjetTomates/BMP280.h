/**
 * Projet Tomates
 * Teensy 3.2
 * 
 * BMP280 Combined humidity and pressure sensor library
 * 
 * @author Benjamin LEBLOND <benjamin.leblond@orange.fr> ; Toyomasa Watarai
 * @version 2.0
 * @date    21-May-2018
 *
 *  Library for "BMP280 temperature, humidity and pressure sensor module" from Switch Science
 *    https://www.switch-science.com/catalog/2236/
 *
 *  For more information about the BMP280:
 *    http://ae-bst.resource.bosch.com/media/products/dokumente/BMP280/BST-BMP280_DS001-10.pdf
 */
 
#ifndef _BMP280_H
#define _BMP280_H

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

#define BMP280_DEFAULT_ADDRESS (0x77 << 1)
 
/** BMP280 class
 *
 *  BMP280: A library to correct environmental data using Boshe BMP280 device
 *
 *  BMP280 is an environmental sensor
 *  @endcode
 */
class BMP280
{
public:

    /** Create a BMP280 instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param sda I2C-bus SDA pin
     * @param scl I2C-bus SCL pin
     * @param slave_adr (option) I2C-bus address (default: 0x76)
     */
    BMP280(PinName sda, PinName sck, char slave_adr = BMP280_DEFAULT_ADDRESS);

    /** Create a BMP280 instance
     *  which is connected to specified I2C pins with specified address
     *
     * @param i2c_obj I2C object (instance)
     * @param slave_adr (option) I2C-bus address (default: 0x76)
     */
    BMP280(I2C* i2c_obj, char slave_adr = BMP280_DEFAULT_ADDRESS);

    /** Destructor of BMP280
     */
    virtual ~BMP280();

    /** Initializa BMP280 sensor
     *
     *  Configure sensor setting and read parameters for calibration
     *
     */
    void initialize(void);

    /** Read the current temperature value (degree Celsius) from BMP280 sensor
     *
     */
    float getTemperature(void);

    /** Read the current pressure value (hectopascal)from BMP280 sensor
     *
     */
    float getPressure(void);

    /** Read the current humidity value (humidity %) from BMP280 sensor
     *
     */
  //  float getHumidity(void);

private:

    I2C*        m_i2c;
    char        m_address;
    uint16_t    dig_T1;
    int16_t     dig_T2, dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint16_t    dig_H1, dig_H3;
    int16_t     dig_H2, dig_H4, dig_H5, dig_H6;
    int32_t     t_fine;

};

#endif // _BMP280_H_
