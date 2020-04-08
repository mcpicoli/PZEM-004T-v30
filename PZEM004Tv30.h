/*
 * PZEM-004Tv30.h
 *
 * Interface library for the upgraded version of PZEM-004T v3.0
 * Based on the PZEM004T library by @olehs https://github.com/olehs/PZEM004T
 *
 * Original version author: Jakub Mandula https://github.com/mandulaj
 * This version: Marcelo C. Picoli https://github.com/mcpicoli
 *
 * Changed at various points noted below in the comments. Abstract:
 * - New ESP32 specific constructor allowing for pin relocation of HardwareSerial serial ports.
 * - "receive" instead "recieve".
 * - Deprecated method "pf()" and replaced it with "powerFactor()".
*/


#ifndef PZEM004TV30_H
#define PZEM004TV30_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Uncomment (or define elsewhere before) this define to be able to use a local version of the SoftwareSerial library instead
// It may be needed in some cases where the library has to be altered locally
// #define LOCAL_SOFTWARESERIAL_LIBRARY

// #define PZEM004_NO_SWSERIAL
// Now tests properly for the ESP32. But a specific version of the SoftwareSerial must be used ("plerup", 5.2.8 and above?)
#if (not defined(PZEM004_NO_SWSERIAL)) && (defined(__AVR__) || defined(ESP8266) || defined(ESP32))
#define PZEM004_SOFTSERIAL
#endif

#ifdef PZEM004_SOFTSERIAL
 //#ifdef LOCAL_SOFTWARESERIAL_LIBRARY
 //   #include "../../SoftwareSerial.h"
 // #else
    #include <SoftwareSerial.h>
  //#endif
#endif

#define PZEM_DEFAULT_ADDR    0xF8

class PZEM004Tv30
{
public:
    #ifdef PZEM004_SOFTSERIAL
    PZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr=PZEM_DEFAULT_ADDR);
    #endif

    PZEM004Tv30(HardwareSerial* port, uint8_t addr=PZEM_DEFAULT_ADDR);

    #if defined(ESP32)
    /**
     * This constructor is specific for the ESP32 boards, since this architecture allows remapping of the pins assigned to most functions to most of the other pins.
     * It allows for retaining the HardwareSerial full features when its default pins are used for other things (like Ethernet PHYs) and avoiding most of the cases 
     * where it would be required to use SoftwareSerial instances with its related problems.
     */
    PZEM004Tv30(HardwareSerial* port, uint8_t receivePin, uint8_t transmitPin, uint8_t addr=PZEM_DEFAULT_ADDR);
    #endif
	
    ~PZEM004Tv30();


    float voltage();
    float current();
    float power();
    float energy();
    float frequency();
	float powerFactor();
    float pf();												// Deprecated! Use powerFactor() instead!

	bool setSlaveAddress(uint8_t addr);
    bool setAddress(uint8_t addr);
    uint8_t getAddress();

    bool setPowerAlarm(uint16_t watts);
    bool getPowerAlarm();

    bool resetEnergy();

private:

    Stream* _serial; // Serial interface
    bool _isSoft;    // Is serial interface software

    uint8_t _addr;   // Device address

    struct {
        float voltage;
        float current;
        float power;
        float energy;
        float frequeny;
        float pf;
        uint16_t alarms;
    }  _currentValues; // Measured values

    uint64_t _lastRead; // Last time values were updated

    void init(uint8_t addr); // Init common to all constructors

    bool updateValues();    // Get most up to date values from device registers and cache them
    uint16_t receive(uint8_t *resp, uint16_t len); // Receive len bytes into a buffer

    bool sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check=false); // Send 8 byte command

    void setCRC(uint8_t *buf, uint16_t len);           // Set the CRC for a buffer
    bool checkCRC(const uint8_t *buf, uint16_t len);   // Check CRC of buffer

    uint16_t CRC16(const uint8_t *data, uint16_t len); // Calculate CRC of buffer
};

#endif // PZEM004T_H
