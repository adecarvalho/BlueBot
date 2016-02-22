///Library for BlueBot
///A.DeCarvalho 18.Fev.2016


#ifndef BLUEBOT_h
#define BLUEBOT_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#ifndef F_CPU
#define  F_CPU 16000000UL
#endif

#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
    uint8_t s1;
    uint8_t s2;
} MePort_Sig;
//
extern MePort_Sig mePort[11];//mePort[0] is nonsense


#define NC 				-1

#define PORT_1 				0x01
#define PORT_2 				0x02
#define PORT_3 				0x03
#define PORT_4 				0x04
#define PORT_5 				0x05
#define PORT_6 				0x06
#define PORT_7 				0x07
#define PORT_8 				0x08
#define M1     				0x09
#define M2     				0x0a

#define SLOT1 				1
#define SLOT2 				2

static bool _isServoBusy = false;

//states of two linefinder sensors
const int8_t NO_LINE=-1;
const int8_t LINE_LEFT=0;
const int8_t LINE_CENTER=1;
const int8_t LINE_RIGHT=2;

//Wire Setup
#define BEGIN_FLAG  		0x1E
#define BEGIN_STATE  		0x91

#define FALSE 0
#define TRUE  1

// Platform specific I/O definitions

#if defined(__AVR__)
#define MePIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define MePIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define MeIO_REG_TYPE uint8_t
#define MeIO_REG_ASM asm("r30")
#define MeDIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define MeDIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask)),((*((base)+2)) |= (mask))//INPUT_PULLUP
#define MeDIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define MeDIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define MeDIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))
#endif
//

///@brief class of MePort,it contains two pin.
class MePort
{
public:
    MePort();
    ///@brief initialize the Port
    ///@param port port number of device
    MePort(uint8_t port);
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    uint8_t getPort();
    uint8_t getSlot();
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead1();
    ///@return the level of pin 2 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead2();
    ///@brief set the level of pin 1 of port
    ///@param value is HIGH or LOW
    void dWrite1(bool value);
    ///@brief set the level of pin 2 of port
    ///@param value is HIGH or LOW
    void dWrite2(bool value);
    ///@return the analog signal of pin 1 of port between 0 to 1023
    int aRead1();
    ///@return the analog signal of pin 2 of port between 0 to 1023
    int aRead2();
    ///@brief set the PWM outpu value of pin 1 of port
    ///@param value between 0 to 255
    void aWrite1(int value);
    ///@brief set the PWM outpu value of pin 2 of port
    ///@param value between 0 to 255
    void aWrite2(int value);
    uint8_t pin1();
    uint8_t pin2();
    void reset(uint8_t port);
    void reset(uint8_t port, uint8_t slot);
protected:
    uint8_t s1;
    uint8_t s2;
    uint8_t _port;
    uint8_t _slot;
};

///@brief class of MeSerial
class MeSerial: public SoftwareSerial, public MePort
{
public:
    MeSerial();
    ///@brief initialize
    ///@param port port number of device
    MeSerial(uint8_t port);
    void setHardware(bool mode);
    ///@brief Sets the data rate in bits per second (baud) for serial data transmission.
    ///@param baudrate use one of these rates: 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
    void begin(long baudrate);
    ///@brief Writes binary data to the serial port. This data is sent as a byte or series of bytes.
    ///@param byte a value to send as a single byte
    size_t write(uint8_t byte);
    ///@brief the first byte of incoming serial data available (or -1 if no data is available) - int
    int read();
    ///@brief Get the number of bytes (characters) available for reading from the serial port. This is data that's already arrived and stored in the serial receive buffer (which holds 64 bytes).
    int available();
    int poll();
    void end();
    bool listen();
    bool isListening();
protected:
    bool _hard;
    bool _polling;
    bool _scratch;
    int _bitPeriod;
    int _byte;
    long _lastTime;
};
///@brief class of MeWire
class MeWire: public MePort
{
public:
    MeWire(uint8_t address);
    ///@brief initialize
    ///@param port port number of device
    MeWire(uint8_t port, uint8_t address);
    ///@brief reset start index of i2c slave address.
    void setI2CBaseAddress(uint8_t baseAddress);
    bool isRunning();
    ///@brief Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
    ///@param address the 7-bit slave address (optional); if not specified, join the bus as a master.
    void begin();
    ///@brief send one byte data request for read one byte from slave address.
    byte read(byte dataAddress);
    void read(byte dataAddress, uint8_t *buf, int len);
    ///@brief send one byte data request for write one byte to slave address.
    void write(byte dataAddress, byte data);
    void request(byte *writeData, byte *readData, int wlen, int rlen);
protected:
    int _slaveAddress;
};

//*********************
// ad class for BlueBot
//*********************

//**********
//LineFoller 
class BlueBotLineFollower: public MePort
{
public:
    BlueBotLineFollower(uint8_t port);
    //
    int8_t read();
	//
    bool readSensor1();
 	//
    bool readSensor2();
};

//****************
//UltrasonicSensor
class BlueBotUltrasonicSensor: public MePort
{
public :
    BlueBotUltrasonicSensor(uint8_t port);
	//
    float readDistanceCm(uint16_t = 200);
  	//
    long measure(unsigned long = 30000);
};

//*******
//DCMotor
class BlueBotDCMotor: public MePort
{
public:
    BlueBotDCMotor(uint8_t port);
	//
    void drive(int speed);	//-255 to 255
	//
    void stop();
    //
    int getSpeed() const;
private:
	int mySpeed;
};

//*********
//Bluetooth
class BlueBotBluetooth: public MeSerial
{
public:
    BlueBotBluetooth(uint8_t port);
};

//***********
//SoundSensor
class BlueBotSoundSensor : public MePort
{
public:
    BlueBotSoundSensor(uint8_t port);
    //
    int read();
};
#endif
