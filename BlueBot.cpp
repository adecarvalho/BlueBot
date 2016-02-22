#include "BlueBot.h"
#include "wiring_private.h"
#include "pins_arduino.h"
#define MeBaseBoard

//
MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
    {NC, NC}, {A2, A3}, {A6, A1}, {A7, A0}, {6, 7}, {5, 4}
};

//
union
{
    byte b[4];
    float fVal;
    long lVal;
} u;

//*********************
/*        Port       */
//*********************
MePort::MePort()
{
    s1 = mePort[0].s1;
    s2 = mePort[0].s2;
    _port = 0;
}
//
MePort::MePort(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
//
uint8_t MePort::getPort()
{
    return _port;
}
//
uint8_t MePort::getSlot()
{
    return _slot;
}
//
bool MePort::dRead1()
{
    bool val;
    pinMode(s1, INPUT);
    val = digitalRead(s1);
    return val;
}
//
bool MePort::dRead2()
{
    bool val;
    pinMode(s2, INPUT);
    val = digitalRead(s2);
    return val;
}
//
void MePort::dWrite1(bool value)
{
    pinMode(s1, OUTPUT);
    digitalWrite(s1, value);
}
//
void MePort::dWrite2(bool value)
{
    pinMode(s2, OUTPUT);
    digitalWrite(s2, value);
}
//
int MePort::aRead1()
{
    int val;
    val = analogRead(s1);
    return val;
}
//
int MePort::aRead2()
{
    int val;
    val = analogRead(s2);
    return val;
}
//
void MePort::aWrite1(int value)
{
    analogWrite(s1, value);
}
//
void MePort::aWrite2(int value)
{
    analogWrite(s2, value);
}
//
uint8_t MePort::pin1()
{
    return s1;
}
//
uint8_t MePort::pin2()
{
    return s2;
}
//
void MePort::reset(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
//
void MePort::reset(uint8_t port, uint8_t slot)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
//**********************************
/*             Wire               */
//**********************************
MeWire::MeWire(uint8_t address): MePort()
{
    _slaveAddress = address + 1;
}
//
MeWire::MeWire(uint8_t port, uint8_t address): MePort(port)
{
    _slaveAddress = address + 1;
}
//
void MeWire::begin()
{
    delay(1000);
    Wire.begin();
    write(BEGIN_FLAG, 0x01);
}
//
bool MeWire::isRunning()
{
    return read(BEGIN_STATE);
}
//
void MeWire::setI2CBaseAddress(uint8_t baseAddress)
{
    byte w[2] = {0};
    byte r[4] = {0};
    w[0] = 0x21;
    w[1] = baseAddress;
    request(w, r, 2, 4);
}
//
byte MeWire::read(byte dataAddress)
{
    byte *b = {0};
    read(dataAddress, b, 1);
    return b[0];
}
//
void MeWire::read(byte dataAddress, uint8_t *buf, int len)
{
    byte rxByte;
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    Wire.endTransmission(); // stop transmitting
    delayMicroseconds(1);
    Wire.requestFrom(_slaveAddress, len); // request 6 bytes from slave device
    int index = 0;
    while(Wire.available()) // slave may send less than requested
    {
        rxByte = Wire.read(); // receive a byte as character
        buf[index] = rxByte;
        index++;
    }
}
//
void MeWire::write(byte dataAddress, byte data)
{
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(data); // sends one byte
    Wire.endTransmission(); // stop transmitting
}
//
void MeWire::request(byte *writeData, byte *readData, int wlen, int rlen)
{

    uint8_t rxByte;
    uint8_t index = 0;

    Wire.beginTransmission(_slaveAddress); // transmit to device

    Wire.write(writeData, wlen);

    Wire.endTransmission();
    delayMicroseconds(2);
    Wire.requestFrom(_slaveAddress, rlen); // request 6 bytes from slave device
    delayMicroseconds(2);
    while(Wire.available()) // slave may send less than requested
    {
        rxByte = Wire.read(); // receive a byte as character

        readData[index] = rxByte;
        index++;
    }
}
//***************************************
/*             Serial                  */
//***************************************
MeSerial::MeSerial(): MePort(), SoftwareSerial(NC, NC)
{
    _hard = true;
    _scratch = true;
    _polling = false;
}
//
MeSerial::MeSerial(uint8_t port): MePort(port), SoftwareSerial(mePort[port].s2, mePort[port].s1)
{
    _scratch = false;
    _hard = false;
    _polling = false;
#if defined(__AVR_ATmega32U4__)
    _polling = getPort() > PORT_5;
    _hard = getPort() == PORT_4;
#else
    _hard = getPort() == PORT_5;
#endif
}
void MeSerial::setHardware(bool mode)
{
    _hard = mode;
}
//
void MeSerial::begin(long baudrate)
{
    _bitPeriod = 1000000 / baudrate;
    if(_hard)
    {
#if defined(__AVR_ATmega32U4__)
        _scratch ? Serial.begin(baudrate) : Serial1.begin(baudrate);
#else
        Serial.begin(baudrate);
#endif
    }
    else
    {
        SoftwareSerial::begin(baudrate);
    }
}
//
void MeSerial::end()
{
    if(_hard)
    {
#if defined(__AVR_ATmega32U4__)
        Serial1.end();
#else
        Serial.end();
#endif
    }
    else
    {
        SoftwareSerial::end();
    }
}
//
size_t MeSerial::write(uint8_t byte)
{
    if(_isServoBusy == true)return -1;
    if(_hard)
    {
#if defined(__AVR_ATmega32U4__)
        return (_scratch ? Serial.write(byte) : Serial1.write(byte));
#else
        return Serial.write(byte);
#endif
    }
    else return SoftwareSerial::write(byte);
}
int MeSerial::read()
{
    if(_isServoBusy == true)return -1;

    if(_polling)
    {
        int temp = _byte;
        _byte = -1;
        return temp > -1 ? temp : poll();
    }
    if(_hard)
    {
#if defined(__AVR_ATmega32U4__)
        return (_scratch ? Serial.read() : Serial1.read());
#else
        return Serial.read();
#endif
    }
    else return SoftwareSerial::read();
}
int MeSerial::available()
{
    if(_polling)
    {
        _byte = poll();
        return _byte > -1 ? 1 : 0;
    }
    if(_hard)
    {
#if defined(__AVR_ATmega32U4__)
        return (_scratch ? Serial.available() : Serial1.available());
#else
        return Serial.available();
#endif
    }
    else return SoftwareSerial::available();
}
bool MeSerial::listen()
{
    if(_hard)
        return true;
    else return SoftwareSerial::listen();
}
//
bool MeSerial::isListening()
{
    if(_hard)
        return true;
    else return SoftwareSerial::isListening();
}
//
int MeSerial::poll()
{
    int val = 0;
    int bitDelay = _bitPeriod - clockCyclesToMicroseconds(50);
    if (digitalRead(s2) == LOW)
    {
        for (int offset = 0; offset < 8; offset++)
        {
            delayMicroseconds(bitDelay);
            val |= digitalRead(s2) << offset;
        }
        delayMicroseconds(bitDelay);
        return val & 0xff;
    }
    return -1;
}
//***************************************
/*             LineFinder              */
//***************************************
BlueBotLineFollower::BlueBotLineFollower(uint8_t port): MePort(port)
{
}
//
int8_t BlueBotLineFollower::read()
{
    bool s1State = MePort::dRead1();
    bool s2State = MePort::dRead2();
    
    if(s1State==true && s2State==true)	//NO_LINE
		return NO_LINE;
	//
	else if(s1State==false && s2State==true)	//NO_LINE_LEFT
		return LINE_RIGHT;
	//
	else if(s1State==true && s2State==false)	//NO_LINE_RIGHT
		return LINE_LEFT;
	//
    return LINE_CENTER;
}
//
bool BlueBotLineFollower::readSensor1()
{
    return MePort::dRead1();
}
//
bool BlueBotLineFollower::readSensor2()
{
    return MePort::dRead2();
}

//****************************************
/*             MotorDriver              */
//****************************************
BlueBotDCMotor::BlueBotDCMotor(uint8_t port): MePort(port)
{
    //The PWM frequency is 976 Hz
#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

    TCCR1A =  _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

    TCCR3A = _BV(WGM30);
    TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

    TCCR4B = _BV(CS42) | _BV(CS41) | _BV(CS40);
    TCCR4D = 0;

#else if defined(__AVR_ATmega328__) // else ATmega328

    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);

#endif
}
//
int BlueBotDCMotor::getSpeed() const
{
	return mySpeed;
}
//
void BlueBotDCMotor::drive(int speed)
{
	if(speed >255)
		speed=255;

	//
	if(speed <-255)
		speed=-255;
    
    mySpeed=speed;

    if(mySpeed >= 0)
    {
        MePort::dWrite2(HIGH);
        MePort::aWrite1(mySpeed);
    }
    else
    {
        MePort::dWrite2(LOW);
        MePort::aWrite1(-mySpeed);
    }
}
//
void BlueBotDCMotor::stop()
{
    BlueBotDCMotor::drive(0);
}

//**********************************************
/*           UltrasonicSenser                 */
//**********************************************
BlueBotUltrasonicSensor::BlueBotUltrasonicSensor(uint8_t port): MePort(port)
{
}

float BlueBotUltrasonicSensor::readDistanceCm(uint16_t MAXcm)
{
    long distance = measure(MAXcm * 55 + 200);
    return (float)distance / 58.0;
}
//
long BlueBotUltrasonicSensor::measure(unsigned long timeout)
{
    long duration;
    MePort::dWrite2(LOW);
    delayMicroseconds(2);
    MePort::dWrite2(HIGH);
    delayMicroseconds(10);
    MePort::dWrite2(LOW);
    pinMode(s2, INPUT);
    duration = pulseIn(s2, HIGH, timeout);
    return duration;
}

//***************************************
/*           Bluetooth                 */
//***************************************
BlueBotBluetooth::BlueBotBluetooth(uint8_t port): MeSerial(port)
{
}
//****************************
/*      Sound Sensor        */
//****************************
BlueBotSoundSensor::BlueBotSoundSensor(uint8_t port) : MePort(port) 
{
}
//
int BlueBotSoundSensor::read()
{
    return MePort::aRead2();
}
//

