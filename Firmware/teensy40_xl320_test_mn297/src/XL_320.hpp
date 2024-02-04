#ifndef XL_320_SERVO
#define XL_320_SERVO

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
// #include <fcntl.h>  // Contains file controls like O_RDWR
// #include <errno.h>  // Error integer and strerror() function
// #include <unistd.h> // write(), read(), close()
// #include <sys/ioctl.h>
// #include <termios.h> // Contains POSIX terminal control definitions
// #include <wiringPi.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <string>

using namespace std;

// Custom definitions
#define ERROR_RESULT_FAIL 0x01
#define ERROR_INSTRUCTION 0x02
#define ERROR_WRONGCNC_SENT 0x03
#define ERROR_DATA_RANGE 0x04
#define ERROR_DATA_LENGTH 0x05
#define ERROR_DATA_LIMIT 0x06
#define ERROR_ACCESS 0x07
#define ERROR_UNKNOWN 11
#define ERROR_SERIAL_GET 12
#define ERROR_SERIAL_SET 13
#define ERROR_SERIAL_READ 14
#define ERROR_NOSTATUS 15
#define ERROR_INCOMPLETESTATUS 16
#define ERROR_WRONGCNC_READ 17

#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_YELLOW 3
#define LED_BLUE 4
#define LED_PURPLE 5
#define LED_CYAN 6
#define LED_WHITE 7

class XL_320
{
private:
    // Error handling
    int errorcode;

    // Serial communication
    // int serial_port;
    // struct termios tty;
    int _pin_enable;
    HardwareSerial *_serial;

    // ID
    unsigned char id;

public:
    // Constructor
    XL_320();

    // Destructor
    ~XL_320();

    void begin();

    // General info
    int model;
    int version;

    // General behavior
    bool verbose;
    string error();

    // Send and receive
    int send(unsigned char Instruction, vector<int> param, bool receive = false);
    unsigned short CRC(unsigned char *data_blk_ptr, unsigned short data_blk_size);
    vector<unsigned char> result;

    // --- Dynamixel protocol 2.0

    int ping();
    int factory_reset();
    int reboot();

    void dispPacket(unsigned char *p, int np, bool status);

    // --- XL-320 specific commands

    // EEPROM
    int getModelNumber(); // Should return 0x15E (350)
    int getFirmwareVersion();

    int setID(int v);
    int getID();

    int setBaudRate(int v);
    int getBaudRate();

    int setReturnDelayTime(int v);
    int getReturnDelayTime();

    int setCWAngleLimit(int v);
    int getCWAngleLimit();

    int setCCWAngleLimit(int v);
    int getCCWAngleLimit();

    int setControlMode(int v);
    int getControlMode();

    int setTemperatureLimit(int v);
    int getTemperatureLimit();

    int setMinVoltageLimit(int v);
    int getMinVoltageLimit();

    int setMaxVoltageLimit(int v);
    int getMaxVoltageLimit();

    int setMaxTorque(int v);
    int getMaxTorque();

    int setStatusReturnLevel(int v);
    int getStatusReturnLevel();

    int setShutdown(int v);
    int getShutdown();

    // ROM
    int setTorqueEnable(int v);
    int getTorqueEnable();

    int setLED(int v);
    int getLED();

    int setDGain(int v);
    int getDGain();

    int setIGain(int v);
    int getIGain();

    int setPGain(int v);
    int getPGain();

    int setGoalPosition(int pos);
    int getGoalPosition();

    int setSpeed(int v);
    int getSpeed();

    int setPunch(int v);
    int getPunch();

    int isMoving();
    int getPresentPosition();
    int getPresentSpeed();
    int getPresentLoad();
    int getPresentVoltage();
    int getPresentTemperature();
    int getRegisteredInstruction();
    int getHardwareErrorStatus();
    void setSerialPort(HardwareSerial &serial);
};

vector<unsigned char> int2arg(int i, int n);
#endif
