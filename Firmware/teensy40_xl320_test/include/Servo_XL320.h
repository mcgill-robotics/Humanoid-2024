
#ifndef SERVO_XL320
#define SERVO_XL320
#include "stdint.h"
#include "control_table_xl320.h"

class ServoXL320
{
private:
    uint16_t model_number = 350;
    uint8_t firmware_version = 0;
    uint8_t id = 1;
    uint8_t baud_rate = 3;
    uint8_t return_delay_time = 250;
    uint16_t cw_angle_limit = 0;
    uint16_t ccw_angle_limit = 1023;
    uint8_t control_mode = 2;
    uint8_t temperature_limit = 65;
    uint8_t min_voltage_limit = 60;
    uint8_t max_voltage_limit = 90;
    uint16_t max_torque = 1023;
    uint8_t status_return_level = 2;
    uint8_t shutdown = 3;

    uint8_t torque_enable = 0;
    uint8_t led = 0;
    uint8_t d_gain = 0;
    uint8_t i_gain = 0;
    uint8_t p_gain = 32;
    uint16_t goal_position = 0;
    uint16_t moving_speed = 0;
    uint16_t torque_limit = 0;
    uint16_t present_position = 0;
    uint16_t present_speed = 0;
    uint16_t present_load = 0;
    uint8_t present_voltage = 0;
    uint8_t present_temperature = 0;
    uint8_t registered_instruction = 0;
    uint8_t moving = 0;
    uint8_t hardware_error_status = 0;
    uint16_t punch = 32;

public:
    uint16_t getModelNumber();
    uint8_t getFirmwareVersion();
    uint8_t getID();
    uint8_t getBaudRate();
    uint8_t getReturnDelayTime();
    uint16_t getCWAngleLimit();
    uint16_t getCCWAngleLimit();
    uint8_t getControlMode();
    uint8_t getTemperatureLimit();
    uint8_t getMinVoltageLimit();
    uint8_t getMaxVoltageLimit();
    uint16_t getMaxTorque();
    uint8_t getStatusReturnLevel();
    uint8_t getShutdown();
    uint8_t getTorqueEnable();
    uint8_t getLED();
    uint8_t getDGain();
    uint8_t getIGain();
    uint8_t getPGain();
    uint16_t getGoalPosition();
    uint16_t getMovingSpeed();
    uint16_t getTorqueLimit();
    uint16_t getPresentPosition();
    uint16_t getPresentSpeed();
    uint16_t getPresentLoad();
    uint8_t getPresentVoltage();
    uint8_t getPresentTemperature();
    uint8_t getRegisteredInstruction();
    uint8_t getMoving();
    uint8_t getHardwareErrorStatus();
    uint16_t getPunch();

    void setModelNumber(uint16_t value);
    void setFirmwareVersion(uint8_t value);
    void setID(uint8_t value);
    void setBaudRate(uint8_t value);
    void setReturnDelayTime(uint8_t value);
    void setCWAngleLimit(uint16_t value);
    void setCCWAngleLimit(uint16_t value);
    void setControlMode(uint8_t value);
    void setTemperatureLimit(uint8_t value);
    void setMinVoltageLimit(uint8_t value);
    void setMaxVoltageLimit(uint8_t value);
    void setMaxTorque(uint16_t value);
    void setStatusReturnLevel(uint8_t value);
    void setShutdown(uint8_t value);
    void setTorqueEnable(uint8_t value);
    void setLED(uint8_t value);
    void setDGain(uint8_t value);
    void setIGain(uint8_t value);
    void setPGain(uint8_t value);
    void setGoalPosition(uint16_t value);
    void setMovingSpeed(uint16_t value);
    void setTorqueLimit(uint16_t value);
    void setPresentPosition(uint16_t value);
    void setPresentSpeed(uint16_t value);
    void setPresentLoad(uint16_t value);
    void setPresentVoltage(uint8_t value);
    void setPresentTemperature(uint8_t value);
    void setRegisteredInstruction(uint8_t value);
    void setMoving(uint8_t value);
    void setHardwareErrorStatus(uint8_t value);
    void setPunch(uint16_t value);
};

#endif // SERVO_XL320