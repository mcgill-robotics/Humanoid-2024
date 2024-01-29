
#ifndef SERVO_XL320
#define SERVO_XL320
#include "stdint.h"

class ServoXL320 {
    private:
        uint16_t model_num = 350;
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
    public:
        uint8_t getID();
};



#endif //SERVO_XL320