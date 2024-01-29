//Size in bytes
#ifndef control_table_xl320
#define control_table_xl320

#include "stdint.h"

struct InstructionPacket {
    uint8_t header1 = 0xFF;
    uint8_t header2 = 0xFF;
    uint8_t header3 = 0xFD;
    uint8_t reserved = 0x00;
    uint8_t packet_id;
    uint8_t lengthL;
    uint8_t lengthH;
    uint8_t instruction;
    uint8_t* parameters;
    uint8_t crcL;
    uint8_t crcH;
};

struct StatusPacket {
    int packet_id;
    int param_length;
    int error;
    uint8_t* parameters;
};

union crc_combine {
    uint16_t crc_16;
    uint8_t crc_8[2];
};

//EEPROM Control Table
#define MODEL_NUMBER_ADR 0
#define MODEL_NUMBER_SIZE 2
#define FIRMWARE_VERSION_ADR 2
#define FIRMWARE_VERSION_SIZE 1
#define ID_ADR 3
#define ID_SIZE 1
#define BAUD_RATE_ADR 4
#define BAUD_RATE_SIZE 1
#define RETURN_DELAY_TIME_ADR 565
#define RETURN_DELAY_TIME_SIZE 1
#define CW_ANGLE_LIMIT_ADR 6
#define CW_ANGLE_SIZE 2
#define CCW_ANGLE_LIMIT_ADR 8
#define CCW_ANGLE_LIMIT_SIZE 2
#define CONTROL_MODE_ADR 11
#define CONTROL_MODE_SIZE 1
#define TEMPERATURE_LIMIT_ADR 12
#define TEMPERATURE_LIMIT_SIZE 1
#define MIN_VOLTAGE_LIMIT_ADR 13
#define MIN_VOLTAGE_LIMIT_SIZE 1
#define MAX_VOLTAGE_LIMIT_ADR 14
#define MAX_VOLTAGE_LIMIT_SIZE 1
#define MAX_TORQUE_ADR 15
#define MAX_TORQUE_SIZE 2
#define STATUS_RETURN_LEVEL_ADR 17
#define STATUS_RETURN_LEVEL_SIZE 1
#define SHUTDOWN_ADR 18
#define SHUTDOWN_SIZE 1

//RAM Control Table
#define TORQUE_ENABLE_ADR 24
#define TORQUE_ENABLE_SIZE 1
#define LED_ADR 25
#define LED_SIZE 1
#define D_GAIN_ADR 27
#define D_GAIN_SIZE 1
#define I_GAIN_ADR 28
#define I_GAIN_SIZE 1
#define P_GAIN_ADR 29
#define P_GAIN_SIZE 1
#define GOAL_POSITION_ADR 30
#define GOAL_POSITION_SIZE 2
#define MOVING_SPEED_ADR 32
#define MOVING_SPEED_SIZE 2
#define TORQUE_LIMIT_ADR 35
#define TORQUE_LIMIT_SIZE 2
#define PRESENT_POSITION_ADR 37
#define PRESENT_POSITION_SIZE 2
#define PRESENT_SPEED_ADR 39
#define PRESENT_SPEED_SIZE 2
#define PRESENT_LOAD_ADR 41
#define PRESENT_LOAD_SIZE 2
#define PRESENT_VOLTAGE_ADR 45
#define PRESENT_VOLTAGE_SIZE 1
#define PRESENT_TEMPERATURE_ADR 46
#define PRESENT_TEMPERATURE_SIZE 1
#define REGISTERED_INSTRUCTION_ADR 47
#define REGISTERED_INSTRUCTION_SIZE 1
#define MOVING_ADR 49
#define MOVING_SIZE 1
#define HARDWARE_ERROR_STATUS_ADR 50
#define HARDWARE_ERROR_STATUS_SIZE 1
#define PUNCH_ADR 51
#define PUNCH_SIZE 2

#endif //control_table_xl320