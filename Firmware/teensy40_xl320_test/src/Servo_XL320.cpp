#include "Servo_XL320.h"
#include "HardwareSerial.h"

// Support:
// Ping, Read, Write, Reg Write, Action, Factory Reset, Reboot, Clear, Control Table Backup

union crc_combine
{
    uint16_t crc_16;
    uint8_t crc_8[2];
};

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    for (j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void writeInstructionPacket(HardwareSerial *serial, uint16_t param_length, uint8_t packet_id, uint8_t instruction, uint8_t *parameters)
{
    uint8_t writeArray[10 + param_length];
    writeArray[0] = 0xFF;
    writeArray[1] = 0xFF;
    writeArray[2] = 0xFD;
    writeArray[3] = packet_id;
    writeArray[4] = (3 + param_length) & 0x00FF;
    writeArray[5] = ((3 + param_length) & 0xFF00) >> 8;
    writeArray[6] = instruction;
    if (param_length > 0)
        memcpy(writeArray + 7, parameters, param_length);
    crc_combine crc_value;
    crc_value.crc_16 = update_crc(0, writeArray, 8 + param_length);
    writeArray[7 + param_length] = crc_value.crc_8[0];
    writeArray[8 + param_length] = crc_value.crc_8[1];

    (*serial).write(writeArray, 10 + param_length);
}

ServoXL320 *getServoByID(ServoXL320 **servo_list, int len, int id)
{
    for (int i = 0; i < len; i++)
    {
        ServoXL320 *item = servo_list[i];
        if ((*item).getID() == id)
        {
            return item;
        }
    }
    return nullptr;
}

int readOneStatus(HardwareSerial *serial, uint8_t *out, int param_length)
{
    int total_length = param_length + 11;
    while ((*serial).available() < total_length)
        ;
    uint8_t buf[total_length];
    (*serial).readBytes((char *)buf, total_length);
    crc_combine crc_value;
    crc_value.crc_16 = update_crc(0, buf, total_length - 2);
    if (buf[total_length - 2] != crc_value.crc_8[0] || buf[total_length - 1] != crc_value.crc_8[1])
        return 1; // bad CRC
    if (buf[8] != 0)
        return buf[8] + 10; // internal servo error
    if ((buf[6] << 8) & buf[5] != param_length + 3)
        return 3; // bad length
    out[0] = buf[4];
    memcpy(out + 1, buf + 9, param_length);
    return 0; // success
}

int pingServo(HardwareSerial *serial, ServoXL320 *servo)
{
    writeInstructionPacket(serial, 0, (*servo).getID(), 0x01, nullptr);
    uint8_t rcv_buf[4];
    int error = readOneStatus(serial, rcv_buf, 3);
    if (error != 0)
        return error;
    (*servo).setModelNumber((rcv_buf[2] << 8) & rcv_buf[1]);
    (*servo).setFirmwareVersion(rcv_buf[3]);
    return 0; // success
}

int *pingAllServos(HardwareSerial *serial, ServoXL320 **servo_list, int len)
{
    writeInstructionPacket(serial, 0, 0xFE, 0x01, nullptr);
    uint8_t rcv_buf[4];
    int error[len];
    ServoXL320 *servo;
    for (int i = 0; i < len; i++)
    {
        error[i] = readOneStatus(serial, rcv_buf, 3);
        if (error[i] == 0)
        {
            servo = getServoByID(servo_list, len, rcv_buf[0]);
        }
    }
    return error;
}

// getters

uint16_t ServoXL320::getModelNumber()
{
    return model_number;
}

uint8_t ServoXL320::getFirmwareVersion()
{
    return firmware_version;
}

uint8_t ServoXL320::getID()
{
    return id;
}

uint8_t ServoXL320::getBaudRate()
{
    return baud_rate;
}

uint8_t ServoXL320::getReturnDelayTime()
{
    return return_delay_time;
}

uint16_t ServoXL320::getCWAngleLimit()
{
    return cw_angle_limit;
}

uint16_t ServoXL320::getCCWAngleLimit()
{
    return ccw_angle_limit;
}

uint8_t ServoXL320::getControlMode()
{
    return control_mode;
}

uint8_t ServoXL320::getTemperatureLimit()
{
    return temperature_limit;
}

uint8_t ServoXL320::getMinVoltageLimit()
{
    return min_voltage_limit;
}

uint8_t ServoXL320::getMaxVoltageLimit()
{
    return max_voltage_limit;
}

uint16_t ServoXL320::getMaxTorque()
{
    return max_torque;
}

uint8_t ServoXL320::getStatusReturnLevel()
{
    return status_return_level;
}

uint8_t ServoXL320::getShutdown()
{
    return shutdown;
}

uint8_t ServoXL320::getTorqueEnable()
{
    return torque_enable;
}

uint8_t ServoXL320::getLED()
{
    return led;
}

uint8_t ServoXL320::getDGain()
{
    return d_gain;
}

uint8_t ServoXL320::getIGain()
{
    return i_gain;
}

uint8_t ServoXL320::getPGain()
{
    return p_gain;
}

uint16_t ServoXL320::getGoalPosition()
{
    return goal_position;
}

uint16_t ServoXL320::getMovingSpeed()
{
    return moving_speed;
}

uint16_t ServoXL320::getTorqueLimit()
{
    return torque_limit;
}

uint16_t ServoXL320::getPresentPosition()
{
    return present_position;
}

uint16_t ServoXL320::getPresentSpeed()
{
    return present_speed;
}

uint16_t ServoXL320::getPresentLoad()
{
    return present_load;
}

uint8_t ServoXL320::getPresentVoltage()
{
    return present_voltage;
}

uint8_t ServoXL320::getPresentTemperature()
{
    return present_temperature;
}

uint8_t ServoXL320::getRegisteredInstruction()
{
    return registered_instruction;
}

uint8_t ServoXL320::getMoving()
{
    return moving;
}

uint8_t ServoXL320::getHardwareErrorStatus()
{
    return hardware_error_status;
}

uint16_t ServoXL320::getPunch()
{
    return punch;
}

// setters

void ServoXL320::setModelNumber(uint16_t value)
{
    model_number = value;
}

void ServoXL320::setFirmwareVersion(uint8_t value)
{
    firmware_version = value;
}

void ServoXL320::setID(uint8_t value)
{
    id = value;
}

void ServoXL320::setBaudRate(uint8_t value)
{
    baud_rate = value;
}

void ServoXL320::setReturnDelayTime(uint8_t value)
{
    return_delay_time = value;
}

void ServoXL320::setCWAngleLimit(uint16_t value)
{
    cw_angle_limit = value;
}

void ServoXL320::setCCWAngleLimit(uint16_t value)
{
    ccw_angle_limit = value;
}

void ServoXL320::setControlMode(uint8_t value)
{
    control_mode = value;
}

void ServoXL320::setTemperatureLimit(uint8_t value)
{
    temperature_limit = value;
}

void ServoXL320::setMinVoltageLimit(uint8_t value)
{
    min_voltage_limit = value;
}

void ServoXL320::setMaxVoltageLimit(uint8_t value)
{
    max_voltage_limit = value;
}

void ServoXL320::setMaxTorque(uint16_t value)
{
    max_torque = value;
}

void ServoXL320::setStatusReturnLevel(uint8_t value)
{
    status_return_level = value;
}

void ServoXL320::setShutdown(uint8_t value)
{
    shutdown = value;
}

void ServoXL320::setTorqueEnable(uint8_t value)
{
    torque_enable = value;
}

void ServoXL320::setLED(uint8_t value)
{
    led = value;
}

void ServoXL320::setDGain(uint8_t value)
{
    d_gain = value;
}

void ServoXL320::setIGain(uint8_t value)
{
    i_gain = value;
}

void ServoXL320::setPGain(uint8_t value)
{
    p_gain = value;
}

void ServoXL320::setGoalPosition(uint16_t value)
{
    goal_position = value;
}

void ServoXL320::setMovingSpeed(uint16_t value)
{
    moving_speed = value;
}

void ServoXL320::setTorqueLimit(uint16_t value)
{
    torque_limit = value;
}

void ServoXL320::setPresentPosition(uint16_t value)
{
    present_position = value;
}

void ServoXL320::setPresentSpeed(uint16_t value)
{
    present_speed = value;
}

void ServoXL320::setPresentLoad(uint16_t value)
{
    present_load = value;
}

void ServoXL320::setPresentVoltage(uint8_t value)
{
    present_voltage = value;
}

void ServoXL320::setPresentTemperature(uint8_t value)
{
    present_temperature = value;
}

void ServoXL320::setRegisteredInstruction(uint8_t value)
{
    registered_instruction = value;
}

void ServoXL320::setMoving(uint8_t value)
{
    moving = value;
}

void ServoXL320::setHardwareErrorStatus(uint8_t value)
{
    hardware_error_status = value;
}

void ServoXL320::setPunch(uint16_t value)
{
    punch = value;
}
