/* XL_320
 *
 * This class is designed to operate the Dynamixel XL320 servo with a Raspberry Pi 4B.
 * The communication runs through UART2 (TC=GPIO0, RX=GPIO1) at 1Mbps.
 * NB: UART2 has to be activated and configured at sufficient speed.
 *
 */
#include <Arduino.h>
#include "XL_320.hpp"

using namespace std;

#undef max
#undef min
#include <stdio.h>
#include <iostream>

using namespace std;

// extern "C"
// {
//     int _write(int fd, char *ptr, int len)
//     {
//         (void)fd;
//         return Serial.write(ptr, len);
//     }
// }

// --------------------------------------------------------------------------
//
//      Constructor & destructor
//
// --------------------------------------------------------------------------

XL_320 ::XL_320() : errorcode(0), _pin_enable(11), id(0x01), model(0), version(0), verbose(false)
{

    pinMode(_pin_enable, OUTPUT);

    _serial = &Serial1;
    _serial->begin(1000000);
}

XL_320 ::~XL_320()
{
    // close(serial_port);
}

string XL_320 ::error()
{
    stringstream es;

    es << "Error (" << errorcode << "): ";
    switch (errorcode)
    {
    case ERROR_UNKNOWN:
        es << "Unknown error.";
        break;
    case ERROR_SERIAL_GET:
        es << "Error " << errno << " from tcgetattr: " << strerror(errno) << ".";
        break;
    case ERROR_SERIAL_SET:
        es << "Error " << errno << " from tcsetattr: " << strerror(errno) << ".";
        break;
    case ERROR_SERIAL_READ:
        es << "Error while reading: " << strerror(errno) << endl;
        break;
    case ERROR_NOSTATUS:
        es << "No Status received. >>> IS THE DEVICE POWERED ON ? <<<";
        break;
    case ERROR_INCOMPLETESTATUS:
        es << "Incomplete status received.";
        break;
    case ERROR_WRONGCNC_READ:
        es << "Wrong CNC, error during transmission.";
        break;
    case ERROR_RESULT_FAIL:
        es << "Failed to process the sent Instruction Packet.";
        break;
    case ERROR_INSTRUCTION:
        es << "Undefined Instruction has been used, or action has been used without Reg Write.";
        break;
    case ERROR_WRONGCNC_SENT:
        es << "CRC of the sent Packet does not match.";
        break;
    case ERROR_DATA_RANGE:
        es << "Data to be written in the corresponding Address is outside the range of the minimum/maximum value.";
        break;
    case ERROR_DATA_LENGTH:
        es << "Attempt to write Data that is shorter than the data length of the corresponding Address (ex: when you attempt to only use 2 bytes of a item that has been defined as 4 bytes).";
        break;
    case ERROR_DATA_LIMIT:
        es << "Data to be written in the corresponding Address is outside of the Limit value.";
        break;
    case ERROR_ACCESS:
        es << "Attempt to write a value in an Address that is Read Only or has not been defined; or attempt to read a value in an Address that is Write Only or has not been defined, or attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock).";
        break;
    }

    es << endl;

    return es.str();
}

void XL_320 ::setSerialPort(HardwareSerial &serial)
{
    _serial = &serial;
}

// --------------------------------------------------------------------------
//
//      Send and receive
//
// --------------------------------------------------------------------------

int XL_320 ::send(unsigned char Instruction, vector<int> param, bool receive)
{

    // Number of parameters
    int np = (int)param.size();

    // Write to serial port
    unsigned char msg[np + 10];

    // Header
    msg[0] = 0xFF;
    msg[1] = 0xFF;
    msg[2] = 0xFD;
    msg[3] = 0x00;

    // ID
    msg[4] = this->id;

    // Length
    msg[5] = np + 3;
    msg[6] = 0x00;

    // Instruction
    msg[7] = Instruction;

    // --- Parameters

    int i = 8;
    for (unsigned char p : param)
    {
        msg[i] = p;
        i++;
    }

    // CRC
    unsigned short crc = CRC(msg, np + 8);
    msg[i] = (crc & 0x00FF);
    msg[i + 1] = (crc >> 8) & 0x00FF;

    // === WRITE INSTRUCTION ================================================

    digitalWrite(_pin_enable, HIGH);
    _serial->write(msg, sizeof(msg));
    delayMicroseconds(100);
    digitalWrite(_pin_enable, LOW);

    // === READ STATUS ======================================================

    if (receive)
    {

        // Read serial buffer
        unsigned char buffer[32];
        int nb = _serial->readBytes(buffer, sizeof(buffer));

        if (nb < 0)
        {

            errorcode = ERROR_SERIAL_READ;
        }
        else if (nb == 0)
        {

            errorcode = ERROR_NOSTATUS;
        }
        else if (nb < 11)
        {

            errorcode = ERROR_INCOMPLETESTATUS;
        }
        else
        {

            // Check CRC
            crc = CRC(buffer, nb - 2);
            if ((buffer[nb - 2] != (crc & 0x00FF)) | (buffer[nb - 1] != ((crc >> 8) & 0x00FF)))
            {

                errorcode = ERROR_WRONGCNC_READ;

                // Device errors
            }
            else if (buffer[8] > 0)
            {

                errorcode = buffer[8];

            } // else {

            // --- Update results
            this->result.clear();
            for (int i = 9; i < nb - 2; i++)
            {
                this->result.push_back(buffer[i]);
            }

            if (verbose)
            {

                // Display sent packet
                dispPacket(msg, np, false);

                // Display returned packet
                cout << endl;
                cout << "Read " << dec << nb << " bytes.";
                dispPacket(buffer, nb - 11, true);
            }

            //}
        }
    }
    else if (verbose)
    {

        // Display sent packet
        dispPacket(msg, np, false);
    }

    return errorcode;
}

unsigned short XL_320 ::CRC(unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j, crc_accum = 0;
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

void XL_320 ::dispPacket(unsigned char *p, int np, bool status)
{

    cout << endl;

    if (status)
    {

        // Title line
        cout << "← |    Header   | ID | Size  | In | Er | ";
        if (np)
        {
            cout << setfill(' ') << setw(np * 3 - 1) << left;
            if (np == 1)
            {
                cout << "Pm | ";
            }
            else
            {
                cout << "Param | ";
            }
        }
        cout << " CRC  |" << endl;

        // Content line
        cout << "  |";
        cout << hex << uppercase;
        for (int i = 0; i < np + 11; i++)
        {
            cout << " " << setfill('0') << setw(2) << right << (int)p[i];
            if ((i == 3) | (i == 4) | (i == 6) | (i == 7) | (i == 8) | (i == np + 8) | (i == np + 10))
            {
                cout << " |";
            }
        }
        cout << endl;

        // Result
        cout << "Result:";
        for (unsigned int i = 0; i < result.size(); i++)
        {
            cout << " " << setfill('0') << setw(2) << right << (int)result[i];
        }
        cout << endl;
    }
    else
    {

        // Title line
        cout << "→ |    Header   | ID | Size  | In | ";
        if (np)
        {
            cout << setfill(' ') << setw(np * 3 - 1) << left << "Param"
                 << " | ";
        }
        cout << " CRC  |" << endl;

        // Content line
        cout << "  |";
        cout << hex << uppercase;
        for (int i = 0; i < np + 10; i++)
        {
            cout << " " << setfill('0') << setw(2) << right << (int)p[i];
            if ((i == 3) | (i == 4) | (i == 6) | (i == 7) | (i == np + 7) | (i == np + 9))
            {
                cout << " |";
            }
        }

        cout << endl;
    }

    return;
}

// --------------------------------------------------------------------------
//
//      Dynamixel protocol 2.0
//
// --------------------------------------------------------------------------

int XL_320 ::ping()
{
    vector<int> param;

    if (send(0x01, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    // Process result
    model = (int)result[1] << 8 | result[0];
    version = result[2];

    if (verbose)
    {
        cout << "Model " << dec << model << " (" << version << ")" << endl;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::factory_reset()
{
    vector<int> param;

    if (send(0x06, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::reboot()
{
    vector<int> param;

    if (send(0x08, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------
//
//      XL-320 specific commands
//
// --------------------------------------------------------------------------

int XL_320 ::getModelNumber()
{
    vector<int> param;

    // Instruction
    param.push_back(0);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::getFirmwareVersion()
{
    vector<int> param;

    // Instruction
    param.push_back(2);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setID(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(3);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getID()
{
    vector<int> param;

    // Instruction
    param.push_back(3);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setBaudRate(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(4);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getBaudRate()
{
    vector<int> param;

    // Instruction
    param.push_back(4);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setReturnDelayTime(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(5);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getReturnDelayTime()
{
    vector<int> param;

    // Instruction
    param.push_back(5);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setCWAngleLimit(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(6);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(v, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getCWAngleLimit()
{
    vector<int> param;

    // Instruction
    param.push_back(6);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::setCCWAngleLimit(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(8);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(v, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getCCWAngleLimit()
{
    vector<int> param;

    // Instruction
    param.push_back(8);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::setControlMode(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(11);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getControlMode()
{
    vector<int> param;

    // Instruction
    param.push_back(11);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setTemperatureLimit(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(12);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getTemperatureLimit()
{
    vector<int> param;

    // Instruction
    param.push_back(12);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setMinVoltageLimit(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(13);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getMinVoltageLimit()
{
    vector<int> param;

    // Instruction
    param.push_back(13);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setMaxVoltageLimit(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(14);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getMaxVoltageLimit()
{
    vector<int> param;

    // Instruction
    param.push_back(14);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setMaxTorque(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(15);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(v, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getMaxTorque()
{
    vector<int> param;

    // Instruction
    param.push_back(15);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::setStatusReturnLevel(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(17);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getStatusReturnLevel()
{
    vector<int> param;

    // Instruction
    param.push_back(17);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setShutdown(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(18);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getShutdown()
{
    vector<int> param;

    // Instruction
    param.push_back(18);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// ==========================================================================

int XL_320 ::setTorqueEnable(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(24);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getTorqueEnable()
{
    vector<int> param;

    // Instruction
    param.push_back(24);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setLED(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(25);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getLED()
{
    vector<int> param;

    // Instruction
    param.push_back(25);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setDGain(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(27);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getDGain()
{
    vector<int> param;

    // Instruction
    param.push_back(27);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setIGain(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(28);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getIGain()
{
    vector<int> param;

    // Instruction
    param.push_back(28);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setPGain(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(29);
    param.push_back(0);

    // Value
    param.push_back(v);

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getPGain()
{
    vector<int> param;

    // Instruction
    param.push_back(29);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::setGoalPosition(int pos)
{
    vector<int> param;

    // Instruction
    param.push_back(30);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(pos, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getGoalPosition()
{
    vector<int> param;

    // Instruction
    param.push_back(30);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::setSpeed(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(32);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(v, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getSpeed()
{
    vector<int> param;

    // Instruction
    param.push_back(32);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::setPunch(int v)
{
    vector<int> param;

    // Instruction
    param.push_back(51);
    param.push_back(0);

    // Value
    vector<unsigned char> arg = int2arg(v, 2);
    param.insert(param.end(), arg.begin(), arg.end());

    // Send
    if (send(0x03, param) > 0)
    {
        cout << error();
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------

int XL_320 ::getPunch()
{
    vector<int> param;

    // Instruction
    param.push_back(51);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::isMoving()
{
    vector<int> param;

    // Instruction
    param.push_back(49);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::getPresentPosition()
{
    vector<int> param;

    // Instruction
    param.push_back(37);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::getPresentSpeed()
{
    vector<int> param;

    // Instruction
    param.push_back(39);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::getPresentLoad()
{
    vector<int> param;

    // Instruction
    param.push_back(41);
    param.push_back(0);
    param.push_back(2);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)(result[1] << 8 | result[0]);
}

// --------------------------------------------------------------------------

int XL_320 ::getPresentVoltage()
{
    vector<int> param;

    // Instruction
    param.push_back(45);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::getPresentTemperature()
{
    vector<int> param;

    // Instruction
    param.push_back(46);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::getRegisteredInstruction()
{
    vector<int> param;

    // Instruction
    param.push_back(47);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------

int XL_320 ::getHardwareErrorStatus()
{
    vector<int> param;

    // Instruction
    param.push_back(50);
    param.push_back(0);
    param.push_back(1);
    param.push_back(0);

    // Send
    if (send(0x02, param, true) > 0)
    {
        cout << error();
        return -1;
    }

    return (int)result[0];
}

// --------------------------------------------------------------------------
//
//      Misc functions
//
// --------------------------------------------------------------------------

vector<unsigned char> int2arg(int i, int n)
{

    vector<unsigned char> vB;

    vB.push_back(i & 0x000000ff);
    if (n > 1)
    {
        vB.push_back((i & 0x0000ff00) >> 8);
    }

    if (n > 2)
    {
        vB.push_back((i & 0x00ff0000) >> 16);
    }

    if (n > 3)
    {
        vB.push_back((i & 0xff000000) >> 24);
    }

    return vB;
}
