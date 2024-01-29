#include "Arduino.h"
#include "control_table_xl320.h"
#include "Servo_XL320.h"

void setup()
{
    Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
    while(!Serial1);

}

void loop()
{


}