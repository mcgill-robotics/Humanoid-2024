#include <Arduino.h>

#undef max
#undef min
#include <stdio.h>
#include <iostream>

#include "XL_320.hpp"

using namespace std;

XL_320 Servo1;

// #define cout Serial
extern "C"
{
    int _write(int fd, char *ptr, int len)
    {
        (void)fd;
        return Serial.write(ptr, len);
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial)
        ;
    Serial.println("START");
    cout << "COUT REDIRECTED!!!\r\n";
    delay(1000);
    cout << "COUT REDIRECTED!!!\r\n";

    Servo1.verbose = true;
}

void loop()
{
    // cout << "Hello, world!" << endl;
    Servo1.ping();

    // Test LED colors
    for (int i = 0; i < 8; i++)
    {
        int status = Servo1.setLED(i);
        cout << "LED status: " << status << endl;
        delayMicroseconds(1000000);
    }
}
