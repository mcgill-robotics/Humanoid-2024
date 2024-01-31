#include "Arduino.h"
#include "IPAddress.h"
#include "Servo_XL320.h"
#include "control_table_xl320.h"

byte local_mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF};
IPAddress local_ip(192, 168, 1, 177);
IPAddress agent_ip(192, 168, 1, 113);
size_t agent_port = 8888;

void setup() {
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  while (!Serial1) {
  }
  set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip,
                                          agent_port);
}

void loop() {}