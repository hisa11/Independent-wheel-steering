#ifndef AMT212C_V_HPP
#define AMT212C_V_HPP

#include <mbed.h>
#include "Rs485.h"

struct Amt21 {
  static constexpr int rotate = 4096;

  uint8_t address;
  int32_t pos;
  uint16_t pre_pos;
  Rs485& rs485;

  Amt21(uint8_t addr, Rs485& rs485_bus) : address(addr), pos(0), pre_pos(0), rs485(rs485_bus) {}

  bool request_pos() {
    rs485.uart_transmit({address});
    uint16_t now_pos;
    bool received = rs485.uart_receive(&now_pos, sizeof(now_pos), 10ms);
    if(received && is_valid(now_pos)) {
      now_pos = (now_pos & 0x3fff) >> 2;
      int16_t diff = now_pos - pre_pos;
      if(diff > rotate / 2) {
        diff -= rotate;
      } else if(diff < -rotate / 2) {
        diff += rotate;
      }
      pos += diff;
      pre_pos = now_pos;
      return true;
    }
    return false;
  }

  void request_reset() {
    uint8_t cmd[2] = {static_cast<uint8_t>(address + 2), 0x75};
    rs485.uart_transmit(cmd);
  }

  static bool is_valid(uint16_t raw_data) {
    bool k1 = raw_data >> 15;
    bool k0 = raw_data >> 14 & 1;
    raw_data <<= 2;
    do {
      k1 ^= raw_data & 0x8000;
      k0 ^= (raw_data <<= 1) & 0x8000;
    } while(raw_data <<= 1);
    return k0 && k1;
  }
};

#endif
