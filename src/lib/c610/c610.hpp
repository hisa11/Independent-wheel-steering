#ifndef RCT_C610_HPP
#define RCT_C610_HPP

#include "mbed.h"

struct c610set
{
    uint16_t angle;
    int16_t rpm;
    int16_t ampere;
    uint8_t temp;
    void set_param(const uint8_t (&data)[8]);
};

class C610
{
public:
    C610(CAN &can);
    bool send_message();
    void set_power(int id, int power);
    void stop();
    void can_reset();

    void param_update();
    int16_t get_rpm(int id);
    uint16_t get_angle(int id);
    int16_t get_ampere(int id);
    int8_t get_temp(int id);
    int16_t get_send_power(int id);

private:
    int16_t send_power[8]; // 1~8のいずれか
    c610set param[8];
    CAN &can;
    const int max_power = 16000;
};

inline C610::C610(CAN &can) : can(can) {}

#endif