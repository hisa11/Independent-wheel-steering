#include "c610.hpp"

bool C610::send_message()
{
    uint8_t buf[16];

    for (int i = 0; i < 8; i++)
    {
        buf[i * 2] = (send_power[i] >> 8) & 0xff;
        buf[i * 2 + 1] = send_power[i] & 0xff;
    }
    bool success = true;
    if (!can.write(CANMessage(0x200, buf, 8)))
    {
        // printf("[c610]can't send(1~4)\n");
        success = false;
    }
    wait_us(500);
    if (!can.write(CANMessage(0x1FF, buf + 8, 8)))
    { // bufの後半部分を送信
        // printf("[c610] can't send(5~8)\n");
        success = false;
    }

    return success;
}

void C610::set_power(int id, int power)
{
    if (power > max_power)
        send_power[id - 1] = max_power;
    else if (power < -max_power)
        send_power[id - 1] = -max_power;
    else
        send_power[id - 1] = power;
}

void C610::stop()
{
    std::fill(std::begin(send_power), std::end(send_power), 0);
}

void c610set::set_param(const uint8_t (&data)[8])
{
    angle = uint16_t(data[0] << 8 | data[1]);
    rpm = int16_t(data[2] << 8 | data[3]);
    ampere = int16_t(data[4] << 8 | data[5]);
    temp = data[6];
}

void C610::param_update()
{
    CANMessage msg;
    if (can.read(msg) && msg.format == CANStandard && msg.type == CANData && msg.len == 8)
    {
        if (msg.id >= 0x200 && msg.id <= 0x208)
        {
            param[msg.id - 0x201u].set_param(msg.data);
        }
    }
}
void C610::can_reset()
{
    can.reset();
}

int16_t C610::get_rpm(int id)
{
    param_update();
    return param[id - 1].rpm;
}

uint16_t C610::get_angle(int id)
{
    param_update();
    return param[id - 1].angle;
}

int16_t C610::get_ampere(int id)
{
    param_update();
    return param[id - 1].ampere;
}

int8_t C610::get_temp(int id)
{
    param_update();
    return param[id - 1].temp;
}

int16_t C610::get_send_power(int id)
{
    return send_power[id - 1];
}