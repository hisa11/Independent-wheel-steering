#ifndef FIRSTPENGUIN_HPP
#define FIRSTPENGUIN_HPP

#include <mbed.h>

class FP
{
public:
    static constexpr int max = INT16_MAX;
    uint32_t send_id;
    int16_t pwm[4] = {};

    struct ReceiveData
    {
        int32_t enc;
        uint32_t adc;
        void set(const uint8_t data[8]) { memcpy(this, data, sizeof(*this)); }
    } receive[4] = {};

    // コンストラクタ
    FP(uint32_t id, CAN &can) : send_id(id), can(can) {}

    // メンバーへのアクセスを可能にする関数
    const ReceiveData *getReceiveData() const { return receive; }
    int16_t *getPwmData() { return pwm; }
    bool send();
    void read(const CANMessage &msg);
    void can_reset() { can.reset(); }

private:
    CAN &can;
};

#endif