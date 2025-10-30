#include "amt212c_v.hpp"

Amt212CV::Amt212CV(PinName tx, PinName rx, PinName dere, uint8_t address, int baud)
    : rs485(tx, rx, baud), de(dere), addr(address) {
    de = 0;
    rs485.set_blocking(false);
    timer.start();
}

void Amt212CV::flush() {
    uint8_t dummy;
    while (rs485.readable()) rs485.read(&dummy, 1);
}

void Amt212CV::send(const void* data, size_t len) {
    de = 1;
    rs485.write(data, len);
    wait_us(5);
    de = 0;
}

bool Amt212CV::recv(void* data, size_t len, std::chrono::microseconds timeout) {
    timer.reset();
    size_t read = 0;
    while (read < len) {
        if (rs485.readable()) {
            int ret = rs485.read((char*)data + read, len - read);
            if (ret > 0) read += ret;
        }
        if (timer.elapsed_time() > timeout) return false;
    }
    return true;
}

bool Amt212CV::is_valid(uint16_t data) {
    auto b = [data](int pos) { return (data >> pos) & 1; };
    bool k1 = b(15);
    bool k0 = b(14);
    bool k1_calc = !(b(13) ^ b(11) ^ b(9) ^ b(7) ^ b(5) ^ b(3) ^ b(1));
    bool k0_calc = !(b(12) ^ b(10) ^ b(8) ^ b(6) ^ b(4) ^ b(2) ^ b(0));
    return (k1 == k1_calc) && (k0 == k0_calc);
}

bool Amt212CV::read_angle(anglelib::Anglef& result, int16_t& delta_ticks) {
    flush();
    send(&addr, 1);
    uint16_t raw;
    if (!recv(&raw, 2, 300us)) return false;
    if (!is_valid(raw)) return false;

    uint16_t raw_ticks = raw & 0x3FFF;
    float new_rad = raw_ticks * ((2.0f * anglelib::PI) / 16384.0f);
    auto new_angle = anglelib::Anglef::from_rad(new_rad);

    int16_t dt = raw_ticks - (ticks & 0x3FFF);
    if (dt > 8192) dt -= 16384;
    if (dt < -8192) dt += 16384;

    delta_ticks = dt;
    result = new_angle;
    return true;
}

bool Amt212CV::update() {
    anglelib::Anglef raw_angle;
    int16_t dt;
    if (!read_angle(raw_angle, dt)) return false;

    if (!initialized) {
        last_angle = raw_angle;
        initialized = true;
        ticks = 0;
    } else {
        ticks += dt;
    }

    switch (mode) {
        case Mode::Wrapped:
            angle = raw_angle;
            break;
        case Mode::Continuous:
            angle = anglelib::Anglef::from_rad(ticks * ((2.0f * anglelib::PI) / 16384.0f));
            break;
    }
    return true;
}

anglelib::Anglef Amt212CV::get_angle() const {
    return angle;
}

int32_t Amt212CV::get_position() const {
    return ticks;
}

void Amt212CV::set_mode(Mode m) {
    mode = m;
}
