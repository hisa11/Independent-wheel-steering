#ifndef AMT212C_V_HPP
#define AMT212C_V_HPP

#include <mbed.h>
#include <cmath>

namespace anglelib {

constexpr float PI = 3.14159265358979323846f;
constexpr float TAU = 2.0f * PI;

template<typename T>
constexpr bool is_equal_approx(T a, T b) {
    constexpr T EPS = 1e-5f;
    return std::abs(a - b) < EPS;
}

template<typename T>
constexpr T wrapf(T value, T min, T max) {
    T range = max - min;
    if (is_equal_approx(range, T(0))) return min;
    T result = value - range * std::floor((value - min) / range);
    return is_equal_approx(result, max) ? min : result;
}

template<typename Rep>
struct Angle {
    Rep value;
    Angle() : value(0) {}
    explicit Angle(Rep rad) : value(rad) {}

    static Angle from_deg(Rep deg) { return Angle(deg * (TAU / 360)); }
    static Angle from_rad(Rep rad) { return Angle(rad); }

    Rep rad() const { return value; }
    Rep deg() const { return value * (360.0f / TAU); }

    Angle operator+(const Angle& rhs) const { return Angle(value + rhs.value); }
    Angle operator-(const Angle& rhs) const { return Angle(value - rhs.value); }
    Angle operator*(Rep rhs) const { return Angle(value * rhs); }
    Angle abs() const { return Angle(std::abs(value)); }
};

using Anglef = Angle<float>;

} // namespace anglelib

class Amt212CV {
public:
    enum class Mode {
        Wrapped,  // 0 ~ 16383 ticks, 0 ~ 2pi radians
        Continuous  // -inf ~ +inf
    };

    Amt212CV(PinName tx, PinName rx, PinName dere, uint8_t address, int baud = 2000000);
    bool update();
    anglelib::Anglef get_angle() const;
    int32_t get_position() const;
    void set_mode(Mode m);

private:
    UnbufferedSerial rs485;
    DigitalOut de;
    Timer timer;
    uint8_t addr;
    float scale = 1.0f;
    anglelib::Anglef offset = anglelib::Anglef::from_rad(0.0f);
    anglelib::Anglef angle{};
    anglelib::Anglef last_angle{};
    int32_t ticks = 0;
    bool initialized = false;
    Mode mode = Mode::Wrapped;

    void flush();
    void send(const void* data, size_t len);
    bool recv(void* data, size_t len, std::chrono::microseconds timeout);
    bool read_angle(anglelib::Anglef& result, int16_t& delta_ticks);
    static bool is_valid(uint16_t data);
};

#endif // AMT212C_V_HPP
