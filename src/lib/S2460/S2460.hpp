#ifndef S2460_HPP
#define S2460_HPP

namespace mbed {
class PwmOut;
}

class S2460 {
   public:
	static const int PULSE_REVERSE_MAX = 125;
	static const int PULSE_STOP = 187;
	static const int PULSE_FORWARD_MAX = 250;

	explicit S2460(int pwm_pin);
	~S2460();

	// 初期化
	void setup();

	// パルス幅(us)で出力
	void write_us(int pulse_us);

   private:
	mbed::PwmOut* servo;  // 実体はS2460.cppで生成
};

#endif	// S2460_HPP