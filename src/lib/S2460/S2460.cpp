#include "S2460.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>

#include "mbed.h"

using namespace std::chrono_literals;
using mbed::PwmOut;

// S2460 実装
S2460::S2460(int pwm_pin)
{
	servo = new PwmOut(static_cast<PinName>(pwm_pin));
	// S2460の仕様に合わせてPWM周波数を100Hz (周期10ms) に設定
	servo->period_ms(10);
}

S2460::~S2460()
{
	delete servo;
	servo = nullptr;
}

void S2460::setup()
{
	printf("--- S2460 Arming Sequence Start ---\n");

	// 1. 最大後退信号 (125us) を1秒間送信
	printf("Step 1: Sending REVERSE signal (125us) for 1s...\n");
	servo->pulsewidth_us(PULSE_REVERSE_MAX);
	ThisThread::sleep_for(1000ms);

	// 2. 停止信号 (187us) を1秒間送信
	printf("Step 2: Sending STOP signal (187us) for 1s...\n");
	servo->pulsewidth_us(PULSE_STOP);
	ThisThread::sleep_for(1000ms);

	printf("--- Arming Sequence Complete ---\n");
}

void S2460::write_us(int pulse_us)
{
	if (pulse_us < PULSE_REVERSE_MAX)
		pulse_us = PULSE_REVERSE_MAX;
	else if (pulse_us > PULSE_FORWARD_MAX)
		pulse_us = PULSE_FORWARD_MAX;
	servo->pulsewidth_us(pulse_us);
}