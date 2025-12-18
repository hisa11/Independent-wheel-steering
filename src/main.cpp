#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"
#include "QEI.h"
#include "amt212c_v.hpp"
#include "c610.hpp"
#include "Rs485.h"
#include "S2460.hpp"
#include "key.hpp"

// Watchdogタイマーのインクルード
Watchdog &watchdog = Watchdog::get_instance();

BufferedSerial pc(USBTX, USBRX, 115200);

DigitalIn button(BUTTON1);
DigitalOut led(LED1);
serial_unit serial(pc);

S2460 esc(D6);

PID steering_position_pid[4] = {
    PID(7.55, 0.0, 0.01, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.01, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.01, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.01, PID::Mode::POSITIONAL)};
PID steering_velocity_pid[4] = {
    PID(0.30, 0.5, 0.0001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.0001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.0001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.0001, PID::Mode::VELOCITY)};
PID tire_pid[4] = {
    PID(0.80, 0.9, 0.01, PID::Mode::VELOCITY),
    PID(0.80, 0.9, 0.01, PID::Mode::VELOCITY),
    PID(0.80, 0.9, 0.01, PID::Mode::VELOCITY),
    PID(0.80, 0.9, 0.01, PID::Mode::VELOCITY)};

CAN can1(PA_11, PA_12, 1000000);
CAN can2(PB_12, PB_13, 1000000);
Mutex can2_mutex;
Mutex encoder_mutex;
Mutex rs485_mutex;

C610 DJI(can1);

// RS485 bus and Amt21 encoders
Rs485 rs485{PA_0, PA_1, (int)2e6, D6};
Amt21 encoder0{0x48, rs485};
Amt21 encoder1{0x54, rs485};
Amt21 encoder2{0x5C, rs485};
Amt21 encoder3{0x58, rs485};

int amt212c_v_error[4] = {326, -24, -1509, 621};
int amt212c_v_position[4] = {0, 0, 0, 0};
float stick_x = 0.00f, stick_y = 0.00f, stick_r = 0.00f;
int16_t tire_power[4] = {0, 0, 0, 0};

const float ALPHA = 0.8f;
float enc_filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f};
int16_t enc[4] = {329, 30, -1476, 632};

// *** Watchdog用のフラグ追加 ***
volatile bool watchdog_fed = false;

int esc_power = S2460::PULSE_STOP;

void encoder_update_thread()
{
    Amt21 *encoders_arr[4] = {&encoder0, &encoder1, &encoder2, &encoder3};
    int current_encoder = 0;
    int debug_counter = 0;
    int error_count[4] = {0, 0, 0, 0};

    while (1)
    {
        // Mutexのタイムアウト付きロック
        if (!rs485_mutex.trylock_for(100ms))
        {
            ThisThread::sleep_for(10ms);
            continue;
        }

        bool update_success = encoders_arr[current_encoder]->request_pos();
        int new_position = 0;

        if (update_success)
        {
            new_position = encoders_arr[current_encoder]->pos - amt212c_v_error[current_encoder];
            error_count[current_encoder] = 0;
        }
        else
        {
            error_count[current_encoder]++;
        }

        rs485_mutex.unlock();

        if (error_count[current_encoder] > 10)
        {
            error_count[current_encoder] = 0;
            ThisThread::sleep_for(2ms);
        }

        if (update_success)
        {
            if (!encoder_mutex.trylock_for(50ms))
            {
                ThisThread::sleep_for(10ms);
                continue;
            }
            amt212c_v_position[current_encoder] = new_position;
            encoder_mutex.unlock();
        }

        current_encoder = (current_encoder + 1) % 4;
        ThisThread::sleep_for(30ms);

        if (++debug_counter >= 140)
        {
            debug_counter = 0;
        }
    }
}

void sensor_thread()
{
    CANMessage enc_msg;
    while (1)
    {
        if (!can2_mutex.trylock_for(50ms))
        {
            ThisThread::sleep_for(10ms);
            continue;
        }

        bool read_success = can2.read(enc_msg, 10);
        can2_mutex.unlock();

        if (read_success)
        {
            if (enc_msg.id == 10)
            {
                for (int i = 0; i < 4; i++)
                {
                    enc[i] = -enc_msg.data[i * 2 + 1] << 8 | enc_msg.data[i * 2];
                    enc_filtered[i] = ALPHA * enc[i] + (1.0f - ALPHA) * enc_filtered[i];
                }
            }
        }

        ThisThread::sleep_for(30ms);
    }
}

void move_aa(std::string msg)
{
    const float COUNTS_PER_90_DEG = 1000.0f;
    const float COUNTS_PER_180_DEG = 2000.0f;
    const float COUNTS_PER_ROTATION = 4000.0f;
    const float COUNTS_PER_RAD = COUNTS_PER_90_DEG / (M_PI / 2.0f);

    const int32_t HW_LIMIT_PLUS = 1000;
    const int32_t HW_LIMIT_MINUS = -1000;

    const float MAX_RPM = 10000.0f;

    const float L = 0.5f;
    const float W = 0.5f;

    msg.erase(0, 2);
    std::vector<double> joys_d = to_numbers(msg);
    std::vector<float> joys(joys_d.begin(), joys_d.end());

    bool all_zero = true;
    for (auto &joy : joys)
    {
        if (joy > -0.08 && joy < 0.08)
        {
            joy = 0.0f;
        }
        else
        {
            all_zero = false;
        }
    }

    if (all_zero)
    {
        for (int i = 0; i < 4; i++)
        {
            tire_pid[i].set_goal(0.0f);
        }
        return;
    }

    float Vx_base = joys[1];
    float Vy_base = joys[0];
    float Omega = joys[2];

    float vx_0 = Vx_base + Omega * (-W);
    float vy_0 = Vy_base + Omega * (-L);

    float vx_1 = Vx_base + Omega * (-W);
    float vy_1 = Vy_base + Omega * L;

    float vx_2 = Vx_base + Omega * W;
    float vy_2 = Vy_base + Omega * L;

    float vx_3 = Vx_base + Omega * W;
    float vy_3 = Vy_base + Omega * (-L);

    float vx[4] = {vx_0, vx_1, vx_2, vx_3};
    float vy[4] = {vy_0, vy_1, vy_2, vy_3};

    float max_speed = 0.0f;
    float wheel_speeds_vec[4];

    for (int i = 0; i < 4; i++)
    {
        wheel_speeds_vec[i] = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
        if (wheel_speeds_vec[i] > max_speed)
        {
            max_speed = wheel_speeds_vec[i];
        }
    }

    if (max_speed > 1.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            wheel_speeds_vec[i] /= max_speed;
        }
    }

    int current_positions[4];
    if (!encoder_mutex.trylock_for(50ms))
    {
        return; // タイムアウト時は処理をスキップ
    }
    for (int i = 0; i < 4; i++)
    {
        current_positions[i] = amt212c_v_position[i];
    }
    encoder_mutex.unlock();

    bool is_moving = (fabsf(Vx_base) > 0.001f || fabsf(Vy_base) > 0.001f || fabsf(Omega) > 0.001f);

    for (int i = 0; i < 4; i++)
    {
        float target_rpm = 0.0f;

        if (is_moving)
        {
            target_rpm = wheel_speeds_vec[i] * MAX_RPM;
        }

        float target_rad = atan2f(vy[i], vx[i]);

        int32_t current_count = current_positions[i];

        float target_count_raw = target_rad * COUNTS_PER_RAD;

        float current_count_normalized = remainderf((float)current_count, COUNTS_PER_ROTATION);

        float target_count_normalized = remainderf(target_count_raw, COUNTS_PER_ROTATION);

        float target_count_adjusted = target_count_normalized;
        float drive_direction = 1.0f;

        if (target_count_adjusted > COUNTS_PER_90_DEG)
        {
            target_count_adjusted -= COUNTS_PER_180_DEG;
            drive_direction = -1.0f;
        }
        else if (target_count_adjusted < -COUNTS_PER_90_DEG)
        {
            target_count_adjusted += COUNTS_PER_180_DEG;
            drive_direction = -1.0f;
        }

        float current_count_adjusted = current_count_normalized;
        if (current_count_adjusted > COUNTS_PER_90_DEG)
        {
            current_count_adjusted -= COUNTS_PER_180_DEG;
        }
        else if (current_count_adjusted < -COUNTS_PER_90_DEG)
        {
            current_count_adjusted += COUNTS_PER_180_DEG;
        }

        float delta_pulse = target_count_adjusted - current_count_adjusted;

        if (delta_pulse > COUNTS_PER_180_DEG)
        {
            delta_pulse -= COUNTS_PER_ROTATION;
        }
        else if (delta_pulse < -COUNTS_PER_180_DEG)
        {
            delta_pulse += COUNTS_PER_ROTATION;
        }

        int32_t final_target_count = (int32_t)current_count_adjusted + (int32_t)delta_pulse;

        if (final_target_count > HW_LIMIT_PLUS)
        {
            final_target_count = HW_LIMIT_PLUS;
        }
        else if (final_target_count < HW_LIMIT_MINUS)
        {
            final_target_count = HW_LIMIT_MINUS;
        }

        if (!is_moving)
        {
            final_target_count = (int32_t)current_count_adjusted;
        }

        float final_target_rpm = target_rpm * drive_direction;

        steering_position_pid[i].set_goal(final_target_count);
        tire_pid[i].set_goal(final_target_rpm);
    }
}

void key_thread()
{
    while (1)
    {
        if (Cross == true)
        {
            esc_power = S2460::PULSE_FORWARD_MAX;
        }
        else
        {
            esc_power = S2460::PULSE_STOP;
        }
    }
}

void pid_thread()
{
    auto pre_time = HighResClock::now();
    for (int i = 0; i < 4; i++)
    {
        steering_position_pid[i].set_output_limits(-8000, 8000);
        steering_velocity_pid[i].set_output_limits(-10000, 10000);
        steering_position_pid[i].set_deadband(20.0f);
        tire_pid[i].set_output_limits(-20000, 20000);
    }

    while (1)
    {
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time - pre_time).count() / 1000000.0f;

        for (int i = 0; i < 4; i++)
        {
            steering_position_pid[i].set_dt(dt);
            steering_velocity_pid[i].set_dt(dt);
            tire_pid[i].set_dt(dt);
        }
        pre_time = now_time;

        int local_encoder_positions[4];
        if (!encoder_mutex.trylock_for(50ms))
        {
            ThisThread::sleep_for(20ms);
            continue;
        }
        for (int i = 0; i < 4; i++)
        {
            local_encoder_positions[i] = amt212c_v_position[i];
        }
        encoder_mutex.unlock();

        for (int i = 0; i < 4; i++)
        {
            if (local_encoder_positions[i] > 1500 || local_encoder_positions[i] < -1500)
            {
                steering_position_pid[i].reset();
                steering_velocity_pid[i].reset();
                steering_velocity_pid[i].set_goal(0);
                tire_pid[i].set_goal(0);
            }
        }

        for (int i = 0; i < 4; i++)
        {
            steering_velocity_pid[i].set_goal(-steering_position_pid[i].do_pid(local_encoder_positions[i]));
            DJI.set_power(i + 1, steering_velocity_pid[i].do_pid(DJI.get_rpm(i + 1)));
            tire_power[i] = tire_pid[i].do_pid(enc_filtered[i]);
        }

        ThisThread::sleep_for(20ms);
    }
}

void led_thread()
{
    while (1)
    {
        led = !led;
        ThisThread::sleep_for(500ms);
    }
}

// *** Watchdog監視専用スレッド ***
void watchdog_thread()
{
    while (1)
    {
        watchdog.kick();
        ThisThread::sleep_for(1000ms); // 1秒ごとにキック
    }
}

int main()
{
    // Watchdogを10秒に延長（より安全なマージン）
    watchdog.start(10000);

    pc.set_blocking(false);

    serial.start_event_mode();

    for (int i = 0; i < 4; i++)
    {
        steering_position_pid[i].set_goal(0);
    }

    // *** Watchdog専用スレッドを最高優先度で起動 ***
    Thread watchdog_thread_handle(osPriorityRealtime, 1024);
    watchdog_thread_handle.start(watchdog_thread);

    Thread thread(osPriorityNormal, 4096);
    thread.start(serial_read);

    Thread encoder_thread_handle(osPriorityHigh, 2048);
    encoder_thread_handle.start(encoder_update_thread);

    Thread pid_thread_handle(osPriorityHigh, 4096);
    pid_thread_handle.start(pid_thread);

    Thread sensor_thread_handle(osPriorityNormal, 2048);
    sensor_thread_handle.start(sensor_thread);

    Thread led_thread_handle(osPriorityLow, 1024);
    led_thread_handle.start(led_thread);

    Thread key_thread_handle(osPriorityNormal, 1024);
    key_thread_handle.start(key_thread);

    esc.setup();

    while (1)
    {
        DJI.send_message();

        CANMessage msg(4, (const uint8_t *)tire_power, 8);
        if (can2_mutex.trylock_for(50ms))
        {
            can2.write(msg);
            can2_mutex.unlock();
        }

        if (encoder_mutex.trylock_for(50ms))
        {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "steering_positions: W0=%d, W1=%d, W2=%d, W3=%d\n",
                     (int)amt212c_v_position[0],
                     (int)amt212c_v_position[1],
                     (int)amt212c_v_position[2],
                     (int)amt212c_v_position[3]);
            pc.write(buffer, strlen(buffer));
            encoder_mutex.unlock();
            esc.write_us(esc_power);
        }

        ThisThread::sleep_for(30ms);
    }
}