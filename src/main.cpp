#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"
#include "QEI.h"
#include "amt212c_v.hpp"
#include "c610.hpp"

BufferedSerial pc(USBTX, USBRX, 115200); // Nucleoのシリアルポート

DigitalIn button(BUTTON1);
serial_unit serial(pc);
PID steering_position_pid(7.55f, 0.0f, 0.0f, PID::Mode::POSITIONAL);
PID steering_velocity_pid(0.3f, 0.5f, 0.000001f, PID::Mode::VELOCITY);

CAN can1(PA_11, PA_12, 1000000);
CAN can2(PB_12, PB_13, 1000000);
C610 DJI(can1);
Amt212CV encoder(PA_9, PA_10, D6, 0x4z8);
int32_t aa_position = 0;
float stick_x = 0.00f, stick_y = 0.00f;

void move_aa(std::string msg)
{
}
void pid_thread()
{
    auto pre_time = HighResClock::now();
    steering_position_pid.set_output_limits(-8000, 8000);
    steering_velocity_pid.set_output_limits(-10000, 10000);
    steering_position_pid.set_deadband(50.0f);
    while (1)
    {
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time -
                                                                         pre_time)
                       .count() /
                   1000000.0f;
        steering_position_pid.set_dt(dt);
        steering_velocity_pid.set_dt(dt);
        pre_time = now_time;
        if (encoder.update())
        {
            aa_position = encoder.get_position() + 1448;
        }
        else
        {
            printf("Encoder update failed!\n");
        }
        steering_velocity_pid.set_goal(-steering_position_pid.do_pid(aa_position));
        if(aa_position > 16232 || aa_position < -14784)
        {
            steering_velocity_pid.set_goal(0);
        }
        DJI.set_power(1, steering_velocity_pid.do_pid(DJI.get_rpm(1)));

        // PID制御ループ
        ThisThread::sleep_for(30ms);
    }
}
int main()
{
    encoder.set_mode(Amt212CV::Mode::Continuous);
    Thread thread;
    thread.start(serial_read);
    Thread pid_thread_handle;
    pid_thread_handle.start(pid_thread);
    while (1)
    {
        if (button.read() == 0)
        {
            steering_position_pid.set_goal(8000);
        }
        else
        {
            steering_position_pid.set_goal(0);
        }
        DJI.send_message();
        // printf("AA Position: %ld\n", aa_position);
        printf("position:%d,send_message:%d,goal_speed:%f\n", aa_position, DJI.get_rpm(1), steering_velocity_pid.get_goal());
        ThisThread::sleep_for(30ms);
    }
}