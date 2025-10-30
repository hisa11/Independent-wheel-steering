#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"
#include "QEI.h"
#include "amt212c_v.hpp"
#include "c610.hpp"

BufferedSerial pc(USBTX, USBRX, 115200); // Nucleoのシリアルポート

DigitalIn button(BUTTON1);
serial_unit serial(pc);
PID steering_position_pid[4] = {
    PID(4.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(4.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(4.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(4.55, 0.0, 0.000001, PID::Mode::POSITIONAL)};
PID steering_velocity_pid[4] = {
    PID(0.30, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(0.30, 0.5, 0.000001, PID::Mode::VELOCITY)};
PID tire_pid[4] = {
    PID(0.50, 0.8, 0.000001, PID::Mode::VELOCITY),
    PID(0.50, 0.8, 0.000001, PID::Mode::VELOCITY),
    PID(0.50, 0.8, 0.000001, PID::Mode::VELOCITY),
    PID(0.50, 0.8, 0.000001, PID::Mode::VELOCITY)};

CAN can1(PA_11, PA_12, 1000000);
CAN can2(PB_12, PB_13, 1000000);
C610 DJI(can1);
Amt212CV encoder0(PA_9, PA_10, D6, 0x48);
Amt212CV encoder1(PA_9, PA_10, D6, 0x50);
Amt212CV encoder2(PA_9, PA_10, D6, 0x5c);
Amt212CV encoder3(PA_9, PA_10, D6, 0x58);
int amt212c_v_error[4] = {5828, 5940, 2076, 2500};
int amt212c_v_position[4] = {0, 0, 0, 0};
float stick_x = 0.00f, stick_y = 0.00f, stick_r = 0.00f;
int tire_power[4] = {0, 0, 0, 0};

const float ALPHA = 0.8f;                         // ローパスフィルタ係数 (0.0-1.0, 小さいほど滑らか)
float enc_filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // フィルタ済みエンコーダー値
int16_t enc[4] = {0, 0, 0, 0};

void sensor_thread()
{
    CANMessage enc_msg;
    while (1)
    {
        can2.read(enc_msg);
        if (enc_msg.id == 10)
        {
            for (int i = 0; i < 4; i++)
            {
                enc[i] = -enc_msg.data[i * 2 + 1] << 8 | enc_msg.data[i * 2];
                // ローパスフィルタ適用: y[n] = α * x[n] + (1 - α) * y[n-1]
                enc_filtered[i] = ALPHA * enc[i] + (1.0f - ALPHA) * enc_filtered[i];
            }
        }
        else
        {
            printf("Unknown CAN ID: %d\n", (int)(enc_msg.id));
        }
        ThisThread::sleep_for(30ms);
    }
}

void move_aa(std::string msg)
{
    // --- 制御用定数 ---
    // 1回転 (2*PIラジアン) に相当するエンコーダのパルス数 (14bit)
    const float PULSE_PER_ROTATION = 16384.0f;
    // 90度 (PI/2ラジアン) に相当するパルス数
    const float PULSE_90_DEG = PULSE_PER_ROTATION / 4.0f; // 4096.0f
    // 180度 (PIラジアン) に相当するパルス数
    const float PULSE_180_DEG = PULSE_PER_ROTATION / 2.0f; // 8192.0f
    // ラジアン -> パルス 換算係数
    const float PULSE_PER_RAD = PULSE_PER_ROTATION / (2.0f * M_PI);

    // ハードウェアリミット (pid_threadから引用)
    // 左右1周 (±16384) よりも小さい値 = 「めいいっぱい使わない」要求に対応
    const int32_t HW_LIMIT_PLUS = 16232;
    const int32_t HW_LIMIT_MINUS = -14784;

    // タイヤ速度リミット (ユーザー指定)
    const float MAX_RPM = 5000.0f;

    // ロボットのジオメトリ (中心からホイールまでの距離) [m]
    // ※※※ この値は実際のロボットに合わせて調整してください ※※※
    const float L = 0.5f; // 中心のX軸(前後)からホイールまでの距離
    const float W = 0.5f; // 中心のY軸(左右)からホイールまでの距離

    // --- 1. スティック入力のパースとデッドバンド ---
msg.erase(0, 2);
std::vector<double> joys_d = to_numbers(msg);
std::vector<float> joys(joys_d.begin(), joys_d.end());
for (auto &joy : joys)
{
    if (joy > -0.08 && joy < 0.08)
    {
        joy = 0.0;
    }
}
// ロボット座標系にマッピング
float Vx_base = joys[1]; // stick_y: 前後 (X軸)
float Vy_base = joys[0]; // stick_x: 左右 (Y軸) ※左を正とする
float Omega = joys[2]; // stick_r: 回転 (Z軸) ※反時計回りを正とする

    // --- 2. スワーブキネマティクス計算 ---
    // 各ホイールの速度ベクトル [Vx, Vy] を計算
    // ホイール位置 (X, Y): 
    //   0:FL(L, W), 1:FR(L, -W), 2:RL(-L, W), 3:RR(-L, -W)
    // V_wheel = V_base + Omega x R_wheel
    // Vx_i = Vx_base - Omega * Y_i
    // Vy_i = Vy_base + Omega * X_i

    // ホイール0: 前左 (Front Left) (L, W)
    float vx_fl = Vx_base - Omega * W;
    float vy_fl = Vy_base + Omega * L;

    // ホイール1: 前右 (Front Right) (L, -W)
    float vx_fr = Vx_base - Omega * (-W);
    float vy_fr = Vy_base + Omega * L;

    // ホイール2: 後左 (Rear Left) (-L, W)
    float vx_rl = Vx_base - Omega * W;
    float vy_rl = Vy_base + Omega * (-L);

    // ホイール3: 後右 (Rear Right) (-L, -W)
    float vx_rr = Vx_base - Omega * (-W);
    float vy_rr = Vy_base + Omega * (-L);

    // 計算された速度ベクトルを配列に格納
    float vx[4] = {vx_fl, vx_fr, vx_rl, vx_rr};
    float vy[4] = {vy_fl, vy_fr, vy_rl, vy_rr};

    // --- 3. 速度の正規化 ---
    float max_speed = 0.0f;
    float wheel_speeds_vec[4]; // 各ホイールの速度ベクトルの大きさ

    for (int i = 0; i < 4; i++) {
        wheel_speeds_vec[i] = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
        if (wheel_speeds_vec[i] > max_speed) {
            max_speed = wheel_speeds_vec[i];
        }
    }

    // 最大速度が 1.0 (スティックの最大入力合成値) を超えた場合、全ホイールの速度を正規化
    if (max_speed > 1.0f) {
        for (int i = 0; i < 4; i++) {
            // vx[i] /= max_speed; // 角度計算のために正規化前の値を使う
            // vy[i] /= max_speed;
            wheel_speeds_vec[i] /= max_speed; // 速度の大きさだけ正規化
        }
    }

    // --- 4. 各ホイールの目標値設定ループ ---
    for (int i = 0; i < 4; i++)
    {
        // a. 目標速度 (RPM)
        // スティックが中央 (Vx, Vy, Omegaがすべて0) の場合、速度は0
        float target_rpm = 0.0f;
        if (fabsf(Vx_base) > 0.001f || fabsf(Vy_base) > 0.001f || fabsf(Omega) > 0.001f) {
             target_rpm = wheel_speeds_vec[i] * MAX_RPM;
        }


        // b. 目標角度 (ラジアン)
        // atan2f(vy, vx) を使い、「前」 (vx > 0, vy = 0) を 0 ラジアンとする
        float target_rad = atan2f(vy[i], vx[i]);

        // c. 90°最適化ロジック
        int32_t current_pulse = amt212c_v_position[i]; // pid_threadで更新される現在角度
        float target_pulse_base = target_rad * PULSE_PER_RAD;
        float current_pulse_norm = remainderf((float)current_pulse, PULSE_PER_ROTATION);
        float delta_pulse = remainderf(target_pulse_base - current_pulse_norm, PULSE_PER_ROTATION);

        float optimized_delta_pulse;
        float drive_direction = 1.0f;

        if (fabsf(delta_pulse) <= PULSE_90_DEG) {
            // A. 差が±90°以内：そのまま
            optimized_delta_pulse = delta_pulse;
            drive_direction = 1.0f;
        } else {
            // B. 差が±90°を超える：180°反転し、モーターを逆回転
            drive_direction = -1.0f;
            if (delta_pulse > 0) {
                optimized_delta_pulse = delta_pulse - PULSE_180_DEG;
            } else {
                optimized_delta_pulse = delta_pulse + PULSE_180_DEG;
            }
        }

        // d. 最終目標値の計算とハードウェアリミット適用
        int32_t final_target_pulse = current_pulse + (int32_t)optimized_delta_pulse;
        
        // スティックが中央付近の場合、ステアリング角度は変更しない
        if (target_rpm < 0.01f) { // ほぼ停止している
            final_target_pulse = current_pulse; // 現在角度を維持
        }

        if (final_target_pulse > HW_LIMIT_PLUS) {
            final_target_pulse = HW_LIMIT_PLUS;
        } else if (final_target_pulse < HW_LIMIT_MINUS) {
            final_target_pulse = HW_LIMIT_MINUS;
        }

        float final_target_rpm = target_rpm * drive_direction;

        // e. PID目標値設定
        steering_position_pid[i].set_goal(final_target_pulse);
        tire_pid[i].set_goal(final_target_rpm);
    }
}

void pid_thread()
{
    auto pre_time = HighResClock::now();
    for (int i = 0; i < 4; i++)
    {
        steering_position_pid[i].set_output_limits(-8000, 8000);
        steering_velocity_pid[i].set_output_limits(-10000, 10000);
        steering_position_pid[i].set_deadband(200.0f);
        tire_pid[i].set_output_limits(-20000, 20000);
    }
    while (1)
    {
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time -
                                                                         pre_time)
                       .count() /
                   1000000.0f;
        for (int i = 0; i < 4; i++)
        {
            steering_position_pid[i].set_dt(dt);
            steering_velocity_pid[i].set_dt(dt);
            tire_pid[i].set_dt(dt);
        }
        pre_time = now_time;
        
        // 各エンコーダーを個別に更新
        Amt212CV* encoders_arr[4] = {&encoder0, &encoder1, &encoder2, &encoder3};
        bool all_updated = true;
        for (int i = 0; i < 4; i++)
        {
            if (encoders_arr[i]->update())
            {
                amt212c_v_position[i] = encoders_arr[i]->get_position() - amt212c_v_error[i];
            }
            else
            {
                all_updated = false;
            }
        }
        
        if (!all_updated)
        {
            printf("Encoder update failed!\n");
        }
        for (int i = 0; i < 4; i++)
        {
            steering_velocity_pid[i].set_goal(-steering_position_pid[i].do_pid(amt212c_v_position[i]));
        }
        
        if (amt212c_v_position[0] > 16232 || amt212c_v_position[0] < -14784 ||
            amt212c_v_position[1] > 16232 || amt212c_v_position[1] < -14784 ||
            amt212c_v_position[2] > 16232 || amt212c_v_position[2] < -14784 ||
            amt212c_v_position[3] > 16232 || amt212c_v_position[3] < -14784)
        {
            for (int i = 0; i < 4; i++)
            {
                steering_velocity_pid[i].set_goal(0);
            }
        }
        for (int i = 0; i < 4; i++)
        {
            DJI.set_power(i + 4, steering_velocity_pid[i].do_pid(DJI.get_rpm(i + 4)));
            tire_power[i] = tire_pid[i].do_pid(enc_filtered[i]);
        }

        // PID制御ループ
        ThisThread::sleep_for(30ms);
    }
}
int main()
{
    for(int i=0;i<4;i++){
        steering_position_pid[i].set_goal(0);
    }
    encoder0.set_mode(Amt212CV::Mode::Continuous);
    encoder1.set_mode(Amt212CV::Mode::Continuous);
    encoder2.set_mode(Amt212CV::Mode::Continuous);
    encoder3.set_mode(Amt212CV::Mode::Continuous);
    Thread thread;
    thread.start(serial_read);
    Thread pid_thread_handle;
    pid_thread_handle.start(pid_thread);
    while (1)
    {

        DJI.send_message();
                CANMessage msg(4, (const uint8_t *)tire_power, 8);
        can2.write(msg);
        printf("amt212c_v_position:%d,%d,%d,%d\n", amt212c_v_position[0], amt212c_v_position[1], amt212c_v_position[2], amt212c_v_position[3]);
        // printf("AA Position: %ld\n", aa_position);
        // printf("position:%d,send_message:%d,goal_speed:%f\n", aa_position, DJI.get_rpm(4), steering_velocity_pid.get_goal());
        ThisThread::sleep_for(30ms);
    }
}