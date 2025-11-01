#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"
#include "QEI.h"
#include "amt212c_v.hpp"
#include "c610.hpp"

BufferedSerial pc(USBTX, USBRX, 115200); // Nucleoのシリアルポート

DigitalIn button(BUTTON1);
DigitalOut led(LED1);
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
    PID(1.10, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(1.10, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(1.10, 0.5, 0.000001, PID::Mode::VELOCITY),
    PID(1.10, 0.5, 0.000001, PID::Mode::VELOCITY)};

CAN can1(PA_11, PA_12, 1000000);
CAN can2(PB_12, PB_13, 1000000);
Mutex can2_mutex;  // can2へのアクセスを保護するミューテックス
Mutex encoder_mutex;  // amt212c_v_positionへのアクセスを保護するミューテックス
// Mutex rs485_mutex;  // RS485バスへのアクセスを保護するミューテックス（不要なので削除）
C610 DJI(can1);
// 共有するRS485バスとDEピンを1つだけ定義
UnbufferedSerial rs485_bus(PA_9, PA_10, 2000000);
DigitalOut rs485_de(D6);
Amt212CV encoder0(rs485_bus, rs485_de, 0x48);
Amt212CV encoder1(rs485_bus, rs485_de, 0x50);
Amt212CV encoder2(rs485_bus, rs485_de, 0x5c);
Amt212CV encoder3(rs485_bus, rs485_de, 0x58);
int amt212c_v_error[4] = {1290, 6472, -6056, 2476};
int amt212c_v_position[4] = {0, 0, 0, 0};
float stick_x = 0.00f, stick_y = 0.00f, stick_r = 0.00f;
int tire_power[4] = {0, 0, 0, 0};

const float ALPHA = 0.8f;                         // ローパスフィルタ係数 (0.0-1.0, 小さいほど滑らか)
float enc_filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // フィルタ済みエンコーダー値
int16_t enc[4] = {0, 0, 0, 0};

// アブソリュートエンコーダー更新専用スレッド
void encoder_update_thread()
{
    Amt212CV *encoders_arr[4] = {&encoder0, &encoder1, &encoder2, &encoder3};
    int current_encoder = 0;
    int debug_counter = 0;
    int error_count[4] = {0, 0, 0, 0};
    while (1)
    {
        bool update_success = encoders_arr[current_encoder]->update();
        int new_position = 0;
        if (update_success)
        {
            new_position = encoders_arr[current_encoder]->get_position() - amt212c_v_error[current_encoder];
            error_count[current_encoder] = 0;
        }
        else
        {
            error_count[current_encoder]++;
            if (error_count[current_encoder] > 10)
            {
                error_count[current_encoder] = 0;
                ThisThread::sleep_for(2ms);
            }
        }

        // エンコーダーの値を更新
        if (update_success)
        {
            encoder_mutex.lock();
            amt212c_v_position[current_encoder] = new_position;
            encoder_mutex.unlock();
        }
        // 次のエンコーダーへ
        current_encoder = (current_encoder + 1) % 4;
        // 更新間隔
        ThisThread::sleep_for(7ms);
        // デバッグ出力（コメントアウト済み）
        // if (++debug_counter >= 140)
        // {
        //     debug_counter = 0;
        //     encoder_mutex.lock();
        //     char buffer[128];
        //     snprintf(buffer, sizeof(buffer), "steering_positions: W0=%d, W1=%d, W2=%d, W3=%d\n",
        //              (int)amt212c_v_position[0],
        //              (int)amt212c_v_position[1],
        //              (int)amt212c_v_position[2],
        //              (int)amt212c_v_position[3]);
        //     encoder_mutex.unlock();
        //     // pc.write(buffer, strlen(buffer));
        // }
    }
}

void sensor_thread()
{
    CANMessage enc_msg;
    while (1)
    {
        // ミューテックスでcan2へのアクセスを保護
        // can2_mutex.lock();
        bool read_success = can2.read(enc_msg, 10);  // 10ms タイムアウトに短縮
        // can2_mutex.unlock();
        
        if (read_success)
        {
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
            }
        }
        else
        {
            // タイムアウト時のエラーハンドリング
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
    float Omega = joys[2];   // stick_r: 回転 (Z軸) ※反時計回りを正とする

    // --- 2. スワーブキネマティクス計算 ---
    // 各ホイールの速度ベクトル [Vx, Vy] を計算
    // 実際のホイール配置 (ロボット中心から見た位置):
    //   0:タイヤ1 = 左前 (Front Left)  (x=+L, y=+W)
    //   1:タイヤ2 = 左後ろ (Rear Left)  (x=-L, y=+W)
    //   2:タイヤ3 = 右後ろ (Rear Right) (x=-L, y=-W)
    //   3:タイヤ4 = 右前 (Front Right) (x=+L, y=-W)
    // 
    // スワーブドライブのキネマティクス:
    // 旋回時、各ホイールはロボット中心からの線に対して直角（接線方向）を向く
    // V_wheel = V_base + ω × r_wheel
    // 外積 ω × r = [0, 0, ω] × [x, y, 0] = [ω*(-y), ω*x, 0]
    // したがって:
    // Vx_wheel = Vx_base + ω * (-y_wheel)  (回転による前後方向の速度)
    // Vy_wheel = Vy_base + ω * x_wheel     (回転による左右方向の速度)

    // ホイール0: タイヤ1 = 左前 (Front Left) (x=+L, y=+W)
    float vx_0 = Vx_base + Omega * (-W);  // Vx + ω*(-W)
    float vy_0 = Vy_base + Omega * L;     // Vy + ω*(+L)

    // ホイール1: タイヤ2 = 左後ろ (Rear Left) (x=-L, y=+W)
    float vx_1 = Vx_base + Omega * (-W);  // Vx + ω*(-W)
    float vy_1 = Vy_base + Omega * (-L);  // Vy + ω*(-L)

    // ホイール2: タイヤ3 = 右後ろ (Rear Right) (x=-L, y=-W)
    float vx_2 = Vx_base + Omega * W;     // Vx + ω*(+W)
    float vy_2 = Vy_base + Omega * (-L);  // Vy + ω*(-L)

    // ホイール3: タイヤ4 = 右前 (Front Right) (x=+L, y=-W)
    float vx_3 = Vx_base + Omega * W;     // Vx + ω*(+W)
    float vy_3 = Vy_base + Omega * L;     // Vy + ω*(+L)

    // 計算された速度ベクトルを配列に格納
    float vx[4] = {vx_0, vx_1, vx_2, vx_3};
    float vy[4] = {vy_0, vy_1, vy_2, vy_3};

    // --- 3. 速度の正規化 ---
    float max_speed = 0.0f;
    float wheel_speeds_vec[4]; // 各ホイールの速度ベクトルの大きさ

    for (int i = 0; i < 4; i++)
    {
        wheel_speeds_vec[i] = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
        if (wheel_speeds_vec[i] > max_speed)
        {
            max_speed = wheel_speeds_vec[i];
        }
    }

    // 最大速度が 1.0 (スティックの最大入力合成値) を超えた場合、全ホイールの速度を正規化
    if (max_speed > 1.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            // vx[i] /= max_speed; // 角度計算のために正規化前の値を使う
            // vy[i] /= max_speed;
            wheel_speeds_vec[i] /= max_speed; // 速度の大きさだけ正規化
        }
    }

    // --- 4. 各ホイールの目標値設定ループ ---
    // 現在のエンコーダー位置を取得（ミューテックス保護）
    int current_positions[4];
    encoder_mutex.lock();
    for (int i = 0; i < 4; i++)
    {
        current_positions[i] = amt212c_v_position[i];
    }
    encoder_mutex.unlock();

    // スティックが動いているかチェック
    bool is_moving = (fabsf(Vx_base) > 0.001f || fabsf(Vy_base) > 0.001f || fabsf(Omega) > 0.001f);

    for (int i = 0; i < 4; i++)
    {
        // a. 目標速度 (RPM)
        // スティックが中央 (Vx, Vy, Omegaがすべて0) の場合、速度は0
        float target_rpm = 0.0f;
        
        if (is_moving)
        {
            target_rpm = wheel_speeds_vec[i] * MAX_RPM;
        }

        // b. 目標角度 (ラジアン)
        // atan2f(vy, vx) を使い、「前」 (vx > 0, vy = 0) を 0 ラジアンとする
        float target_rad = atan2f(vy[i], vx[i]);

        // c. 90°最適化ロジック（改良版：±π/2制限対応）
        int32_t current_pulse = current_positions[i];
        
        // 目標パルス値を計算（-180°～180°の範囲）
        float target_pulse_raw = target_rad * PULSE_PER_RAD;
        
        // 現在位置を-180°～180°の範囲に正規化
        float current_pulse_normalized = remainderf((float)current_pulse, PULSE_PER_ROTATION);
        
        // 目標位置も-180°～180°の範囲に正規化
        float target_pulse_normalized = remainderf(target_pulse_raw, PULSE_PER_ROTATION);
        
        // 目標角度を±90°範囲に収める（±180°反転で対応）
        float target_pulse_adjusted = target_pulse_normalized;
        float drive_direction = 1.0f;
        
        if (target_pulse_adjusted > PULSE_90_DEG)
        {
            // 90°～180° → -90°～0° に変換してタイヤ逆回転
            target_pulse_adjusted -= PULSE_180_DEG;
            drive_direction = -1.0f;
        }
        else if (target_pulse_adjusted < -PULSE_90_DEG)
        {
            // -180°～-90° → 0°～90° に変換してタイヤ逆回転
            target_pulse_adjusted += PULSE_180_DEG;
            drive_direction = -1.0f;
        }
        
        // 正規化された角度間の最短距離を計算（現在位置も±90°に収める）
        float current_pulse_adjusted = current_pulse_normalized;
        if (current_pulse_adjusted > PULSE_90_DEG)
        {
            current_pulse_adjusted -= PULSE_180_DEG;
        }
        else if (current_pulse_adjusted < -PULSE_90_DEG)
        {
            current_pulse_adjusted += PULSE_180_DEG;
        }
        
        float delta_pulse = target_pulse_adjusted - current_pulse_adjusted;
        
        // -180°～180°の範囲に収める（最短経路）
        if (delta_pulse > PULSE_180_DEG)
        {
            delta_pulse -= PULSE_PER_ROTATION;
        }
        else if (delta_pulse < -PULSE_180_DEG)
        {
            delta_pulse += PULSE_PER_ROTATION;
        }

        // d. 最終目標値の計算（-πrad～πradの範囲に制限、計算上は±π/2に収まる）
        // 正規化された現在位置に差分を加算
        int32_t final_target_pulse = (int32_t)current_pulse_adjusted + (int32_t)delta_pulse;
        
        // 安全のため-180°～180°の範囲に制限（通常は±90°に収まる）
        if (final_target_pulse > PULSE_180_DEG)
        {
            final_target_pulse = PULSE_180_DEG;
        }
        else if (final_target_pulse < -PULSE_180_DEG)
        {
            final_target_pulse = -PULSE_180_DEG;
        }

        // スティックが中央付近の場合、ステアリング角度は変更しない
        if (!is_moving)
        {
            final_target_pulse = (int32_t)current_pulse_adjusted; // 現在角度を維持
        }

        float final_target_rpm = target_rpm * drive_direction;

        // e. PID目標値設定
        steering_position_pid[i].set_goal(final_target_pulse);
        tire_pid[i].set_goal(final_target_rpm);
    }
    
    // // f. 全ホイールの目標角度を表示
    // if (is_moving)
    // {
    //     const float PULSE_PER_RAD_CONST = PULSE_PER_ROTATION / (2.0f * M_PI);
    //     printf("Target angles: W0=%.1f° W1=%.1f° W2=%.1f° W3=%.1f°\n",
    //            steering_position_pid[0].get_goal() / PULSE_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[1].get_goal() / PULSE_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[2].get_goal() / PULSE_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[3].get_goal() / PULSE_PER_RAD_CONST * 180.0f / M_PI);
    // }
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

        // PID制御にはミューテックスで保護された最新のエンコーダー値を使用
        int local_encoder_positions[4];
        encoder_mutex.lock();
        for (int i = 0; i < 4; i++)
        {
            local_encoder_positions[i] = amt212c_v_position[i];
        }
        encoder_mutex.unlock();

        // ハードウェアリミットチェック
        bool limit_exceeded = false;
        for (int i = 0; i < 4; i++)
        {
            if (local_encoder_positions[i] > 16232 || local_encoder_positions[i] < -14784)
            {
                limit_exceeded = true;
                // 位置PIDの積分項をリセットして異常な蓄積を防ぐ
                steering_position_pid[i].reset();
                steering_velocity_pid[i].reset();
                steering_velocity_pid[i].set_goal(0);
            }
        }

        // 通常のPID制御
        if (!limit_exceeded)
        {
            for (int i = 0; i < 4; i++)
            {
                float position_pid_output = steering_position_pid[i].do_pid(local_encoder_positions[i]);
                steering_velocity_pid[i].set_goal(-position_pid_output);
                
                // 異常な出力を検出
                if (std::abs(position_pid_output) > 15000.0f)
                {
                    // PIDをリセット
                    steering_position_pid[i].reset();
                    steering_velocity_pid[i].reset();
                }
            }
        }
        for (int i = 0; i < 4; i++)
        {
            DJI.set_power(i + 1, steering_velocity_pid[i].do_pid(DJI.get_rpm(i + 1)));
            tire_power[i] = -tire_pid[i].do_pid(enc_filtered[i]);
        }
        // pc.write("PID loop executed\n", 18);
        // PID制御ループ
        ThisThread::sleep_for(20ms);
    }
}

void led_thread()
{
    while (1)
    {
        led = !led;
        // pc.write("LED toggled\n", 13);
        ThisThread::sleep_for(500ms);
    }
}

int main()
{
    // シリアルポートを非ブロッキングモードに設定（重要！）
    pc.set_blocking(false);
    // 共有バスの初期化
    rs485_de = 0;
    rs485_bus.set_blocking(false);

    for (int i = 0; i < 4; i++)
    {
        steering_position_pid[i].set_goal(0);
    }
    encoder0.set_mode(Amt212CV::Mode::Continuous);
    encoder1.set_mode(Amt212CV::Mode::Continuous);
    encoder2.set_mode(Amt212CV::Mode::Continuous);
    encoder3.set_mode(Amt212CV::Mode::Continuous);

    Thread thread;
    thread.start(serial_read);
    Thread encoder_thread_handle;
    encoder_thread_handle.start(encoder_update_thread);
    // 優先度を最高に設定
    encoder_thread_handle.set_priority(osPriorityRealtime);
    Thread pid_thread_handle;
    pid_thread_handle.start(pid_thread);
    Thread sensor_thread_handle;
    sensor_thread_handle.start(sensor_thread);
    Thread led_thread_handle;
    led_thread_handle.start(led_thread);

    while (1)
    {
        DJI.send_message();
        // ...existing code...
        CANMessage msg(4, (const uint8_t *)tire_power, 8);
        can2.write(msg);
        ThisThread::sleep_for(30ms);
    }
}