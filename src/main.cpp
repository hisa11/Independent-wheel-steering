#include "mbed.h"
#include "pid.hpp"
#include "serial_read.hpp"
#include "QEI.h"
#include "amt212c_v.hpp"
#include "c610.hpp"
#include "Rs485.h"

BufferedSerial pc(USBTX, USBRX, 115200); // Nucleoã®ã‚·ãƒªã‚¢ãƒ«ãƒãƒ¼ãƒˆ

DigitalIn button(BUTTON1);
DigitalOut led(LED1);
serial_unit serial(pc);
PID steering_position_pid[4] = {
    PID(7.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.000001, PID::Mode::POSITIONAL),
    PID(7.55, 0.0, 0.000001, PID::Mode::POSITIONAL)};
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
Mutex can2_mutex;    // can2ã¸ã®ã‚¢ã‚¯ã‚»ã‚¹ã‚’ä¿è­·ã™ã‚‹ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹
Mutex encoder_mutex; // amt212c_v_positionã¸ã®ã‚¢ã‚¯ã‚»ã‚¹ã‚’ä¿è­·ã™ã‚‹ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹
Mutex rs485_mutex;   // RS485ãƒã‚¹ã¸ã®ã‚¢ã‚¯ã‚»ã‚¹ã‚’ä¿è­·ã™ã‚‹ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹
C610 DJI(can1);
// RS485 bus and Amt21 encoders
Rs485 rs485{PA_9, PA_10, (int)2e6, D6};
Amt21 encoder0{0x48, rs485};
Amt21 encoder1{0x54, rs485};
Amt21 encoder2{0x5C, rs485};
Amt21 encoder3{0x58, rs485};
int amt212c_v_error[4] = {321, -1471, -1526, 622};
int amt212c_v_position[4] = {0, 0, 0, 0};
float stick_x = 0.00f, stick_y = 0.00f, stick_r = 0.00f;
int tire_power[4] = {0, 0, 0, 0};

const float ALPHA = 0.8f;                         // ãƒ­ãƒ¼ãƒ‘ã‚¹ãƒ•ã‚£ãƒ«ã‚¿ä¿‚æ•° (0.0-1.0, å°ã•ã„ã»ã©æ»‘ã‚‰ã‹)
float enc_filtered[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // ãƒ•ã‚£ãƒ«ã‚¿æ¸ˆã¿ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼å€¤
int16_t enc[4] = {0, 0, 0, 0};

// ã‚¢ãƒ–ã‚½ãƒªãƒ¥ãƒ¼ãƒˆã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼æ›´æ–°å°‚ç"¨ã‚¹ãƒ¬ãƒƒãƒ‰
void encoder_update_thread()
{
    Amt21 *encoders_arr[4] = {&encoder0, &encoder1, &encoder2, &encoder3};
    int current_encoder = 0;
    int debug_counter = 0;
    int error_count[4] = {0, 0, 0, 0};
    while (1)
    {
        // RS485ãƒã‚¹ã¸ã®ã‚¢ã‚¯ã‚»ã‚¹ã‚'ä¿è­·
        rs485_mutex.lock();
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
            if (error_count[current_encoder] > 10)
            {
                error_count[current_encoder] = 0;
                // ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹ã‚'ä¿æŒã—ãŸã¾ã¾ã‚¹ãƒªãƒ¼ãƒ—
                ThisThread::sleep_for(2ms);
            }
        }
        rs485_mutex.unlock();

        // ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼ã®å€¤ã‚'æ›´æ–°
        if (update_success)
        {
            encoder_mutex.lock();
            amt212c_v_position[current_encoder] = new_position;
            encoder_mutex.unlock();
        }
        // æ¬¡ã®ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼ã¸
        current_encoder = (current_encoder + 1) % 4;
        // æ›´æ–°é–"éš"
        ThisThread::sleep_for(30ms);
        // ãƒ‡ãƒãƒƒã‚°å‡ºåŠ›
        if (++debug_counter >= 140)
        {
            debug_counter = 0;

            // pc.write(buffer, strlen(buffer));
        }
    }
}

void sensor_thread()
{
    CANMessage enc_msg;
    while (1)
    {
        // ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹ã§can2ã¸ã®ã‚¢ã‚¯ã‚»ã‚¹ã‚’ä¿è­·
        can2_mutex.lock();
        bool read_success = can2.read(enc_msg, 10); // 10ms ã‚¿ã‚¤ãƒ ã‚¢ã‚¦ãƒˆã«çŸ­ç¸®
        can2_mutex.unlock();

        if (read_success)
        {
            if (enc_msg.id == 10)
            {
                for (int i = 0; i < 4; i++)
                {
                    enc[i] = -enc_msg.data[i * 2 + 1] << 8 | enc_msg.data[i * 2];
                    // ãƒ­ãƒ¼ãƒ‘ã‚¹ãƒ•ã‚£ãƒ«ã‚¿é©ç”¨: y[n] = Î± * x[n] + (1 - Î±) * y[n-1]
                    enc_filtered[i] = ALPHA * enc[i] + (1.0f - ALPHA) * enc_filtered[i];
                }
            }
            else
            {
            }
        }
        else
        {
            // ã‚¿ã‚¤ãƒ ã‚¢ã‚¦ãƒˆæ™‚ã®ã‚¨ãƒ©ãƒ¼ãƒãƒ³ãƒ‰ãƒªãƒ³ã‚°
        }
        ThisThread::sleep_for(30ms);
    }
}

void move_aa(std::string msg)
{
    // --- åˆ¶å¾¡ç"¨å®šæ•° ---
    // ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼ã®ã‚«ã‚¦ãƒ³ãƒˆæ•°ï¼š90åº¦ = 1000ã‚«ã‚¦ãƒ³ãƒˆã€360åº¦ = 4000ã‚«ã‚¦ãƒ³ãƒˆç›¸å½"
    const float COUNTS_PER_90_DEG = 1000.0f;   // 90åº¦ã«ç›¸å½"ã™ã‚‹ã‚«ã‚¦ãƒ³ãƒˆ
    const float COUNTS_PER_180_DEG = 2000.0f;  // 180åº¦ã«ç›¸å½"ã™ã‚‹ã‚«ã‚¦ãƒ³ãƒˆ
    const float COUNTS_PER_ROTATION = 4000.0f; // 1å›žè»¢ã«ç›¸å½"ã™ã‚‹ã‚«ã‚¦ãƒ³ãƒˆ
    // ãƒ©ã‚¸ã‚¢ãƒ³ -> ã‚«ã‚¦ãƒ³ãƒˆ æ›ç®—ä¿‚æ•° (1000 / (PI/2))
    const float COUNTS_PER_RAD = COUNTS_PER_90_DEG / (M_PI / 2.0f);

    // ãƒãƒ¼ãƒ‰ã‚¦ã‚§ã‚¢ãƒªãƒŸãƒƒãƒˆ (-1000 ~ 1000 ã®ç¯„å›²)
    const int32_t HW_LIMIT_PLUS = 1000;
    const int32_t HW_LIMIT_MINUS = -1000;

    // ã‚¿ã‚¤ãƒ¤é€Ÿåº¦ãƒªãƒŸãƒƒãƒˆ (ãƒ¦ãƒ¼ã‚¶ãƒ¼æŒ‡å®š)
    const float MAX_RPM = 5000.0f;

    // ãƒ­ãƒœãƒƒãƒˆã®ã‚¸ã‚ªãƒ¡ãƒˆãƒª (ä¸­å¿ƒã‹ã‚‰ãƒ›ã‚¤ãƒ¼ãƒ«ã¾ã§ã®è·é›¢) [m]
    // â€»â€»â€» ã“ã®å€¤ã¯å®Ÿéš›ã®ãƒ­ãƒœãƒƒãƒˆã«åˆã‚ã›ã¦èª¿æ•´ã—ã¦ãã ã•ã„ â€»â€»â€»
    const float L = 0.5f; // ä¸­å¿ƒã®Xè»¸(å‰å¾Œ)ã‹ã‚‰ãƒ›ã‚¤ãƒ¼ãƒ«ã¾ã§ã®è·é›¢
    const float W = 0.5f; // ä¸­å¿ƒã®Yè»¸(å·¦å³)ã‹ã‚‰ãƒ›ã‚¤ãƒ¼ãƒ«ã¾ã§ã®è·é›¢

    // --- 1. ã‚¹ãƒ†ã‚£ãƒƒã‚¯å…¥åŠ›ã®ãƒ‘ãƒ¼ã‚¹ã¨ãƒ‡ãƒƒãƒ‰ãƒãƒ³ãƒ‰ ---
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
    // ãƒ­ãƒœãƒƒãƒˆåº§æ¨™ç³»ã«ãƒžãƒƒãƒ”ãƒ³ã‚°
    float Vx_base = joys[1]; // stick_y: å‰å¾Œ (Xè»¸)
    float Vy_base = joys[0]; // stick_x: å·¦å³ (Yè»¸) â€»å·¦ã‚’æ­£ã¨ã™ã‚‹
    float Omega = joys[2];   // stick_r: å›žè»¢ (Zè»¸) â€»åæ™‚è¨ˆå›žã‚Šã‚’æ­£ã¨ã™ã‚‹

    // --- 2. ã‚¹ãƒ¯ãƒ¼ãƒ–ã‚­ãƒãƒžãƒ†ã‚£ã‚¯ã‚¹è¨ˆç®— ---
    // å„ãƒ›ã‚¤ãƒ¼ãƒ«ã®é€Ÿåº¦ãƒ™ã‚¯ãƒˆãƒ« [Vx, Vy] ã‚’è¨ˆç®—
    // å®Ÿéš›ã®ãƒ›ã‚¤ãƒ¼ãƒ«é…ç½® (ãƒ­ãƒœãƒƒãƒˆä¸­å¿ƒã‹ã‚‰è¦‹ãŸä½ç½®):
    //   0:ã‚¿ã‚¤ãƒ¤1 = å·¦å‰ (Front Left)  (x=+L, y=+W)
    //   1:ã‚¿ã‚¤ãƒ¤2 = å·¦å¾Œã‚ (Rear Left)  (x=-L, y=+W)
    //   2:ã‚¿ã‚¤ãƒ¤3 = å³å¾Œã‚ (Rear Right) (x=-L, y=-W)
    //   3:ã‚¿ã‚¤ãƒ¤4 = å³å‰ (Front Right) (x=+L, y=-W)
    //
    // ã‚¹ãƒ¯ãƒ¼ãƒ–ãƒ‰ãƒ©ã‚¤ãƒ–ã®ã‚­ãƒãƒžãƒ†ã‚£ã‚¯ã‚¹:
    // æ—‹å›žæ™‚ã€å„ãƒ›ã‚¤ãƒ¼ãƒ«ã¯ãƒ­ãƒœãƒƒãƒˆä¸­å¿ƒã‹ã‚‰ã®ç·šã«å¯¾ã—ã¦ç›´è§’ï¼ˆæŽ¥ç·šæ–¹å‘ï¼‰ã‚’å‘ã
    // V_wheel = V_base + Ï‰ Ã— r_wheel
    // å¤–ç© Ï‰ Ã— r = [0, 0, Ï‰] Ã— [x, y, 0] = [Ï‰*(-y), Ï‰*x, 0]
    // ã—ãŸãŒã£ã¦:
    // Vx_wheel = Vx_base + Ï‰ * (-y_wheel)  (å›žè»¢ã«ã‚ˆã‚‹å‰å¾Œæ–¹å‘ã®é€Ÿåº¦)
    // Vy_wheel = Vy_base + Ï‰ * x_wheel     (å›žè»¢ã«ã‚ˆã‚‹å·¦å³æ–¹å‘ã®é€Ÿåº¦)

    // ãƒ›ã‚¤ãƒ¼ãƒ«0: ã‚¿ã‚¤ãƒ¤1 = å·¦å‰ (Front Left) (x=+L, y=+W)
    float vx_0 = Vx_base + Omega * (-W); // Vx + Ï‰*(-W)
    float vy_0 = Vy_base + Omega * L;    // Vy + Ï‰*(+L)

    // ãƒ›ã‚¤ãƒ¼ãƒ«1: ã‚¿ã‚¤ãƒ¤2 = å·¦å¾Œã‚ (Rear Left) (x=-L, y=+W)
    float vx_1 = Vx_base + Omega * (-W); // Vx + Ï‰*(-W)
    float vy_1 = Vy_base + Omega * (-L); // Vy + Ï‰*(-L)

    // ãƒ›ã‚¤ãƒ¼ãƒ«2: ã‚¿ã‚¤ãƒ¤3 = å³å¾Œã‚ (Rear Right) (x=-L, y=-W)
    float vx_2 = Vx_base + Omega * W;    // Vx + Ï‰*(+W)
    float vy_2 = Vy_base + Omega * (-L); // Vy + Ï‰*(-L)

    // ãƒ›ã‚¤ãƒ¼ãƒ«3: ã‚¿ã‚¤ãƒ¤4 = å³å‰ (Front Right) (x=+L, y=-W)
    float vx_3 = Vx_base + Omega * W; // Vx + Ï‰*(+W)
    float vy_3 = Vy_base + Omega * L; // Vy + Ï‰*(+L)

    // è¨ˆç®—ã•ã‚ŒãŸé€Ÿåº¦ãƒ™ã‚¯ãƒˆãƒ«ã‚’é…åˆ—ã«æ ¼ç´
    float vx[4] = {vx_0, vx_1, vx_2, vx_3};
    float vy[4] = {vy_0, vy_1, vy_2, vy_3};

    // --- 3. é€Ÿåº¦ã®æ­£è¦åŒ– ---
    float max_speed = 0.0f;
    float wheel_speeds_vec[4]; // å„ãƒ›ã‚¤ãƒ¼ãƒ«ã®é€Ÿåº¦ãƒ™ã‚¯ãƒˆãƒ«ã®å¤§ãã•

    for (int i = 0; i < 4; i++)
    {
        wheel_speeds_vec[i] = sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);
        if (wheel_speeds_vec[i] > max_speed)
        {
            max_speed = wheel_speeds_vec[i];
        }
    }

    // æœ€å¤§é€Ÿåº¦ãŒ 1.0 (ã‚¹ãƒ†ã‚£ãƒƒã‚¯ã®æœ€å¤§å…¥åŠ›åˆæˆå€¤) ã‚’è¶…ãˆãŸå ´åˆã€å…¨ãƒ›ã‚¤ãƒ¼ãƒ«ã®é€Ÿåº¦ã‚’æ­£è¦åŒ–
    if (max_speed > 1.0f)
    {
        for (int i = 0; i < 4; i++)
        {
            // vx[i] /= max_speed; // è§’åº¦è¨ˆç®—ã®ãŸã‚ã«æ­£è¦åŒ–å‰ã®å€¤ã‚’ä½¿ã†
            // vy[i] /= max_speed;
            wheel_speeds_vec[i] /= max_speed; // é€Ÿåº¦ã®å¤§ãã•ã ã‘æ­£è¦åŒ–
        }
    }

    // --- 4. å„ãƒ›ã‚¤ãƒ¼ãƒ«ã®ç›®æ¨™å€¤è¨­å®šãƒ«ãƒ¼ãƒ— ---
    // ç¾åœ¨ã®ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼ä½ç½®ã‚’å–å¾—ï¼ˆãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹ä¿è­·ï¼‰
    int current_positions[4];
    encoder_mutex.lock();
    for (int i = 0; i < 4; i++)
    {
        current_positions[i] = amt212c_v_position[i];
    }
    encoder_mutex.unlock();

    // ã‚¹ãƒ†ã‚£ãƒƒã‚¯ãŒå‹•ã„ã¦ã„ã‚‹ã‹ãƒã‚§ãƒƒã‚¯
    bool is_moving = (fabsf(Vx_base) > 0.001f || fabsf(Vy_base) > 0.001f || fabsf(Omega) > 0.001f);

    for (int i = 0; i < 4; i++)
    {
        // a. ç›®æ¨™é€Ÿåº¦ (RPM)
        // ã‚¹ãƒ†ã‚£ãƒƒã‚¯ãŒä¸­å¤® (Vx, Vy, OmegaãŒã™ã¹ã¦0) ã®å ´åˆã€é€Ÿåº¦ã¯0
        float target_rpm = 0.0f;

        if (is_moving)
        {
            target_rpm = wheel_speeds_vec[i] * MAX_RPM;
        }

        // b. ç›®æ¨™è§’åº¦ (ãƒ©ã‚¸ã‚¢ãƒ³)
        // atan2f(vy, vx) ã‚’ä½¿ã„ã€ã€Œå‰ã€ (vx > 0, vy = 0) ã‚’ 0 ãƒ©ã‚¸ã‚¢ãƒ³ã¨ã™ã‚‹
        float target_rad = atan2f(vy[i], vx[i]);

        // c. 90Â°æœ€é©åŒ–ãƒ­ã‚¸ãƒƒã‚¯ï¼ˆæ”¹è‰¯ç‰ˆï¼šÂ±Ï€/2åˆ¶é™å¯¾å¿œï¼‰
        int32_t current_count = current_positions[i];

        // ç›®æ¨™ãƒ‘ãƒ«ã‚¹å€¤ã‚’è¨ˆç®—ï¼ˆ-180Â°ï½ž180Â°ã®ç¯„å›²ï¼‰
        float target_count_raw = target_rad * COUNTS_PER_RAD;

        // ç¾åœ¨ä½ç½®ã‚’-180Â°ï½ž180Â°ã®ç¯„å›²ã«æ­£è¦åŒ–
        float current_count_normalized = remainderf((float)current_count, COUNTS_PER_ROTATION);

        // ç›®æ¨™ä½ç½®ã‚‚-180Â°ï½ž180Â°ã®ç¯„å›²ã«æ­£è¦åŒ–
        float target_count_normalized = remainderf(target_count_raw, COUNTS_PER_ROTATION);

        // ç›®æ¨™è§’åº¦ã‚’Â±90Â°ç¯„å›²ã«åŽã‚ã‚‹ï¼ˆÂ±180Â°åè»¢ã§å¯¾å¿œï¼‰
        float target_count_adjusted = target_count_normalized;
        float drive_direction = 1.0f;

        if (target_count_adjusted > COUNTS_PER_90_DEG)
        {
            // 90Â°ï½ž180Â° â†’ -90Â°ï½ž0Â° ã«å¤‰æ›ã—ã¦ã‚¿ã‚¤ãƒ¤é€†å›žè»¢
            target_count_adjusted -= COUNTS_PER_180_DEG;
            drive_direction = -1.0f;
        }
        else if (target_count_adjusted < -COUNTS_PER_90_DEG)
        {
            // -180Â°ï½ž-90Â° â†’ 0Â°ï½ž90Â° ã«å¤‰æ›ã—ã¦ã‚¿ã‚¤ãƒ¤é€†å›žè»¢
            target_count_adjusted += COUNTS_PER_180_DEG;
            drive_direction = -1.0f;
        }

        // æ­£è¦åŒ–ã•ã‚ŒãŸè§’åº¦é–“ã®æœ€çŸ­è·é›¢ã‚’è¨ˆç®—ï¼ˆç¾åœ¨ä½ç½®ã‚‚Â±90Â°ã«åŽã‚ã‚‹ï¼‰
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

        // -180Â°ï½ž180Â°ã®ç¯„å›²ã«åŽã‚ã‚‹ï¼ˆæœ€çŸ­çµŒè·¯ï¼‰
        if (delta_pulse > COUNTS_PER_180_DEG)
        {
            delta_pulse -= COUNTS_PER_ROTATION;
        }
        else if (delta_pulse < -COUNTS_PER_180_DEG)
        {
            delta_pulse += COUNTS_PER_ROTATION;
        }

        // d. æœ€çµ‚ç›®æ¨™å€¤ã®è¨ˆç®—ï¼ˆ-1000ï½ž1000 (-90Â°ï½ž90Â°) ã®ç¯„å›²ã«åˆ¶é™ï¼‰
        // æ­£è¦åŒ–ã•ã‚ŒãŸç¾åœ¨ä½ç½®ã«å·®åˆ†ã‚'åŠ ç®—
        int32_t final_target_count = (int32_t)current_count_adjusted + (int32_t)delta_pulse;

        // å®‰å…¨ã®ãŸã‚-1000ï½ž1000ã®ç¯„å›²ã«åˆ¶é™ï¼ˆÂ±90Â°ï¼‰
        if (final_target_count > HW_LIMIT_PLUS)
        {
            final_target_count = HW_LIMIT_PLUS;
        }
        else if (final_target_count < HW_LIMIT_MINUS)
        {
            final_target_count = HW_LIMIT_MINUS;
        }

        // ã‚¹ãƒ†ã‚£ãƒƒã‚¯ãŒä¸­å¤®ä»˜è¿‘ã®å ´åˆã€ã‚¹ãƒ†ã‚¢ãƒªãƒ³ã‚°è§’åº¦ã¯å¤‰æ›´ã—ãªã„
        if (!is_moving)
        {
            final_target_count = (int32_t)current_count_adjusted; // ç¾åœ¨è§’åº¦ã‚’ç¶­æŒ
        }

        float final_target_rpm = target_rpm * drive_direction;

        // e. PIDç›®æ¨™å€¤è¨­å®š
        steering_position_pid[i].set_goal(final_target_count);
        tire_pid[i].set_goal(final_target_rpm);
    }

    // // f. å…¨ãƒ›ã‚¤ãƒ¼ãƒ«ã®ç›®æ¨™è§’åº¦ã‚’è¡¨ç¤º
    // if (is_moving)
    // {
    //     const float COUNTS_PER_RAD_CONST = COUNTS_PER_ROTATION / (2.0f * M_PI);
    //     printf("Target angles: W0=%.1fÂ° W1=%.1fÂ° W2=%.1fÂ° W3=%.1fÂ°\n",
    //            steering_position_pid[0].get_goal() / COUNTS_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[1].get_goal() / COUNTS_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[2].get_goal() / COUNTS_PER_RAD_CONST * 180.0f / M_PI,
    //            steering_position_pid[3].get_goal() / COUNTS_PER_RAD_CONST * 180.0f / M_PI);
    // }
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

        // PIDåˆ¶å¾¡ã«ã¯ãƒŸãƒ¥ãƒ¼ãƒ†ãƒƒã‚¯ã‚¹ã§ä¿è­·ã•ã‚ŒãŸæœ€æ–°ã®ã‚¨ãƒ³ã‚³ãƒ¼ãƒ€ãƒ¼å€¤ã‚’ä½¿ç”¨
        int local_encoder_positions[4];
        encoder_mutex.lock();
        for (int i = 0; i < 4; i++)
        {
            local_encoder_positions[i] = amt212c_v_position[i];
        }
        encoder_mutex.unlock();

        // ハードウェアリミットチェック (-1000 ~ 1000, 緊急停止 ±1500)
        bool emergency_stop = false;
        for (int i = 0; i < 4; i++)
        {
            // ±1500を超えたら緊急停止
            if (local_encoder_positions[i] > 1500 || local_encoder_positions[i] < -1500)
            {
                emergency_stop = true;
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
            tire_power[i] = -tire_pid[i].do_pid(enc_filtered[i]);
        }
        // pc.write("PID loop executed\n", 18);
        // PIDåˆ¶å¾¡ãƒ«ãƒ¼ãƒ—
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
    // ã‚·ãƒªã‚¢ãƒ«ãƒãƒ¼ãƒˆã‚’éžãƒ–ãƒ­ãƒƒã‚­ãƒ³ã‚°ãƒ¢ãƒ¼ãƒ‰ã«è¨­å®šï¼ˆé‡è¦ï¼ï¼‰
    pc.set_blocking(false);

    // ã‚·ãƒªã‚¢ãƒ«é€šä¿¡ã‚'ã‚¤ãƒ™ãƒ³ãƒˆæ–¹å¼ã§åˆæœŸåŒ–
    serial.start_event_mode();

    for (int i = 0; i < 4; i++)
    {
        steering_position_pid[i].set_goal(0);
    }

    Thread thread(osPriorityHigh);
    thread.start(serial_read);
    Thread encoder_thread_handle(osPriorityRealtime);
    encoder_thread_handle.start(encoder_update_thread);
    Thread pid_thread_handle(osPriorityHigh);
    pid_thread_handle.start(pid_thread);
    Thread sensor_thread_handle(osPriorityHigh);
    sensor_thread_handle.start(sensor_thread);
    Thread led_thread_handle(osPriorityLow);
    led_thread_handle.start(led_thread);

    while (1)
    {
        DJI.send_message();
        // ...existing code...
        CANMessage msg(4, (const uint8_t *)tire_power, 8);
        can2_mutex.lock();
        can2.write(msg);
        can2_mutex.unlock();
        encoder_mutex.lock();
        // char buffer[128];
        // snprintf(buffer, sizeof(buffer), "steering_positions: W0=%d, W1=%d, W2=%d, W3=%d\n",
        //          (int)amt212c_v_position[0],
        //          (int)amt212c_v_position[1],
        //          (int)amt212c_v_position[2],
        //          (int)amt212c_v_position[3]);
        // pc.write(buffer, strlen(buffer));
        char buffer2[128];
        snprintf(buffer2, sizeof(buffer2), "tire4:pos:%d,goal:%d,power:%d\n",
                 (int)amt212c_v_position[2],
                 (int)steering_position_pid[2].get_goal(),
                 (int)steering_velocity_pid[2].get_goal());
        pc.write(buffer2, strlen(buffer2));
        encoder_mutex.unlock();
        ThisThread::sleep_for(30ms);
    }
}
