#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    enum class Mode {
        POSITIONAL, // 位置型PID
        VELOCITY    // 速度型(増分型)PIDz
    };

    PID(float kp, float ki, float kd, Mode mode = Mode::POSITIONAL);
    
    // --- メイン API ---
    float do_pid(float current_value);
    void reset();

    // --- 設定用 API ---
    void set_goal(float goal);
    void set_dt(float dt_sec);
    void set_gains(float kp, float ki, float kd);
    void set_mode(Mode mode);
    void set_output_limits(float min, float max);
    
    /**
     * @brief 静止摩擦を打ち消すための最小駆動力を設定します (速度型PIDでのみ有効)
     * @param min_power モーターが動き出す最小の出力値 (常に正の値)
     */
    void set_min_drive_power(float min_power); // ★★★ ここを追加 ★★★

    /**
     * @brief 誤差の不感帯（デッドバンド）を設定します
     * @param deadband 不感帯の幅。この値以下の誤差は0として扱われます
     */
    void set_deadband(float deadband);

    // --- 高度な設定用 API ---
    void enable_anti_windup(bool enable);
    void enable_derivative_on_measurement(bool enable);
    void enable_feedforward(bool enable, float kf = 0.0f);
    void set_output_offset(float offset);
    void gravity_reduction(float gravity);
    void reset_gravity_offset();
    void set_gravity_offset(float gravity);

    // --- 状態取得 API ---
    float get_goal() const { return _goal; }
    float get_gravity_offset() const { return _gravity_offset; }

private:
    // PIDゲイン
    float _kp, _ki, _kd;
    
    // 制御状態
    float _goal = 0.0;
    float _dt = 0.01;
    Mode _mode = Mode::POSITIONAL;
    bool _initialized = false;

    // 履歴データ
    float _integral = 0.0;          // 位置型用積分値
    float _previous_error = 0.0;
    float _previous_error2 = 0.0;
    float _previous_value = 0.0;
    float _previous_value2 = 0.0;
    float _last_output = 0.0;
    float _last_goal = 0.0;         // FF用

    // 出力制限
    float _output_min = -1.0;
    float _output_max = 1.0;

    // オプション機能
    bool _anti_windup_enabled = true;
    bool _derivative_on_measurement_enabled = false;
    bool _feedforward_enabled = false;
    float _kf = 0.0;
    float _output_offset = 0.0;
    float _gravity_offset = 0.0;

    // ★★★ ここを追加 ★★★
    float _min_drive_power = 0.0; // 静止摩擦補償のための最小駆動力
    float _deadband = 0.0;         // 誤差の不感帯（デッドバンド）

    // ヘルパー関数
    float constrain(float value, float min, float max);
};

#endif // PID_HPP