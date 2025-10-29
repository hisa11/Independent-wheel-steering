#include "pid.hpp"
#include <cassert>
#include <cmath>

PID::PID(float kp, float ki, float kd, Mode mode) {
    set_gains(kp, ki, kd);
    set_mode(mode);
    reset();
}

float PID::do_pid(float current_value) {
    assert(_dt > 0 && "PID error: dt is not set or is zero.");

    float error = _goal - current_value;
    
    // 不感帯処理：誤差が設定値以下の場合は誤差を0として扱う
    if (_deadband > 0.0f && std::abs(error) <= _deadband) {
        error = 0.0f;
    }
    
    float output = 0.0;
    float output_before_min_power = 0.0;  // _min_drive_power適用前の値を保存用

    if (_mode == Mode::POSITIONAL) {
        // --- 位置型PID ---
        // P項
        float p_term = _kp * error;
        
        // I項
        _integral += error * _dt;
        float i_term = _ki * _integral;

        // D項
        float d_term;
        if (_derivative_on_measurement_enabled) {
            d_term = -_kd * (current_value - _previous_value) / _dt;
        } else {
            d_term = _kd * (error - _previous_error) / _dt;
        }

        // 出力計算と制限（重力補償を追加）
        float pre_saturated_output = p_term + i_term + d_term + _gravity_offset;
        output = constrain(pre_saturated_output, _output_min, _output_max);
        output_before_min_power = output;

        // アンチワインドアップ（クランピング法）
        if (_anti_windup_enabled && _ki != 0.0f) {
            float saturation_error = output - pre_saturated_output;
            _integral += saturation_error / _ki;
        }

    } else { // Mode::VELOCITY
        // --- 速度型PID ---
        // 初回計算時は前回値などを保存して終了
        if (!_initialized) {
            _initialized = true;
            _previous_error = error;
            _previous_error2 = error;
            _previous_value = current_value;
            _previous_value2 = current_value;
            _last_output = 0.0;
            _last_goal = _goal;
            return 0.0;
        }
        
        // P項（変化量）
        float p_term_diff = _kp * (error - _previous_error);

        // I項（変化量）
        float i_term_diff = _ki * error * _dt;

        // D項（変化量）- 2階差分を使用
        float d_term_diff;
        if (_derivative_on_measurement_enabled) {
            // 微分先行型（測定値の2階差分）
            d_term_diff = -_kd * (current_value - 2.0f * _previous_value + _previous_value2) / _dt;
        } else {
            // 誤差の2階差分
            d_term_diff = _kd * (error - 2.0f * _previous_error + _previous_error2) / _dt;
        }

        // 今回の出力変化量
        float delta_output = p_term_diff + i_term_diff + d_term_diff;

        // 新しい出力を計算
        float new_output = _last_output + delta_output;

        // フィードフォワードの追加（目標値の変化に追従）
        if (_feedforward_enabled && _kf > 0.0f) {
            new_output += _kf * (_goal - _last_goal);
        }
        
        // 静的オフセット
        new_output += _output_offset;

        // 出力制限（_min_drive_power適用前、重力補償前）
        new_output = constrain(new_output, _output_min, _output_max);
        output_before_min_power = new_output;  // 重力補償を含まない値を保存

        // 重力補償を追加
        new_output += _gravity_offset;

        // 静止摩擦補償（目標値が0以外なら最小駆動力を加算）
        output = new_output;
        if (_min_drive_power > 0.0f && std::abs(_goal) > 0.1f) {
            // ゴールの符号に応じて最小駆動力を加算
            float min_power_offset = (_goal > 0.0f) ? _min_drive_power : -_min_drive_power;
            output += min_power_offset;
        }

        // 目標値が0で現在値も0に近い場合は出力を0にして振動を停止
        if (std::abs(_goal) < 1.0f && std::abs(current_value) < 100.0f) {
            output = 0.0;
            output_before_min_power = 0.0;
        }
    }

    // 履歴を更新
    _previous_error2 = _previous_error;
    _previous_error = error;
    _previous_value2 = _previous_value;
    _previous_value = current_value;
    
    // _last_outputには_min_drive_powerを加える前の値を保存
    _last_output = output_before_min_power;
    _last_goal = _goal;

    return output;
}

// ★★★ ここからが追加/修正された関数実装 ★★★

/**
 * @brief 静止摩擦を打ち消すための最小駆動力を設定します。
 * この値は、PID出力がゼロから動き出す際に、静止摩擦や不感帯を
 * 乗り越えるために必要な最低限の出力値を指定します。
 * @param min_power モーターが動き出す最小の出力値。常に正の値に変換されます。
 */
void PID::set_min_drive_power(float min_power) {
    _min_drive_power = std::abs(min_power);
}

/**
 * @brief 誤差の不感帯（デッドバンド）を設定します。
 * この値以下の誤差は0として扱われ、PID制御の出力も0になります。
 * @param deadband 不感帯の幅。この値以下の誤差は0として扱われます。
 */
void PID::set_deadband(float deadband) {
    _deadband = std::abs(deadband);
}

void PID::set_goal(float goal) { 
    _goal = goal; 
}

void PID::set_dt(float dt_sec) { 
    if (dt_sec > 0) _dt = dt_sec; 
}

void PID::set_gains(float kp, float ki, float kd) { 
    _kp = kp; 
    _ki = ki; 
    _kd = kd; 
}

void PID::set_mode(Mode mode) { 
    if (_mode != mode) {
        _mode = mode; 
        reset();
    }
}

void PID::set_output_limits(float min, float max) { 
    _output_min = min; 
    _output_max = max; 
}

void PID::enable_anti_windup(bool enable) { 
    _anti_windup_enabled = enable; 
}

void PID::enable_derivative_on_measurement(bool enable) { 
    _derivative_on_measurement_enabled = enable; 
}

void PID::enable_feedforward(bool enable, float kf) {
    _feedforward_enabled = enable;
    _kf = kf;
}

void PID::set_output_offset(float offset) {
    _output_offset = offset;
}

void PID::gravity_reduction(float gravity) {
    _gravity_offset += gravity;
}

void PID::reset_gravity_offset() {
    _gravity_offset = 0.0;
}

void PID::set_gravity_offset(float gravity) {
    _gravity_offset = gravity;
}

void PID::reset() {
    _integral = 0.0;
    _previous_error = 0.0;
    _previous_error2 = 0.0;
    _previous_value = 0.0;
    _previous_value2 = 0.0;
    _last_output = 0.0;
    _last_goal = 0.0;
    _gravity_offset = 0.0;
    _initialized = false;
}

float PID::constrain(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}