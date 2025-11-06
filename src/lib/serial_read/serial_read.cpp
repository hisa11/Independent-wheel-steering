#include "serial_read.hpp"
#include "mbed.h"
#include "key.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>

std::vector<double> to_numbers(const std::string &input) {
    std::vector<double> numbers;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, ':')) { // ':'で区切る
        if (!token.empty() && token.back() == '|') {  // 最後の '|' を削除
            token.pop_back();
        }
        numbers.push_back(std::stod(token)); // 文字列をdoubleに変換
    }
    return numbers;
}

void serial_unit::start_event_mode()
{
    // イベントコールバックを設定
    men_serial.sigio(callback(this, &serial_unit::rx_event_callback));
    men_serial.set_blocking(false);
}

void serial_unit::rx_event_callback()
{
    // 割り込みハンドラでは最小限の処理のみ行う
    // フラグを立てるだけで、実際の処理は別で行う
    data_available = true;
}

void serial_unit::process_received_data()
{
    if (!data_available) {
        return;
    }
    
    data_available = false;
    
    char buffer[64];  // 一時バッファ
    
    while (men_serial.readable()) {
        ssize_t read_count = men_serial.read(buffer, sizeof(buffer));
        
        if (read_count > 0) {
            // 受信データをバッファに追加
            for (ssize_t i = 0; i < read_count; i++) {
                if (buffer[i] == '|') {
                    // メッセージ終端文字を検出
                    if (!rx_buffer.empty()) {
                        queue_mutex.lock();
                        // キューがいっぱいでない場合のみ追加
                        if (message_queue.size() < MAX_QUEUE_SIZE) {
                            message_queue.push(rx_buffer);
                        } else {
                            // キューがいっぱいの場合、古いメッセージを破棄
                            message_queue.pop();
                            message_queue.push(rx_buffer);
                        }
                        queue_mutex.unlock();
                        rx_buffer.clear();
                    }
                } else {
                    // バッファオーバーフロー防止
                    if (rx_buffer.length() < MAX_BUFFER_SIZE) {
                        rx_buffer += buffer[i];
                    } else {
                        // バッファが大きすぎる場合はクリア（破損データ）
                        rx_buffer.clear();
                    }
                }
            }
        }
    }
}

bool serial_unit::get_message(std::string &msg)
{
    // データ処理を実行（割り込みではなくメインスレッドで）
    process_received_data();
    
    queue_mutex.lock();
    if (!message_queue.empty()) {
        msg = message_queue.front();
        message_queue.pop();
        queue_mutex.unlock();
        return true;
    }
    queue_mutex.unlock();
    return false;
}


void serial_read() {
    uint32_t skip_count = 0;  // â€» ã‚¹ã‚­ãƒƒãƒ—ã‚«ã‚¦ãƒ³ã‚¿ãƒ¼
    while (1) {
        std::string msg;
        if (serial.get_message(msg)) {
            if (!msg.empty()) {
                // â€» å…¥åŠ›ãŒ0ã®å ´åˆã€90%ã®ãƒ¡ãƒƒã‚»ãƒ¼ã‚¸ã‚'ã‚¹ã‚­ãƒƒãƒ—ã—ã¦è² è·è»½æ¸›
                if (msg.find("n:0.00:0.00:0.00") != std::string::npos) {
                    skip_count++;
                    if (skip_count % 10 != 0) {  // 10å›žã«1å›žã ã'å‡¦ç†
                        ThisThread::sleep_for(50ms);
                        continue;
                    }
                }
                else {
                    skip_count = 0;  // éžã‚¼ãƒ­å…¥åŠ›ã®æ™‚ã¯ãƒªã‚»ãƒƒãƒˆ
                }
                
                if (msg[0] == 'n') {
                    move_aa(msg);
                } else {
                    key_puress(msg);
                }
            }
        }
        // メッセージがない場合は短時間スリープしてCPUを解放
        ThisThread::sleep_for(50ms);
    }
}