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

    while (std::getline(ss, token, ':')) {
        if (!token.empty() && token.back() == '|') {
            token.pop_back();
        }
        numbers.push_back(std::stod(token));
    }
    return numbers;
}

void serial_unit::start_event_mode()
{
    // ブロッキングモードに設定（100ms間隔なのでイベント不要）
    men_serial.set_blocking(true);
}

// シンプルなブロッキング読み取り
bool serial_unit::get_message(std::string &msg)
{
    msg.clear();
    char c;
    int timeout_count = 0;
    const int MAX_TIMEOUT = 150; // 150ms待機（100ms間隔より少し長め）
    
    // '|'まで読み取る
    while (timeout_count < MAX_TIMEOUT) {
        ssize_t result = men_serial.read(&c, 1);
        
        if (result == 1) {
            if (c == '|') {
                // メッセージ完了
                return !msg.empty();
            } else if (c == '\n' || c == '\r') {
                // 改行は無視
                continue;
            } else {
                // バッファオーバーフロー防止
                if (msg.length() < MAX_BUFFER_SIZE) {
                    msg += c;
                } else {
                    // バッファが大きすぎる場合はクリア（破損データ）
                    msg.clear();
                    return false;
                }
            }
            timeout_count = 0; // データ受信したらタイムアウトカウントリセット
        } else {
            // データなし、1msスリープ
            ThisThread::sleep_for(1ms);
            timeout_count++;
        }
    }
    
    // タイムアウト
    return false;
}

void serial_read() {
    uint32_t skip_count = 0;
    
    while (1) {
        std::string msg;
        
        // ブロッキング読み取り（メッセージが来るまで待機）
        if (serial.get_message(msg)) {
            if (!msg.empty()) {
                // *** 入力が0の場合、90%のメッセージをスキップして負荷軽減 ***
                if (msg.find("n:0.00:0.00:0.00:0.00") != std::string::npos || 
                    msg.find("n:0.0:0.0:0.0:0.0") != std::string::npos) {
                    skip_count++;
                    if (skip_count % 10 != 0) {
                        continue; // スリープ不要、すぐ次のメッセージ待ち
                    }
                }
                else {
                    skip_count = 0;
                }
                
                if (msg[0] == 'n') {
                    move_aa(msg);
                } else {
                    key_puress(msg);
                }
            }
        }
        // get_message内でタイムアウト処理しているので、ここではスリープ不要
    }
}