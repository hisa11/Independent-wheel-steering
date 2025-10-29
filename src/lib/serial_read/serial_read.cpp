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

std::string serial_unit::read_serial()
{
    static std::string str = "";
    char buf[1];

    while (men_serial.readable())
    {
        men_serial.read(buf, 1);
        if (buf[0] == '|')
        {
            std::string buff_str = str;
            str.clear();
            return buff_str;
        }
        else
        {
            str += buf[0];
        }
    }
    return "";
}


void serial_read() {
    while (1) {
        // printf("serial_read\n");
        std::string msg = serial.read_serial();
        if (msg != "") {
            if (msg[0] == 'n') {
                // if (R3 && Right) {
                //     msg = "n:-0.500000:0.000000:0.000000:0.000000|";
                //     printf("Updated msg: %s\n", msg.c_str()); // 増加した値を表示
                // } else if (R3 && Left) {
                //     msg = "n:0.500000:0.000000:0.000000:0.000000|";
                //     printf("Updated msg: %s\n", msg.c_str()); // 増加した値を表示
                // } else if (R3 && Up) {
                //     msg = "n:0.000000:0.500000:0.000000:0.000000|";
                //     printf("Updated msg: %s\n", msg.c_str()); // 増加した値を表示
                // } else if (R3 && Down) {
                //     msg = "n:0.000000:-0.500000:0.000000:0.000000|";
                //     printf("Updated msg: %s\n", msg.c_str()); // 増加した値を表示
                // }
                move_aa(msg);
            } else {
                key_puress(msg);
            }
        }
        // ThisThread::sleep_for(10ms);
    }
}