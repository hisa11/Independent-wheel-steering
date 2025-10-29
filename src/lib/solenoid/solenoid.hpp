#ifndef SOLENOID_HPP
#define SOLENOID_HPP
#include <mbed.h>

class solenoid {
    public:
        solenoid(CAN &can, int id);
        bool sendmessage(); // メンバ関数に修正
        void data(int id,bool state);
        bool getstatus(int id);

        bool status[8];
        CAN &can; // CAN参照をメンバ変数として保持
        int id;



};

#endif