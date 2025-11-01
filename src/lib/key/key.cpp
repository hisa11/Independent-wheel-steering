// language: cpp
#include "key.hpp"
#include "mbed.h"

// 変数の定義
bool Circle = false;
bool Cross = false;
bool Square = false;
bool Triangle = false;
bool Up = false;
bool Right = false;
bool Down = false;
bool Left = false;
bool L1 = false;
bool R1 = false;
bool L2 = false;
bool R2 = false;
bool SHARE = false;
bool OPTION = false;
bool PS = false;
bool L3 = false;
bool R3 = false;


void key_puress(std::string &msg) {
    // ROS2のフォーマットに対応: "button:pressing" または "button:no_pressing"
    if (msg == "circle:pressing")
        Circle = true;
    else if (msg == "circle:no_pressing")
        Circle = false;

    if (msg == "cross:pressing")
        Cross = true;
    else if (msg == "cross:no_pressing")
        Cross = false;

    if (msg == "square:pressing")
        Square = true;
    else if (msg == "square:no_pressing")
        Square = false;

    if (msg == "triangle:pressing")
        Triangle = true;
    else if (msg == "triangle:no_pressing")
        Triangle = false;

    if (msg == "L1:pressing")
        L1 = true;
    else if (msg == "L1:no_pressing")
        L1 = false;

    if (msg == "R1:pressing")
        R1 = true;
    else if (msg == "R1:no_pressing")
        R1 = false;

    if (msg == "L2:pressing")
        L2 = true;
    else if (msg == "L2:no_pressing")
        L2 = false;

    if (msg == "R2:pressing")
        R2 = true;
    else if (msg == "R2:no_pressing")
        R2 = false;

    if (msg == "SHARE:pressing")
        SHARE = true;
    else if (msg == "SHARE:no_pressing")
        SHARE = false;

    if (msg == "OPTIONS:pressing")
        OPTION = true;
    else if (msg == "OPTIONS:no_pressing")
        OPTION = false;

    if (msg == "PS:pressing")
        PS = true;
    else if (msg == "PS:no_pressing")
        PS = false;

    if (msg == "up:pressing")
        Up = true;
    else if (msg == "up:no_pressing")
        Up = false;

    if (msg == "down:pressing")
        Down = true;
    else if (msg == "down:no_pressing")
        Down = false;

    if (msg == "left:pressing")
        Left = true;
    else if (msg == "left:no_pressing")
        Left = false;

    if (msg == "right:pressing")
        Right = true;
    else if (msg == "right:no_pressing")
        Right = false;

    // L3とR3はROS2側で送信されていないため、デフォルトのまま
    if (msg == "L3:pressing")
        L3 = true;
    else if (msg == "L3:no_pressing")
        L3 = false;

    if (msg == "R3:pressing")
        R3 = true;
    else if (msg == "R3:no_pressing")
        R3 = false;
}
