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
    if (msg == "ci:p")
        Circle = true;
    else if (msg == "ci:no_p")
        Circle = false;

    if (msg == "cr:p")
        Cross = true;
    else if (msg == "cr:no_p")
        Cross = false;

    if (msg == "sq:p")
        Square = true;
    else if (msg == "sq:no_p")
        Square = false;

    if (msg == "tri:p")
        Triangle = true;
    else if (msg == "tri:no_p")
        Triangle = false;

    if (msg == "L1:p")
        L1 = true;
    else if (msg == "L1:no_p")
        L1 = false;

    if (msg == "R1:p")
        R1 = true;
    else if (msg == "R1:no_p")
        R1 = false;

    if (msg == "L2:p")
        L2 = true;
    else if (msg == "L2:no_p")
        L2 = false;

    if (msg == "R2:p")
        R2 = true;
    else if (msg == "R2:no_p")
        R2 = false;

    if (msg == "SH:p")
        SHARE = true;
    else if (msg == "SH:no_p")
        SHARE = false;

    if (msg == "OP:p")
        OPTION = true;
    else if (msg == "OP:no_p")
        OPTION = false;

    if (msg == "PS:p")
        PS = true;
    else if (msg == "PS:no_p")
        PS = false;

    if (msg == "u:p")
        Up = true;
    else if (msg == "u:no_p")
        Up = false;

    if (msg == "d:p")
        Down = true;
    else if (msg == "d:no_p")
        Down = false;

    if (msg == "l:p")
        Left = true;
    else if (msg == "l:no_p")
        Left = false;

    if (msg == "r:p")
        Right = true;
    else if (msg == "r:no_p")
        Right = false;

    if (msg == "L3:p")
        L3 = true;
    else if (msg == "L3:no_p")
        L3 = false;

    if (msg == "R3:p")
        R3 = true;
    else if (msg == "R3:no_p")
        R3 = false;
}
