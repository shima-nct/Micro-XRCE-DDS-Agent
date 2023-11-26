#ifndef COMMON_H
#define COMMON_H

#include <gpiod.h>

// HIGH and LOW constants
constexpr int HIGH = 1;
constexpr int LOW = 0;

// ピンモードを定義する
enum GPIOMode {
    INPUT,
    OUTPUT
};

// 関数宣言
void pinMode(int pin, GPIOMode mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
unsigned char bitRead(unsigned long x, unsigned char bitPosition);

#endif // COMMON_H
