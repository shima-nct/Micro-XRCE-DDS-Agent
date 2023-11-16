#ifndef ARDUINO_H
#define ARDUINO_H

#include <string.h>
#include "Stream.h"
#include "HardwareSerial.h"
#include "WString.h"
#include "wiring_digital.h"

using byte = unsigned char;
#define F(x) (x)

unsigned long millis();

#define HIGH 0x1
#define LOW  0x0

void pinMode(int pin, GPIOMode mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
unsigned char bitRead(unsigned long x, unsigned char bitPosition);

#endif // ARDUINO_H