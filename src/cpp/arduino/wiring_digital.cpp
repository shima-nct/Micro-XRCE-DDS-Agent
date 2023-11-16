#include <chrono>
#include <iostream>
#include <gpiod.h>

// ピンモードを定義する
enum GPIOMode {
    INPUT,
    OUTPUT
};


constexpr auto chipName = "gpiochip0";

// libgpiod チップとラインを取得するためのヘルパー関数
gpiod_line *getLine(int pin, gpiod_chip *&chip)
{
    chip = gpiod_chip_open_by_name(chipName);
    if (!chip)
    {
        std::cerr << "Cannot open GPIO chip" << std::endl;
        return nullptr;
    }

    gpiod_line *line = gpiod_chip_get_line(chip, pin);
    if (!line)
    {
        std::cerr << "Cannot get line " << pin << std::endl;
        gpiod_chip_close(chip);
        return nullptr;
    }

    return line;
}

void pinMode(int pin, GPIOMode mode)
{
    gpiod_chip *chip;
    gpiod_line *line = getLine(pin, chip);
    if (!line)
        return;

    if (mode == INPUT)
    { // INPUT
        if (gpiod_line_request_input(line, "gpio") < 0)
        {
            std::cerr << "Cannot set line as input" << std::endl;
        }
    }
    else
    { // OUTPUT
        if (gpiod_line_request_output(line, "gpio", 0) < 0)
        {
            std::cerr << "Cannot set line as output" << std::endl;
        }
    }

    gpiod_chip_close(chip);
}

void digitalWrite(int pin, int value)
{
    gpiod_chip *chip;
    gpiod_line *line = getLine(pin, chip);
    if (!line)
        return;

    if (gpiod_line_set_value(line, value) < 0)
    {
        std::cerr << "Cannot set line value" << std::endl;
    }

    gpiod_chip_close(chip);
}

int digitalRead(int pin)
{
    gpiod_chip *chip;
    gpiod_line *line = getLine(pin, chip);
    if (!line)
        return -1;

    int value = gpiod_line_get_value(line);
    if (value < 0)
    {
        std::cerr << "Cannot read line value" << std::endl;
    }

    gpiod_chip_close(chip);
    return value;
}

// Function to read a bit at a given position
unsigned char bitRead(unsigned long x, unsigned char bitPosition) {
    return (x >> bitPosition) & 0x01;
}
