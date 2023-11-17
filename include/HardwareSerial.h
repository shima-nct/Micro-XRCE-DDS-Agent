#ifndef LINUX_HARDWARESERIAL_H
#define LINUX_HARDWARESERIAL_H

#include "Stream.h"
#include <termios.h>
#include <functional>

// Define config for Serial.begin(baud, config);
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06

#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E

#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26

#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E

#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36

#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

class HardwareSerial : public Stream
{
private:
    unsigned long timeout = 1000; // デフォルトのタイムアウト（ミリ秒）

public:
    HardwareSerial();
    ~HardwareSerial();

    int open(const std::string &device, unsigned long baud = 9600, uint8_t config = SERIAL_8N1);
    void begin(unsigned long baud, uint8_t config = SERIAL_8N1)
    {
#ifdef DEFAULT_DEVICE
        open(DEFAULT_DEVICE, baud, config);
#else
        open("/dev/ttyS0", baud, config);
#endif
    }

    int available();
    void flush();
    int peek(void);
    int read(void);
    size_t read(uint8_t *buffer, size_t size);
    inline size_t read(char *buffer, size_t size)
    {
        return read((uint8_t *)buffer, size);
    }
    size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    size_t write(const uint8_t *buffer, size_t size);
    void end();
    int getFd() { return serial_fd; }

private:
    int serial_fd;
    struct termios tty;
    int peek_buffer;
    bool peek_flag;
};

extern HardwareSerial Serial1;

#endif // LINUX_HARDWARESERIAL_H