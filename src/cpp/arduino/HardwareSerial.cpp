#include "HardwareSerial.h"
#include "wiring_digital.h"
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <sys/ioctl.h>

HardwareSerial::HardwareSerial() : serial_fd{-1}
{
}

HardwareSerial::~HardwareSerial()
{
    // Close file descriptor if valid
    if (serial_fd != -1)
    {
        close(serial_fd);
    }
}

void HardwareSerial::begin(unsigned long baud, uint8_t config)
{
    if (serial_fd < 0)
    {
        return;
    }
    struct termios tty_config;
    // struct termios tty_config;
    memset(&tty_config, 0, sizeof(tty_config));

    if (0 != tcgetattr(serial_fd, &tty_config))
    {
        int errno_saved = errno;
        std::cout << "SERIAL: tgetattr error " << errno_saved << std::endl;
    }

    /* Setting CONTROL OPTIONS. */
    tty_config.c_cflag |= CREAD;  // Enable read.
    tty_config.c_cflag |= CLOCAL; // Set local mode.
    if (config <= SERIAL_8N2)
    {
        tty_config.c_cflag &= ~PARENB; // Disable parity.
    }
    else
    {
        tty_config.c_cflag |= PARENB; // Enable parity.
        if (SERIAL_5E1 <= config && config <= SERIAL_8E2)
        {
            tty_config.c_cflag &= ~PARODD; // Set even parity.
        }
        else
        {
            tty_config.c_cflag |= PARODD; // Set odd parity.
        }
    }
    tty_config.c_cflag &= ~CSIZE; // Mask the character size bits.
    if (config <= SERIAL_8N1 || (SERIAL_5E1 < config && config <= SERIAL_8E1) || (SERIAL_5O1 <= config && config <= SERIAL_8O1))
    {
        tty_config.c_cflag &= ~CSTOPB; // Set one stop bit.
    }
    else
    {
        tty_config.c_cflag |= CSTOPB; // Set two stop bits.
    }
    if (config == SERIAL_5N1 || config == SERIAL_5N2 || config == SERIAL_5E1 || config == SERIAL_5E2 || config == SERIAL_5O1 || config == SERIAL_5O2)
    {
        tty_config.c_cflag |= CS5;
    }
    else if (config == SERIAL_6N1 || config == SERIAL_6N2 || config == SERIAL_6E1 || config == SERIAL_6E2 || config == SERIAL_6O1 || config == SERIAL_6O2)
    {
        tty_config.c_cflag |= CS6;
    }
    else if (config == SERIAL_7N1 || config == SERIAL_7N2 || config == SERIAL_7E1 || config == SERIAL_7E2 || config == SERIAL_7O1 || config == SERIAL_7O2)
    {
        tty_config.c_cflag |= CS7;
    }
    else
    {
        tty_config.c_cflag |= CS8;
    }

    if (config <= SERIAL_8N1 || (SERIAL_5E1 < config && config <= SERIAL_8E1) || (SERIAL_5O1 <= config && config <= SERIAL_8O1))
    {
        tty_config.c_cflag &= ~CSTOPB; // Set one stop bit.
    }
    else
    {
        tty_config.c_cflag |= CSTOPB; // Set two stop bits.
    }

    tty_config.c_cflag &= ~CRTSCTS; // Disable hardware flow control.

    /* Setting LOCAL OPTIONS. */
    tty_config.c_lflag &= ~ICANON; // Set non-canonical input.
    tty_config.c_lflag &= ~ECHO;   // Disable echoing of input characters.
    tty_config.c_lflag &= ~ECHOE;  // Disable echoing the erase character.
    tty_config.c_lflag &= ~ISIG;   // Disable SIGINTR, SIGSUSP, SIGDSUSP
                                   // and SIGQUIT signals.

    /* Setting INPUT OPTIONS. */
    tty_config.c_iflag &= ~IXON;   // Disable output software flow control.
    tty_config.c_iflag &= ~IXOFF;  // Disable input software flow control.
    tty_config.c_iflag &= ~INPCK;  // Disable parity check.
    tty_config.c_iflag &= ~ISTRIP; // Disable strip parity bits.
    tty_config.c_iflag &= ~IGNBRK; // No ignore break condition.
    tty_config.c_iflag &= ~IGNCR;  // No ignore carrier return.
    tty_config.c_iflag &= ~INLCR;  // No map NL to CR.
    tty_config.c_iflag &= ~ICRNL;  // No map CR to NL.

    /* Setting OUTPUT OPTIONS. */
    tty_config.c_oflag &= ~OPOST; // Set raw output.

    /* Setting OUTPUT CHARACTERS. */
    tty_config.c_cc[VMIN] = 1;
    tty_config.c_cc[VTIME] = 1;

    /* Setting BAUD RATE. */
    cfsetispeed(&tty_config, getBaudRate(baud));
    cfsetospeed(&tty_config, getBaudRate(baud));

    if (0 != tcsetattr(serial_fd, TCSANOW, &tty_config))
    {
        std::cerr << "SERIAL: tgetattr error " << strerror(errno) << std::endl;
    }
}

int HardwareSerial::open(const std::string &device, unsigned long baud, uint8_t config)
{
    if (0 < serial_fd)
    {
        ::close(serial_fd);
    }
    if (0 < (serial_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)))
    {
        begin(baud, config);
    }
    else
    {
        std::cerr << "SERIAL: open error " << strerror(errno) << std::endl;
        return -1;
    }
    return serial_fd;
}

int HardwareSerial::available()
{
    int bytesAvailable;
    ioctl(serial_fd, FIONREAD, &bytesAvailable);
    return bytesAvailable;
}

int HardwareSerial::read()
{
    char buf;
    size_t n = readBytes( &buf, 1ul);
    if(n == 0)
    {
        return -1;
    }
    return buf;
}

size_t HardwareSerial::readBytes( char *buffer, size_t length) 
{
    if (peek_flag)
    {
        peek_flag = false;
        return peek_buffer;
    }

    uint8_t buf;
    ssize_t n = ::read(serial_fd, &buf, 1ul);

    if (n <= 0)
    {
        return 0;
    }

    return n;
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
    ssize_t n = ::write(serial_fd, buffer, size);
    if (n < 0)
    {
        std::cerr << "SERIAL: write error " << strerror(errno) << std::endl;
        return 0;
    }
    return n;
}

size_t HardwareSerial::write(uint8_t c)
{
    return write(&c, 1);
}

void HardwareSerial::end()
{
    if (serial_fd != -1)
    {
        serial_fd = close(serial_fd);
        serial_fd = -1;
    }
}

void HardwareSerial::flush()
{
    tcflush(serial_fd, TCIOFLUSH);
}

int HardwareSerial::peek(void)
{
    if (peek_flag)
    {
        return peek_buffer;
    }

    // 非ブロッキング読み取りを試みる
    char c;
    ssize_t size = ::read(serial_fd, &c, 1);
    if (size > 0)
    {
        peek_buffer = c;
        peek_flag = true;
        return c;
    }

    return -1;
}

HardwareSerial Serial1;