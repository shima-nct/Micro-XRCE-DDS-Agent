// Copyright 2017-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uxr/agent/transport/serial/LLCC68Agent.hpp>
#include <uxr/agent/transport/serial/SerialAgentLinux.hpp>
#include <uxr/agent/utils/Conversion.hpp>
#include <uxr/agent/logger/Logger.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
namespace eprosima {
namespace uxr {

LLCC68Agent::LLCC68Agent(
        char const* dev,
        int open_flags,
        termios const& termios_attrs,
        uint8_t addr,
        Middleware::Kind middleware_kind)
    : SerialAgent(addr, middleware_kind)
    , dev_{dev}
    , open_flags_{open_flags}
    , termios_attrs_{termios_attrs}
    , lora_e220_(&Serial1, Serial1.getBaudRate(termios_attrs.c_ispeed))
    , lora_addr{18030u}
    , ADDH{static_cast<uint8_t>((lora_addr >> 8) & 0xFFu)}
    , ADDL{static_cast<uint8_t>(lora_addr & 0xFFu)}
    , CHAN{12u}
    {
        framing_io_ = FramingIO(
        addr,
        std::bind(&LLCC68Agent::write_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
        std::bind(&LLCC68Agent::read_data, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    }

    LLCC68Agent::~LLCC68Agent()
    {
        try
        {
            stop();
        stop();
    }
    catch (std::exception& e)
    {
        UXR_AGENT_LOG_CRITICAL(
            UXR_DECORATE_RED("error stopping server"),
            "exception: {}",
            e.what());
    }
}

bool LLCC68Agent::init()
{
    bool rv = false;

    // Check if serial port exist
    std::chrono::steady_clock::time_point begin;
    int serial_exist = 0;
    int error_count = 0;

    do
    {
        if (serial_exist != 0)
        {
            std::this_thread::sleep_for((std::chrono::milliseconds) 10);

            if (EACCES == errno || EBUSY == errno)
            {
                // Increase error count
                error_count++;

                if (error_count > 10)
                {
                    // Resource busy or superuser privileges required
                    break;
                }
            }
            else if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - begin).count())
            {
                begin = std::chrono::steady_clock::now();
                UXR_AGENT_LOG_INFO(
                    UXR_DECORATE_YELLOW("Serial port not found."),
                    "device: {}, error {}, waiting for connection...",
                    dev_, errno);
            }
        }

        serial_exist = access(dev_.c_str(), W_OK );
    }
    while (serial_exist != 0);

    // poll_fd_.fd = open(dev_.c_str(), open_flags_);
    poll_fd_.fd = Serial1.open(dev_.c_str());
    lora_e220_.begin();
    if (0 < poll_fd_.fd)
    {
         rv = true;
        struct termios new_attrs;
        memset(&new_attrs, 0, sizeof(new_attrs));
        if (0 == tcgetattr(poll_fd_.fd, &new_attrs))
        {
//             new_attrs.c_cflag = termios_attrs_.c_cflag;
//             new_attrs.c_lflag = termios_attrs_.c_lflag;
//             new_attrs.c_iflag = termios_attrs_.c_iflag;
//             new_attrs.c_oflag = termios_attrs_.c_oflag;
//             new_attrs.c_cc[VMIN] = termios_attrs_.c_cc[VMIN];
//             new_attrs.c_cc[VTIME] = termios_attrs_.c_cc[VTIME];

// #if _HAVE_STRUCT_TERMIOS_C_ISPEED || __APPLE__
//             cfsetispeed(&new_attrs, termios_attrs_.c_ispeed);
// #endif
// #if _HAVE_STRUCT_TERMIOS_C_OSPEED || __APPLE__
//             cfsetospeed(&new_attrs, termios_attrs_.c_ospeed);
// #endif

            if (0 == tcsetattr(poll_fd_.fd, TCSANOW, &new_attrs))
            {
                rv = true;
                poll_fd_.events = POLLIN;

                tcflush(poll_fd_.fd, TCIOFLUSH);

                UXR_AGENT_LOG_INFO(
                    UXR_DECORATE_GREEN("running..."),
                    "fd: {}",
                    poll_fd_.fd);
            }
            else
            {
                UXR_AGENT_LOG_ERROR(
                    UXR_DECORATE_RED("set termios attributes error"),
                    "errno: {}",
                    errno);
            }
        }
        else
        {
            UXR_AGENT_LOG_ERROR(
                UXR_DECORATE_RED("get termios attributes error"),
                "errno: {}",
                errno);
        }
    }
    else
    {
        UXR_AGENT_LOG_ERROR(
            UXR_DECORATE_RED("open device error"),
            "device: {}, errno: {}{}",
            dev_, errno,
            (EACCES == errno) ? ". Please re-run with superuser privileges." : "");
    }
    return rv;
}

bool LLCC68Agent::fini()
{
    if (-1 == poll_fd_.fd)
    {
        return true;
    }

    bool rv = false;
    Serial1.end();
    poll_fd_.fd = Serial1.getFd();
    if (0 > poll_fd_.fd)
    {
        UXR_AGENT_LOG_INFO(
            UXR_DECORATE_GREEN("server stopped"),
            "fd: {}",
            poll_fd_.fd);
        rv = true;
    }
    else
    {
        UXR_AGENT_LOG_ERROR(
            UXR_DECORATE_RED("close server error"),
            "fd: {}, errno: {}",
            poll_fd_.fd, errno);
    }

    poll_fd_.fd = -1;
    fifo_buffer_ = std::queue<uint8_t>();
    return rv;
}

void printBuffer(const unsigned char *buffer, size_t length)
{
    printf("len: %zu, data: \n", length);
    for (size_t i = 0; i < length; ++i)
    {
        if (i % 0x20 == 0)
        {
            printf("%04lX: ", i);
        }
        printf("%02X ", buffer[i]);
        if ((i + 1) % 0x20 == 0 || i + 1 == length)
        {
            printf("\n");
        }
    }
}

ssize_t LLCC68Agent::write_data(
        uint8_t* buf,
        size_t len,
        TransportRc& transport_rc)
{
    // std::vector<byte> message(buf, buf+len);
    const size_t lora_header_size = 3;
    std::vector<byte> message(len + lora_header_size);
    message.data()[0] = ADDH;
    message.data()[1] = ADDL;
    message.data()[2] = CHAN;
    memmove(message.data() + lora_header_size, buf, len);
    size_t rv = 0;
    ssize_t bytes_written = ::write(poll_fd_.fd, message.data(), message.size());
    bytes_written -= lora_header_size;
    int ret = tcdrain(poll_fd_.fd);
    struct timespec req, rem;
    // 0.1秒（100ミリ秒）を設定
    req.tv_sec = 0;          // 秒
    req.tv_nsec = 50000000; // ナノ秒

    // nanosleepを呼び出す
    if (nanosleep(&req, &rem) < 0) {
        std::cout << "Can't sleep: ";
    }
    // bytes_written -= 3;
    // ResponseStatus rs = lora_e220_.sendFixedBinaryMessage(ADDH, ADDL, CHAN, message);

    if (-1 !=bytes_written && ret == 0)
    // if (E220_SUCCESS == rs.code)
    {
        rv = bytes_written;

        std::cout << "Sent: ";
        printBuffer(message.data(), bytes_written);
    }
    else
    {
        transport_rc = TransportRc::server_error;
    }
    return rv;
}

ssize_t LLCC68Agent::read_data(
        uint8_t* buf,
        size_t len,
        int timeout,
        TransportRc& transport_rc)
{
    ssize_t bytes_read = 0;
    int poll_rv = poll(&poll_fd_, 1, timeout);
    if(poll_fd_.revents & (POLLERR+POLLHUP))
    {
        transport_rc = TransportRc::server_error;;
    }
    else if (0 < poll_rv)
    {
        bytes_read = read(poll_fd_.fd, buf, len);
        printf("Received: ");
        printBuffer(buf, bytes_read);
        if (0 > bytes_read)
        {
            transport_rc = TransportRc::server_error;
        }
        // ResponseBinaryMessageContainer rc = lora_e220_.receiveBinaryMessage(len);
        // if (E220_SUCCESS == rc.status.code)
        // {
        //     bytes_read = rc.data.size();
        //     memcpy(buf, rc.data.data(), bytes_read);
        // }
        // else
        // {
        //     transport_rc = TransportRc::server_error;
        // }
    }
    else
    {
        transport_rc = (poll_rv == 0) ? TransportRc::timeout_error : TransportRc::server_error;
    }
    return bytes_read;
}

bool LLCC68Agent::handle_error(
        TransportRc /*transport_rc*/)
{
    return fini() && init();
}

} // namespace uxr
} // namespace eprosima
