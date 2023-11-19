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

#ifndef UXR_AGENT_TRANSPORT_SERIAL_LLCC68AGENT_HPP_
#define UXR_AGENT_TRANSPORT_SERIAL_LLCC68AGENT_HPP_

#include <uxr/agent/transport/serial/SerialAgentLinux.hpp>

#include <termios.h>
#include "HardwareSerial.h"
#include "LoRa_E220.h"
#include <queue>

namespace eprosima {
namespace uxr {

class LLCC68Agent : public SerialAgent
{
public:
    LLCC68Agent(
            char const * dev,
            int open_flags,
            termios const & termios_attrs,
            uint8_t addr,
            Middleware::Kind middleware_kind);

    ~LLCC68Agent();

    int getfd() { return poll_fd_.fd; };
    
    ssize_t write_data(
            uint8_t* buf,
            size_t len,
            TransportRc& transport_rc);

    ssize_t read_data(
            uint8_t* buf,
            size_t len,
            int timeout,
            TransportRc& transport_rc);

private:
    bool init() final;
    bool fini() final;
    bool handle_error(
            TransportRc transport_rc) final;

private:
    const std::string dev_;
    const int open_flags_;
    const termios termios_attrs_;
    LoRa_E220 lora_e220_;
    uint16_t lora_addr;
    byte ADDH;
	byte ADDL;
    byte CHAN;
    std::queue<uint8_t> fifo_buffer_;
};

} // namespace uxr
} // namespace eprosima

#endif // UXR_AGENT_TRANSPORT_SERIAL_LLCC68AGENT_HPP_