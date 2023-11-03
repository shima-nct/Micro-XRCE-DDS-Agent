// Copyright 2021-present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <uxr/agent/transport/custom/CustomAgent.hpp>
#include <uxr/agent/transport/endpoint/IPv4EndPoint.hpp>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/**
 * This custom XRCE Agent example attempts to show how easy is for the user to define a custom
 * Micro XRCE-DDS Agent behaviour, in terms of transport initialization and closing, and also
 * regarding read and write operations.
 * For this simple case, an UDP socket is opened on port 8888. Additionally, some information
 * messages are being printed to demonstrate the custom behaviour.
 * As the endpoint is already defined, we are using the provided
 * `eprosima::uxr::IPv4EndPoint` by the library.
 * Other transport protocols might need to implement their own endpoint struct.
 */

int main(int argc, char** argv)
{
  const char* device_name = nullptr;
  speed_t baud = B9600;

  int opt;
  while ((opt = getopt(argc, argv, "D:b:")) != -1)
  {
    switch (opt)
    {
      case 'D':
        device_name = optarg;
        break;
      case 'b': {
        int baud_rate = std::stoi(optarg);
        switch (baud_rate)
        {
          case 9600:
            baud = B9600;
            break;
          case 19200:
            baud = B19200;
            break;
          case 38400:
            baud = B38400;
            break;
          // 他のボーレートに対するケースもここに追加
          default:
            std::cerr << "Unsupported baud rate: " << baud_rate << std::endl;
            return 1;
        }
        break;
      }
      default:
        std::cerr << "Usage: " << argv[0] << " -D <device> -b <baud rate>" << std::endl;
        return 1;
    }
  }

  if (device_name == nullptr)
  {
    std::cerr << "Device name is required." << std::endl;
    return 1;
  }

  eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);
  uint16_t agent_port(8888);

  struct pollfd poll_fd;

  /**
   * @brief Agent's initialization behaviour description.
   */
  eprosima::uxr::CustomAgent::InitFunction init_function = [&]() -> bool {
    bool rv = false;
    poll_fd.fd = open(device_name, O_RDWR | O_NOCTTY | O_SYNC);

    if (-1 != poll_fd.fd)
    {
      struct termios tty
      {
      };
      memset(&tty, 0, sizeof tty);

      if (tcgetattr(poll_fd.fd, &tty) == 0)
      {
        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                      // disable break processing
        tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing
        tty.c_oflag = 0;                             // no remapping, no delays
        tty.c_cc[VMIN] = 0;                          // read doesn't block
        tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls,
                                            // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
        tty.c_cflag |= 0;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(poll_fd.fd, TCSANOW, &tty) == 0)
        {
          poll_fd.events = POLLIN;
          rv = true;

          UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent INIT function"),
                             "Serial port: "
                             "/dev/ttyUSB0 "
                             "is now open "
                             "with 9600 "
                             "bps.",
                             NULL);
        }
      }
    }

    return rv;
  };

  /**
   * @brief Agent's destruction actions.
   */
  eprosima::uxr::CustomAgent::FiniFunction fini_function = [&]() -> bool {
    if (-1 == poll_fd.fd)
    {
      return true;
    }

    if (0 == close(poll_fd.fd))
    {
      poll_fd.fd = -1;
      UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent FINI function"),
                         "Serial port: ", device_name);

      return true;
    }
    else
    {
      return false;
    }
  };

  /**
   * @brief Agent's incoming data functionality.
   */
  eprosima::uxr::CustomAgent::RecvMsgFunction recv_msg_function =
      [&](eprosima::uxr::CustomEndPoint* source_endpoint, uint8_t* buffer, size_t buffer_length, int timeout,
          eprosima::uxr::TransportRc& transport_rc) -> ssize_t {
    ssize_t bytes_received = -1;
    int poll_rv = poll(&poll_fd, 1, timeout);

    if (0 < poll_rv)
    {
      bytes_received = read(poll_fd.fd, buffer, buffer_length);
      transport_rc = (-1 != bytes_received) ? eprosima::uxr::TransportRc::ok : eprosima::uxr::TransportRc::server_error;
    }
    else
    {
      transport_rc =
          (0 == poll_rv) ? eprosima::uxr::TransportRc::timeout_error : eprosima::uxr::TransportRc::server_error;
      bytes_received = 0;
    }

    if (eprosima::uxr::TransportRc::ok == transport_rc)
    {
      UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent RECV_MSG function"),
                         "Serial port: ", device_name);
    }

    return bytes_received;
  };

  /**
   * @brief Agent's outgoing data flow definition.
   */
  eprosima::uxr::CustomAgent::SendMsgFunction send_msg_function =
      [&](const eprosima::uxr::CustomEndPoint* destination_endpoint, uint8_t* buffer, size_t message_length,
          eprosima::uxr::TransportRc& transport_rc) -> ssize_t {
    ssize_t bytes_sent = write(poll_fd.fd, buffer, message_length);
    transport_rc = (-1 != bytes_sent) ? eprosima::uxr::TransportRc::ok : eprosima::uxr::TransportRc::server_error;

    if (eprosima::uxr::TransportRc::ok == transport_rc)
    {
      UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent SEND_MSG function"),
                         "Serial port: ", device_name);
    }

    return bytes_sent;
  };

  /**
   * Run the main application.
   */
  try
  {
    /**
     * EndPoint definition for this transport. We define an address and a port.
     */
    eprosima::uxr::CustomEndPoint custom_endpoint;

    /**
     * Create a custom agent instance.
     */
    eprosima::uxr::CustomAgent custom_agent("SERIAL_CUSTOM", &custom_endpoint, mw_kind, false, init_function,
                                            fini_function, send_msg_function, recv_msg_function);

    /**
     * Set verbosity level
     */
    custom_agent.set_verbose_level(6);

    /**
     * Run agent and wait until receiving an stop signal.
     */
    custom_agent.start();

    int n_signal = 0;
    sigset_t signals;
    sigwait(&signals, &n_signal);

    /**
     * Stop agent, and exit.
     */
    custom_agent.stop();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }
}