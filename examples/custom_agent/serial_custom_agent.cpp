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
#include <uxr/agent/utils/ArgumentParser.hpp>
#include <uxr/agent/transport/serial/TermiosAgentLinux.hpp>

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

int main(int argc, char **argv)
{
  eprosima::uxr::agent::parser::ArgumentParser<eprosima::uxr::TermiosAgent> argparser(argc, argv, eprosima::uxr::agent::TransportKind::SERIAL);
  eprosima::uxr::agent::parser::ParseResult parse_result = argparser.parse_arguments();
  eprosima::uxr::agent::parser::SerialArgs<eprosima::uxr::TermiosAgent> serial_args;
  const char *default_args[] = {"-D", "/dev/ttyS0", "-b", "9600"};
  char **default_argv = const_cast<char **>(default_args);
  int default_argc = 0;
  while (default_args[default_argc] != nullptr)
  {
    ++default_argc;
  }
  serial_args.parse(default_argc, default_argv);
  bool is_serial_args = serial_args.parse(argc, argv);
  termios attrs = argparser.init_termios(serial_args.baud_rate().c_str());

  eprosima::uxr::Middleware::Kind mw_kind(eprosima::uxr::Middleware::Kind::FASTDDS);

  struct pollfd poll_fd;

  /**
   * @brief Agent's initialization behaviour description.
   */
  eprosima::uxr::CustomAgent::InitFunction init_function = [&]() -> bool
  {
    bool rv = false;
    poll_fd.fd = open(serial_args.dev().c_str(), O_RDWR | O_NOCTTY);

    if (0 < poll_fd.fd)
    {
      struct termios new_attrs;
      memset(&new_attrs, 0, sizeof(new_attrs));
      if (0 == tcgetattr(poll_fd.fd, &new_attrs))
      {
        new_attrs.c_cflag = attrs.c_cflag;
        new_attrs.c_lflag = attrs.c_lflag;
        new_attrs.c_iflag = attrs.c_iflag;
        new_attrs.c_oflag = attrs.c_oflag;
        // new_attrs.c_cc[VMIN] = attrs.c_cc[VMIN];
        // new_attrs.c_cc[VTIME] = attrs.c_cc[VTIME];
        new_attrs.c_cc[VMIN] = 64;
        new_attrs.c_cc[VTIME] = 5;

#if _HAVE_STRUCT_TERMIOS_C_ISPEED
        cfsetispeed(&new_attrs, attrs.c_ispeed);
#endif
#if _HAVE_STRUCT_TERMIOS_C_OSPEED
        cfsetospeed(&new_attrs, attrs.c_ospeed);
#endif

        if (0 == tcsetattr(poll_fd.fd, TCSANOW, &new_attrs))
        {
          rv = true;
          poll_fd.events = POLLIN;

          tcflush(poll_fd.fd, TCIOFLUSH);

          UXR_AGENT_LOG_INFO(
              UXR_DECORATE_GREEN("running..."),
              "fd: {}",
              poll_fd.fd);
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
          serial_args.dev(), errno,
          (EACCES == errno) ? ". Please re-run with superuser privileges." : "");
    }

    UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent INIT function"),
                       "Serial port: " + serial_args.dev() +
                           " is now open "
                           "with " +
                           serial_args.baud_rate().c_str() +
                           " bps.",
                       NULL);

    return rv;
  };

  /**
   * @brief Agent's destruction actions.
   */
  eprosima::uxr::CustomAgent::FiniFunction fini_function = [&]() -> bool
  {
    if (-1 == poll_fd.fd)
    {
      return true;
    }

    if (0 == close(poll_fd.fd))
    {
      poll_fd.fd = -1;
      UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent FINI function"),
                         "Serial port: ", serial_args.dev());

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
      [&](eprosima::uxr::CustomEndPoint *source_endpoint, uint8_t *buffer, size_t buffer_length, int timeout,
          eprosima::uxr::TransportRc &transport_rc) -> ssize_t
  {
    ssize_t bytes_read = 0;
    int poll_rv = poll(&poll_fd, 1, timeout);
    if (poll_fd.revents & (POLLERR + POLLHUP))
    {
      transport_rc = eprosima::uxr::TransportRc::server_error;
      ;
    }
    else if (0 < poll_rv)
    {
      bytes_read = read(poll_fd.fd, buffer, buffer_length);
      if (0 > bytes_read)
      {
        transport_rc = eprosima::uxr::TransportRc::server_error;
      }
    }
    else
    {
      transport_rc = (poll_rv == 0) ? eprosima::uxr::TransportRc::timeout_error : eprosima::uxr::TransportRc::server_error;
    }

    if (eprosima::uxr::TransportRc::ok == transport_rc)
    {
      UXR_AGENT_LOG_INFO(
          UXR_DECORATE_GREEN(
              "This is an example of a custom Micro XRCE-DDS Agent RECV_MSG function"),
          "Serial port: {}",
          serial_args.dev());
    }

    return bytes_read;
  };

  /**
   * @brief Agent's outgoing data flow definition.
   */
  eprosima::uxr::CustomAgent::SendMsgFunction send_msg_function =
      [&](const eprosima::uxr::CustomEndPoint *destination_endpoint, uint8_t *buffer, size_t message_length,
          eprosima::uxr::TransportRc &transport_rc) -> ssize_t
  {
    ssize_t bytes_sent = write(poll_fd.fd, buffer, message_length);
    transport_rc = (-1 != bytes_sent) ? eprosima::uxr::TransportRc::ok : eprosima::uxr::TransportRc::server_error;

    if (eprosima::uxr::TransportRc::ok == transport_rc)
    {
      UXR_AGENT_LOG_INFO(UXR_DECORATE_GREEN("This is an example of a custom Micro XRCE-DDS Agent SEND_MSG function"),
                         "Serial port: {}", serial_args.dev());
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
  catch (const std::exception &e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }
}