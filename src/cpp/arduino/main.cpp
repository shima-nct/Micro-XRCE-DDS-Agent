#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <netdb.h>

#include <termios.h>
#include <rmw/init.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/custom_transport.h>

#include <functional>
#include <errno.h>
#include <iostream>
#include "Arduino.h"
#include "LoRa_E220.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            return 1;                                                                    \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// --- micro-ROS Transports ---

typedef struct
{
    struct pollfd poll_fd;
} custom_transport_data_t;

std::string device = "/dev/ttyUSB0"; 

speed_t baudrate = B9600; // デフォルトボーレート
typedef struct
{
    byte ADDH;
    byte ADDL;
    byte CHAN;
} lora_e200_addr_ch_t;
lora_e200_addr_ch_t lora_e200_addr_ch;

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

LoRa_E220 e220ttl(&Serial1);

bool custom_transport_open(struct uxrCustomTransport *transport)
{
    custom_transport_data_t *transport_data = (custom_transport_data_t *)transport->args;
    Serial1.open(device.c_str(), baudrate);
    e220ttl.begin();
    transport_data->poll_fd.fd = Serial1.getFd();
    transport_data->poll_fd.events = POLLIN;
    tcflush(transport_data->poll_fd.fd, TCIOFLUSH);

    printf("running... fd: %d\n",
           transport_data->poll_fd.fd);

    return true;
}

bool custom_transport_close(struct uxrCustomTransport *transport)
{
    custom_transport_data_t *transport_data = (custom_transport_data_t *)transport->args;
    if (-1 == transport_data->poll_fd.fd)
    {
        return true;
    }
    Serial1.end();
    transport_data->poll_fd.fd = Serial1.getFd();
    return transport_data->poll_fd.fd < 0;
}

size_t custom_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
    custom_transport_data_t *transport_data = (custom_transport_data_t *)transport->args;
    size_t rv = 0;
    ResponseStatus rs = e220ttl.sendFixedMessage(lora_e200_addr_ch.ADDH, lora_e200_addr_ch.ADDL, lora_e200_addr_ch.CHAN, (void *)buf, len);

    if (E220_SUCCESS == rs.code)
    {
        rv = len;
        *errcode = 0;
        std::cout << "Sent: " << rs.getResponseDescription() << std::endl;
        printBuffer(buf, rv);
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

size_t custom_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    custom_transport_data_t *transport_data = (custom_transport_data_t *)transport->args;
    size_t rv = 0;
    int poll_rv = poll(&transport_data->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        // ssize_t bytes_received = read(transport_data->poll_fd.fd, (void *)buf, len);
        if (e220ttl.available() > 0)
        {
            ResponseContainer rsc = e220ttl.receiveMessage();
            if (E220_SUCCESS == rsc.status.code && (rv = rsc.data.length() < len))
            {
                rsc.data.getBytes(buf, rv);
                *errcode = 0;
                printf("Received: ");
                printBuffer(buf, rv);
            }
            else
            {
                *errcode = 1;
            }
        }
    }
    else
    {
        *errcode = (0 == poll_rv) ? 0 : 1;
    }
    return rv;
}

// --- micro-ROS App ---

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        printf("Sent: %d\n", msg.data);
        msg.data++;
    }
}

int main(int argc, char *const argv[])
{
    int opt;

    while ((opt = getopt(argc, argv, "D:b:")) != -1)
    {
        switch (opt)
        {
        case 'D':
            device = optarg;
            break;
        case 'b':
            baudrate = atoi(optarg);
            switch (baudrate)
            {
            case 1200:
                baudrate = B1200;
                break;
            case 4800:
                baudrate = B4800;
                break;
            case 9600:
                baudrate = B9600;
                break;
            case 19200:
                baudrate = B19200;
                break;
            case 38400:
                baudrate = B38400;
                break;
            case 57600:
                baudrate = B57600;
                break;
            case 115200:
                baudrate = B115200;
                break;

            default:
                fprintf(stderr, "Unsupported baud rate: %d\n", baudrate);
                exit(EXIT_FAILURE);
            }
            break;
        default:
            fprintf(stderr, "Usage: %s [-D device] [-b baudrate]\n", argv[0]);
            exit(EXIT_FAILURE);
        }
    }

    custom_transport_data_t custom_transport_data;
    uint16_t addr = 18030;
    uint8_t addrHigh = (addr >> 8) & 0xFF; // 上位バイトを取得
    uint8_t addrLow = addr & 0xFF;         // 下位バイトを取得
    lora_e200_addr_ch.ADDH = addrHigh;
    lora_e200_addr_ch.ADDL = addrLow;
    lora_e200_addr_ch.CHAN = 28;

    RCCHECK(rmw_uros_set_custom_transport(
        true,
        (void *)&custom_transport_data,
        custom_transport_open,
        custom_transport_close,
        custom_transport_write,
        custom_transport_read))

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABA, rmw_options))

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "custom_transport_node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "std_msgs_msg_Int32"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg.data = 0;

    rclc_executor_spin(&executor);

    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    return 0;
}
