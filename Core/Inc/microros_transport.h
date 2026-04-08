/**
 * microros_transport.h  —  micro-ROS UART transport declarations
 *
 * UART1:  PA9 = TX   PA10 = RX   2 Mbaud
 * Connect to Raspberry Pi / PC running:
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 2000000
 */
#ifndef MICROROS_TRANSPORT_H
#define MICROROS_TRANSPORT_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Called by rmw_uros_set_custom_transport() */
bool transport_open (struct uxrCustomTransport *t);
bool transport_close(struct uxrCustomTransport *t);
size_t transport_write(struct uxrCustomTransport *t,
                       const uint8_t *buf, size_t len, uint8_t *err);
size_t transport_read (struct uxrCustomTransport *t,
                       uint8_t *buf, size_t len,
                       int timeout_ms, uint8_t *err);

#endif /* MICROROS_TRANSPORT_H */
