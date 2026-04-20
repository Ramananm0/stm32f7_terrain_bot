/**
 * microros_transport.c  —  UART transport for micro-ROS
 * USART6  PG14(TX)  PG9(RX)  2 Mbaud  STM32F746G-DISCO
 * CubeMX: enable USART6, TX=PG14, RX=PG9, baud=2000000
 */
#include "microros_transport.h"
#include "main.h"        /* has extern UART_HandleTypeDef huart6 */
#include <uxr/client/transport.h>

extern UART_HandleTypeDef huart6;

bool transport_open(struct uxrCustomTransport *t)
{
    (void)t;
    return true;   /* UART already initialised by MX_USART6_UART_Init() */
}

bool transport_close(struct uxrCustomTransport *t)
{
    (void)t;
    return true;
}

size_t transport_write(struct uxrCustomTransport *t,
                       const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t;
    HAL_StatusTypeDef s = HAL_UART_Transmit(&huart6,
                                            (uint8_t *)buf,
                                            (uint16_t)len, 100);
    *err = (s != HAL_OK) ? 1 : 0;
    return (s == HAL_OK) ? len : 0;
}

size_t transport_read(struct uxrCustomTransport *t,
                      uint8_t *buf, size_t len,
                      int timeout_ms, uint8_t *err)
{
    (void)t;
    HAL_StatusTypeDef s = HAL_UART_Receive(&huart6,
                                           buf, (uint16_t)len,
                                           (uint32_t)timeout_ms);
    *err = (s != HAL_OK && s != HAL_TIMEOUT) ? 1 : 0;
    return (s == HAL_OK) ? len : 0;
}
