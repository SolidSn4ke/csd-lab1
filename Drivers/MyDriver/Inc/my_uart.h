#ifndef MY_UART_H
#define MY_UART_H

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>

typedef enum { MY_UART_POLL, MY_UART_INTERRUPT } MyUART_Mode;

typedef enum {
  MY_UART_OK,
  MY_UART_EMPTY,
  MY_UART_BUSY,
  MY_UART_FULL,
  MY_UART_ERROR
} MyUART_Status;

#ifndef MY_UART_RX_QUEUE_SIZE
#define MY_UART_RX_QUEUE_SIZE 256U
#endif

#ifndef MY_UART_TX_QUEUE_SIZE
#define MY_UART_TX_QUEUE_SIZE 256U
#endif

#if MY_UART_RX_QUEUE_SIZE < 2
#error "MY_UART_RX_QUEUE_SIZE must be at least 2"
#endif

#if MY_UART_TX_QUEUE_SIZE < 2
#error "MY_UART_TX_QUEUE_SIZE must be at least 2"
#endif

typedef struct {
  USART_TypeDef *Instance;
  uint32_t BaudRate;
  uint32_t WordLength;
  uint32_t StopBits;
  uint32_t Parity;
  uint32_t HwFlowCtl;
  uint32_t OverSampling;
  MyUART_Mode Mode;
} MyUART_Config;

MyUART_Status MX_MyUSART6_Init(MyUART_Mode mode);
MyUART_Status my_uart_init(const MyUART_Config *config);

MyUART_Status my_uart_receive(uint8_t *byte);
MyUART_Status my_uart_transmit(const uint8_t *data, size_t size);

MyUART_Status my_uart_rx_status(void);
MyUART_Status my_uart_tx_status(void);

void my_uart_irq_handler(void);

MyUART_Status my_uart_set_mode(MyUART_Mode mode);

#endif
