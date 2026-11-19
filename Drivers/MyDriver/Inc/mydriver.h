#ifndef MY_DRIVER_H
#define MY_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx_hal.h"

typedef enum
{
    GREEN,
    RED,
    YELLOW
} Lights;

void turn_on_light(Lights color);
void turn_off_light(Lights color);

int check_button(uint32_t now);

typedef enum
{
    INTR,
    POLL
} Modes;

/* First call initializes the register driver; subsequent calls switch modes.
 * Call from main only. RX bytes are preserved across mode changes.
 * HAL_StatusTypeDef is retained solely for compatibility with the console.
 * Queue sizes are MY_UART_RX_QUEUE_SIZE / MY_UART_TX_QUEUE_SIZE in my_uart.h.
 */
HAL_StatusTypeDef uart_receive_init(Modes mode);

/* mode must match uart_receive_init. HAL_BUSY means no received byte is available. */
HAL_StatusTypeDef receive_char(Modes mode, char *recieved_char);
typedef struct
{
    size_t index;
    int overflow;
} LineReceiver;

typedef enum
{
    LINE_PENDING,
    LINE_READY,
    LINE_OVERFLOW,
    LINE_ERROR
} LineStatus;

LineStatus receive_str(Modes mode, LineReceiver *rx, char *buf, size_t size);
/* Feed an already received byte (for callers implementing echo). */
LineStatus receive_line_char(LineReceiver *rx, char *buf, size_t size, char ch);
/* INTR copies the complete message into the queue, or returns HAL_BUSY without
 * accepting any bytes. Software queue capacity is MY_UART_TX_QUEUE_SIZE - 1.
 * HAL_OK means accepted, not physically transmitted. POLL waits for TX.
 * mode must match uart_receive_init. Call these functions from main only.
 */
HAL_StatusTypeDef transmit_char(Modes mode, char ch);
HAL_StatusTypeDef transmit_str(Modes mode, const char *ptr, size_t size);
/* Compatibility no-op: the register driver starts transmission itself. */
HAL_StatusTypeDef uart_transmit_init(void);

#endif
