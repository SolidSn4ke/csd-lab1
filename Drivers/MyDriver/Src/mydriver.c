#include "../Inc/mydriver.h"
#include "../Inc/my_uart.h"
#include "gpio.h"
#include "my_gpio.h"

#define BUTTON_DEBOUNCE_MS 30U

static uint16_t light_pin(Lights color) {
  switch (color) {
  case GREEN:
    return GPIO_PIN_13;

  case YELLOW:
    return GPIO_PIN_14;

  case RED:
    return GPIO_PIN_15;

  default:
    return 0U;
  }
}

void turn_on_light(Lights color) {
  uint16_t pin = light_pin(color);

  if (pin != 0U) {
    My_GPIO_WritePins(GPIOD, pin, pin);
  }
}

void turn_off_light(Lights color) {
  uint16_t pin = light_pin(color);

  if (pin != 0U) {
    My_GPIO_WritePins(GPIOD, pin, 0U);
  }
}

int check_button(uint32_t now) {
  static int last_raw = 0;
  static int stable_state = 0;
  static uint32_t changed_at = 0U;

  int raw = (My_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET);

  if (raw != last_raw) {
    last_raw = raw;
    changed_at = now;
  }

  if ((uint32_t)(now - changed_at) >= BUTTON_DEBOUNCE_MS) {
    stable_state = last_raw;
  }

  return stable_state;
}

/* Compatibility with the console API. No HAL UART functions are used here.
 * Call these wrappers from main only; the hardware IRQ calls my_uart_irq_handler.
 */
static Modes active_mode = POLL;
static int uart_initialized;

static HAL_StatusTypeDef console_uart_status(MyUART_Status status) {
  switch (status) {
  case MY_UART_OK:
    return HAL_OK;
  case MY_UART_EMPTY:
  case MY_UART_BUSY:
  case MY_UART_FULL:
    return HAL_BUSY;
  default:
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef uart_receive_init(Modes mode) {
  if (mode != INTR && mode != POLL)
    return HAL_ERROR;

  /* The two enum types have opposite numeric ordering. */
  MyUART_Mode requested = mode == INTR ? MY_UART_INTERRUPT : MY_UART_POLL;
  MyUART_Status status = uart_initialized ? my_uart_set_mode(requested)
                                          : MX_MyUSART6_Init(requested);
  if (status == MY_UART_OK) {
    uart_initialized = 1;
    active_mode = mode;
  }
  return console_uart_status(status);
}

HAL_StatusTypeDef uart_transmit_init(void) {
  /* Kept for the existing console: TXEIE is armed by my_uart_transmit itself. */
  return uart_initialized ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef receive_char(Modes mode, char *recieved_char) {
  if (!uart_initialized || mode != active_mode || recieved_char == NULL)
    return HAL_ERROR;

  uint8_t byte;
  MyUART_Status status = my_uart_receive(&byte);
  if (status == MY_UART_OK)
    *recieved_char = (char)byte;
  return console_uart_status(status);
}

LineStatus receive_str(Modes mode, LineReceiver *rx, char *buf, size_t size) {
  if (rx == NULL || buf == NULL || size == 0 || rx->index >= size)
    return LINE_ERROR;

  char ch;
  HAL_StatusTypeDef status = receive_char(mode, &ch);

  if (status == HAL_TIMEOUT || status == HAL_BUSY)
    return LINE_PENDING;

  if (status != HAL_OK) {
    /* A lost byte must not turn a damaged line into a valid command. */
    rx->index = 0;
    rx->overflow = 1;
    buf[0] = '\0';
    return LINE_ERROR;
  }

  return receive_line_char(rx, buf, size, ch);
}

LineStatus receive_line_char(LineReceiver *rx, char *buf, size_t size,
                             char ch) {
  if (rx == NULL || buf == NULL || size == 0 || rx->index >= size)
    return LINE_ERROR;

  if (ch == '\0') {
    rx->overflow = 1;
    return LINE_PENDING;
  }

  if (ch == '\r' || ch == '\n') {
    if (rx->overflow) {
      rx->index = 0;
      rx->overflow = 0;
      buf[0] = '\0';
      return LINE_OVERFLOW;
    }

    if (rx->index == 0)
      return LINE_PENDING;

    buf[rx->index] = '\0';
    rx->index = 0;
    return LINE_READY;
  }

  if (rx->overflow)
    return LINE_PENDING;

  if (ch == '\b' || ch == '\x7f') {
    if (rx->index > 0)
      buf[--rx->index] = '\0';
    return LINE_PENDING;
  }

  if (rx->index >= size - 1) {
    rx->overflow = 1;
    return LINE_PENDING;
  }

  buf[rx->index++] = ch;
  buf[rx->index] = '\0';
  return LINE_PENDING;
}

HAL_StatusTypeDef transmit_char(Modes mode, char ch) {
  return transmit_str(mode, &ch, 1);
}

HAL_StatusTypeDef transmit_str(Modes mode, const char *ptr, size_t size) {
  if (!uart_initialized || mode != active_mode || ptr == NULL || size == 0)
    return HAL_ERROR;

  return console_uart_status(my_uart_transmit((const uint8_t *)ptr, size));
}
