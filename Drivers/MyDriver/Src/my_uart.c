
#include "../Inc/my_uart.h"
#include "../Inc/byte_queue.h"
#include <stddef.h>

#define MY_UART_RX_ERRORS                                                      \
  (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)

static uint8_t rx_storage[MY_UART_RX_QUEUE_SIZE];
static uint8_t tx_storage[MY_UART_TX_QUEUE_SIZE];

static ByteQueue rx_queue = {rx_storage, sizeof(rx_storage), 0U, 0U};

static ByteQueue tx_queue = {tx_storage, sizeof(tx_storage), 0U, 0U};

static volatile MyUART_Mode uart_mode = MY_UART_POLL;
static volatile uint8_t uart_initialized = 0U;
static volatile uint8_t rx_error = 0U;

MyUART_Status MX_MyUSART6_Init(MyUART_Mode mode) {
  MyUART_Config config = {.Instance = USART6,
                          .BaudRate = 57600,
                          .WordLength = UART_WORDLENGTH_8B,
                          .StopBits = UART_STOPBITS_1,
                          .Parity = UART_PARITY_NONE,
                          .HwFlowCtl = UART_HWCONTROL_NONE,
                          .OverSampling = UART_OVERSAMPLING_16,
                          .Mode = mode};

  return my_uart_init(&config);
}

MyUART_Status my_uart_init(const MyUART_Config *config) {
  if (config == NULL || config->Instance != USART6 || config->BaudRate == 0U) {
    return MY_UART_ERROR;
  }

  if (config->Mode != MY_UART_POLL && config->Mode != MY_UART_INTERRUPT) {
    return MY_UART_ERROR;
  }

  if (uart_initialized) {
    return MY_UART_BUSY;
  }

  if (config->WordLength != UART_WORDLENGTH_8B ||
      config->Parity != UART_PARITY_NONE ||
      (config->StopBits != UART_STOPBITS_1 &&
       config->StopBits != UART_STOPBITS_2) ||
      config->HwFlowCtl != UART_HWCONTROL_NONE ||
      (config->OverSampling != UART_OVERSAMPLING_8 &&
       config->OverSampling != UART_OVERSAMPLING_16)) {
    return MY_UART_ERROR;
  }

  // Определяем частоту APB2
  uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13U;
  uint32_t apb2_div = (ppre2 < 4U) ? 1U : (1U << (ppre2 - 3U));

  uint32_t pclk = SystemCoreClock / apb2_div;

  // Вычисляем значение регистра BRR
  uint64_t divider =
      ((uint64_t)pclk + config->BaudRate / 2U) / config->BaudRate;

  uint32_t brr;

  if (config->OverSampling == UART_OVERSAMPLING_16) {
    if (divider < 16U || divider > 0xFFFFU) {
      return MY_UART_ERROR;
    }

    brr = (uint32_t)divider;
  } else {
    if (divider < 8U || divider > 0x7FFFU) {
      return MY_UART_ERROR;
    }

    brr = (uint32_t)(((divider & ~0x7ULL) << 1U) | (divider & 0x7ULL));
  }

  USART_TypeDef *uart = config->Instance;

  // Включаем тактирование GPIOC и USART6
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

  (void)RCC->AHB1ENR;
  (void)RCC->APB2ENR;

  // Проверяем, не выполняется ли передача
  if ((uart->CR1 & USART_CR1_UE) && !(uart->SR & USART_SR_TC)) {
    return MY_UART_BUSY;
  }

  // Отключаем прерывания и USART
  NVIC_DisableIRQ(USART6_IRQn);
  uart->CR1 &= ~USART_CR1_UE;

  // PC6 - TX, PC7 - RX
  GPIOC->MODER &= ~((3U << 12) | (3U << 14));
  GPIOC->MODER |= (2U << 12) | (2U << 14);

  // Push-pull
  GPIOC->OTYPER &= ~((1U << 6) | (1U << 7));

  // Very high speed
  GPIOC->OSPEEDR |= (3U << 12) | (3U << 14);

  // Без подтяжек
  GPIOC->PUPDR &= ~((3U << 12) | (3U << 14));

  // AF8 для USART6
  GPIOC->AFR[0] &= ~((0xFU << 24) | (0xFU << 28));
  GPIOC->AFR[0] |= (8U << 24) | (8U << 28);

  // Сбрасываем управляющие регистры
  uart->CR1 = 0U;
  uart->CR2 = 0U;
  uart->CR3 = 0U;

  // Настройка длины слова, четности и oversampling
  uart->CR1 |= config->WordLength;
  uart->CR1 |= config->Parity;
  uart->CR1 |= config->OverSampling;

  // Настройка стоповых битов
  uart->CR2 |= config->StopBits;

  // Настройка скорости передачи
  uart->BRR = brr;

  // Сбрасываем старые флаги приема и ошибок
  (void)uart->SR;
  (void)uart->DR;

  // Разрешаем передачу и прием
  uart->CR1 |= USART_CR1_TE | USART_CR1_RE;

  // Настройка режима работы
  if (config->Mode == MY_UART_INTERRUPT) {
    uart->CR1 |= USART_CR1_RXNEIE;
  }

  // Подготавливаем программные очереди
  byte_queue_clear(&rx_queue);
  byte_queue_clear(&tx_queue);

  rx_error = 0U;
  uart_mode = config->Mode;
  uart_initialized = 1U;

  // Включаем USART
  uart->CR1 |= USART_CR1_UE;

  NVIC_ClearPendingIRQ(USART6_IRQn);

  if (config->Mode == MY_UART_INTERRUPT) {
    NVIC_SetPriority(USART6_IRQn, 0U);
    NVIC_EnableIRQ(USART6_IRQn);
  }

  return MY_UART_OK;
}

MyUART_Status my_uart_receive(uint8_t *byte) {
  if (!uart_initialized || byte == NULL) {
    return MY_UART_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if (rx_error) {
    byte_queue_clear(&rx_queue);
    rx_error = 0U;

    __set_PRIMASK(primask);
    return MY_UART_ERROR;
  }

  int has_data = byte_queue_pop(&rx_queue, byte);

  __set_PRIMASK(primask);

  if (has_data) {
    return MY_UART_OK;
  }

  if (uart_mode == MY_UART_INTERRUPT) {
    return MY_UART_EMPTY;
  }

  // Polling: проверяем наличие данных
  uint32_t sr = USART6->SR;

  if (sr & MY_UART_RX_ERRORS) {
    (void)USART6->DR;
    return MY_UART_ERROR;
  }

  if (!(sr & USART_SR_RXNE)) {
    return MY_UART_EMPTY;
  }

  *byte = (uint8_t)(USART6->DR & 0xFFU);

  return MY_UART_OK;
}

MyUART_Status my_uart_transmit(const uint8_t *data, size_t size) {
  if (!uart_initialized || data == NULL) {
    return MY_UART_ERROR;
  }

  if (size == 0U) {
    return MY_UART_OK;
  }

  if (uart_mode == MY_UART_POLL) {
    for (size_t i = 0; i < size; i++) {
      while (!(USART6->SR & USART_SR_TXE)) {
      }

      USART6->DR = data[i];
    }

    while (!(USART6->SR & USART_SR_TC)) {
    }

    return MY_UART_OK;
  }

  // Interrupt: проверяем свободное место в очереди
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  size_t used = (tx_queue.head + tx_queue.size - tx_queue.tail) % tx_queue.size;

  size_t available = tx_queue.size - 1U - used;

  if (size > available) {
    __set_PRIMASK(primask);
    return MY_UART_FULL;
  }

  // Помещаем данные в очередь
  for (size_t i = 0; i < size; i++) {
    (void)byte_queue_push(&tx_queue, data[i]);
  }

  // Разрешаем прерывание передачи
  USART6->CR1 |= USART_CR1_TXEIE;

  __set_PRIMASK(primask);

  return MY_UART_OK;
}

MyUART_Status my_uart_rx_status(void) {
  if (!uart_initialized) {
    return MY_UART_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t error = rx_error;
  int has_data = (rx_queue.head != rx_queue.tail);
  MyUART_Mode mode = uart_mode;

  __set_PRIMASK(primask);

  if (error) {
    return MY_UART_ERROR;
  }

  if (has_data) {
    return MY_UART_OK;
  }

  if (mode == MY_UART_INTERRUPT) {
    return MY_UART_EMPTY;
  }

  uint32_t sr = USART6->SR;

  if (sr & MY_UART_RX_ERRORS) {
    return MY_UART_ERROR;
  }

  return (sr & USART_SR_RXNE) ? MY_UART_OK : MY_UART_EMPTY;
}

MyUART_Status my_uart_tx_status(void) {
  if (!uart_initialized) {
    return MY_UART_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Проверяем, остались ли данные в очереди
  int has_data = (tx_queue.head != tx_queue.tail);

  // Проверяем полное завершение передачи
  int complete = ((USART6->SR & USART_SR_TC) != 0U);

  // Проверяем активность прерывания передачи
  int tx_interrupt = ((USART6->CR1 & USART_CR1_TXEIE) != 0U);

  __set_PRIMASK(primask);

  if (has_data || !complete || tx_interrupt) {
    return MY_UART_BUSY;
  }

  return MY_UART_OK;
}

void my_uart_irq_handler(void) {
  if (!uart_initialized || uart_mode != MY_UART_INTERRUPT) {
    return;
  }

  uint32_t sr = USART6->SR;

  // Обработка приема данных
  if (sr & (USART_SR_RXNE | MY_UART_RX_ERRORS)) {
    uint8_t byte = (uint8_t)(USART6->DR & 0xFFU);

    if (sr & MY_UART_RX_ERRORS) {
      rx_error = 1U;
    } else if (!rx_error && (sr & USART_SR_RXNE)) {
      if (!byte_queue_push(&rx_queue, byte)) {
        rx_error = 1U;
      }
    }
  }

  // Обработка передачи данных
  if ((USART6->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {

    uint8_t byte;

    if (byte_queue_pop(&tx_queue, &byte)) {
      USART6->DR = byte;
    } else {
      USART6->CR1 &= ~USART_CR1_TXEIE;
    }
  }
}

MyUART_Status my_uart_set_mode(MyUART_Mode mode) {
  if (mode != MY_UART_POLL && mode != MY_UART_INTERRUPT) {
    return MY_UART_ERROR;
  }

  if (!uart_initialized) {
    return MY_UART_ERROR;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // Если режим уже установлен, ничего не меняем
  if (uart_mode == mode) {
    __set_PRIMASK(primask);
    return MY_UART_OK;
  }

  // Проверяем, завершена ли передача
  if (tx_queue.head != tx_queue.tail || (USART6->CR1 & USART_CR1_TXEIE) ||
      !(USART6->SR & USART_SR_TC)) {

    __set_PRIMASK(primask);
    return MY_UART_BUSY;
  }

  // Отключаем прерывания USART6
  NVIC_DisableIRQ(USART6_IRQn);
  USART6->CR1 &= ~USART_CR1_RXNEIE;

  // Проверяем, остался ли непрочитанный байт
  uint32_t sr = USART6->SR;

  if (sr & (USART_SR_RXNE | MY_UART_RX_ERRORS)) {
    uint8_t byte = (uint8_t)(USART6->DR & 0xFFU);

    if (sr & MY_UART_RX_ERRORS) {
      rx_error = 1U;
    } else if (!rx_error && (sr & USART_SR_RXNE)) {
      if (!byte_queue_push(&rx_queue, byte)) {
        rx_error = 1U;
      }
    }
  }

  // Сохраняем новый режим
  uart_mode = mode;

  NVIC_ClearPendingIRQ(USART6_IRQn);

  if (mode == MY_UART_INTERRUPT) {
    USART6->CR1 |= USART_CR1_RXNEIE;

    NVIC_SetPriority(USART6_IRQn, 0U);
    NVIC_EnableIRQ(USART6_IRQn);
  }

  __set_PRIMASK(primask);

  return MY_UART_OK;
}
