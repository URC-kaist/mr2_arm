/**
 * Simple retargeting of printf/scanf to USART2.
 *
 * CubeMX generates _write/_read stubs in syscalls.c that call __io_putchar
 * / __io_getchar. By implementing those here and linking this file, all
 * printf output is transmitted over USART2 and getchar/scanf will block on it.
 */
#include "main.h"

extern UART_HandleTypeDef huart2;

int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart2, &c, 1, HAL_MAX_DELAY);
  return ch;
}

int __io_getchar(void) {
  uint8_t c;
  HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY);
  return (int)c;
}
