#ifndef UART_H
#define UART_H

#include <avr/io.h>

void uart_init(unsigned int baud);
void uart_transmit(unsigned char data);
void uart_print(const char *str);
void uart_println(const char *str);

#endif