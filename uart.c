#include "uart.h"

void uart_init(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud-1;
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void uart_transmit(unsigned char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print(const char *str) {
    while (*str) {
        uart_transmit(*str++);
    }
}

void uart_println(const char *str) {
    uart_print(str);
    uart_transmit('\r');
    uart_transmit('\n');
}