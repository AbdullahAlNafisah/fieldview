#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart0_init(uint32_t baud);
void uart0_putc(char c);
void uart0_print(const char *s);
void uart0_println(const char *s);
void uart0_print_int(int32_t v);
void uart0_print_hex8(uint8_t x);

#ifdef __cplusplus
}
#endif

int  uart0_available(void);   // >0 if a byte is waiting
char uart0_getc(void);        // read one byte (non-blocking if you checked available)
