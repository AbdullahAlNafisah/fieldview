#include "uart.h"
#include <avr/io.h>

void uart0_init(uint32_t baud){
  UCSR0A = _BV(U2X0);                       // 1) enable double speed
  uint16_t ubrr = (uint16_t)((F_CPU / (8UL * baud)) - 1UL + 0.5);  // 2) compute for U2X
  UBRR0H = (uint8_t)(ubrr >> 8);            // 3) load UBRR
  UBRR0L = (uint8_t)(ubrr & 0xFF);
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);         // RX+TX
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);       // 8N1
  while (UCSR0A & _BV(RXC0)) (void)UDR0;    // flush junk
}


void uart0_putc(char c){ while(!(UCSR0A&(1<<UDRE0))){} UDR0=c; }

void uart0_print(const char*s){ while(*s) uart0_putc(*s++); }

void uart0_println(const char*s){ uart0_print(s); uart0_putc('\r'); uart0_putc('\n'); }

void uart0_print_int(int32_t v){
  char b[16]; bool neg=(v<0); uint32_t n=neg?(uint32_t)(-v):(uint32_t)v; int i=0;
  do{ b[i++] = (char)('0'+(n%10)); n/=10; }while(n && i<(int)sizeof(b)-1);
  if(neg && i<(int)sizeof(b)) b[i++]='-';
  while(i) uart0_putc(b[--i]);
}

void uart0_print_hex8(uint8_t x){
  const char*h="0123456789ABCDEF"; uart0_putc(h[(x>>4)&0x0F]); uart0_putc(h[x&0x0F]);
}

int uart0_available(void){
  return (UCSR0A & (1<<RXC0)) ? 1 : 0;  // data in RX buffer?
}

char uart0_getc(void){
  return UDR0;  // caller should check uart0_available() first
}