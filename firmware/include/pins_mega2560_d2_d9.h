#pragma once
#include <avr/io.h>

/* Mega2560 — four software I2C buses on D2..D9
   Bus0: SDA=D2 (PE4),  SCL=D3 (PE5)
   Bus1: SDA=D4 (PG5),  SCL=D5 (PE3)
   Bus2: SDA=D6 (PH3),  SCL=D7 (PH4)
   Bus3: SDA=D8 (PH5),  SCL=D9 (PH6)
*/

#if !defined(__AVR_ATmega2560__)
# error "pins_mega2560_d2_d9.h targets ATmega2560 only"
#endif

// Bus0 D2/D3
#define BUS0_SDA_PORT  &PORTE
#define BUS0_SDA_DDR   &DDRE
#define BUS0_SDA_PIN   &PINE
#define BUS0_SDA_BIT   4
#define BUS0_SCL_PORT  &PORTE
#define BUS0_SCL_DDR   &DDRE
#define BUS0_SCL_PIN   &PINE
#define BUS0_SCL_BIT   5

// Bus1 D4/D5
#define BUS1_SDA_PORT  &PORTG
#define BUS1_SDA_DDR   &DDRG
#define BUS1_SDA_PIN   &PING
#define BUS1_SDA_BIT   5
#define BUS1_SCL_PORT  &PORTE
#define BUS1_SCL_DDR   &DDRE
#define BUS1_SCL_PIN   &PINE
#define BUS1_SCL_BIT   3

// Bus2 D6/D7
#define BUS2_SDA_PORT  &PORTH
#define BUS2_SDA_DDR   &DDRH
#define BUS2_SDA_PIN   &PINH
#define BUS2_SDA_BIT   3
#define BUS2_SCL_PORT  &PORTH
#define BUS2_SCL_DDR   &DDRH
#define BUS2_SCL_PIN   &PINH
#define BUS2_SCL_BIT   4

// Bus3 D8/D9
#define BUS3_SDA_PORT  &PORTH
#define BUS3_SDA_DDR   &DDRH
#define BUS3_SDA_PIN   &PINH
#define BUS3_SDA_BIT   5
#define BUS3_SCL_PORT  &PORTH
#define BUS3_SCL_DDR   &DDRH
#define BUS3_SCL_PIN   &PINH
#define BUS3_SCL_BIT   6
