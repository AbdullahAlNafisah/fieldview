#pragma once
#include <stdint.h>
#include <avr/io.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  volatile uint8_t *sda_port, *sda_ddr, *sda_pin; uint8_t sda_bit;
  volatile uint8_t *scl_port, *scl_ddr, *scl_pin; uint8_t scl_bit;
  const char *name;
} SoftI2C;

/* User-tunable timing (~50–80 kHz default) */
#ifndef T_RISE_US
#define T_RISE_US   1
#endif
#ifndef T_HIGH_US
#define T_HIGH_US   6
#endif
#ifndef T_LOW_US
#define T_LOW_US    8
#endif
#ifndef STRETCH_TIMEOUT
#define STRETCH_TIMEOUT  8000
#endif
// #ifndef T_RISE_US
// #define T_RISE_US   0   // was 1
// #endif
// #ifndef T_HIGH_US
// #define T_HIGH_US   2   // was 6
// #endif
// #ifndef T_LOW_US
// #define T_LOW_US    3   // was 8
// #endif
// #ifndef STRETCH_TIMEOUT
// #define STRETCH_TIMEOUT  1500  // was 8000 (shorter = fail fast)
// #endif


/* Core primitives */
void i2c_init(SoftI2C *b);
bool i2c_start(SoftI2C *b);
bool i2c_restart(SoftI2C *b);
void i2c_stop(SoftI2C *b);
bool i2c_write_byte(SoftI2C *b, uint8_t x);
bool i2c_read_byte(SoftI2C *b, uint8_t *x, bool ack);

/* Debug helpers */
bool i2c_line_selftest(SoftI2C *b);
void i2c_bus_recover(SoftI2C *b);
bool i2c_scan_bus(SoftI2C *b, uint8_t target);

#ifdef __cplusplus
}
#endif
