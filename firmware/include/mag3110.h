#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "softi2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAG3110_ADDR 0x0E
#define MAG3110_DR_STATUS 0x00
#define MAG3110_OUT_X_MSB 0x01
#define MAG3110_WHO_AM_I 0x07
#define MAG3110_SYSMOD 0x08
#define MAG3110_CTRL_REG1 0x10
#define MAG3110_CTRL_REG2 0x11
#define MAG3110_DIE_TEMP 0x0F

#define CTRL1_AC_ACTIVE 0x01
#define CTRL2_AUTO_MRST_EN 0x80
#define CTRL2_RAW 0x20
#define CTRL2_MAG_RST 0x10

bool mag_rdN(SoftI2C *b, uint8_t reg, uint8_t *buf, uint8_t len);
bool mag_rd1(SoftI2C *b, uint8_t reg, uint8_t *val);
bool mag_wr(SoftI2C *b, uint8_t reg, uint8_t val);

bool mag_standby(SoftI2C *b);
bool mag_active(SoftI2C *b);
bool mag_cfg2(SoftI2C *b, uint8_t v);

bool mag_read_xyz(SoftI2C *b, int16_t *x, int16_t *y, int16_t *z);
bool mag_init_and_status(SoftI2C *b);

#ifdef __cplusplus
}
#endif
