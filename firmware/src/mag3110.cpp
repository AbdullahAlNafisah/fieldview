#include "mag3110.h"
#include "uart.h"
#include <util/delay.h>

bool mag_wr(SoftI2C*b, uint8_t reg, uint8_t val){
  if(!i2c_start(b)) return false;
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|0)){ i2c_stop(b); return false; }
  if(!i2c_write_byte(b,reg)){ i2c_stop(b); return false; }
  if(!i2c_write_byte(b,val)){ i2c_stop(b); return false; }
  i2c_stop(b); return true;
}

bool mag_rdN(SoftI2C*b, uint8_t reg, uint8_t*buf, uint8_t len){
  if(!i2c_start(b)) return false;
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|0)){ i2c_stop(b); return false; }
  if(!i2c_write_byte(b,reg)){ i2c_stop(b); return false; }
  if(!i2c_restart(b)){ i2c_stop(b); return false; }
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|1)){ i2c_stop(b); return false; }
  for(uint8_t i=0;i<len;i++){
    bool more=(i+1<len);
    if(!i2c_read_byte(b,&buf[i],more)){ i2c_stop(b); return false; }
  }
  i2c_stop(b); return true;
}

bool mag_rd1(SoftI2C*b, uint8_t reg, uint8_t*val){ return mag_rdN(b, reg, val, 1); }

bool mag_standby(SoftI2C*b){ return mag_wr(b, MAG3110_CTRL_REG1, 0x00); }
bool mag_active (SoftI2C*b){ return mag_wr(b, MAG3110_CTRL_REG1, CTRL1_AC_ACTIVE); }
bool mag_cfg2   (SoftI2C*b, uint8_t v){ return mag_wr(b, MAG3110_CTRL_REG2, v); }

bool mag_read_xyz(SoftI2C*b, int16_t *x, int16_t *y, int16_t *z){
  uint8_t d[6]; if(!mag_rdN(b, MAG3110_OUT_X_MSB, d, 6)) return false;
  *x=(int16_t)((d[0]<<8)|d[1]);
  *y=(int16_t)((d[2]<<8)|d[3]);
  *z=(int16_t)((d[4]<<8)|d[5]);
  return true;
}

bool mag_init_and_status(SoftI2C*b){
  uint8_t who=0;
  if(!mag_rd1(b, MAG3110_WHO_AM_I, &who)){ uart0_print(b->name); uart0_println(": WHO_AM_I read failed"); return false; }
  uart0_print(b->name); uart0_print(": WHO_AM_I=0x"); uart0_print_hex8(who); uart0_putc('\r'); uart0_putc('\n');
  if(who!=0xC4){ uart0_print(b->name); uart0_println(": unexpected WHO_AM_I (want 0xC4)"); return false; }

  if(!mag_standby(b)) { uart0_print(b->name); uart0_println(": standby FAIL"); return false; }
  if(!mag_cfg2(b, CTRL2_AUTO_MRST_EN /*| CTRL2_RAW*/ )){ uart0_print(b->name); uart0_println(": CTRL2 write FAIL"); return false; }
  if(!mag_active(b)) { uart0_print(b->name); uart0_println(": active FAIL"); return false; }

  uint8_t sys=0,dr=0,c1=0,c2=0,tmp=0;
  (void)mag_rd1(b, MAG3110_SYSMOD,    &sys);
  (void)mag_rd1(b, MAG3110_DR_STATUS, &dr);
  (void)mag_rd1(b, MAG3110_CTRL_REG1, &c1);
  (void)mag_rd1(b, MAG3110_CTRL_REG2, &c2); // write-only bits read 0
  (void)mag_rd1(b, MAG3110_DIE_TEMP,  &tmp);

  uart0_print(b->name); uart0_print(": SYSMOD="); uart0_print_int(sys);
  uart0_print(" DR_STATUS=0x"); uart0_print_hex8(dr);
  uart0_print(" CTRL1=0x"); uart0_print_hex8(c1);
  uart0_print(" CTRL2=0x"); uart0_print_hex8(c2);
  uart0_print(" DIE_TEMP="); uart0_print_int((int8_t)tmp); uart0_println(" C");
  return true;
}
