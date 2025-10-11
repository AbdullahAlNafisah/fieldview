#include "softi2c.h"
#include "uart.h"
#include <util/delay.h>

/* timing helpers */
static inline void dly_high(){ _delay_us(T_HIGH_US); }
static inline void dly_low (){ _delay_us(T_LOW_US); }
static inline void dly_rise(){ _delay_us(T_RISE_US); }

/* open-drain helpers */
static inline void sda_release(SoftI2C*b){ *(b->sda_ddr) &= ~(1<<b->sda_bit); }
static inline void sda_low    (SoftI2C*b){ *(b->sda_port)&= ~(1<<b->sda_bit); *(b->sda_ddr)|=(1<<b->sda_bit); }
static inline void scl_release(SoftI2C*b){ *(b->scl_ddr) &= ~(1<<b->scl_bit); }
static inline void scl_low    (SoftI2C*b){ *(b->scl_port)&= ~(1<<b->scl_bit); *(b->scl_ddr)|=(1<<b->scl_bit); }
static inline uint8_t sda_read(SoftI2C*b){ return ((*(b->sda_pin)) & (1<<b->sda_bit)) != 0; }
static inline uint8_t scl_read(SoftI2C*b){ return ((*(b->scl_pin)) & (1<<b->scl_bit)) != 0; }

static bool scl_wait_high(SoftI2C*b){
  uint16_t g=STRETCH_TIMEOUT;
  while(!scl_read(b)){ if(--g==0) return false; }
  return true;
}

/* core */
void i2c_init(SoftI2C*b){
  *(b->sda_port)&=~(1<<b->sda_bit);
  *(b->scl_port)&=~(1<<b->scl_bit);
  sda_release(b); scl_release(b); _delay_us(10);
}

bool i2c_start(SoftI2C*b){
  sda_release(b); scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  sda_low(b); dly_low(); scl_low(b); dly_low(); return true;
}

bool i2c_restart(SoftI2C*b){
  sda_release(b); scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  sda_low(b); dly_low(); scl_low(b); dly_low(); return true;
}

void i2c_stop(SoftI2C*b){
  sda_low(b); dly_low(); scl_release(b); dly_rise(); (void)scl_wait_high(b);
  sda_release(b); dly_rise();
}

static bool i2c_write_bit(SoftI2C*b, uint8_t v){
  if(v) sda_release(b); else sda_low(b);
  scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  dly_high(); scl_low(b); dly_low(); return true;
}

static bool i2c_read_bit(SoftI2C*b, uint8_t*bit){
  sda_release(b); scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  dly_high(); *bit = sda_read(b); scl_low(b); dly_low(); return true;
}

bool i2c_write_byte(SoftI2C*b, uint8_t x){
  for(uint8_t m=0x80;m;m>>=1) if(!i2c_write_bit(b, (x&m)!=0)) return false;
  uint8_t ack; if(!i2c_read_bit(b,&ack)) return false; return (ack==0);
}

bool i2c_read_byte(SoftI2C*b, uint8_t*x, bool ack){
  uint8_t bit,v=0; for(uint8_t i=0;i<8;i++){ if(!i2c_read_bit(b,&bit)) return false; v=(uint8_t)((v<<1)|(bit?1:0)); }
  *x=v; return i2c_write_bit(b, ack?0:1);
}

/* debug helpers */
void i2c_bus_recover(SoftI2C*b){
  sda_release(b);
  for(uint8_t i=0;i<9;i++){ scl_release(b); dly_rise(); dly_high(); scl_low(b); dly_low(); }
  i2c_stop(b);
}

bool i2c_line_selftest(SoftI2C*b){
  bool ok=true; uart0_print(b->name); uart0_print(": line test: ");
  sda_release(b); scl_release(b); _delay_ms(1);
  if(!sda_read(b)){ uart0_print("[SDA LOW] "); ok=false; }
  if(!scl_read(b)){ uart0_print("[SCL LOW] "); ok=false; }
  sda_low(b); scl_low(b); _delay_ms(1);
  if( sda_read(b)){ uart0_print("[SDA not pulling LOW] "); ok=false; }
  if( scl_read(b)){ uart0_print("[SCL not pulling LOW] "); ok=false; }
  sda_release(b); scl_release(b); _delay_ms(1);
  uart0_print(ok? "OK":"FAIL"); uart0_putc('\r'); uart0_putc('\n');
  return ok;
}

bool i2c_scan_bus(SoftI2C*b, uint8_t target){
  bool any=false, hit=false; uart0_print(b->name); uart0_print(": scan:");
  for(uint8_t a=1;a<127;a++){
    if(!i2c_start(b)){ uart0_print(" [startERR]"); break; }
    bool ack = i2c_write_byte(b, (uint8_t)((a<<1)|0)); i2c_stop(b);
    if(ack){ any=true; uart0_putc(' '); uart0_print_hex8(a); if(a==target) hit=true; }
  }
  if(!any) uart0_print(" none");
  uart0_putc('\r'); uart0_putc('\n');
  return hit;
}
