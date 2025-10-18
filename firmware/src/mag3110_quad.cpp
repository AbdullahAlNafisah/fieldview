/*  Board: Arduino Mega 2560
 *  Sensors: MAG3110 x4 firmware with direct register R/W over UART "four software I2C buses on D2..D9 (SDA/SCL pairs)"
 *  Datasheet refs: 
 *    WHO_AM_I=0xC4;
 *    CTRL_REG1 (0x10) [DR2..0, OS1..0, FR, TM, AC]; 
 *    CTRL_REG2 (0x11) [AUTO_MRST_EN, RAW, Mag_RST]
 *  On power-up: sensors remain STANDBY. Host can INIT/ACTIVE/READ/STREAM/CAL and do raw RD/WR per register.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* ---------------- UART ---------------- */
static void uart0_init(uint32_t baud){
  UCSR0A = _BV(U2X0);
  uint16_t ubrr = (uint16_t)((F_CPU / (8UL * baud)) - 1UL + 0.5);
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr & 0xFF);
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
  while (UCSR0A & _BV(RXC0)) (void)UDR0;
}
static inline int uart0_available(void){ return (UCSR0A & (1<<RXC0)) ? 1 : 0; }
static inline char uart0_getc(void){ return UDR0; }
static inline void uart0_putc(char c){ while(!(UCSR0A&(1<<UDRE0))){} UDR0=c; }
static void uart_print(const char*s){ while(*s) uart0_putc(*s++); }
static void uart_println(const char*s){ uart_print(s); uart0_putc('\r'); uart0_putc('\n'); }
static void uart_print_int(int32_t v){
  char b[16]; bool neg=(v<0); uint32_t n=neg?(uint32_t)(-v):(uint32_t)v; int i=0;
  do{ b[i++] = (char)('0'+(n%10)); n/=10; }while(n && i<(int)sizeof(b)-1);
  if(neg && i<(int)sizeof(b)) b[i++]='-';
  while(i) uart0_putc(b[--i]);
}
static void uart_hex8(uint8_t x){ const char*h="0123456789ABCDEF"; uart0_putc(h[(x>>4)&0xF]); uart0_putc(h[x&0xF]); }

/* ---------------- Soft I2C (bit-bang) ---------------- */
typedef struct {
  volatile uint8_t* sda_port; volatile uint8_t* sda_ddr; volatile uint8_t* sda_pin; uint8_t sda_bit;
  volatile uint8_t* scl_port; volatile uint8_t* scl_ddr; volatile uint8_t* scl_pin; uint8_t scl_bit;
  const char* name;
} SoftI2C;

/* Pins for Mega2560 D2..D9:
   Bus0: SDA=D2 (PE4),  SCL=D3 (PE5)
   Bus1: SDA=D4 (PG5),  SCL=D5 (PE3)
   Bus2: SDA=D6 (PH3),  SCL=D7 (PH4)
   Bus3: SDA=D8 (PH5),  SCL=D9 (PH6)
*/
#define BUS0_SDA_PORT  &PORTE
#define BUS0_SDA_DDR   &DDRE
#define BUS0_SDA_PIN   &PINE
#define BUS0_SDA_BIT   4
#define BUS0_SCL_PORT  &PORTE
#define BUS0_SCL_DDR   &DDRE
#define BUS0_SCL_PIN   &PINE
#define BUS0_SCL_BIT   5

#define BUS1_SDA_PORT  &PORTG
#define BUS1_SDA_DDR   &DDRG
#define BUS1_SDA_PIN   &PING
#define BUS1_SDA_BIT   5
#define BUS1_SCL_PORT  &PORTE
#define BUS1_SCL_DDR   &DDRE
#define BUS1_SCL_PIN   &PINE
#define BUS1_SCL_BIT   3

#define BUS2_SDA_PORT  &PORTH
#define BUS2_SDA_DDR   &DDRH
#define BUS2_SDA_PIN   &PINH
#define BUS2_SDA_BIT   3
#define BUS2_SCL_PORT  &PORTH
#define BUS2_SCL_DDR   &DDRH
#define BUS2_SCL_PIN   &PINH
#define BUS2_SCL_BIT   4

#define BUS3_SDA_PORT  &PORTH
#define BUS3_SDA_DDR   &DDRH
#define BUS3_SDA_PIN   &PINH
#define BUS3_SDA_BIT   5
#define BUS3_SCL_PORT  &PORTH
#define BUS3_SCL_DDR   &DDRH
#define BUS3_SCL_PIN   &PINH
#define BUS3_SCL_BIT   6

/* I2C helpers */
static inline void sda_release(SoftI2C*b){ *(b->sda_ddr) &= ~(1<<b->sda_bit); } // input = Hi-Z, external pull-up
static inline void scl_release(SoftI2C*b){ *(b->scl_ddr) &= ~(1<<b->scl_bit); }
static inline void sda_low(SoftI2C*b){ *(b->sda_port) &= ~(1<<b->sda_bit); *(b->sda_ddr) |= (1<<b->sda_bit); }
static inline void scl_low(SoftI2C*b){ *(b->scl_port) &= ~(1<<b->scl_bit); *(b->scl_ddr) |= (1<<b->scl_bit); }
static inline uint8_t sda_read(SoftI2C*b){ return ((*(b->sda_pin)) & (1<<b->sda_bit)) ? 1 : 0; }
static inline uint8_t scl_read(SoftI2C*b){ return ((*(b->scl_pin)) & (1<<b->scl_bit)) ? 1 : 0; }
static inline void dly_rise(){ _delay_us(2); }
static inline void dly_high(){ _delay_us(2); }
static inline void dly_low(){ _delay_us(2); }

static void i2c_init(SoftI2C*b){ *(b->sda_port)&=~(1<<b->sda_bit); *(b->scl_port)&=~(1<<b->scl_bit); sda_release(b); scl_release(b); _delay_us(10); }

static bool scl_wait_high(SoftI2C*b){ // simple clock-stretch wait
  for(uint16_t i=0;i<2000;i++){ if(scl_read(b)) return true; _delay_us(1); }
  return false;
}

static bool i2c_start(SoftI2C*b){
  sda_release(b); scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  sda_low(b); dly_low(); scl_low(b); dly_low(); return true;
}
static bool i2c_restart(SoftI2C*b){
  sda_release(b); scl_release(b); dly_rise(); if(!scl_wait_high(b)) return false;
  sda_low(b); dly_low(); scl_low(b); dly_low(); return true;
}
static void i2c_stop(SoftI2C*b){
  sda_low(b); dly_low(); scl_release(b); dly_rise(); (void)scl_wait_high(b); sda_release(b); dly_rise();
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
static bool i2c_write_byte(SoftI2C*b, uint8_t x){
  for(uint8_t m=0x80;m;m>>=1) if(!i2c_write_bit(b, (x&m)!=0)) return false;
  uint8_t ack; if(!i2c_read_bit(b,&ack)) return false; return (ack==0); // ack==0 means ACK
}
static bool i2c_read_byte(SoftI2C*b, uint8_t*x, bool ack){
  uint8_t bit,v=0; for(uint8_t i=0;i<8;i++){ if(!i2c_read_bit(b,&bit)) return false; v=(uint8_t)((v<<1)|(bit?1:0)); }
  *x=v; return i2c_write_bit(b, ack?0:1);
}
static void i2c_bus_recover(SoftI2C*b){
  sda_release(b);
  for(uint8_t i=0;i<9;i++){ scl_release(b); dly_rise(); dly_high(); scl_low(b); dly_low(); }
  i2c_stop(b);
}
static bool i2c_line_selftest(SoftI2C*b){
  bool ok=true; uart_print(b->name); uart_print(": line: ");
  sda_release(b); scl_release(b); _delay_ms(1);
  if(!sda_read(b)){ uart_print("[SDA LOW] "); ok=false; }
  if(!scl_read(b)){ uart_print("[SCL LOW] "); ok=false; }
  sda_low(b); scl_low(b); _delay_ms(1);
  if( sda_read(b)){ uart_print("[SDA not LOW] "); ok=false; }
  if( scl_read(b)){ uart_print("[SCL not LOW] "); ok=false; }
  sda_release(b); scl_release(b); _delay_ms(1);
  uart_println(ok? "OK":"FAIL");
  return ok;
}

/* ---------------- MAG3110 low-level ---------------- */
#define MAG3110_ADDR      0x0E  // I2C 7-bit addr (standard part) :contentReference[oaicite:10]{index=10}
#define REG_DR_STATUS     0x00
#define REG_OUT_X_MSB     0x01
#define REG_OUT_X_LSB     0x02
#define REG_OUT_Y_MSB     0x03
#define REG_OUT_Y_LSB     0x04
#define REG_OUT_Z_MSB     0x05
#define REG_OUT_Z_LSB     0x06
#define REG_WHO_AM_I      0x07 // expects 0xC4 :contentReference[oaicite:11]{index=11}
#define REG_SYSMOD        0x08
#define REG_DIE_TEMP      0x0F
#define REG_CTRL1         0x10 // DR2 DR1 DR0 OS1 OS0 FR TM AC :contentReference[oaicite:12]{index=12}
#define REG_CTRL2         0x11 // AUTO_MRST_EN .. RAW .. Mag_RST :contentReference[oaicite:13]{index=13}

/* CTRL1 bits */
#define CTRL1_AC          0x01 // ACTIVE
#define CTRL1_TM          0x02 // trigger
#define CTRL1_FR          0x04 // fast read (MSB only)
#define CTRL1_OS0         0x08
#define CTRL1_OS1         0x10
#define CTRL1_DR0         0x20
#define CTRL1_DR1         0x40
#define CTRL1_DR2         0x80
/* CTRL2 bits */
#define CTRL2_AUTO_MRST_EN 0x80
#define CTRL2_RAW          0x20
#define CTRL2_MAG_RST      0x10

static bool mag_wr(SoftI2C*b, uint8_t reg, uint8_t val){
  if(!i2c_start(b)) return false;
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|0)) { i2c_stop(b); return false; }
  if(!i2c_write_byte(b,reg)) { i2c_stop(b); return false; }
  if(!i2c_write_byte(b,val)) { i2c_stop(b); return false; }
  i2c_stop(b); return true;
}
static bool mag_rdN(SoftI2C*b, uint8_t reg, uint8_t*buf, uint8_t len){
  if(!i2c_start(b)) return false;
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|0)) { i2c_stop(b); return false; }
  if(!i2c_write_byte(b,reg)) { i2c_stop(b); return false; }
  if(!i2c_restart(b)) { i2c_stop(b); return false; }
  if(!i2c_write_byte(b,(MAG3110_ADDR<<1)|1)) { i2c_stop(b); return false; }
  for(uint8_t i=0;i<len;i++){
    bool more=(i+1<len);
    if(!i2c_read_byte(b,&buf[i],more)){ i2c_stop(b); return false; }
  }
  i2c_stop(b); return true;
}
static bool mag_rd1(SoftI2C*b, uint8_t reg, uint8_t*val){ return mag_rdN(b, reg, val, 1); }

static bool mag_set_standby(SoftI2C*b){ return mag_wr(b, REG_CTRL1, 0x00); } // STANDBY by clearing AC
static bool mag_set_active (SoftI2C*b, uint8_t ctrl1){ return mag_wr(b, REG_CTRL1, (uint8_t)(ctrl1 | CTRL1_AC)); }

static bool mag_xyz(SoftI2C*b, int16_t* x, int16_t* y, int16_t* z){
  uint8_t d[6]; if(!mag_rdN(b, REG_OUT_X_MSB, d, 6)) return false; // preferred burst starting at 0x01 :contentReference[oaicite:14]{index=14}
  *x = (int16_t)((d[0]<<8)|d[1]); *y = (int16_t)((d[2]<<8)|d[3]); *z = (int16_t)((d[4]<<8)|d[5]);
  return true;
}

/* ---------------- App State ---------------- */
static SoftI2C bus[4] = {
  {BUS0_SDA_PORT, BUS0_SDA_DDR, BUS0_SDA_PIN, BUS0_SDA_BIT, BUS0_SCL_PORT, BUS0_SCL_DDR, BUS0_SCL_PIN, BUS0_SCL_BIT, "Bus0"},
  {BUS1_SDA_PORT, BUS1_SDA_DDR, BUS1_SDA_PIN, BUS1_SDA_BIT, BUS1_SCL_PORT, BUS1_SCL_DDR, BUS1_SCL_PIN, BUS1_SCL_BIT, "Bus1"},
  {BUS2_SDA_PORT, BUS2_SDA_DDR, BUS2_SDA_PIN, BUS2_SDA_BIT, BUS2_SCL_PORT, BUS2_SCL_DDR, BUS2_SCL_PIN, BUS2_SCL_BIT, "Bus2"},
  {BUS3_SDA_PORT, BUS3_SDA_DDR, BUS3_SDA_PIN, BUS3_SDA_BIT, BUS3_SCL_PORT, BUS3_SCL_DDR, BUS3_SCL_PIN, BUS3_SCL_BIT, "Bus3"},
};

static bool line_ok[4]  = {false,false,false,false};
static bool has_mag[4]  = {false,false,false,false};
static bool ready  [4]  = {false,false,false,false};
static uint8_t ctrl1_cached[4] = {0x00,0x00,0x00,0x00}; // keep last non-AC config bits

static void print_info_one(int i){
  uint8_t who=0, sys=0, c1=0, c2=0, t=0;
  uart_print("INFO: "); uart_print(bus[i].name); uart_print(": ");
  if(!has_mag[i]){ uart_println("no device"); return; }
  (void)mag_rd1(&bus[i], REG_WHO_AM_I, &who);
  (void)mag_rd1(&bus[i], REG_SYSMOD, &sys);
  (void)mag_rd1(&bus[i], REG_CTRL1,  &c1);
  (void)mag_rd1(&bus[i], REG_CTRL2,  &c2); // write-only bits read 0
  (void)mag_rd1(&bus[i], REG_DIE_TEMP,&t);
  uart_print("WHO=0x"); uart_hex8(who);
  uart_print(" SYSMOD="); uart_print_int(sys);
  uart_print(" CTRL1=0x"); uart_hex8(c1);
  uart_print(" CTRL2=0x"); uart_hex8(c2);
  uart_print(" TEMP=");   uart_print_int((int8_t)t); uart_println(" C");
}

static bool scan_for_mag(SoftI2C*b){
  // ultra-compact address probe
  bool ok=false;
  if(i2c_start(b)){
    ok = i2c_write_byte(b,(MAG3110_ADDR<<1)|0);
    i2c_stop(b);
  }
  return ok;
}

static void banner_and_selftest(){
  uart_println("=== MAG3110 x4 (Mega2560 D2..D9) ===");
  for(int i=0;i<4;i++){ i2c_init(&bus[i]); }
  _delay_ms(5);
  // Line self-test and recover
  for(int i=0;i<4;i++){
    line_ok[i] = i2c_line_selftest(&bus[i]);
    if(!line_ok[i]){ uart_print(bus[i].name); uart_println(": try recover"); i2c_bus_recover(&bus[i]); line_ok[i]=i2c_line_selftest(&bus[i]); }
  }
  // Quick scan
  for(int i=0;i<4;i++){ has_mag[i] = line_ok[i] && scan_for_mag(&bus[i]); }
  // Initial state: keep sensors STANDBY (do not touch CTRL1 unless asked)
  uart_println("READY");
  for(int i=0;i<4;i++) print_info_one(i);
}

/* Line reader */
static bool read_line(char*buf, uint8_t cap){
  static uint8_t idx=0;
  while(uart0_available()){
    char c=uart0_getc();
    if(c=='\r') continue;
    if(c=='\n'){ buf[idx]=0; idx=0; return true; }
    if(idx+1<cap) buf[idx++]=c; else idx=0;
  }
  return false;
}

static int parse_bus_or_all(const char* s){ // returns bitmask for buses
  if(!s || !*s) return 0xF;
  if(!strcmp(s,"ALL")) return 0xF;
  if(s[0]=='B'||s[0]=='b'){ int n=atoi(s+3); if(n>=0&&n<4) return (1<<n); }
  int n=atoi(s); if(n>=0&&n<4) return (1<<n);
  return 0;
}

static uint32_t millis_like(){ // coarse timestamp from _delay_ms; not exact
  // We don't have a SysTick here; host should timestamp. We just increment a static.
  static uint32_t t=0; t+=1; return t;
}

int main(void){
  uart0_init(115200);
  banner_and_selftest();

  bool streaming=false;
  uint8_t stream_mask=0x0F;
  char cmdbuf[64];

  while(1){
    if(read_line(cmdbuf,sizeof(cmdbuf))){
      // tokenize
      char *cmd = strtok(cmdbuf," ");
      if(!cmd) continue;
      for(char*p=cmd;*p;++p) if(*p>='a'&&*p<='z') *p = (char)(*p - 'a' + 'A');

      if(!strcmp(cmd,"HELLO")){ banner_and_selftest(); continue; }
      if(!strcmp(cmd,"SCAN")){
        for(int i=0;i<4;i++){ has_mag[i] = line_ok[i] && scan_for_mag(&bus[i]); ready[i]=false; }
        uart_println("ACK SCAN");
        continue;
      }
      if(!strcmp(cmd,"INFO")){
        for(int i=0;i<4;i++) print_info_one(i);
        continue;
      }
      if(!strcmp(cmd,"INIT")){
        char* a = strtok(NULL," ");
        int mask = parse_bus_or_all(a);
        for(int i=0;i<4;i++) if(mask&(1<<i)){
          if(has_mag[i]){
            (void)mag_set_standby(&bus[i]);           // required for most CTRL1 updates :contentReference[oaicite:15]{index=15}
            (void)mag_wr(&bus[i], REG_CTRL2, CTRL2_AUTO_MRST_EN); // recommended AUTO_MRST_EN :contentReference[oaicite:16]{index=16}
            ctrl1_cached[i] = 0; // default DR=80Hz OS=16 (all zeros) with AC=0
            ready[i]=true;
          }
        }
        uart_println("ACK INIT");
        continue;
      }
      if(!strcmp(cmd,"ACTIVE")){
        char* a = strtok(NULL," ");
        int mask = parse_bus_or_all(a);
        for(int i=0;i<4;i++) if(mask&(1<<i)) if(ready[i]) (void)mag_set_active(&bus[i], ctrl1_cached[i]);
        uart_println("ACK ACTIVE");
        continue;
      }
      if(!strcmp(cmd,"STANDBY")){
        char* a = strtok(NULL," ");
        int mask = parse_bus_or_all(a);
        for(int i=0;i<4;i++) if(mask&(1<<i)) if(has_mag[i]) (void)mag_set_standby(&bus[i]);
        uart_println("ACK STANDBY");
        continue;
      }
      if(!strcmp(cmd,"CFG1")){
        char* a = strtok(NULL," ");
        char* v = strtok(NULL," ");
        if(!a||!v){ uart_println("ERR CFG1 usage"); continue; }
        int idx = atoi(a); uint8_t val=(uint8_t)strtoul(v,NULL,16);
        if(idx>=0&&idx<4&&has_mag[idx]){
          // NOTE: Except AC/TM, device should be STANDBY to change CTRL1 fields. :contentReference[oaicite:17]{index=17}
          (void)mag_wr(&bus[idx], REG_CTRL1, val);
          ctrl1_cached[idx] = (uint8_t)(val & ~CTRL1_AC);
          uart_println("ACK CFG1");
        } else uart_println("ERR CFG1 bad bus");
        continue;
      }
      if(!strcmp(cmd,"CFG2")){
        char* a = strtok(NULL," ");
        char* v = strtok(NULL," ");
        if(!a||!v){ uart_println("ERR CFG2 usage"); continue; }
        int idx = atoi(a); uint8_t val=(uint8_t)strtoul(v,NULL,16);
        if(idx>=0&&idx<4&&has_mag[idx]){ (void)mag_wr(&bus[idx], REG_CTRL2, val); uart_println("ACK CFG2"); }
        else uart_println("ERR CFG2 bad bus");
        continue;
      }
      if(!strcmp(cmd,"WR")){
        char* a = strtok(NULL," "); char* r = strtok(NULL," "); char* v = strtok(NULL," ");
        if(!a||!r||!v){ uart_println("ERR WR usage"); continue; }
        int idx=atoi(a); uint8_t reg=(uint8_t)strtoul(r,NULL,16), val=(uint8_t)strtoul(v,NULL,16);
        if(idx>=0&&idx<4&&has_mag[idx]){ if(mag_wr(&bus[idx],reg,val)) uart_println("ACK WR"); else uart_println("ERR WR I2C"); }
        else uart_println("ERR WR bad bus");
        continue;
      }
      if(!strcmp(cmd,"RD")){
        char* a = strtok(NULL," "); char* r = strtok(NULL," "); char* l = strtok(NULL," ");
        if(!a||!r||!l){ uart_println("ERR RD usage"); continue; }
        int idx=atoi(a); uint8_t reg=(uint8_t)strtoul(r,NULL,16); int len=atoi(l); if(len<=0||len>32) len=1;
        if(idx>=0&&idx<4&&has_mag[idx]){
          uint8_t buf[32]; if(mag_rdN(&bus[idx],reg,buf,(uint8_t)len)){
            uart_print("RD "); uart_print(bus[idx].name); uart_print(" ");
            for(int i=0;i<len;i++){ uart_hex8(buf[i]); if(i+1<len) uart0_putc(' '); }
            uart0_putc('\r'); uart0_putc('\n');
          } else uart_println("ERR RD I2C");
        } else uart_println("ERR RD bad bus");
        continue;
      }
      if(!strcmp(cmd,"READXYZ")){
        char* a = strtok(NULL," ");
        int mask = parse_bus_or_all(a);
        uint32_t ts = millis_like();
        for(int i=0;i<4;i++) if(mask&(1<<i)){
          int16_t x=0,y=0,z=0; uint8_t t=0; (void)mag_rd1(&bus[i],REG_DIE_TEMP,&t);
          bool ok = has_mag[i] && mag_xyz(&bus[i],&x,&y,&z);
          uart_print("XYZ,"); uart_print_int((int32_t)ts); uart_print(","); uart_print(bus[i].name); uart_print(",X=");
          uart_print_int(ok?x:0); uart_print(" Y="); uart_print_int(ok?y:0); uart_print(" Z="); uart_print_int(ok?z:0);
          uart_print(" T="); uart_print_int((int8_t)t); uart0_putc('\r'); uart0_putc('\n');
        }
        continue;
      }
      if(!strcmp(cmd,"STREAM")){
        char* sub = strtok(NULL," ");
        if(sub && !strcmp(sub,"START")){
          char* m = strtok(NULL," ");
          stream_mask = (m? (uint8_t)strtoul(m,NULL,0) : 0x0F);
          streaming=true; uart_println("ACK STREAM START");
        }else if(sub && !strcmp(sub,"STOP")){
          streaming=false; uart_println("ACK STREAM STOP");
        }else{
          uart_println("ERR STREAM usage");
        }
        continue;
      }
      if(!strcmp(cmd,"CAL")){
        char* sub = strtok(NULL," ");
        if(sub && !strcmp(sub,"START")){ uart_println("ACK CAL START"); }
        else if(sub && !strcmp(sub,"STOP")){ uart_println("ACK CAL STOP"); }
        else { uart_println("ERR CAL usage"); }
        continue;
      }

      // Unknown
      uart_print("NAK "); uart_println(cmd);
    }

    // If streaming, push one sample set per loop (host controls pacing)
    if(streaming){
      uint32_t ts = millis_like();
      for(int i=0;i<4;i++) if(stream_mask&(1<<i)){
        int16_t x=0,y=0,z=0; uint8_t t=0; (void)mag_rd1(&bus[i],REG_DIE_TEMP,&t);
        bool ok = has_mag[i] && mag_xyz(&bus[i],&x,&y,&z);
        uart_print("XYZ,"); uart_print_int((int32_t)ts); uart_print(","); uart_print(bus[i].name); uart_print(",X=");
        uart_print_int(ok?x:0); uart_print(" Y="); uart_print_int(ok?y:0); uart_print(" Z="); uart_print_int(ok?z:0);
        uart_print(" T="); uart_print_int((int8_t)t); uart0_putc('\r'); uart0_putc('\n');
      }
      _delay_ms(10); // ~100 Hz overall; host can decimate
    }
  }
  return 0;
}
