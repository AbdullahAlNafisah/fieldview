#include <util/delay.h>
#include "uart.h"
#include "softi2c.h"
#include "mag3110.h"
#include "pins_mega2560_d2_d9.h"
#include <string.h>  // for strcmp


// --- simple line parser for commands from host ---
static bool read_line(char *buf, uint8_t cap){
  static uint8_t idx = 0;
  while (uart0_available()){
    char c = uart0_getc();
    if (c == '\r') continue;
    if (c == '\n'){
      buf[idx] = 0;
      idx = 0;
      return true;          // got a full line
    }
    if (idx + 1 < cap) buf[idx++] = c;    // leave room for NUL
    else idx = 0; // overflow: reset
  }
  return false;
}

int main(void){
  uart0_init(115200);

  SoftI2C bus[4] = {
    { BUS0_SDA_PORT, BUS0_SDA_DDR, BUS0_SDA_PIN, BUS0_SDA_BIT,
      BUS0_SCL_PORT, BUS0_SCL_DDR, BUS0_SCL_PIN, BUS0_SCL_BIT, "Bus0" },
    { BUS1_SDA_PORT, BUS1_SDA_DDR, BUS1_SDA_PIN, BUS1_SDA_BIT,
      BUS1_SCL_PORT, BUS1_SCL_DDR, BUS1_SCL_PIN, BUS1_SCL_BIT, "Bus1" },
    { BUS2_SDA_PORT, BUS2_SDA_DDR, BUS2_SDA_PIN, BUS2_SDA_BIT,
      BUS2_SCL_PORT, BUS2_SCL_DDR, BUS2_SCL_PIN, BUS2_SCL_BIT, "Bus2" },
    { BUS3_SDA_PORT, BUS3_SDA_DDR, BUS3_SDA_PIN, BUS3_SDA_BIT,
      BUS3_SCL_PORT, BUS3_SCL_DDR, BUS3_SCL_PIN, BUS3_SCL_BIT, "Bus3" }
  };

  uart0_println("\r\n=== MAG3110 x4 — Debug bring-up (Mega2560 D2..D9) ===");
  for(int i=0;i<4;i++) i2c_init(&bus[i]);
  _delay_ms(10);

  uart0_println("\r\n-- 1) Line self-test --");
  bool lines_ok[4]={0};
  for(int i=0;i<4;i++) lines_ok[i]=i2c_line_selftest(&bus[i]);

  uart0_println("\r\n-- 2) Bus recovery --");
  for(int i=0;i<4;i++){
    // If SDA is low, try recovery
    *(bus[i].sda_ddr) &= ~(1<<bus[i].sda_bit);
    *(bus[i].scl_ddr) &= ~(1<<bus[i].scl_bit);
    _delay_ms(1);
    if( ((*(bus[i].sda_pin)) & (1<<bus[i].sda_bit)) == 0 ){
      uart0_print(bus[i].name); uart0_println(": SDA low -> recovering");
      i2c_bus_recover(&bus[i]);
    } else {
      uart0_print(bus[i].name); uart0_println(": SDA high (ok)");
    }
  }

  uart0_println("\r\n-- 3) I2C scan (expect 0x0E) --");
  bool has_mag[4]={0};
  for(int i=0;i<4;i++){
    if(lines_ok[i]) has_mag[i]=i2c_scan_bus(&bus[i], MAG3110_ADDR);
    else { uart0_print(bus[i].name); uart0_println(": skipped (line test failed)"); }
  }

  uart0_println("\r\n-- 4) Init + status --");
  bool ready[4]={0};
  for(int i=0;i<4;i++){
    if(has_mag[i]) ready[i]=mag_init_and_status(&bus[i]);
    else { uart0_print(bus[i].name); uart0_println(": no 0x0E -> skip"); }
  }

  uart0_println("\r\n-- 5) Streaming XYZ --");
  // --------------------------------------------
  // Command-driven loop from the host (Python GUI)
  // Accept single-letter commands too:
  //   'S' or "START" -> arm
  //   'T' or "STOP"  -> disarm
  //   'R' or "REQ"   -> one-shot batch for all buses
  // --------------------------------------------
  uart0_println("\r\nREADY");
  bool armed = false;
  char cmdbuf[32];

  while (1) {
    if (read_line(cmdbuf, sizeof(cmdbuf))) {
      // normalize first char (case-insensitive)
      char c = cmdbuf[0];
      if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

      if (c == 'S') {                // START
        armed = true;
        uart0_println("ACK START");
      }
      else if (c == 'T') {           // STOP
        armed = false;
        uart0_println("ACK STOP");
      }
      else if (c == 'R') {           // REQ
        if (armed) {
          for (int i = 0; i < 4; i++) {
            uart0_print("Bus"); uart0_putc('0' + i); uart0_print(": ");
            if (ready[i]) {
              int16_t x, y, z;
              if (mag_read_xyz(&bus[i], &x, &y, &z)) {
                uart0_print("X="); uart0_print_int(x);
                uart0_print(" Y="); uart0_print_int(y);
                uart0_print(" Z="); uart0_print_int(z);
              } else {
                uart0_print("X=0 Y=0 Z=0");
              }
            } else {
              uart0_print("X=0 Y=0 Z=0");
            }
            uart0_putc('\r'); uart0_putc('\n');
          }
        } else {
          uart0_println("NAK REQ (not started)");
        }
      }
      else {
        uart0_print("NAK "); uart0_println(cmdbuf);
      }
    }
    _delay_ms(1); // tiny yield
  }
  return 0;
}
