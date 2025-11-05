#include <Arduino.h>
#include <SoftWire.h>
#include <AsyncDelay.h>

extern "C" {
  #include "bmm350.h"
}

// ---- pins / address ----
constexpr uint8_t SCL_PIN = 2;
constexpr uint8_t SDA_PIN = 3;
constexpr uint8_t BMM350_ADDR = 0x14;  // use 0x15 if your board straps ADSEL=HIGH

// ---- soft I2C ----
SoftWire sw(SDA_PIN, SCL_PIN);
char swTxBuffer[32];
char swRxBuffer[32];

// ---- Bosch device ----
static bmm350_dev dev;

AsyncDelay readInterval;
volatile uint16_t sampleCount = 0;
constexpr uint16_t MAX_SAMPLES = 1000;

// ---- I2C glue ----
static int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  uint8_t addr = *(uint8_t*)intf_ptr;
  sw.beginTransmission(addr);
  sw.write(reg_addr);
  if (sw.endTransmission(false) != 0) return BMM350_E_COM_FAIL; // repeated start
  int got = sw.requestFrom(addr, (uint8_t)len);
  if (got != (int)len) return BMM350_E_COM_FAIL;
  for (uint32_t i = 0; i < len; ++i) data[i] = sw.read();
  return BMM350_OK;
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  uint8_t addr = *(uint8_t*)intf_ptr;
  sw.beginTransmission(addr);
  sw.write(reg_addr);
  for (uint32_t i = 0; i < len; ++i) sw.write(data[i]);
  return (sw.endTransmission() == 0) ? BMM350_OK : BMM350_E_COM_FAIL;
}

static void delay_us(uint32_t us, void*) {
  if (us >= 1000) delay(us/1000);
  else delayMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  // LED to show “done”
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // SoftWire setup
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(25);       // slower bitbang
  sw.setTimeout(2000);      // more forgiving
  sw.begin();

  sw.begin();

  // Bosch device init
  static uint8_t i2c_addr = BMM350_ADDR;
  dev.read     = i2c_read;
  dev.write    = i2c_write;
  dev.delay_us = delay_us;
  dev.intf_ptr = &i2c_addr;

  int8_t rslt = bmm350_init(&dev);
  if (rslt != BMM350_OK) {
    Serial.print("bmm350_init failed: "); Serial.println(rslt);
    while (1) { delay(1000); }
  }

  // Configure (ODR/averaging/axes/mode)
  bmm350_set_odr_performance(BMM350_DATA_RATE_50HZ, BMM350_AVERAGING_8, &dev); // ~50 Hz
  bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
  bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);

  // 50 Hz -> ~20 ms period
  readInterval.start(20, AsyncDelay::MILLIS);

  // Print a tiny header so the PC script knows we’re sending CSV
  Serial.println("# BMM350 CSV x,y,z");
}

void loop() {
  // Stop after MAX_SAMPLES
  if (sampleCount >= MAX_SAMPLES) {
    // Put sensor to sleep and notify PC
    bmm350_set_powermode(BMM350_SUSPEND_MODE, &dev);
    Serial.println("DONE");
    digitalWrite(LED_BUILTIN, HIGH); // solid LED indicates completion
    while (1) { delay(1000); }       // halt further reads
  }

  if (!readInterval.isExpired()) return;
  readInterval.restart();

  bmm350_mag_temp_data m = {0};
  if (bmm350_get_compensated_mag_xyz_temp_data(&m, &dev) == BMM350_OK) {
    // CSV: x,y,z (microtesla)
    Serial.print(m.x, 6); Serial.print(',');
    Serial.print(m.y, 6); Serial.print(',');
    Serial.println(m.z, 6);
    sampleCount++;
  }
}
