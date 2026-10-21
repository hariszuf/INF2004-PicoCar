#include "lsm303d.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#define BUS i2c0
#define SDA_PIN 16
#define SCL_PIN 17
#define ADDR 0x1D
#define REG_WHO_AM_I 0x0F
#define REG_CTRL1 0x20
#define REG_CTRL2 0x21
#define REG_ACCEL_OUT 0x28

static bool write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_timeout_us(BUS, ADDR, data, sizeof data, false, 30000) == 2;
}

static bool read_regs(uint8_t reg, uint8_t *buffer, size_t len) {
    // Register bit 7 enables auto-increment during multi-byte reads.
    return i2c_write_timeout_us(BUS, ADDR, &reg, 1, true, 30000) == 1 &&
           i2c_read_timeout_us(BUS, ADDR, buffer, len, false, 30000) == (int)len;
}

void lsm303d_bus_init(void) {
    i2c_init(BUS, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

bool lsm303d_init(void) {
    uint8_t who = 0;
    if (!read_regs(REG_WHO_AM_I, &who, 1)) {
        printf("LSM303D chip ID read failed\n");
        return false;
    }
    printf("WHO_AM_I = 0x%02X (LSM303D expected 0x49)\n", who);
    if (who != 0x49) return false; // ST LSM303D identification byte.
    // 100 Hz accelerometer output, all axes enabled, block data update.
    if (!write_reg(REG_CTRL1, 0x5F)) return false;
    // +/-2 g full scale. High resolution is enabled by CTRL1.
    if (!write_reg(REG_CTRL2, 0x00)) return false;
    sleep_ms(100);
    return true;
}

bool lsm303d_read_accel(accel_g_t *out) {
    if (!out) return false;
    uint8_t bytes[6];
    if (!read_regs(REG_ACCEL_OUT | 0x80, bytes, sizeof bytes)) return false;
    int16_t raw_x = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    int16_t raw_y = (int16_t)((uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8));
    int16_t raw_z = (int16_t)((uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8));
    // LSM303D high resolution, +/-2g: 0.061 mg/count.
    const float scale = 0.000061f;
    out->x = raw_x * scale;
    out->y = raw_y * scale;
    out->z = raw_z * scale;
    return true;
}
