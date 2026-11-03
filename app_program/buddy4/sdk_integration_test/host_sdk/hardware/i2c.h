#ifndef INTEGRATION_HOST_I2C_H
#define INTEGRATION_HOST_I2C_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef struct { uint8_t reg[256],pointer; } i2c_inst_t;
extern i2c_inst_t *const i2c0;
unsigned i2c_init(i2c_inst_t *bus,unsigned baudrate);
int i2c_write_timeout_us(i2c_inst_t *bus,uint8_t address,const uint8_t *data,
    size_t length,bool no_stop,unsigned timeout);
int i2c_read_timeout_us(i2c_inst_t *bus,uint8_t address,uint8_t *data,
    size_t length,bool no_stop,unsigned timeout);
#endif
