#ifndef INTEGRATION_HOST_I2C_H
#define INTEGRATION_HOST_I2C_H
#include "../../../../barrc_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef struct
{
    uint8_t reg[256];
    uint8_t pointer;
} i2c_inst_t;
extern i2c_inst_t * const i2c0;
uint32_t i2c_init (i2c_inst_t * p_bus, uint32_t baudrate);
int32_t i2c_write_timeout_us (i2c_inst_t * p_bus, uint8_t address,
                          const uint8_t * data,
                          size_t length, bool b_no_stop, uint32_t timeout);
int32_t i2c_read_timeout_us (i2c_inst_t * p_bus, uint8_t address,
                             uint8_t * data,
                         size_t length, bool b_no_stop, uint32_t timeout);
#endif

/*** end of file ***/
