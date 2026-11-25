#ifndef INTEGRATION_HOST_PICO_STDLIB_H
#define INTEGRATION_HOST_PICO_STDLIB_H
#include "../../../../barrc_types.h"
#include <stdbool.h>
#include <stdint.h>
#define GPIO_FUNC_I2C 3u
#define PICO_ERROR_TIMEOUT (-1)
bool stdio_init_all (void);
uint64_t time_us_64 (void);
int32_t getchar_timeout_us (uint32_t timeout);
void tight_loop_contents (void);
void gpio_set_function (uint32_t pin, uint32_t function);
void gpio_pull_up (uint32_t pin);
#endif

/*** end of file ***/
