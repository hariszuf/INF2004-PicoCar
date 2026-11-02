#ifndef INTEGRATION_HOST_PICO_STDLIB_H
#define INTEGRATION_HOST_PICO_STDLIB_H
#include <stdbool.h>
#include <stdint.h>
#define GPIO_FUNC_I2C 3u
#define PICO_ERROR_TIMEOUT (-1)
bool stdio_init_all(void);
uint64_t time_us_64(void);
int getchar_timeout_us(uint32_t timeout);
void tight_loop_contents(void);
void gpio_set_function(unsigned pin,unsigned function);
void gpio_pull_up(unsigned pin);
#endif
