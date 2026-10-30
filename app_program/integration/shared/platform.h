#ifndef B5_LINE_PLATFORM_H
#define B5_LINE_PLATFORM_H
#include <stdint.h>
#include <stdbool.h>
int32_t b5_platform_init(void);
uint64_t b5p_time_us_64(void);
void b5_echo_irq(uint32_t events,uint32_t timestamp_us);
void b5_line_command_received(unsigned command);
bool b5_line_take_stop(void);
int b5_line_take_command(void);
bool b5_line_usb_connected(void);
bool line_controller_active(void);
#endif
