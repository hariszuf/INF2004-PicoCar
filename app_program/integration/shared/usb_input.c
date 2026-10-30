#ifdef B5_FULL_TEST
/* Compiled with SDK/TinyUSB headers, never with tkernel.h. */
#include "tusb.h"
extern void b5_line_command_received(unsigned command);
/* Link wrapper is active only in the full-test build. TinyUSB remains owned by
 * the original USB service task. We replace its input-discard call. */
void __wrap_tm_tusb_discard_input(void) {
 unsigned budget=64;
 while(budget-- && tud_cdc_available()) {
  int c=tud_cdc_read_char();
  if(c>=0) b5_line_command_received((unsigned)c);
 }
}
#endif
