#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <stdint.h>
#include "platform.h"
#include "compat/hardware/gpio.h"
#include "../buddy5/ultrasonic.h"
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
enum { OFF, WAIT_RISE, WAIT_FALL, DONE, BAD };
static volatile uint32_t phase, rise_us, width_us;
void b5_echo_irq(uint32_t events,uint32_t now) {
 if(phase==OFF) return;
 if(events==(GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL)) { phase=BAD; return; }
 if(phase==WAIT_RISE && (events&GPIO_IRQ_EDGE_RISE)) { rise_us=now; phase=WAIT_FALL; }
 else if(phase==WAIT_FALL && (events&GPIO_IRQ_EDGE_FALL)) {
  width_us=(uint32_t)(now-rise_us); phase=DONE;
 } else if(phase!=DONE) phase=BAD;
}
void ultrasonic_init(void) {
 phase=OFF;
 b5p_gpio_init(28); b5p_gpio_set_dir(28,true);
 b5p_gpio_init(7); b5p_gpio_set_irq_enabled(7,12u,true);
}
uint32_t ultrasonic_echo_to_mm(uint32_t us) { return (us*343u+1000u)/2000u; }
ultrasonic_result_t ultrasonic_read(void) {
 ultrasonic_result_t result={US_OK,0,0};
 uint32_t start, current;
 UW saved;
 if(b5p_gpio_get(7)) { result.status=US_ECHO_STUCK_HIGH; return result; }
 /* Only the 10us trigger is protected. Echo capture uses interrupts, and
  * the task sleeps between checks so motor PID can keep its schedule. */
 DI(saved);
 REG(0x400140f0u)=12u<<28;
 rise_us=width_us=0; phase=WAIT_RISE;
 REG(0xd0000018u)=1u<<28;
 start=(uint32_t)b5p_time_us_64();
 while((uint32_t)((uint32_t)b5p_time_us_64()-start)<3u) { }
 REG(0xd0000014u)=1u<<28;
 start=(uint32_t)b5p_time_us_64();
 while((uint32_t)((uint32_t)b5p_time_us_64()-start)<10u) { }
 REG(0xd0000018u)=1u<<28;
 EI(saved);
 for(;;) {
  DI(saved);
  current=phase;
  uint32_t elapsed=(uint32_t)((uint32_t)b5p_time_us_64()-
                    (current==WAIT_FALL?rise_us:start));
  if(current==DONE) {
   result.echo_us=width_us; phase=OFF; EI(saved); break;
  }
  if(current==BAD || elapsed>=30000u) {
   phase=OFF; EI(saved);
   result.status=current==BAD?US_OUT_OF_RANGE:
                 current==WAIT_FALL?US_FALL_TIMEOUT:US_RISE_TIMEOUT;
   return result;
  }
  EI(saved);
  tk_dly_tsk(1);
 }
 if(result.echo_us<116u || result.echo_us>23200u) result.status=US_OUT_OF_RANGE;
 else result.distance_mm=ultrasonic_echo_to_mm(result.echo_us);
 return result;
}
#endif