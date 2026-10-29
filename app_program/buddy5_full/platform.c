#ifdef B5_FULL_TEST
#include <tk/tkernel.h>
#include <stdint.h>
#include <stdbool.h>
#include "platform.h"
#include "compat/hardware/gpio.h"
#include "compat/hardware/pwm.h"
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
static b5_gpio_callback_t encoder_callback;
uint64_t b5p_time_us_64(void) {
 uint32_t h,l,h2;
 do { h=REG(0x40054024u); l=REG(0x40054028u); h2=REG(0x40054024u); } while(h!=h2);
 return ((uint64_t)h<<32)|l;
}
void b5p_gpio_set_function(uint32_t p,uint32_t function) {
 REG(0x4001c004u+4*p)=(REG(0x4001c004u+4*p)|(1u<<6))&~(1u<<7);
 REG(0x40014004u+8*p)=function;
}
void b5p_gpio_set_dir(uint32_t p,bool output) {
 REG(output?0xd0000024u:0xd0000028u)=1u<<p;
}
void b5p_gpio_init(uint32_t p) {
 b5p_gpio_set_dir(p,false);
 REG(0xd0000018u)=1u<<p;
 b5p_gpio_set_function(p,5u);
 REG(0x4001c004u+4*p)&=~((1u<<2)|(1u<<3));
}
void b5p_gpio_pull_up(uint32_t p) {
 REG(0x4001c004u+4*p)=(REG(0x4001c004u+4*p)|(1u<<3))&~(1u<<2);
}
bool b5p_gpio_get(uint32_t p) { return (REG(0xd0000004u)&(1u<<p))!=0; }
void b5p_pwm_init(uint32_t slice,const pwm_config *c,bool enable) {
 uint32_t base=0x40050000u+slice*0x14u;
 REG(base)=0; REG(base+4)=c->divider; REG(base+8)=0;
 REG(base+12)=0; REG(base+16)=c->wrap; REG(base)=enable?1u:0u;
}
void b5p_pwm_set_gpio_level(uint32_t pin,uint16_t level) {
 uint32_t address=0x4005000cu+((pin>>1)&7u)*0x14u, shift=(pin&1u)*16u;
 REG(address)=(REG(address)&~(0xffffu<<shift))|((uint32_t)level<<shift);
}
void b5p_gpio_set_irq_enabled(uint32_t p,uint32_t events,bool enabled) {
 uint32_t offset=(p/8u)*4u, mask=(events&12u)<<((p%8u)*4u);
 UW saved;
 DI(saved);
 REG(0x400140f0u+offset)=mask; /* W1C stale edge status */
 if(enabled) REG(0x40014100u+offset)|=mask;
 else REG(0x40014100u+offset)&=~mask;
 EI(saved);
}
void b5p_gpio_set_irq_enabled_with_callback(uint32_t p,uint32_t events,
                                          bool enabled,b5_gpio_callback_t callback) {
 encoder_callback=callback;
 b5p_gpio_set_irq_enabled(p,events,enabled);
}
static void gpio_dispatch(UINT intno) {
 uint32_t status=REG(0x40014120u);
 uint32_t now=REG(0x40054028u);
 (void)intno;
 /* Echo first, then both encoder pins. Clear only owned edge bits. */
 uint32_t echo=(status>>28)&12u;
 if(echo) { REG(0x400140f0u)=echo<<28; b5_echo_irq(echo,now); }
 for(uint32_t p=4;p<=6;p+=2) {
  uint32_t events=(status>>(p*4u))&12u;
  if(events) {
   REG(0x400140f0u)=events<<(p*4u);
   if(encoder_callback) encoder_callback(p,events);
  }
 }
}
int32_t b5_platform_init(void) {
 T_DINT definition={TA_HLNG,(FP)gpio_dispatch};
 uint32_t start=(uint32_t)b5p_time_us_64();
 REG(0x4000f000u)=1u<<14; /* release PWM reset, do not reset other slices */
 while(!(REG(0x4000c008u)&(1u<<14)))
  if((uint32_t)((uint32_t)b5p_time_us_64()-start)>10000u) return E_SYS;
 ER e=tk_def_int(13,&definition);
 if(e<E_OK) return e;
 ClearInt(13); EnableInt(13,0);
 return E_OK;
}
#endif