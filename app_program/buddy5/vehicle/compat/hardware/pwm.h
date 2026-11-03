#ifndef B5_COMPAT_PWM_H
#define B5_COMPAT_PWM_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { uint32_t divider; uint16_t wrap; } pwm_config;
static inline uint32_t pwm_gpio_to_slice_num(uint32_t pin) { return (pin>>1)&7u; }
static inline pwm_config pwm_get_default_config(void) { return (pwm_config){16u,65535u}; }
static inline void pwm_config_set_clkdiv(pwm_config *c,float v) { c->divider=(uint32_t)(v*16.0f); }
static inline void pwm_config_set_wrap(pwm_config *c,uint16_t v) { c->wrap=v; }
void b5p_pwm_init(uint32_t slice,const pwm_config *c,bool enable);
void b5p_pwm_set_gpio_level(uint32_t pin,uint16_t level);
#define pwm_init b5p_pwm_init
#define pwm_set_gpio_level b5p_pwm_set_gpio_level
#endif
