#ifndef B5_COMPAT_GPIO_H
#define B5_COMPAT_GPIO_H
#include <stdint.h>
#include <stdbool.h>
#ifndef GPIO_IN
#define GPIO_IN 0
#endif
#ifndef GPIO_OUT
#define GPIO_OUT 1
#endif
#define GPIO_FUNC_PWM 4u
#define GPIO_IRQ_EDGE_FALL 4u
#define GPIO_IRQ_EDGE_RISE 8u
typedef void (*b5_gpio_callback_t)(unsigned int,uint32_t);
void b5p_gpio_init(uint32_t pin);
void b5p_gpio_set_dir(uint32_t pin,bool output);
void b5p_gpio_pull_up(uint32_t pin);
bool b5p_gpio_get(uint32_t pin);
void b5p_gpio_set_function(uint32_t pin,uint32_t function);
void b5p_gpio_set_irq_enabled(uint32_t pin,uint32_t events,bool enabled);
void b5p_gpio_set_irq_enabled_with_callback(uint32_t pin,uint32_t events,
                                           bool enabled,b5_gpio_callback_t callback);
#define gpio_init b5p_gpio_init
#define gpio_set_dir b5p_gpio_set_dir
#define gpio_pull_up b5p_gpio_pull_up
#define gpio_get b5p_gpio_get
#define gpio_set_function b5p_gpio_set_function
#define gpio_set_irq_enabled b5p_gpio_set_irq_enabled
#define gpio_set_irq_enabled_with_callback b5p_gpio_set_irq_enabled_with_callback
#endif
