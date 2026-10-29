#include <tk/tkernel.h>
#include <stdint.h>
#include "buddy5_config.h"
#include "buddy5_servo.h"
#define B5_REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define B5_PWM_SLICE (0x40050000u + 6u*0x14u)
#if B5_SERVO_PIN != 12u
#error "This driver owns PWM slice 6A / GP12; update mapping before changing pin."
#endif
static uint64_t clock_us(void) {
    uint32_t hi,lo,again;
    do {
        hi=B5_REG(0x40054024u); lo=B5_REG(0x40054028u);
        again=B5_REG(0x40054024u);
    } while(hi!=again);
    return ((uint64_t)hi<<32)|lo;
}
uint32_t buddy5_clock_ms(void) { return (uint32_t)(clock_us()/1000u); }
bool buddy5_servo_set_angle(int degrees) {
    int32_t pulse;
    if(degrees < -60 || degrees > 60) return false;
    if(degrees>=0)
        pulse=(int32_t)B5_SERVO_CENTRE_US +
          degrees*((int32_t)B5_SERVO_LEFT_US-(int32_t)B5_SERVO_CENTRE_US)/60;
    else
        pulse=(int32_t)B5_SERVO_CENTRE_US +
          (-degrees)*((int32_t)B5_SERVO_RIGHT_US-(int32_t)B5_SERVO_CENTRE_US)/60;
    if(pulse<500 || pulse>2500) return false;
    B5_REG(B5_PWM_SLICE+0x0cu) =
        (B5_REG(B5_PWM_SLICE+0x0cu)&0xffff0000u)|(uint32_t)pulse;
    return true;
}
bool buddy5_servo_init(void) {
    uint32_t start=buddy5_clock_ms();
    /* Release PWM reset without resetting motor slices. */
    B5_REG(0x4000f000u)=1u<<14; /* RESETS atomic CLR alias, RESET offset 0 */
    while(!(B5_REG(0x4000c008u)&(1u<<14)))
        if((uint32_t)(buddy5_clock_ms()-start)>10u) return false;
    /* This project configures clk_sys=125MHz. Divider gives 1us ticks.
     * Own ALL of slice 6 (GP12/13); no other PWM client may use that slice. */
    B5_REG(B5_PWM_SLICE)=0;
    B5_REG(B5_PWM_SLICE+4u)=TMCLK_MHz<<4;
    B5_REG(B5_PWM_SLICE+8u)=0;
    B5_REG(B5_PWM_SLICE+0x10u)=19999u; /* 50Hz */
    if(!buddy5_servo_set_angle(0)) return false;
    B5_REG(0x4001c004u+4u*B5_SERVO_PIN) &= ~(1u<<7); /* output enabled */
    B5_REG(0x40014004u+8u*B5_SERVO_PIN)=4u; /* PWM function */
    B5_REG(B5_PWM_SLICE)=1u;
    return true;
}