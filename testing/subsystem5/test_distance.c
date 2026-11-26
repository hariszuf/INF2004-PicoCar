#include <assert.h>
#include <stdint.h>
#include <stdio.h>
/* Independent formula tests; hardware driver is compiled separately on Pico. */
static uint32_t convert (uint32_t us)
{
    return (us * 343u + 1000u) / 2000u;
}

int main (void)
{
    assert(343 == convert(2000)); /* 34.3 cm */
    assert(172 == convert(1000)); /* 17.2 cm rounded */
    assert(99 == convert(580));   /* approximately 10 cm */
    puts("Distance conversion tests passed");
    return 0;
}

/*** end of file ***/
