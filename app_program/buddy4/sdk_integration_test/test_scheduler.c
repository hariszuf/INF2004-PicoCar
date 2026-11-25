#include <assert.h>
#include "scheduler.h"
int main (void)
{
    task_t t = {.next = 100, .period = 20};
    assert(!due(&t, 99));
    assert(due(&t, 100));
    assert(120 == t.next);
    assert(due(&t, 181));
    assert((3 == t.skipped) && (200 == t.next) && (61 == t.late_max_us));
    assert(!due(&t, 181));
    assert(due(&t, 200));
    assert(3 == t.skipped);
    task_t wide = {.next = UINT64_C(0x100000000), .period = 5000};
    assert((due(&wide, UINT64_C(0x100000001))) && (1 == wide.late_max_us));
    return 0;
}

/*** end of file ***/
