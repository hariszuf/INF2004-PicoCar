#include <assert.h>
#include "scheduler.h"
int main(void) {
    task_t t = {.next=100, .period=20};
    assert(!due(&t,99)); assert(due(&t,100)); assert(t.next==120);
    assert(due(&t,181)); assert(t.skipped==3 && t.next==200 && t.late_max_us==61);
    assert(!due(&t,181)); assert(due(&t,200)); assert(t.skipped==3);
    task_t wide = {.next=UINT64_C(0x100000000), .period=5000};
    assert(due(&wide,UINT64_C(0x100000001)) && wide.late_max_us==1);
    return 0;
}
