"""Host check of the exact diagnostic LED function, with fake radio/netif/time."""
from pathlib import Path
import subprocess
import argparse

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
source = (args.repo / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c').read_text()
build = root / 'build-led'
build.mkdir(exist_ok=True)
start = source.index('static void startup_led_poll(void)')
end = source.index('\n}\n#endif\n\nvoid cyw43_utk_task', start) + 2
function = source[start:end]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define CYW43_ITF_STA 0
struct netif { unsigned up, link, ip; };
static struct { unsigned itf_state; struct netif netif[1]; } cyw43_state;
static struct { int init_result; } radio_status;
static unsigned writes;
static int observed_level, mqtt_connected;
static uint64_t clock_us;
#define netif_is_up(n) ((n)->up)
#define netif_is_link_up(n) ((n)->link)
#define netif_ip4_addr(n) (&(n)->ip)
#define ip4_addr_isany_val(ip) ((ip) == 0)
static uint64_t time_us_64(void) { return clock_us; }
static int mqtt_utk_is_connected(void) { return mqtt_connected; }
static int cyw43_gpio_set(void *state, int gpio, bool level) {
    assert(state == &cyw43_state); assert(gpio == 0);
    assert(cyw43_state.itf_state == 1);
    observed_level = level; writes++; return 0;
}
/* FUNCTION */
static void check(unsigned pulses) {
    unsigned before = writes;
    for(unsigned slot = 0; slot < 40; slot++) {
        clock_us = slot * 200000ULL;
        startup_led_poll();
        assert(observed_level == (pulses == 0 ||
            (slot % 20 < 2 * pulses && (slot & 1) == 0)));
        unsigned after = writes;
        for(unsigned repeat = 0; repeat < 100; repeat++) startup_led_poll();
        assert(writes == after); /* No extra radio I/O at the 2ms poll rate. */
    }
    assert(writes - before <= 40);
}
int main(void) {
    startup_led_poll(); assert(writes == 0); /* No radio init via LED ioctl. */
    cyw43_state.itf_state = 1;
    check(2); /* Direct join no longer waits for a preliminary scan. */
    cyw43_state.netif[0].link = 1;
    cyw43_state.netif[0].up = 1; check(3);
    cyw43_state.netif[0].ip = 0x1234;
    cyw43_state.netif[0].up = 0; check(3);
    cyw43_state.netif[0].up = 1; check(4);
    mqtt_connected = 1; check(0);
    radio_status.init_result = -1; check(5);
    unsigned before = writes;
    cyw43_state.itf_state = 0; startup_led_poll(); assert(writes == before);
    return 0;
}
'''.replace('/* FUNCTION */', function)
c_path = build / 'led-check.c'
exe_path = build / 'led-check.exe'
c_path.write_text(harness)
subprocess.run([args.cc, '-std=c11', '-O2', str(c_path), '-o', str(exe_path)], check=True)
subprocess.run([str(exe_path)], check=True)
print('PASS: actual LED function, all stages, radio-init guard and transition-only writes')
