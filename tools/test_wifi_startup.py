"""Compile the production join-control block with a fake clock and radio.

Run: python tools/test_wifi_startup.py [--cc gcc] [--source path/to/cyw43_utk.c]
This checks scheduling decisions, not hardware, DHCP or driver behaviour.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default='gcc')
parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[1] / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c')
args = parser.parse_args()
source = args.source.read_text()
block = source[source.index('        radio_status.poll_count++;'):source.index('        status_publish_end();')]
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define TM_WIFI_JOIN 1
#define CYW43_JOIN_TIMEOUT_US 30000000ULL
#define CYW43_JOIN_RETRY_US 5000000ULL
#define CYW43_ITF_STA 0
#define CYW43_LINK_DOWN 0
#define CYW43_ETIMEDOUT 1
#define CYW43_CHANNEL_NONE 0
#define UTK_WIFI_AUTH 0
static struct {
    unsigned poll_count, scan_active, scan_complete;
    unsigned join_started, join_complete, link_up, join_elapsed_ms;
    int join_request_result, join_result, link_status, link_rssi;
    uint8_t bssid[6];
} radio_status;
static int cyw43_state, calls, request_result, link;
static uint64_t now_us, join_started_us, join_failed_us;
static const uint8_t wifi_ssid[] = "fake", wifi_password[] = "fakepassword";
static uint64_t time_us_64(void) { return now_us; }
static void mp_memory_barrir(void) {}
static void startup_led_poll(void) {}
static bool cyw43_wifi_scan_active(void *state) { (void)state; return false; }
static int cyw43_wifi_join(void *state, unsigned sl, const uint8_t *ssid,
    unsigned pl, const uint8_t *password, int auth, void *bssid, int channel) {
    (void)state; (void)sl; (void)ssid; (void)pl; (void)password;
    (void)auth; (void)bssid; (void)channel;
    calls++; return request_result;
}
static int cyw43_wifi_link_status(void *state, int itf) {
    (void)state; (void)itf; return link;
}
static int cyw43_wifi_get_bssid(void *state, void *bssid) {
    (void)state; (void)bssid; return 0;
}
static int cyw43_wifi_get_rssi(void *state, int32_t *rssi) {
    (void)state; *rssi = -50; return 0;
}
static void poll_join(void) {
    bool was_scanning = radio_status.scan_active != 0;
    int result;
/* PRODUCTION_BLOCK */
}
static void reset(void) {
    memset(&radio_status, 0, sizeof(radio_status));
    radio_status.scan_active = 1;
    now_us = 1000; join_started_us = join_failed_us = 0;
    calls = request_result = link = 0;
}
int main(void) {
    /* Immediate request failure: retry only after a full cooldown. */
    reset(); request_result = -2; poll_join(); assert(calls == 1);
    now_us += 4999999; poll_join(); assert(calls == 1);
    now_us++; request_result = 0; poll_join();
#if TM_WIFI_MQTT
    assert(calls == 2); assert(!radio_status.join_complete);
    /* Successful association must remain undisturbed indefinitely. */
    radio_status.link_up = 1; poll_join();
    assert(radio_status.join_result == 0);
    now_us += 60000000; poll_join(); assert(calls == 2);
#else
    assert(calls == 1); /* qualification profile retains one-shot behaviour */
#endif
    /* Pending request: no overlapping join; timeout gets its own cooldown. */
    reset(); poll_join(); now_us += 29999999; poll_join(); assert(calls == 1);
    now_us++; poll_join(); assert(radio_status.join_complete);
    now_us += 4999999; poll_join(); assert(calls == 1);
    now_us++; poll_join(); assert(calls == (TM_WIFI_MQTT ? 2 : 1));
    /* Terminal driver failure, followed by a late link before retry. */
    reset(); poll_join(); now_us += 100; link = -3; poll_join();
    assert(radio_status.join_complete && radio_status.join_result == -3);
    radio_status.link_up = 1; now_us += 5000000; poll_join(); assert(calls == 1);
    /* Terminal failure without a late link does retry in MQTT builds. */
    reset(); poll_join(); link = -3; poll_join();
    now_us += 5000000; link = 0; poll_join();
    assert(calls == (TM_WIFI_MQTT ? 2 : 1));
    return 0;
}
'''.replace('/* PRODUCTION_BLOCK */', block)
with tempfile.TemporaryDirectory(prefix='picocar-join-test-') as directory:
    folder = Path(directory)
    c_file = folder / 'join_test.c'
    c_file.write_text(harness)
    for mqtt in (0, 1):
        exe = folder / f'join_test_{mqtt}.exe'
        subprocess.run([args.cc, '-std=c11', '-O2', f'-DTM_WIFI_MQTT={mqtt}', str(c_file), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(f'PASS: production join-control block, TM_WIFI_MQTT={mqtt}')
