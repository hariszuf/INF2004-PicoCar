"""Exercise the production owner task with a fake driver, netif and clock.

Documents recovery gaps as well as working retries. This does not emulate
hardware, the scheduler, DHCP packets or the driver's internal auto-rejoin.
"""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
repo = args.repo.resolve()
source = (repo / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c').read_text()
task = source[source.index('void cyw43_utk_task('):source.index('void cyw43_utk_get_status(')]
header = (repo / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h').as_posix()
build = Path(__file__).resolve().parent / 'build-owner'
build.mkdir(exist_ok=True)
harness = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "STATUS_HEADER"
#define TM_WIFI_MQTT 1
#define TM_WIFI_JOIN 1
#define TM_WIFI_NETIF 1
#define CYW43_ITF_STA 0
#define CYW43_COUNTRY_WORLDWIDE 0
#define CYW43_LINK_DOWN 0
#define CYW43_EIO 5
#define CYW43_ETIMEDOUT 110
#define CYW43_CHANNEL_NONE 0
#define CYW43_JOIN_RETRY_US 5000000ULL
#define CYW43_JOIN_TIMEOUT_US 30000000ULL
#define CYW43_POLL_MS 2
#define UTK_WIFI_AUTH 0
#define CYW43_THIS_PRC() 1
static T_CYW43_UTK_STATUS radio_status;
static struct { unsigned itf_state; int netif[1]; } cyw43_state;
static const uint8_t wifi_ssid[]="fake", wifi_password[]="fakepassword";
static unsigned now_ms, setup_calls, join_calls, radio_polls, mqtt_polls, timers;
static unsigned join_times[8], good_join_time;
static int cyw43_utk_poll_requested;
static const char *mode;
static jmp_buf done;
static void (*cyw43_poll)(void);
static uint64_t time_us_64(void) { return (uint64_t)now_ms*1000; }
static void mp_memory_barrir(void) {}
static void status_publish_begin(void) {}
static void status_publish_end(void) {}
static void cyw43_thread_enter(void) {}
static void cyw43_thread_exit(void) {}
static void lwip_init(void) {}
static void sys_check_timeouts(void) { timers++; }
static void mqtt_utk_poll(void *netif) { (void)netif; mqtt_polls++; }
static void startup_led_poll(void) {}
static void fake_poll(void) { radio_polls++; }
static void cyw43_init(void *state) { (void)state; cyw43_poll=NULL; }
static void cyw43_wifi_set_up(void *state, int itf, bool up, int country) {
    (void)state; (void)itf; (void)up; (void)country; setup_calls++;
    if(strcmp(mode,"init_failure")) {
        cyw43_state.itf_state=1; cyw43_poll=fake_poll;
    }
}
static int cyw43_wifi_get_mac(void *state, int itf, void *mac) {
    (void)state; (void)itf; (void)mac; return 0;
}
static bool cyw43_wifi_scan_active(void *state) { (void)state; return false; }
static int cyw43_wifi_join(void *state, unsigned sl, const uint8_t *ssid,
    unsigned pl, const uint8_t *password, int auth, void *bssid, int channel) {
    (void)state; (void)sl; (void)ssid; (void)pl; (void)password;
    (void)auth; (void)bssid; (void)channel;
    assert(join_calls<8); join_times[join_calls++]=now_ms;
    if((!strcmp(mode,"join_rejected") || !strcmp(mode,"late_link")) && join_calls==1) return -2;
    good_join_time=now_ms; return 0;
}
static int cyw43_wifi_link_status(void *state, int itf) { (void)state; (void)itf; return CYW43_LINK_DOWN; }
static int cyw43_wifi_get_bssid(void *state, void *bssid) { (void)state; (void)bssid; return 0; }
static int cyw43_wifi_get_rssi(void *state, int32_t *rssi) { (void)state; *rssi=-50; return 0; }
static void update_netif_status(void) {
    bool up=join_calls && now_ms>=good_join_time+100;
    if(!strcmp(mode,"join_timeout")) up=false;
    if(!strcmp(mode,"join_rejected") && join_calls<2) up=false;
    if(!strcmp(mode,"link_lost") && now_ms>=1000) up=false;
    radio_status.link_up=radio_status.netif_link_up=up;
}
static int32_t tk_dly_tsk(uint32_t delay) {
    now_ms+=delay;
    if(now_ms>=60000) longjmp(done,1);
    return 0;
}
/* TASK */
int main(int argc, char **argv) {
    assert(argc==2); mode=argv[1];
    if(setjmp(done)==0) cyw43_utk_task(0,NULL);
    assert(setup_calls==1);
    if(!strcmp(mode,"init_failure")) {
        assert(radio_status.init_result==-CYW43_EIO && !radio_status.ready);
        assert(join_calls==0 && radio_polls==0 && mqtt_polls==0 && timers==0);
        puts("CONFIRMED LIMIT: init failure receives no retry or network polling over 60 simulated seconds");
        return 0;
    }
    assert(radio_polls>0 && radio_polls==mqtt_polls && timers==mqtt_polls);
    if(!strcmp(mode,"join_rejected")) {
        assert(join_calls==2 && join_times[1]-join_times[0]==5000 && radio_status.link_up);
        puts("PASS: rejected first join retries after 5 seconds and reaches link-up");
    } else if(!strcmp(mode,"join_timeout")) {
        assert(join_calls==2 && join_times[1]-join_times[0]==35000);
        puts("PASS: pending join times out after 30 seconds and retries after 5 more seconds");
    } else if(!strcmp(mode,"late_link")) {
        assert(join_calls==1 && radio_status.link_up);
        puts("PASS: late link-up suppresses a potentially disruptive retry");
    } else if(!strcmp(mode,"link_lost")) {
        assert(join_calls==1 && radio_status.join_complete && radio_status.join_result==0 && !radio_status.link_up);
        puts("CONFIRMED LIMIT: no application rejoin after successful association later drops; driver auto-rejoin not modeled");
    } else {
        assert(!strcmp(mode,"normal") && join_calls==1 && radio_status.link_up);
        puts("PASS: successful startup keeps radio, timeout and MQTT polling active without USB code");
    }
    return 0;
}
'''.replace('STATUS_HEADER', header).replace('/* TASK */', task)
c_path = build / 'owner_test.c'
c_path.write_text(harness)
exe = build / 'owner_test.exe'
subprocess.run([args.cc, '-std=c11', '-O2', str(c_path), '-o', str(exe)], check=True)
outputs = []
for case in ('normal', 'init_failure', 'join_rejected', 'join_timeout', 'late_link', 'link_lost'):
    result = subprocess.run([str(exe), case], capture_output=True, text=True, timeout=10)
    outputs.append(result.stdout + result.stderr)
    print(outputs[-1], end='')
    if result.returncode:
        (build / 'results.log').write_text(''.join(outputs))
        raise SystemExit(result.returncode)
(build / 'results.log').write_text(''.join(outputs))
