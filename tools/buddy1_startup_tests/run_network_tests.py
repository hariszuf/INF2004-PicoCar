"""Build real lwIP + firmware MQTT against an in-memory Ethernet peer.

No sockets, USB, radio or physical hardware are used. The clock, Ethernet wire,
console and Buddy IPC endpoints are simulated; selected tests inject client
allocation or connect-call failure. DHCP/TCP/MQTT protocol code is real.
"""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--sdk', type=Path, required=True, help='Pico SDK containing lib/lwip')
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
args.repo = args.repo.resolve()
args.sdk = args.sdk.resolve()
root = Path(__file__).resolve().parent
build = root / 'build'
(build / 'tm').mkdir(parents=True, exist_ok=True)
(build / 'tk').mkdir(exist_ok=True)
lwip = args.sdk / 'lib/lwip/src'
options = (args.repo / 'lib/libnet/lwip/include/lwipopts.h').as_posix()
(build / 'lwipopts.h').write_text(f'#include "{options}"\n#undef MEM_ALIGNMENT\n#define MEM_ALIGNMENT 8\n')
(build / 'tm/tmonitor.h').write_text('''#pragma once
#include <stdint.h>
typedef unsigned char UB;
typedef int INT;
int tm_printf(const UB *format, ...);
int tm_sprintf(UB *buffer, const UB *format, ...);
''')
(build / 'tk/tkernel.h').write_text('''#pragma once
#include <stdint.h>
#include <stddef.h>
#define LOCAL static
#define E_OK 0
#define E_PAR -17
#define E_OBJ -41
#define E_TMOUT -50
#define TMO_FEVR -1
#define TMO_POL 0
#define TA_TFIFO 0
#define TA_HLNG 1
#define TA_RNG3 0
typedef unsigned char UB;
typedef uint32_t UW;
typedef int32_t INT, ER, ID;
typedef void (*FP)(INT, void *);
typedef struct {int itskpri, stksz; FP task; unsigned tskatr;} T_CTSK;
typedef struct {unsigned mbfatr, bufsz, maxmsz; void *bufptr;} T_CMBF;
ID tk_cre_mbf(const T_CMBF *);
ER tk_snd_mbf(ID, const void *, INT, INT);
INT tk_rcv_mbf(ID, void *, INT);
ID tk_cre_tsk(const T_CTSK *);
ER tk_sta_tsk(ID, INT);
''')
buddy_sources = [args.repo / 'app_program/buddy1' / name for name in ('command.c', 'telemetry.c')]
(build / 'firmware_buddy.c').write_text(''.join(f'#include "{p.as_posix()}"\n' for p in buddy_sources))
target = (args.repo / 'lib/libnet/lwip/mqtt_utk.c').as_posix()
(build / 'firmware_mqtt.c').write_text(f'#include "{target}"\n')
radio = (args.repo / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c').read_text()
dhcp_start = radio.rindex('static void copy_ip4(')
dhcp_end = radio.index('\n#endif', dhcp_start)
(build / 'firmware_dhcp.c').write_text(radio[dhcp_start:dhcp_end])
radio_header = (args.repo / 'lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h').as_posix()
(build / 'firmware_radio_status.h').write_text(f'#include "{radio_header}"\n')
sources = list((lwip / 'core').glob('*.c')) + list((lwip / 'core/ipv4').glob('*.c'))
sources += [lwip / 'netif/ethernet.c', lwip / 'apps/mqtt/mqtt.c', root / 'network_test.c', root / 'buddy_ipc_test.c']
defines = ['TM_WIFI_MQTT=1', 'TM_WIFI_DHCP=1', 'TM_WIFI_STATIC=0', 'TM_WIFI_TCP=0']
includes = [build, args.repo / 'config', lwip / 'include', args.sdk / 'src/rp2_common/pico_lwip/include']
exe = build / 'network_test.exe'
command = [args.cc, '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra']
command += ['-D' + d for d in defines] + ['-I' + str(p) for p in includes]
command += [str(p) for p in sources] + ['-o', str(exe)]
result = subprocess.run(command, capture_output=True, text=True)
(build / 'compile.log').write_text(result.stdout + result.stderr)
if result.returncode:
    print(result.stdout + result.stderr)
    raise SystemExit(result.returncode)
outputs = []
for case in ('dhcp', 'late_dhcp', 'late_dhcp_mqtt', 'dhcp_start_error', 'sync_error', 'alloc_error', 'no_arp', 'tcp_timeout', 'tcp_reset', 'connack_timeout', 'connack_refused', 'success', 'buddy_io'):
    result = subprocess.run([str(exe), case], capture_output=True, text=True)
    outputs.append(result.stdout + result.stderr)
    print(outputs[-1], end='')
    if result.returncode:
        (build / 'results.log').write_text(''.join(outputs))
        raise SystemExit(result.returncode)
(build / 'results.log').write_text(''.join(outputs))
