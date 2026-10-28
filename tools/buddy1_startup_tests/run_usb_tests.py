"""Check the production USB console with a fake kernel and TinyUSB boundary.

This verifies finite output buffering and sleep requests, not RP2040 USB
hardware, interrupt behaviour, or actual scheduler fairness.
"""
import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
args.repo = args.repo.resolve()
root = Path(__file__).resolve().parent
build = root / 'build-usb'
(build / 'tk').mkdir(parents=True, exist_ok=True)
(build / 'sys').mkdir(exist_ok=True)
(build / 'tk/tkernel.h').write_text('''#pragma once
#include <stdint.h>
#include <stddef.h>
#define TK_SUPPORT_SMP 0
#define USE_TMONITOR 1
#define TM_CONSOLE_USB_CDC 1
#define IMPORT extern
#define EXPORT
#define LOCAL static
#define DI(x) ((x)=0)
#define EI(x) ((void)(x))
#define E_OK 0
#define TA_HLNG 1
#define TA_RNG0 0
typedef unsigned char UB;
typedef uint32_t UW, UINT;
typedef int32_t W, INT, ER, ID;
typedef void (*FP)(INT, void *);
typedef struct {void *exinf; unsigned tskatr, assprc; FP task;
    int itskpri, stksz; void *bufptr;} T_CTSK;
int tk_get_tid(void);
void tk_ext_tsk(void);
int tk_slp_tsk(int);
int tk_cre_tsk(T_CTSK *);
int tk_sta_tsk(int, int);
int tk_del_tsk(int);
''')
(build / 'tk/typedef.h').write_text('#include <tk/tkernel.h>\n')
(build / 'sys/sysdef.h').write_text('')
target = (args.repo / 'lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c').as_posix()
(build / 'firmware_usb.c').write_text(f'#include "{target}"\n')
exe = build / 'usb_test.exe'
result = subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', '-I' + str(build),
                         str(root / 'usb_test.c'), '-o', str(exe)], capture_output=True, text=True)
(build / 'compile.log').write_text(result.stdout + result.stderr)
if result.returncode:
    print(result.stdout + result.stderr)
    raise SystemExit(result.returncode)
outputs = []
for case in ('unmounted', 'mounted_blocked', 'event_burst', 'init_failure'):
    result = subprocess.run([str(exe), case], capture_output=True, text=True)
    outputs.append(result.stdout + result.stderr)
    print(outputs[-1], end='')
    if result.returncode:
        (build / 'results.log').write_text(''.join(outputs))
        raise SystemExit(result.returncode)
(build / 'results.log').write_text(''.join(outputs))
