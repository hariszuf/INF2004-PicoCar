"""Compile the real console path with register/clock host adapters only.

Preserve formatting, nested DI/EI, sink selection and the UART polling loop.
FIFO/baud timing is a model, not a physical latency measurement.
"""
from pathlib import Path
import re
import subprocess

trial = Path(__file__).resolve().parent
source = trial / 'build/source'
out = trial / 'build/investigation_e/console'
out.mkdir(parents=True, exist_ok=True)
units = []
for name in ['lib/libtm/sysdepend/pico_rp2040/tm_com.c',
             'lib/libtm/libtm.c', 'lib/libtm/libtm_printf.c']:
    code = (source / name).read_text(encoding='utf-8-sig')
    # Substitute headers by the minimal host types below, not library logic.
    code = re.sub(r'^#include[^\n]*$', '', code, flags=re.M)
    if name.endswith('tm_com.c'):
        for register in ['DR', 'FR', 'IBRD', 'FBRD', 'LCR_H', 'CR']:
            adapter = {'DR': '(*uart_slot())', 'FR': 'uart_flags()'}.get(register, 'uart_config')
            code, count = re.subn(r'^#define UART_' + register + r'\s+[^\n]*$',
                                  '#define UART_' + register + ' ' + adapter, code, flags=re.M)
            assert count == 1, register
    units.append(code)

header = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdarg.h>
typedef uint8_t UB;
typedef uint32_t UW, UINT;
typedef int32_t W;
typedef int INT;
typedef int16_t H;
typedef uint16_t UH;
#define LOCAL static
#define EXPORT
#define IMPORT extern
#define PICO_RP2040 1
#define USE_TMONITOR 1
#define USE_TM_PRINTF 1
#define TM_CONSOLE_USB_CDC 1
#define TK_SUPPORT_SMP 0
#define TM_OUTBUF_SZ 0
#define CHR_CR 13
#define CHR_LF 10
#define CHR_ETX 3
static UW now_us, primask, mask_start, max_mask_us, next_drain=87;
static UW fifo_used=32, uart_bytes, uart_dr, uart_config;
static UB usb_data[512];
static unsigned usb_bytes;
static UW mask_interrupts(void) {
    UW old=primask; if(!old)mask_start=now_us; primask=1; return old;
}
static void restore_interrupts(UW old) {
    primask=old;
    if(!old && now_us-mask_start>max_mask_us)max_mask_us=now_us-mask_start;
}
#define DI(x) ((x)=mask_interrupts())
#define EI(x) restore_interrupts(x)
static UW uart_flags(void) {
    now_us++; /* Register reads consume modeled CPU time while FIFO drains. */
    while(now_us>=next_drain) { if(fifo_used)fifo_used--; next_drain+=87; }
    return fifo_used>=32 ? (1u<<5) : 0;
}
static UW *uart_slot(void) { assert(fifo_used<32); fifo_used++; uart_bytes++; return &uart_dr; }
static void tm_usb_console_put(const UB *p,INT n) {
    assert(usb_bytes+n<sizeof(usb_data)); memcpy(usb_data+usb_bytes,p,n); usb_bytes+=n;
    now_us++; /* Bounded copy; no host or UART wait. */
}
INT tm_putchar(INT c);
'''
fmt = '     IRQ trace (us from request): elapsed=%u last_irq=%u count=%u bits=0x%X done=%u entry_state=%u NVIC enabled=%u pending=%u\\n'
assert fmt in (source / 'app_program/app_main.c').read_text()
footer = r'''
int main(void) {
    tm_printf((UB *)"     IRQ trace (us from request): elapsed=%u last_irq=%u count=%u bits=0x%X done=%u entry_state=%u NVIC enabled=%u pending=%u\n",
        19500u,18900u,8u,0x200u,18900u,6u,1u,0u);
    const char *expected="     IRQ trace (us from request): elapsed=19500 last_irq=18900 count=8 bits=0x200 done=18900 entry_state=6 NVIC enabled=1 pending=0\r\n";
    assert(!primask && usb_bytes==strlen(expected) && !memcmp(usb_data,expected,usb_bytes));
    if(TM_CONSOLE_UART) { assert(uart_bytes==usb_bytes && max_mask_us>10000); }
    else { assert(uart_bytes==0 && max_mask_us<1000); }
    printf("Console UART=%d: identical %u USB bytes, modeled max IRQ mask=%u us\n",TM_CONSOLE_UART,usb_bytes,max_mask_us);
}
'''
generated = out / 'console.c'
generated.write_text(header + '\n'.join(units) + footer)
mk = (source / 'build_make/pico_rp2040.mk').read_text()
assert '-DTM_CONSOLE_USB_CDC=1 -DTM_CONSOLE_UART=0' in mk
for uart in (1, 0):
    exe = out / f'console_uart{uart}.exe'
    subprocess.run(['gcc', '-std=c11', '-Wno-pointer-to-int-cast', '-UNDEBUG',
                    f'-DTM_CONSOLE_UART={uart}', str(generated), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
