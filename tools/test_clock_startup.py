"""Run production RP2040 clock startup against fake registers, not hardware."""
import argparse
from pathlib import Path
import re
import subprocess

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--repo', type=Path, required=True)
p.add_argument('--cc', default='gcc')
args = p.parse_args()
repo = args.repo.resolve()
source = (repo / 'kernel/sysdepend/pico_rp2040/cpu_clock.c').read_text()
body = source[source.index('LOCAL void init_pll'):source.index('EXPORT void shutdown_clock')]
headers = [repo / 'include/sys/sysdepend/cpu/rp2040/sysdef.h',
           repo / 'include/sys/sysdepend/pico_rp2040/sysdef.h',
           repo / 'kernel/sysdepend/pico_rp2040/sysdepend.h']
definitions = {}
for header in headers:
    for line in header.read_text().splitlines():
        m = re.match(r'#define\s+(\w+)\b', line)
        if m:
            definitions[m[1]] = line
needed = set(re.findall(r'\b[A-Z][A-Za-z_0-9]*\b', body)) & definitions.keys()
while True:
    expanded = needed | (set(re.findall(r'\b\w+\b', '\n'.join(definitions[n] for n in needed))) & definitions.keys())
    if expanded == needed:
        break
    needed = expanded
harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t UW, UINT, ATR;
typedef volatile uint32_t _UW;
typedef uint64_t UD;
#define LOCAL static
#define EXPORT
DEFINITIONS
static uint32_t registers[0x10000];
static unsigned tick_writes;
static unsigned index_of(uint32_t a) { return (a & 0x3ffffu) / 4u; }
static uint32_t in_w(uint32_t a) {
    if (a == XOSC_STATUS) return XOSC_STATUS_STABLE;
    if (a == PLL_SYS_BASE + PLL_CS || a == PLL_USB_BASE + PLL_CS) return PLL_CS_LOCK;
    if (a == CLK_REF + CLK_x_SELECTED) return 1u << (registers[index_of(CLK_REF + CLK_x_CTRL)] & CLK_REF_CTRL_SRC);
    if (a == CLK_SYS + CLK_x_SELECTED) return 1u << (registers[index_of(CLK_SYS + CLK_x_CTRL)] & CLK_SYS_CTRL_SRC);
    return registers[index_of(a)];
}
static void out_w(uint32_t a, uint32_t v) {
    if (a == 0x4005802cu) {
        assert(in_w(CLK_REF + CLK_x_SELECTED) == 4u);
        assert(in_w(CLK_REF + CLK_x_DIV) == 256u);
        assert(v == 0x20cu);
        tick_writes++;
    }
    registers[index_of(a)] = v;
}
static void set_w(uint32_t a, uint32_t v) { out_w(a, in_w(a) | v); }
static void clr_w(uint32_t a, uint32_t v) { out_w(a, in_w(a) & ~v); }
PRODUCTION
int main(void) {
    const uint32_t inherited_ticks[] = {0x200u, 0x20cu, 0xffffffffu};
    for (unsigned i = 0; i < 3; i++) {
        tick_writes = 0;
        registers[index_of(0x4005802cu)] = inherited_ticks[i];
        /* Prove startup doesn't reset the hardware time count. */
        registers[index_of(0x40054000u)] = 0x12345678u;
        registers[index_of(0x40054004u)] = 0x87654321u;
        registers[index_of(0x40058000u)] = 0u;
        startup_clock(CLKATR_USB | CLKATR_ADC | CLKATR_RTC | CLKATR_PREI);
        assert(tick_writes == 1);
        assert(in_w(0x4005802cu) == 0x20cu);
        assert(in_w(0x40054000u) == 0x12345678u);
        assert(in_w(0x40054004u) == 0x87654321u);
        assert(in_w(0x40058000u) == 0u); /* Watchdog reset counter remains off. */
    }
    puts("PASS clock startup: cold/inherited tick states -> 1 MHz after clk_ref setup; timer count and watchdog control preserved");
}
'''
harness = harness.replace('DEFINITIONS', '\n'.join(definitions[n] for n in sorted(needed))).replace('PRODUCTION', body)
build = Path(__file__).resolve().parent / 'build-clock'
build.mkdir(exist_ok=True)
test_source = build / 'clock_test.c'
test_source.write_text(harness)
exe = build / 'clock_test.exe'
subprocess.run([args.cc, '-std=c11', '-Wall', '-Wextra', str(test_source), '-o', str(exe)], check=True)
raise SystemExit(subprocess.run([str(exe)]).returncode)
