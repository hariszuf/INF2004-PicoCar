"""Check the actual optional MTK CSV formatter, including invalid NaN masking."""
from pathlib import Path
import subprocess

trial = Path(__file__).resolve().parent
source = trial / 'build/source'
app = (source / 'app_program/app_main.c').read_text()
start = app.index('            tm_printf((UB *)"B4,%')
formatter = app[start:app.index('            #else', start)]
header = next(line for line in app.splitlines() if '# B4_MTK: seq' in line)
assert len(header.split('seq ', 1)[1].split(r'\n', 1)[0].split()) + 1 == 16
out = trial / 'build/quality_csv'
out.mkdir(exist_ok=True)
cfile, exe = out / 'csv.c', out / 'csv.exe'
cfile.write_text(r'''
#define BUDDY4_MTK_HOST_TEST
#include "buddy4_task.h"
#include <math.h>
#include <stdio.h>
#define tm_printf(format, ...) printf((const char *)(format), __VA_ARGS__)
static buddy4_task_snapshot_t buddy4_diagnostic;
static void row(void) {
''' + formatter + r'''
}
int main(void) {
 buddy4_diagnostic.valid=true;buddy4_diagnostic.tilt_valid=true;
 buddy4_diagnostic.pitch_deg=-0.14f;buddy4_diagnostic.roll_deg=6;
 buddy4_diagnostic.height_cm=1.18f;buddy4_diagnostic.acceleration_valid=true;
 buddy4_diagnostic.measurement_quality=1;row();
 buddy4_diagnostic.tilt_valid=false;buddy4_diagnostic.acceleration_valid=false;
 buddy4_diagnostic.pitch_deg=buddy4_diagnostic.roll_deg=buddy4_diagnostic.height_cm=NAN;
 buddy4_diagnostic.measurement_quality=2;buddy4_diagnostic.clipped_axes=7;
 buddy4_diagnostic.clipped_samples=buddy4_diagnostic.unsuitable_samples=1;row();
}
''')
subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                '-I' + str(source / 'app_program/buddy4'), str(cfile),
                '-o', str(exe)], check=True)
rows = [r.split(',') for r in subprocess.check_output([str(exe)], text=True).splitlines()]
assert len(rows) == 2 and all(len(r) == 17 for r in rows)
assert rows[0][4:7] == ['-140', '6000', '1180']
assert rows[1][4:7] == ['-2147483647'] * 3
assert rows[1][-6:] == ['0', '0', '2', '7', '1', '1']
print('MTK CSV: 17 fields, actual formatter, quality fields and NaN masking passed')
