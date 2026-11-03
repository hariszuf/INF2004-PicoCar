"""Exercise the real trial readable-output helpers with captured host output."""
from pathlib import Path
import subprocess

trial = Path(__file__).resolve().parent
source = trial / 'build/source'
out = trial / 'build/peak_console'
out.mkdir(parents=True, exist_ok=True)
app = (source / 'app_program/app_main.c').read_text(encoding='utf-8-sig')
begin = app.index('static void buddy4_decimal(')
helpers = app[begin:app.index('\n#endif', begin)]
core = (source / 'app_program/buddy4/buddy4_imu.c').read_text(encoding='utf-8-sig')
labels = core[core.index('const char *buddy4_terrain_name'):core.index('static void finish_side_event')]
labels += core[core.index('const char *buddy4_quality_name'):core.index('static int classify_angle')]
code = r'''
#define BUDDY4_MTK_HOST_TEST
#include "buddy4_task.h"
#include "buddy4_imu.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static char captured[16384];
static INT tm_printf(const UB *format, ...) {
    size_t used=strlen(captured);
    va_list ap;va_start(ap,format);
    int n=vsnprintf(captured+used,sizeof(captured)-used,(const char*)format,ap);
    va_end(ap);assert(n>=0&&(size_t)n<sizeof(captured)-used);return n;
}
''' + labels + helpers + r'''
int main(void) {
    buddy4_task_snapshot_t s={0}, before;
    char value[16];
    buddy4_decimal(value,-0.14f);assert(strcmp(value,"-0.14")==0);
    buddy4_decimal(value,NAN);assert(strcmp(value,"N/A")==0);
    s.valid=true;s.tilt_valid=true;s.measurement_quality=1;s.pitch_deg=-0.14f;s.roll_deg=0.08f;s.height_cm=0.02f;
    before=s;buddy4_print_readable(&s);
    assert(memcmp(&before,&s,sizeof(s))==0);
    assert(strstr(captured,"[B4] NOW")&&strstr(captured,"Pitch: -0.14 deg"));
    assert(strstr(captured,"B4-MEASUREMENT-QUALITY-I USB-only"));
    assert(strstr(captured,"Possible-impact detector NOW: INVALID"));
    assert(strstr(captured,"Saved cumulative possible-impact count: 0"));
    assert(!strstr(captured,"B4 RESULT")&&!strstr(captured,"Previous tilt"));
    captured[0]=0;s.completed_events=1;s.last_event.side=-1;
    s.last_event.end_us=20000000;s.last_event.stable_valid=true;
    s.last_event.stable_cm=0.98f;
    buddy4_print_readable(&s);
    assert(strstr(captured,"Completed side-tilt #1 | Side: LEFT_WHEEL_UP"));
    assert(strstr(captured,"Ended at: 20 s since boot"));
    assert(strstr(captured,"SAVED RESULT, not current tilt"));
    assert(strstr(captured,"OBSERVATIONAL TILT, NOT CONFIRMED HUMP CROSSING"));
    assert(strstr(captured,"Stable peak (cm): 0.98 | Median peak candidate (cm): UNCONFIRMED"));
    captured[0]=0;buddy4_print_readable(&s);assert(!strstr(captured,"B4 RESULT"));
    /* A count change reports retained history even when the one-sample pulse is
       already gone and current detector data is invalid. */
    captured[0]=0;s.impact_valid=false;s.possible_impact=false;
    s.impact_count=1;s.impact_time_us=12345678;
    before=s;buddy4_print_readable(&s);
    assert(memcmp(&before,&s,sizeof(s))==0);
    assert(strstr(captured,"Possible-impact detector NOW: INVALID"));
    assert(strstr(captured,"Saved cumulative possible-impact count: 1"));
    assert(strstr(captured,"Saved latest possible-impact time: 12.345 s"));
    assert(strstr(captured,"[B4 POSSIBLE IMPACT] Observational event #1"));
    assert(strstr(captured,"Retained event time: 12.345 s"));
    assert(strstr(captured,"SAVED EVENT, NOT CURRENT DATA, NOT CONFIRMED COLLISION"));
    captured[0]=0;s.possible_impact=true;buddy4_print_readable(&s);
    assert(!strstr(captured,"[B4 POSSIBLE IMPACT]")); /* Pulse is not the trigger. */
    captured[0]=0;s.impact_valid=true;s.possible_impact=false;
    s.impact_count=4;s.impact_time_us=9876543;buddy4_print_readable(&s);
    assert(strstr(captured,"Possible-impact detector NOW: VALID"));
    assert(strstr(captured,"2 intermediate possible-impact event timestamp(s) not retained"));
    assert(strstr(captured,"Observational event #4"));
    assert(strstr(captured,"Retained event time: 9.876 s"));
    captured[0]=0;buddy4_print_readable(&s);
    assert(!strstr(captured,"[B4 POSSIBLE IMPACT]"));
    captured[0]=0;s.valid=false;s.pitch_deg=123.45f;s.completed_events=2;
    s.last_event.stable_valid=false;s.last_event.median_valid=false;
    buddy4_print_readable(&s);
    assert(strstr(captured,"IMU: INVALID")&&!strstr(captured,"123.45"));
    assert(strstr(captured,"Stable peak (cm): UNCONFIRMED | Median peak candidate (cm): UNCONFIRMED"));
    captured[0]=0;s.valid=true;s.completed_events=3;s.last_event.side=1;
    s.last_event.stable_valid=s.last_event.median_valid=true;
    s.last_event.stable_cm=1.02f;s.last_event.median_cm=1.04f;
    before=s;buddy4_print_readable(&s);
    assert(memcmp(&before,&s,sizeof(s))==0);
    assert(strstr(captured,"RIGHT_WHEEL_UP")&&strstr(captured,"1.02")&&strstr(captured,"1.04"));
    assert(strstr(captured,"NOT CONFIRMED HUMP CROSSING"));
    captured[0]=0;s.completed_events=5;buddy4_print_readable(&s);
    assert(strstr(captured,"1 intermediate event(s) not captured"));
    captured[0]=0;buddy4_print_readable(&s);assert(!strstr(captured,"B4 RESULT"));
    captured[0]=0;s.valid=true;s.tilt_valid=false;s.measurement_quality=2;
    s.clipped_axes=7;s.clipped_samples=1;s.unsuitable_samples=1;
    s.raw[0]=s.raw[1]=s.raw[2]=-32768;s.magnitude_g=3.6249f;
    s.acceleration_g[0]=-2.06f;s.pitch_deg=-35.67f;s.height_cm=NAN;
    s.impact_valid=false;buddy4_print_readable(&s);
    assert(strstr(captured,"Current tilt/height: N/A")&&!strstr(captured,"Pitch: -35.67"));
    assert(strstr(captured,"Acquisition fresh=1 | Tilt usable=0 | Quality=CLIPPED"));
    assert(strstr(captured,"Raw XYZ=(-32768,-32768,-32768)"));
    assert(strstr(captured,"Magnitude=3.62 g")&&strstr(captured,"UNKNOWN_CLIPPED"));
    assert(strstr(captured,"Saved clipped samples=1 (impact assessment indeterminate)"));
    assert(strstr(captured,"Sensor readback:"));
    captured[0]=0;s.clipped_axes=0;s.tilt_valid=true;s.measurement_quality=1;
    buddy4_print_readable(&s);
    assert(strstr(captured,"Saved clipped samples=1 (impact assessment indeterminate)"));
    puts("Tilt/peak/possible-impact console checks passed: current validity vs saved history, count-triggered event output, missed-detail warning, pulse independence and read-only snapshots.");
}
'''
cfile=out/'test_peak_console.c'
exe=out/'test_peak_console.exe'
cfile.write_text(code)
subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-UNDEBUG',
                '-I'+str(source/'app_program/buddy4'),str(cfile),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
