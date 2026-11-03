# Buddy 3 interface

The integration build compiles `line_follow.c` and `line_sensors.c` directly
from `app_program/buddy3/`. The vehicle coordinator starts or stops line
following, and Buddy 5 receives line reacquisition feedback through the
shared scan link. No duplicate Buddy 3 source files are kept here.

