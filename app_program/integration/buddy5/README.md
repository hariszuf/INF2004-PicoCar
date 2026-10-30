# Buddy 5 obstacle scanning integration

This folder owns the HC-SR04+ echo driver, SG90 servo driver, filtered scan
task, and obstacle profile state machine. `scan_link.h` in
`app_program/integration/shared/` is the request/feedback interface to the
vehicle controller. Buddy 5 does not issue motor commands directly.

The `buddy5_core.c` profile is provisional until it is tested on the full
course with the supplied obstacle. `TIGHT_COURSE=1` is a small-course
demonstration setting and must not be used to claim clearance for a 20 cm box.
