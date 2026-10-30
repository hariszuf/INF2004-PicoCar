# Full-test vehicle integration

`main.c` owns the autonomous vehicle controller and its μT-Kernel task. It
receives Buddy 5 obstacle requests, calls Buddy 2 motion primitives through
`vehicle_nav.c`, and starts or resumes Buddy 3 line following. The GPIO interrupt
dispatcher and Pico SDK compatibility headers live here because they are
shared by the encoder, ultrasonic echo, motor, and IR drivers.

Build from `build_make/` with `picocar_integration.mk`, `SMP=0`, and `WIFI=none`.
The integration uses Buddy 1's local telemetry queue for USB diagnostics; it
does not require MQTT or a network connection. The normal and headless UF2
targets remain separate. This is an integration profile, not a replacement for
any buddy's standalone build.

