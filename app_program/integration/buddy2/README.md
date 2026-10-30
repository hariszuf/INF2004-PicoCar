# Buddy 2 interface

The integration build compiles `motion_control.c`, `motion_pid.c`,
`motion_motor.c`, and `motion_encoder.c` directly from `app_program/buddy2/`.
The vehicle coordinator in `../shared/vehicle_nav.c` calls these motion APIs.
The shared compatibility headers let the selected Pico SDK-facing files run
in this μT-Kernel profile without replacing Buddy 2's original sources.

