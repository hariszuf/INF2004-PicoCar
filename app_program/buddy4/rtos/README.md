# Buddy 4 in the integrated driving image

The integrated build links the portable IMU, impact, motion and encoder-yaw
modules from `app_program/buddy4/`. This folder contains the native μT-Kernel
task and device adapter adapted from Buddy 4's isolated prototype. The driver
uses I2C0 on GP16/GP17 and one exclusive `iica` owner. The controller supplies
fresh signed wheel speeds after each Buddy 2 update and forwards valid tilt to
Buddy 2 through `motion_set_terrain`. The task runs every 20 ms and publishes a
mutex-protected snapshot. An unavailable IMU does not prevent line following;
the controller then uses Buddy 2's flat-ground behavior.

The native obstruction observer and its command-context adapter are still
pending. The integrated car does not use Buddy 4's possible-impact result as an
automatic motor stop. That event and the IMU fields need final telemetry/MQTT
ownership and hardware qualification. Buddy 4's standalone and isolated test
applications remain separate from the single integrated `usermain`.

