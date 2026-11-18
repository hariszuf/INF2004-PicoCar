# Buddy 4 in the integrated driving image

The integrated build links the portable IMU, impact, motion and encoder-yaw
modules (IMU, lift/overturn detector, motion, encoder-yaw) from `app_program/buddy4/`. This folder contains the native μT-Kernel
task and device adapter adapted from Buddy 4's isolated prototype. The driver
uses I2C0 on GP16/GP17 and one exclusive `iica` owner. The controller supplies
fresh signed wheel speeds after each Buddy 2 update and forwards valid tilt to
Buddy 2 through `motion_set_terrain`. The task runs every 20 ms and publishes a
mutex-protected snapshot. An unavailable IMU does not prevent line following;
the controller then uses Buddy 2's flat-ground behavior.

IMU collision detection has been removed (it conflicts with obstacle
avoidance). Instead the IMU detects that the robot is lifted or overturned
(`buddy4_lift.c`): gravity tilted 20 degrees or more for 100 ms (hand lifts tilt the car about
25-30 degrees), upside-down, or a sustained 0.10 g acceleration for 100 ms. The
state latches until the robot is level (10 degrees or less) and still for 1 s. The vehicle controller stops all
motors immediately (halt reason 14, "robot lifted or overturned") and needs a
START after the robot is set down; lifting the robot ends the demonstration run.
Telemetry: source 4, type 20 (see `TELEMETRY_TYPE_LIFT`), shown in the
"Robot lift detection" section of the dashboard's Buddy 4 page. Thresholds are
provisional (`buddy4_lift_default_config`) and need tuning on the car.

The native obstruction observer and its command-context adapter are still
pending. Buddy 4's standalone and isolated test
applications remain separate from the single integrated `usermain`.

