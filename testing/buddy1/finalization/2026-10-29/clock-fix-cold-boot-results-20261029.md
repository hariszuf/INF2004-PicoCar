# Repeated power-bank cold boots — 29 October 2026

Firmware: buddy1-clock-fix-no-usb.uf2, SHA256
`ed1ceeaeb06e8bca56ce19031d9796f7db68eada4899388c2552d95f47f3b893`.

Local source commit: `0f07d6df448da5ca5f581cf4cb89f229c51ba475`.

The user performs the power cycles, using the same power bank without
BOOTSEL, motors disabled and no wall charger. The assistant reads MQTT,
dashboard state and broker sockets. Physical power-on instants are not
independently measured. The continuous subscriber log is
`clock-fix-cold-boots-20261029.log`; it records received MQTT traffic, not broker
CONNECT/CONNACK messages.

| Repetition | Result | Evidence |
|---|---|---|
| 1 | PASS | User confirmed reconnection; telemetry restarted at seq=1 at 15:08:14 +08:00. Car port changed 55252 → 59555. Heartbeats and telemetry progressed; dashboard freshness true in all three verification readings. |
| 2 | PASS | User confirmed reconnection; new car port 53525 replaces 59555. Heartbeat restarted at seq=0 at 15:11:16 and reached seq=1 at 15:11:22. Freshness true after connection; telemetry advanced 22 → 28. |
| 3 | PASS | User confirmed reconnection; telemetry restarted at seq=1 at 15:12:37. Car port changed 53525 → 63041. Heartbeats 0/1/2/3/4 arrived at 15:12:42/:47/:52/:57/15:13:02; telemetry and dashboard freshness confirmed. |

Boot 1 verification snapshots, 15:08:47–15:09:00:
heartbeat 5 → 6 → 8, telemetry 49 → 56 → 62, heartbeat ages 3.2/4.9/1.0 s,
same established port 59555 throughout. Intermediate heartbeat messages are
present in the continuous log. Snapshot polling can skip a sequence without
meaning that the publication was lost.

Raw evidence: `clock-fix-cold-boot-1-verified.json`. The earlier
`clock-fix-cold-boot-1.json` was sampled just before reconnection completed and
still shows cached previous-run messages; it is not the passing result.

Boot 2 snapshots are in `clock-fix-cold-boot-2-verified.json`. Its first
reading at 15:11:10 still contains cached previous-run values while the car
is reconnecting. The 15:11:16 and 15:11:22 readings show the new established
connection and fresh status. This normal transition is not counted as a
post-connection heartbeat failure. A final, separate subscriber capture for
boot 3 is `clock-fix-cold-boot-3-mqtt-20261029.log`.

Boot 3 snapshots are in `clock-fix-cold-boot-3-verified.json`. The first,
15:12:36, still shows the cached previous-run messages during reconnection.
The next three readings, 15:12:43–15:12:55, show heartbeat 0 → 1 → 2,
telemetry 21 → 27 → 33, heartbeat ages 0.2/1.4/2.6 s and the same new
established port 63041. The subscriber log independently records the
five-second heartbeat cadence.

**Outcome: all three user-performed power-bank cold boots passed fresh
connection, advancing telemetry and heartbeat recovery checks.** Exact startup
latency was not independently measured. Firmware, local commit, successful
publishing log and these repetition results are saved in
`known-good/2026-10-29-clock-fix/`.

These checks assess repeated connection and heartbeat recovery. They do not
constitute motor tests, a long-duration network stress test or a precise boot
latency benchmark. Startup telemetry backlog may be sent in a burst and may
contain queue-related sequence gaps; that is recorded separately from heartbeat
cadence and is not silently treated as loss-free telemetry.
