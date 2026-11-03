# SDK possible-obstruction hardware evidence, 2 November 2026

Originals remain at their supplied locations, unchanged:

- [Test 1, 15:42](../Pasted%20text%2820261102-074240%29.txt)
- [Test 2, 15:44](../Pasted%20text%2820261102-074448%29.txt)

The [derived manifest](foam_success_manifest.json) records exact bytes, SHA-256,
row counts, event identities and timing. Reproduce its checks without modifying
originals: `python app_program/buddy4/evidence/verify_foam_success.py`.

| Recorded observation | Test 1 | Test 2 |
| --- | ---: | ---: |
| Complete 86-column rows | 722 | 669 |
| Rejected numeric fragments | 5 | 2 |
| Driving rows | 178 | 169 |
| GRAVITY_LIKE rows | 722 | 669 |
| I2C error count | 0 throughout | 0 throughout |
| First event host timestamp | 15:42:25.084 | 15:44:36.218 |
| Firmware event observation time, us | 9,160,237 | 10,880,237 |
| Obstruction count | 0 -> 1 | 0 -> 1 |
| Event-flag rows / distinct events | 2 / 1 | 2 / 1 |
| Slowdown span reported at event | 116 ms | 116 ms |
| Wheel speeds at event, left/right cm/s | 0.007 / 0.008 | 0.004 / 0.004 |
| Firmware SERIAL_STOP control time, us | 9,303,734 | 11,166,630 |
| Event to firmware SERIAL_STOP | 143.497 ms | 286.393 ms |
| Event to first stopped CSV row, firmware times | 161.651 ms | 301.681 ms |
| New possible-impact events | 0 | 0 |

Both events occur while running=1, manual forward targets remain 3/3 cm/s,
motor state is EXECUTING and motor/stop faults are zero. Earlier rows show
establishment. The subsequent stop is SERIAL_STOP, with ERROR/ESTOP afterward;
CSV continues and the saved event count stays one. The event itself does not
stop motors. A repeated event-flag row with the same event timestamp/count is
not a second event or a new encoder acquisition.

The operator explicitly confirms physical foam contact for **Test 1**. Test 2
is a second recorded possible-obstruction observation during the reported foam
test; contact details and contact instant are not independently verified.
Neither contact instant has an independent timestamp. Do not calculate physical
contact-to-detection latency or claim confirmed collision classification.

Timing distinctions matter: Test 1's host event-to-first-stopped-row interval is
160 ms (25.084 to 25.244), matching the approximate supplied description. Test
2's 328 ms interval is host event-to-**S-send annotation** (36.218 to 36.546),
not the firmware event-to-S-processing interval. The host annotations interleave
CSV fragments and can appear later than the corresponding received CONTROL
line. Use the firmware's monotonic timestamps for its event/stop ordering;
the log cannot measure serial transport or operator reaction latency reliably.
The event timestamp describes a qualified software observation, not a timestamp
of underlying encoder acquisition or physical contact.

The operator associates both recordings with the updated SDK firmware. No
binary hash is embedded in the recording. The currently verified SDK artifact
has SHA-256 `c5f3f5599d2b3cb19cf0ae07c87838e994b37ba5a4fd3a2e200cd19eaea67e73`.

These results support controlled encoder-assisted possible-obstruction behavior
for this setup. They do not establish general collision sensitivity, a zero
false-positive rate, numerical yaw accuracy, or final team MTK coexistence.
Earlier misses and abnormal IMU recordings remain preserved in this directory.
No firmware algorithm was changed to analyse these successful captures.
