# Correctness and performance

## Stage 1: controller flags and fault mask
- Host test `testing/system/test_robot_flags.c`: 18 checks pass (set, clear, update, test-and-clear, balanced locking).
- Firmware builds (make exit code 0). Flash +224 B, RAM unchanged.
- On-car check (to record): STOP halts, START restarts, dashboard shows Running / Stopped.

## Stage 2: Buddy 5 structs
- Host test `testing/system/test_bitpack.c`: 9 checks pass (all widths 1..32, 13 scan slots independent, clamp).
- Behaviour unchanged: the host simulation (`testing/buddy5/new_logic/run_tests.sh 2`) and the scan-only run
  (`run_scanonly.sh`) print byte-for-byte identical output before and after the change (diff is empty).
- On-car check (to record): obstacle bypass and scan-only width reading.
