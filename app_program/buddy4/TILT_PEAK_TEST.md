# Buddy 4: friend tilt and peak test

Use the new B4-TILT-PEAK-POSSIBLE-IMPACT-H USB-only UF2, not an old file with the same name.
Motor power must remain disconnected. USB powers the Pico. This IMU-only MTK
image does not drive and has no M/R/G commands. Do not enable motors for it.

## Send back

Copy the serial monitor text, including the build marker, NOW, timing/error
lines and any B4 RESULT lines. Label each recording with what actually happened,
any measured reference height, and whether it was held or rolled. Keep full
text rather than screenshots of a few rows. Serial monitor does not save a file
automatically: paste into the supplied template or use its capture function.

## A. Communication and tilt (motor power disconnected)

1. Flash by BOOTSEL/copy UF2. Open the Pico USB serial port at 115200.
2. Confirm B4-TILT-PEAK-POSSIBLE-IMPACT-H and leave the robot naturally resting on a firm, flat
   surface, all normal supports contacting it. Record 15 seconds untouched.
3. Record separate front-up, front-down, left-wheel-up and right-wheel-up
   positions. Tilt the whole chassis enough to cross the detection threshold;
   keep the IMU fixed to the chassis. Hold each position steady for about
   5 seconds, then return to the original surface for 5 seconds. Label which
   physical side was raised. Handling tests verify label direction only.
4. Return all supports to the baseline and record 10 seconds. Save the results.

NOW describes the present state. A completed side event can print B4 RESULT
after returning toward level or changing sides. That is a saved result, not
the current tilt. Pitch-only tilt need not create a side event/height peak.
Labels have hysteresis, so they do not necessarily switch at the same angle
when entering versus leaving a slope.

## B. Optional stationary height check

Measure the support's actual vertical rise in cm. Raise one wheel with a stable
support, leaving the opposite wheel and caster naturally supported. Do not use
a hand to balance the chassis during the settled recording. Record 10 seconds
settled per side and the baseline before/after. A 1 cm rise can be near the
4-degree side-entry threshold, so a missing completed event is not a zero
height result. Compare settled NOW estimates, not a saved placement peak,
against the reference. Document contact/placement and surface slope.

## C. Peak preview without motor operation (optional)

If available, centre the smaller measured ramp in ONE tyre's path; the other
wheel stays on the lower surface. Record baseline, gently roll the chassis
continuously over it with motor power disconnected, then release it at rest
on the baseline. Do not lift or tilt it by hand to simulate wheel height.
Wait 5 seconds after the end for the result. Label it MANUALLY ROLLED and note
hand contact, side and reference height. Three previews per side are useful
if practical, but do not fabricate a result or force an unconfirmed peak.

These previews include external force and are not autonomous driving validation.
Actual unassisted crossing/height validation remains pending until the working
driving firmware/final motor integration is available. A stationary held tilt
also does not validate a moving peak. Do not flash the old SDK driving firmware
and expect the new MTK output: they are different applications.

## Reading the results

- B4 RESULT Stable peak / Median peak candidate: preliminary wheel-height
  difference for the completed side-tilt event, each with separate validity.
- UNCONFIRMED: estimator did not obtain its required fresh/stable sample window.
- Handling can create larger peaks than the true surface height. Identify it
  from the physical test label; a peak alone cannot prove a hump crossing.
- Multiple quick events can be overwritten between 1 Hz monitor reads. If the
  monitor reports missing intermediate events, repeat only that clearly labelled
  test with pauses; do not silently treat the latest result as all the events.
- If I2C errors/skipped slots rise or IMU becomes invalid, keep the text and
  report it before interpreting height accuracy. Do not adjust calibration.

Tests A/B/C provide different evidence. None alone closes all final course,
collision, yaw-rate or team-MTK validation tasks.
