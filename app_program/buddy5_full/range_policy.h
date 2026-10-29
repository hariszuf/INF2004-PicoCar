#ifndef B5_RANGE_POLICY_H
#define B5_RANGE_POLICY_H
#include "buddy5_core.h"
#include "../buddy5/ultrasonic.h"
/* The sensor is mounted above the chassis.  Echoes below this range are from
 * the car/mounting rather than an obstacle that can be avoided at 200 mm. */
#define B5_SELF_ECHO_DEAD_ZONE_MM 90u
/* Prototype assumption: timeout can mean an open scene, not a measured wall.
 * Keep that explicit, and require three such observations before accepting it.
 * A disconnected sensor cannot be distinguished from open space by this test. */
static inline b5_range_t b5_range_from_raw(ultrasonic_result_t raw) {
 if(raw.status==US_OK) {
  if(raw.distance_mm<B5_SELF_ECHO_DEAD_ZONE_MM)
   return (b5_range_t){true,B5_NO_ECHO_MM};
  return (b5_range_t){true,raw.distance_mm};
 }
 if(raw.status==US_RISE_TIMEOUT || raw.status==US_FALL_TIMEOUT ||
    (raw.status==US_OUT_OF_RANGE && raw.echo_us>23200u))
  return (b5_range_t){true,B5_NO_ECHO_MM};
 return (b5_range_t){false,0};
}
#endif
