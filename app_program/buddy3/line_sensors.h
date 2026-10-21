/**
 * @file line_sensors.h
 * @brief IR sensor input. Bits are by car side, not by pin.
 */
#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <stdint.h>

#define LINE_SENSOR_LEFT    (0x01u)   /* line sensor on the car's left */
#define LINE_SENSOR_RIGHT   (0x02u)   /* line sensor on the car's right */
#define LINE_SENSOR_BARCODE (0x04u)   /* barcode sensor (not used for steering) */

void line_sensors_init(void);

/** Bit mask of the sensors currently over black. */
uint8_t line_sensors_read(void);

#endif /* LINE_SENSORS_H */
