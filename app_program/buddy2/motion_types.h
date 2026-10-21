/**
 * @file motion_types.h
 * @brief Types shared by the Motion Control modules (Buddy 2).
 */
#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

/** Wheel selector. */
typedef enum
{
    MOTION_SIDE_LEFT = 0,
    MOTION_SIDE_RIGHT = 1,
    MOTION_SIDE_COUNT = 2
} motion_side_t;

#endif /* MOTION_TYPES_H */
