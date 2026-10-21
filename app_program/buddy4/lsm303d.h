#ifndef BUDDY4_LSM303D_H
#define BUDDY4_LSM303D_H
#include <stdbool.h>

typedef struct { float x, y, z; } accel_g_t;

void lsm303d_bus_init(void);
bool lsm303d_init(void);
bool lsm303d_read_accel(accel_g_t *out);

#endif
