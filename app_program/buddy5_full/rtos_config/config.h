#ifndef B5_LINE_RTOS_CONFIG_H
#define B5_LINE_RTOS_CONFIG_H
#include "../../../config/config.h"
/* Only selected by buddy5_line.mk; the team config file is not edited. */
#undef CNF_TIMER_PERIOD
#define CNF_TIMER_PERIOD 1
#undef USE_SDEV_DRV
#define USE_SDEV_DRV 0
#endif