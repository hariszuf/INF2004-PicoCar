#ifndef BUDDY5_H
#define BUDDY5_H

#include <tk/tkernel.h>

/* Phase 1: Initialize the ultrasonic sensor */
ER buddy5_init(void); // Prepare GPIO and ultrasonic sensor for operation

/* Phase 2: Start the ultrasonic RTOS task */
ER buddy5_start(void); // Create and start the RTOS task

#endif