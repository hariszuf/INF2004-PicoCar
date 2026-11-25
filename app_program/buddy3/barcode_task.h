#ifndef BUDDY3_BARCODE_TASK_H
#define BUDDY3_BARCODE_TASK_H
#include <tk/tkernel.h>
#include <stdbool.h>
#include <stdint.h>
ER buddy3_barcode_start (void);
bool buddy3_barcode_latest (char * letter, uint32_t * p_read_ms);
#endif

/*** end of file ***/
