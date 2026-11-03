/**
 * @file barcode_scan.h
 * @brief Turns the barcode IR sensor (GP26) into decoded Code 39 letters.
 *
 * Call barcode_scan_update() regularly (every few ms) with the sensor state.
 * It debounces the signal, measures how long each black bar and white gap lasts,
 * and feeds those runs to the Code 39 decoder (barcode39.h). Only the ratio of
 * bar widths matters, so the speed over the barcode does not need to be exact.
 *
 * Line following calls this automatically when built with -DLINE_BARCODE_DECODE=1
 * (then also compile barcode_scan.c and barcode39.c).
 */
#ifndef BARCODE_SCAN_H
#define BARCODE_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "barcode39.h"

/** An edge is accepted only after the new level has lasted this long (rejects glitches). */
#ifndef BARCODE_DEBOUNCE_US
#define BARCODE_DEBOUNCE_US     (8000u)     /* TUNE: must be well below a narrow bar's duration */
#endif
/** A half-read barcode is thrown away if no edge arrives for this long. */
#ifndef BARCODE_IDLE_RESET_US
#define BARCODE_IDLE_RESET_US   (1500000u)  /* TUNE */
#endif

void barcode_scan_init(void);

/** black = barcode sensor sees black. now_us = any free-running microsecond clock. */
void barcode_scan_update(bool black, uint32_t now_us);

/** Returns true once for each newly decoded barcode and copies it to *out. */
bool barcode_scan_take(barcode39_result_t *out);

/** Diagnostics for the serial test: failed-looking-barcode count and reason (B39_FAIL_*). */
uint32_t barcode_scan_fail_count(void);
uint8_t barcode_scan_last_fail(void);

/** Number of barcodes decoded since init. */
uint32_t barcode_scan_count(void);

/* Optional hooks. Weak no-op defaults; define a strong version to use them
 * (for example to print on serial). Called from barcode_scan_update(). */
void barcode_scan_on_decoded(const barcode39_result_t *p_result);
void barcode_scan_on_run(bool black, uint32_t width_us);

#endif /* BARCODE_SCAN_H */
