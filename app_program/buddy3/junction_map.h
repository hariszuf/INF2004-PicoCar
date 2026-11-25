/*
 * junction_map - which way to go at a "+" junction, chosen by the barcode
 * letter.
 *
 * Hardware independent, no RTOS. The A-Z table starts with the built-in
 * defaults (A = left, F = U-turn,
 * W = right, every other letter = no instruction, so the line follower goes
 * straight) and can be replaced at run
 * time (dashboard -> MQTT "MAP" command), so the map announced on test day
 * needs no reflash.
 */
#ifndef JUNCTION_MAP_H
#define JUNCTION_MAP_H

#include "../barrc_types.h"
#include <stdint.h>

typedef enum
{
    JUNCTION_NONE = 0,     /* not a letter A-Z, or no instruction for it */
    JUNCTION_LEFT,
    JUNCTION_STRAIGHT,
    JUNCTION_RIGHT,
    JUNCTION_UTURN
} junction_action_t;

#define JUNCTION_MAP_LETTERS 26

/* Action for a barcode letter. Upper or lower case A-Z; anything else gives
 * JUNCTION_NONE. */
junction_action_t junction_action_for_letter (char letter);

/** "LEFT", "STRAIGHT", "RIGHT" or "NONE", for logs. */
const char * junction_action_name (junction_action_t action);


/** Restore the built-in defaults. */
void junction_map_reset (void);

/**
 * Replace the whole table. `codes` must hold exactly JUNCTION_MAP_LETTERS
 * characters for A..Z, each one of
 * L (left) S (straight) R (right) U (U-turn) N (no instruction), upper or lower
 * case.
 * Returns 0 on success; -1 if anything is wrong, in which case the table is
 * left unchanged.
 */
int32_t junction_map_apply (const char * p_codes, uint32_t length);

/* Packed table for telemetry: 3 bits per letter (the junction_action_t value),
 * letters A-J in out[0], K-T in out[1], U-Z in out[2]. */
void junction_map_pack (int32_t out[3]);

/* Counts every successful change (including reset); 0 = still the built-in
 * defaults. */
uint16_t junction_map_version (void);

#endif /* JUNCTION_MAP_H */

/*** end of file ***/
