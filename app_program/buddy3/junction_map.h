/*
 * junction_map - which way to go at a "+" junction, chosen by the barcode letter.
 *
 * Hardware independent, no RTOS. The letters repeat in groups of three:
 *   A = Left      B = Straight   C = Right
 *   D = Left      E = Straight   F = Right
 *   ... all the way to Z (Z = Straight)
 * i.e. (letter - 'A') % 3 : 0 = left, 1 = straight, 2 = right.
 */
#ifndef JUNCTION_MAP_H
#define JUNCTION_MAP_H

typedef enum
{
    JUNCTION_NONE = 0,     /* not a letter A-Z: no instruction */
    JUNCTION_LEFT,
    JUNCTION_STRAIGHT,
    JUNCTION_RIGHT,
    JUNCTION_UTURN
} junction_action_t;

/** Action for a barcode letter. Upper or lower case A-Z; anything else gives JUNCTION_NONE. */
junction_action_t junction_action_for_letter(char letter);

/** "LEFT", "STRAIGHT", "RIGHT" or "NONE", for logs. */
const char *junction_action_name(junction_action_t action);

#endif /* JUNCTION_MAP_H */
