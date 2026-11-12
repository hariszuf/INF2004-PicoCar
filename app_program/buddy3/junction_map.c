#include "junction_map.h"

junction_action_t junction_action_for_letter(char letter)
{
    if ((letter >= 'a') && (letter <= 'z'))
    {
        letter = (char)(letter - 'a' + 'A');
    }

    if ((letter < 'A') || (letter > 'Z'))
    {
        return JUNCTION_NONE;
    }

    if (letter == 'F')
    {
        return JUNCTION_UTURN;
    }

    if (letter == 'A')
    {
        return JUNCTION_LEFT;
    }

    if (letter == 'W')
    {
        return JUNCTION_RIGHT;
    }

    return JUNCTION_NONE;
}

const char *junction_action_name(junction_action_t action)
{
    switch (action)
    {
    case JUNCTION_LEFT:
        return "LEFT";
    case JUNCTION_STRAIGHT:
        return "STRAIGHT";
    case JUNCTION_RIGHT:
        return "RIGHT";
    case JUNCTION_UTURN:
        return "UTURN";
    default:
        return "NONE";
    }
}