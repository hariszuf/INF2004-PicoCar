/* junction_map - see junction_map.h. */
#include "junction_map.h"

junction_action_t junction_action_for_letter(char letter)
{
    int index;

    if ((letter >= 'a') && (letter <= 'z'))
    {
        letter = (char)(letter - 'a' + 'A');
    }
    if ((letter < 'A') || (letter > 'Z'))
    {
        return JUNCTION_NONE;
    }
    index = (letter - 'A') % 3;
    if (index == 0)
    {
        return JUNCTION_LEFT;
    }
    if (index == 1)
    {
        return JUNCTION_STRAIGHT;
    }
    return JUNCTION_RIGHT;
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
    default:
        return "NONE";
    }
}
