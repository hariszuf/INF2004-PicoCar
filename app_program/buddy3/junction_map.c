#include "junction_map.h"

static uint8_t s_map[JUNCTION_MAP_LETTERS];
static uint16_t s_version = 0u;
static uint8_t s_ready = 0u;

static void load_defaults(void)
{
    unsigned i;

    for (i = 0u; i < JUNCTION_MAP_LETTERS; i++)
    {
        s_map[i] = (uint8_t)JUNCTION_NONE;   /* any other letter: no instruction (the line follower then goes straight) */
    }
    s_map['A' - 'A'] = (uint8_t)JUNCTION_LEFT;
    s_map['F' - 'A'] = (uint8_t)JUNCTION_UTURN;
    s_map['W' - 'A'] = (uint8_t)JUNCTION_RIGHT;
    s_ready = 1u;
}

junction_action_t junction_action_for_letter(char letter)
{
    if (!s_ready)
    {
        load_defaults();
    }

    if ((letter >= 'a') && (letter <= 'z'))
    {
        letter = (char)(letter - 'a' + 'A');
    }

    if ((letter < 'A') || (letter > 'Z'))
    {
        return JUNCTION_NONE;
    }

    return (junction_action_t)s_map[letter - 'A'];
}

void junction_map_reset(void)
{
    load_defaults();
    s_version++;
}

int junction_map_apply(const char *codes, unsigned length)
{
    uint8_t next[JUNCTION_MAP_LETTERS];
    unsigned i;

    if ((codes == 0) || (length != JUNCTION_MAP_LETTERS))
    {
        return -1;
    }

    for (i = 0u; i < JUNCTION_MAP_LETTERS; i++)
    {
        char c = codes[i];

        if ((c >= 'a') && (c <= 'z'))
        {
            c = (char)(c - 'a' + 'A');
        }

        switch (c)
        {
        case 'L': next[i] = (uint8_t)JUNCTION_LEFT; break;
        case 'S': next[i] = (uint8_t)JUNCTION_STRAIGHT; break;
        case 'R': next[i] = (uint8_t)JUNCTION_RIGHT; break;
        case 'U': next[i] = (uint8_t)JUNCTION_UTURN; break;
        case 'N': next[i] = (uint8_t)JUNCTION_NONE; break;
        default: return -1;
        }
    }

    for (i = 0u; i < JUNCTION_MAP_LETTERS; i++)
    {
        s_map[i] = next[i];
    }
    s_ready = 1u;
    s_version++;
    return 0;
}

void junction_map_pack(int32_t out[3])
{
    unsigned i;

    if (!s_ready)
    {
        load_defaults();
    }

    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    for (i = 0u; i < JUNCTION_MAP_LETTERS; i++)
    {
        out[i / 10u] |= (int32_t)((uint32_t)s_map[i] << (3u * (i % 10u)));
    }
}

uint16_t junction_map_version(void)
{
    return s_version;
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
