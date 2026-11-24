/*
 * bitpack - read and write fields of any width (1..32 bits) at any bit position inside a byte array.
 *
 * Bit 0 of the array is the least significant bit of byte 0 (little-endian bit order). Used for arrays of small
 * values (for example 13 scan slots of 13+2 bits) and for the binary telemetry frames. Plain loops, no heap, no
 * hardware access, so it compiles for the host tests and for the Pico.
 */
#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>

static inline uint32_t bitpack_get(const uint8_t *buf, unsigned bit, unsigned width)
{
    uint32_t value = 0u;
    for (unsigned i = 0u; i < width; i++) {
        unsigned b = bit + i;
        value |= (uint32_t)((buf[b >> 3] >> (b & 7u)) & 1u) << i;
    }
    return value;
}

static inline void bitpack_put(uint8_t *buf, unsigned bit, unsigned width, uint32_t value)
{
    for (unsigned i = 0u; i < width; i++) {
        unsigned b = bit + i;
        uint8_t mask = (uint8_t)(1u << (b & 7u));
        if ((value >> i) & 1u) buf[b >> 3] |= mask; else buf[b >> 3] &= (uint8_t)~mask;
    }
}

/* Bytes needed to hold the given number of bits. */
#define BITPACK_BYTES(bits) (((bits) + 7u) / 8u)

#endif /* BITPACK_H */
