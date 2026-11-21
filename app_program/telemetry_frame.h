/*
 * telemetry_frame - compact binary MQTT frames (car <-> dashboard).
 *
 * Replaces the text lines ("src=2 type=3 seq=1234 left_speed_mm_s=100 ... flags=0") with a few bytes. Plain C, no heap,
 * no hardware access: the firmware, the host tests and the Python dashboard backend (dashboard/telemetry_frame.py)
 * all implement the same layout.
 *
 * Telemetry frame (car -> dashboard), 5 to 15 bytes:
 *   byte 0   bit 7     1 = binary frame (text payloads are ASCII, bit 7 is always 0)
 *            bits 6..4 source (0..7)
 *            bits 3..2 size class of value 1: 0 = value is 0 (no bytes), 1 = int8, 2 = int16, 3 = int32
 *            bits 1..0 size class of value 2, same classes
 *   byte 1   bits 5..0 type (0..63)
 *            bit 6     1 = a 16-bit flags field follows the sequence
 *            bit 7     reserved, 0
 *   bytes 2..4         sequence, low 24 bits, little-endian
 *   [2 bytes]          flags, little-endian, only if byte 1 bit 6 is set
 *   [0/1/2/4 bytes]    value 1, little-endian two's complement, width from its size class
 *   [0/1/2/4 bytes]    value 2, same
 *
 * Command frame (dashboard -> car):
 *   byte 0   bit 7 = 1, bits 6..0 = opcode (TF_CMD_*)
 *   TF_CMD_MAP is followed by 10 bytes: 26 junction actions (A..Z) of 3 bits each, bit-packed (see bitpack.h),
 *   action values 0 none, 1 left, 2 straight, 3 right, 4 U-turn.
 */
#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdbool.h>
#include <stdint.h>
#include "bitpack.h"

#define TF_BINARY_MARK 0x80u
#define TF_TELEMETRY_MAX 15u
#define TF_SEQ_BITS 24u

#define TF_CMD_START 1u
#define TF_CMD_STOP 2u
#define TF_CMD_PAUSE 3u
#define TF_CMD_RESUME 4u
#define TF_CMD_MAP 5u
#define TF_CMD_MAPRESET 6u

#define TF_MAP_LETTERS 26u
#define TF_MAP_ACTION_BITS 3u
#define TF_MAP_BYTES BITPACK_BYTES(TF_MAP_LETTERS * TF_MAP_ACTION_BITS)
#define TF_CMD_MAP_LENGTH (1u + TF_MAP_BYTES)

typedef struct {
    uint8_t source, type;
    uint16_t flags;
    int32_t value_1, value_2;
    uint32_t sequence;   /* low 24 bits only */
} tf_telemetry_t;

static inline unsigned tf_size_class(int32_t v)
{
    if (v == 0) return 0u;
    if (v >= -128 && v <= 127) return 1u;
    if (v >= -32768 && v <= 32767) return 2u;
    return 3u;
}

static inline unsigned tf_class_bytes(unsigned cls) { return cls == 0u ? 0u : cls == 1u ? 1u : cls == 2u ? 2u : 4u; }

static inline unsigned tf_put_value(uint8_t *out, int32_t v, unsigned cls)
{
    unsigned n = tf_class_bytes(cls);
    uint32_t u = (uint32_t)v;
    for (unsigned i = 0u; i < n; i++) out[i] = (uint8_t)(u >> (8u * i));
    return n;
}

static inline int32_t tf_get_value(const uint8_t *in, unsigned cls)
{
    unsigned n = tf_class_bytes(cls);
    uint32_t u = 0u;
    for (unsigned i = 0u; i < n; i++) u |= (uint32_t)in[i] << (8u * i);
    if (n == 1u) return (int32_t)(int8_t)u;
    if (n == 2u) return (int32_t)(int16_t)u;
    return (int32_t)u;
}

/* Encode one telemetry message into out (at least TF_TELEMETRY_MAX bytes). Returns the frame length. */
static inline unsigned tf_encode_telemetry(uint8_t *out, uint8_t source, uint8_t type, uint16_t flags,
                                         int32_t value_1, int32_t value_2, uint32_t sequence)
{
    unsigned c1 = tf_size_class(value_1), c2 = tf_size_class(value_2);
    unsigned n = 5u;
    out[0] = (uint8_t)(TF_BINARY_MARK | ((source & 7u) << 4) | (c1 << 2) | c2);
    out[1] = (uint8_t)((type & 63u) | (flags != 0u ? 0x40u : 0u));
    out[2] = (uint8_t)sequence;
    out[3] = (uint8_t)(sequence >> 8);
    out[4] = (uint8_t)(sequence >> 16);
    if (flags != 0u) { out[n++] = (uint8_t)flags; out[n++] = (uint8_t)(flags >> 8); }
    n += tf_put_value(out + n, value_1, c1);
    n += tf_put_value(out + n, value_2, c2);
    return n;
}

/* Decode a telemetry frame. Returns false if it is not a well-formed frame (wrong marker or wrong length). */
static inline bool tf_decode_telemetry(const uint8_t *in, unsigned len, tf_telemetry_t *out)
{
    if (len < 5u || (in[0] & TF_BINARY_MARK) == 0u || (in[1] & 0x80u) != 0u) return false;
    unsigned c1 = (in[0] >> 2) & 3u, c2 = in[0] & 3u;
    bool has_flags = (in[1] & 0x40u) != 0u;
    unsigned n = 5u + (has_flags ? 2u : 0u);
    if (len != n + tf_class_bytes(c1) + tf_class_bytes(c2)) return false;
    out->source = (uint8_t)((in[0] >> 4) & 7u);
    out->type = (uint8_t)(in[1] & 63u);
    out->sequence = (uint32_t)in[2] | ((uint32_t)in[3] << 8) | ((uint32_t)in[4] << 16);
    out->flags = has_flags ? (uint16_t)(in[5] | (in[6] << 8)) : 0u;
    out->value_1 = tf_get_value(in + n, c1);
    out->value_2 = tf_get_value(in + n + tf_class_bytes(c1), c2);
    return true;
}

/* Command: opcode only. Returns the frame length (1). */
static inline unsigned tf_encode_command(uint8_t *out, unsigned opcode)
{
    out[0] = (uint8_t)(TF_BINARY_MARK | (opcode & 0x7Fu));
    return 1u;
}

/* Command: set the barcode map. codes = 26 characters L, S, R, U or N (upper case) for A..Z. Returns TF_CMD_MAP_LENGTH,
 * or 0 if a character is not one of them. */
static inline unsigned tf_encode_map(uint8_t *out, const char *codes)
{
    out[0] = (uint8_t)(TF_BINARY_MARK | TF_CMD_MAP);
    for (unsigned i = 0u; i < TF_MAP_BYTES; i++) out[1u + i] = 0u;
    for (unsigned i = 0u; i < TF_MAP_LETTERS; i++) {
        unsigned a;
        switch (codes[i]) {
        case 'N': a = 0u; break;
        case 'L': a = 1u; break;
        case 'S': a = 2u; break;
        case 'R': a = 3u; break;
        case 'U': a = 4u; break;
        default: return 0u;
        }
        bitpack_put(out + 1, i * TF_MAP_ACTION_BITS, TF_MAP_ACTION_BITS, a);
    }
    return TF_CMD_MAP_LENGTH;
}

/* Decode the 10 bytes after the TF_CMD_MAP opcode into 26 characters plus a terminating NUL (out has 27 bytes).
 * Returns false if an action value is above 4. */
static inline bool tf_decode_map(const uint8_t *map_bytes, char *out)
{
    static const char letters[5] = { 'N', 'L', 'S', 'R', 'U' };
    for (unsigned i = 0u; i < TF_MAP_LETTERS; i++) {
        uint32_t a = bitpack_get(map_bytes, i * TF_MAP_ACTION_BITS, TF_MAP_ACTION_BITS);
        if (a > 4u) return false;
        out[i] = letters[a];
    }
    out[TF_MAP_LETTERS] = '\0';
    return true;
}

#endif /* TELEMETRY_FRAME_H */
