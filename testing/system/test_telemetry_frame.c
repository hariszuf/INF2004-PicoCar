/* Host test for telemetry_frame.h (binary MQTT frames). Prints the frames as hex so the Python decoder test can use
 * the exact same bytes (dashboard/test_app.py).
 *   gcc -std=c11 -Wall -Wextra -Iapp_program testing/system/test_telemetry_frame.c -o /tmp/tf && /tmp/tf */
#include <stdio.h>
#include <string.h>
#include "telemetry_frame.h"

static int s_failures;
#define CHECK(cond, text) do { if (cond) { printf("  PASS  %s\n", text); } \
    else { printf("  FAIL  %s\n", text); s_failures++; } } while (0)

static void hex(const char *label, const uint8_t *b, size_t n)
{
    printf("  FRAME %s ", label);
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

static bool roundtrip(uint8_t s, uint8_t t, uint16_t f, int32_t a, int32_t b, uint32_t q, size_t *len_out)
{
    uint8_t buf[TF_TELEMETRY_MAX];
    tf_telemetry_t d;
    size_t n = tf_encode_telemetry(buf, s, t, f, a, b, q);
    if (len_out) *len_out = n;
    return n <= TF_TELEMETRY_MAX && tf_decode_telemetry(buf, n, &d) && d.source == s && d.type == t && d.flags == f &&
           d.value_1 == a && d.value_2 == b && d.sequence == (q & 0xFFFFFFu);
}

int main(void)
{
    size_t n;
    uint8_t buf[TF_TELEMETRY_MAX];

    CHECK(roundtrip(2, 3, 0, 100, -100, 1234, &n) && n == 7u, "motor speeds: 7 bytes");
    CHECK(roundtrip(2, 3, 0, 0, 0, 1, &n) && n == 5u, "both values zero: 5 bytes (header + sequence)");
    CHECK(roundtrip(5, 51, 0, 189, 300, 42, &n) && n == 9u, "box width: 9 bytes");
    CHECK(roundtrip(5, 59, 1, 2147483647, -2147483647 - 1, 16777215u, &n) && n == 15u, "largest frame: 15 bytes");
    CHECK(roundtrip(6, 57, 65535, -1, 1, 0, &n), "source 6, flags 0xFFFF, negative value");
    CHECK(roundtrip(3, 6, 0, 65, 1500, 99, &n), "barcode letter A and age");

    int ok = 1;
    const int32_t samples[] = { 0, 1, -1, 127, -128, 128, -129, 32767, -32768, 32768, -32769, 2147483647, -2147483647 - 1 };
    for (unsigned i = 0; i < sizeof samples / sizeof samples[0] && ok; i++)
        for (unsigned j = 0; j < sizeof samples / sizeof samples[0] && ok; j++)
            if (!roundtrip(5, 56, (uint16_t)(i * 7u), samples[i], samples[j], 1000u + i, NULL)) ok = 0;
    CHECK(ok, "every pair of boundary values round-trips");

    ok = 1;
    for (uint32_t t = 0; t < 64u; t++) for (uint32_t s = 0; s < 8u; s++) if (!roundtrip((uint8_t)s, (uint8_t)t, 0, 5, 5, 7, NULL)) ok = 0;
    CHECK(ok, "all 8 sources and 64 types");

    tf_telemetry_t d;
    n = tf_encode_telemetry(buf, 2, 3, 0, 1, 2, 3);
    CHECK(!tf_decode_telemetry(buf, n - 1u, &d), "short frame is rejected");
    CHECK(!tf_decode_telemetry(buf, n + 1u, &d), "long frame is rejected");
    buf[0] &= 0x7Fu;
    CHECK(!tf_decode_telemetry(buf, n, &d), "text-like first byte is rejected");
    n = tf_encode_telemetry(buf, 2, 3, 0, 1, 2, 3);
    CHECK((buf[0] & 0x80u) != 0u, "binary frames always have bit 7 set (never ASCII)");
    n = tf_encode_telemetry(buf, 2, 3, 0, 100, -100, 1234); hex("2/3 speeds   ", buf, n);
    n = tf_encode_telemetry(buf, 5, 51, 0, 189, 300, 42); hex("5/51 width   ", buf, n);
    n = tf_encode_telemetry(buf, 5, 59, 1, 2147483647, -2147483647 - 1, 16777215u); hex("5/59 max     ", buf, n);

    /* Commands. */
    uint8_t cmd[TF_CMD_MAP_LENGTH];
    CHECK(tf_encode_command(cmd, TF_CMD_START) == 1u && cmd[0] == 0x81u, "START = 0x81");
    CHECK(tf_encode_command(cmd, TF_CMD_STOP) == 1u && cmd[0] == 0x82u, "STOP = 0x82");
    CHECK(tf_encode_command(cmd, TF_CMD_MAPRESET) == 1u && cmd[0] == 0x86u, "MAPRESET = 0x86");
    const char *map = "LNNNNUNNNNNNNNNNNNNNNNRNNN";
    char back[TF_MAP_LETTERS + 1];
    CHECK(tf_encode_map(cmd, map) == 11u, "MAP command is 11 bytes (was 30 characters of text)");
    CHECK(tf_decode_map(cmd + 1, back) && strcmp(back, map) == 0, "MAP round-trips (A left, F U-turn, W right)");
    hex("MAP default   ", cmd, 11u);
    CHECK(tf_encode_map(cmd, "LNNNNUNNNNNNNNNNNNNNNNRNNX") == 0u, "MAP with a bad letter is rejected");
    ok = 1;
    for (int k = 0; k < 5 && ok; k++) {
        char all[TF_MAP_LETTERS + 1];
        memset(all, "NLSRU"[k], TF_MAP_LETTERS); all[TF_MAP_LETTERS] = '\0';
        tf_encode_map(cmd, all);
        if (!tf_decode_map(cmd + 1, back) || strcmp(back, all) != 0) ok = 0;
    }
    CHECK(ok, "MAP with all 26 letters the same, for each action");
    memset(cmd + 1, 0xFF, TF_MAP_BYTES);
    CHECK(!tf_decode_map(cmd + 1, back), "MAP with an action value above 4 is rejected");

    printf("%s\n", s_failures == 0 ? "ALL PASSED" : "SOME FAILED");
    return s_failures == 0 ? 0 : 1;
}
