/* Execute the actual SDK main loop, controls, IMU/observers and read-only
 * Buddy 2/3 processing on the existing motor/encoder simulation. No hardware,
 * USB device, COM port or real motor API is accessed. Transport below is a
 * bounded/drop-output model, not a claim to emulate TinyUSB or electrical
 * faults. */
#include "../../barrc_types.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_hw.h"
#include "host_platform.h"
static int32_t capture_printf (const char * p_format, ...);
#define printf capture_printf
#define main actual_sdk_main
#include "main.c"
#undef main
#undef printf

static jmp_buf s_finished;
static int32_t s_scenario;
static bool s_stop_received;
static bool s_driven;
static bool s_seen_error;
static uint32_t s_key_index;
static uint32_t s_rows_before;
static uint32_t s_rows_after;
static uint32_t s_invalid_rows;
static uint32_t s_header_count;
static uint32_t s_attempted;
static uint32_t s_dropped;
static uint32_t s_received_after_resume;
static uint64_t s_previous_seq;
static uint64_t s_last_row_us;
static uint64_t s_first_stop_us;
static bool s_have_seq;
static i2c_inst_t s_fake_sensor;
i2c_inst_t * const i2c0 = &s_fake_sensor;

bool stdio_init_all (void)
{
    return true;
}

uint64_t time_us_64 (void)
{
    return host_time_us();
}

bool stdio_usb_connected (void)
{
    return !((2 == s_scenario) && (host_time_us() >= 1500000)
             && (host_time_us() < 4000000));
}

int32_t getchar_timeout_us (uint32_t timeout)
{
    assert(0 == timeout);
    static const struct
    {
        uint64_t when;
        int32_t key;
    }

    commands[] = {
        {100000, 'M'}, {200000, 'R'}, {300000, 'G'}, {2200000, 'S'},
        {2400000, 'S'}, {3000000, 'G'}, {4500000, ' '}
    };

    if ((s_key_index < sizeof(commands) / sizeof(commands[0]))
        && (host_time_us() >= commands[s_key_index].when))
    {
        int32_t key = commands[s_key_index++].key;

        if (('S' == key) && (!s_stop_received))
        {
            s_stop_received = true;
            s_first_stop_us = host_time_us();
        }

        return key;
    }

    return PICO_ERROR_TIMEOUT;
}

void gpio_set_function (uint32_t pin, uint32_t function)
{
    assert(((16 == pin) || (17 == pin)) && (GPIO_FUNC_I2C == function));
}

void gpio_pull_up (uint32_t pin)
{
    assert((16 == pin) || (17 == pin));
}

void line_sensors_init (void)
{
}

uint8_t line_sensors_read (void)
{
    return LINE_SENSOR_LEFT | LINE_SENSOR_RIGHT;
}

uint32_t i2c_init (i2c_inst_t * p_bus, uint32_t baudrate)
{
    assert((p_bus == i2c0) && (100000 == baudrate));
    memset(p_bus, 0, sizeof(*p_bus));
    p_bus->reg[0x0f] = 0x49;
    p_bus->reg[0x27] = 8;
    /* Recorded healthy stationary vector. */
    const int16_t raw[] = {901, 456, -14521};

    for (uint32_t idx = 0; idx < 3; idx++)
    {
        p_bus->reg[0x28 + 2 * idx] = (uint16_t)raw[idx] & 255;
        p_bus->reg[0x29 + 2 * idx] = (uint16_t)raw[idx] >> 8;
    }

    return baudrate;
}

static bool io_error (void)
{
    return (3 == s_scenario) && (host_time_us() >= 3000000)
            && (host_time_us() < 4000000);
}

int32_t i2c_write_timeout_us (i2c_inst_t * p_bus, uint8_t address,
                              const uint8_t * p,
                          size_t n, bool b_no_stop, uint32_t timeout)
{
    assert((p_bus == i2c0) && (0x1d == address)
           && (BUDDY4_I2C_TIMEOUT_US == timeout));

    if (io_error())
    {
        return -1;
    }

    if (1 == n)
    {
        assert(b_no_stop);
        p_bus->pointer = p[0];
    }
    else
    {
        assert((2 == n) && (!b_no_stop));
        p_bus->reg[p[0]] = p[1];
    }

    return (int32_t)n;
}

int32_t i2c_read_timeout_us (i2c_inst_t * p_bus, uint8_t address, uint8_t * p,
                         size_t n, bool b_no_stop, uint32_t timeout)
{
    assert((p_bus == i2c0) && (0x1d == address) && (!b_no_stop)
           && (BUDDY4_I2C_TIMEOUT_US == timeout));

    if (io_error())
    {
        return -1;
    }

    for (size_t idx = 0; idx < n; idx++)
    {
        p[idx] = p_bus->reg[(p_bus->pointer & 0x7f) + idx];
    }

    return (int32_t)n;
}

static int32_t capture_printf (const char * p_format, ...)
{
    char text[4096];
    va_list ap;
    va_start(ap, p_format);
    int32_t count = vsnprintf(text, sizeof(text), p_format, ap);
    va_end(ap);
    assert((count >= 0) && ((size_t)count < sizeof(text)));
    s_attempted++;

    if ((1 == s_scenario) && (host_time_us() >= 1500000)
        && (host_time_us() < 4000000))
    {
        s_dropped++;
        sim_advance_ms(1); /* Model finite TX wait/drop while RX works. */
        return count;
    }

    if (!strncmp(text, "seq,", 4))
    {
        s_header_count++;
        return count;
    }

    if (strncmp(p_format, "%lu,%llu,", 9))
    {
        return count;
    }

    char * field[87];
    uint32_t n = 0;

    for (char * p = strtok(text, ","); p && (n < 87); p = strtok(NULL, ","))
    {
        field[n++] = p;
    }

    assert(86 == n);
    uint64_t seq = strtoull(field[0], NULL, 10),
    row_us = strtoull(field[1], NULL, 10);
    assert((!s_have_seq)
           || ((seq > s_previous_seq) && (row_us > s_last_row_us)));
    s_previous_seq = seq;
    s_last_row_us = row_us;
    s_have_seq = true;

    if (atoi(field[34]))
    {
        s_driven = true;
    }

    if (s_stop_received)
    {
        s_rows_after++;
        assert(!atoi(field[34]));
        assert((!control.running) && (!control.armed));

        if (row_us > s_first_stop_us + 20000)
        {
            assert(MOTION_STATE_ERROR == atoi(field[45]));
            assert(MOTION_FAULT_ESTOP == atoi(field[46]));
            s_seen_error = true;
        }

        if (!atoi(field[3]))
        {
            s_invalid_rows++;
        }
    }
    else
    {
        s_rows_before++;
    }

    if (row_us >= 4000000)
    {
        s_received_after_resume++;
    }

    return count;
}

void tight_loop_contents (void)
{
    if (s_stop_received)
    {
        assert((!control.running) && (!control.armed));
        assert((sim_is_braked(MOTION_SIDE_LEFT))
               && (sim_is_braked(MOTION_SIDE_RIGHT)));
    }

    if (host_time_us() >= 6500000)
    {
        longjmp(s_finished, 1);
    }

    sim_advance_ms(1);
}

int main (int argc, char ** argv)
{
    assert(2 == argc);
    s_scenario = atoi(argv[1]);
    assert((s_scenario >= 0) && (s_scenario <= 3));

    if (!setjmp(s_finished))
    {
        actual_sdk_main();
    }

    assert(s_driven && s_stop_received && s_seen_error);
    assert((s_rows_before > 20) && (s_rows_after > 100)
           && (s_received_after_resume > 100));
    assert((s_last_row_us > 6400000) && (sim_is_braked(MOTION_SIDE_LEFT))
           && (sim_is_braked(MOTION_SIDE_RIGHT)));
    assert(s_header_count == (2 == s_scenario ? 2u : 1u));

    if (1 == s_scenario)
    {
        assert((s_dropped > 50)
               && (s_attempted > s_rows_before + s_rows_after));
    }

    if (3 == s_scenario)
    {
        assert(s_invalid_rows > 20);
    }

    printf("Actual SDK main-loop scenario %d: %u received rows before S, %u "
           "after S, %u dropped writes, motors remain latched stopped\n",
           s_scenario, s_rows_before, s_rows_after, s_dropped);
    return 0;
}

/*** end of file ***/
