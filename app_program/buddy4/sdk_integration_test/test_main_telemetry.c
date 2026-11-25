/* Execute the actual SDK main loop, controls, IMU/observers and read-only
 * Buddy 2/3 processing on the existing motor/encoder simulation. No hardware,
 * USB device, COM port or real motor API is accessed. Transport below is a
 * bounded/drop-output model, not a claim to emulate TinyUSB or electrical
 * faults. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_hw.h"
#include "host_platform.h"
static int capture_printf (const char * format, ...);
#define printf capture_printf
#define main actual_sdk_main
#include "main.c"
#undef main
#undef printf

static jmp_buf finished;
static int scenario;
static bool stop_received;
static bool driven;
static bool seen_error;
static uint32_t key_index;
static uint32_t rows_before;
static uint32_t rows_after;
static uint32_t invalid_rows;
static uint32_t header_count;
static uint32_t attempted;
static uint32_t dropped;
static uint32_t received_after_resume;
static uint64_t previous_seq;
static uint64_t last_row_us;
static uint64_t first_stop_us;
static bool have_seq;
static i2c_inst_t fake_sensor;
i2c_inst_t * const i2c0 = &fake_sensor;

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
    return !((2 == scenario) && (host_time_us() >= 1500000)
             && (host_time_us() < 4000000));
}

int getchar_timeout_us (uint32_t timeout)
{
    assert(0 == timeout);
    static const struct
    {
        uint64_t when;
        int key;
    }

    commands[] = {
        {100000, 'M'}, {200000, 'R'}, {300000, 'G'}, {2200000, 'S'},
        {2400000, 'S'}, {3000000, 'G'}, {4500000, ' '}
    };

    if ((key_index < sizeof(commands) / sizeof(commands[0]))
        && (host_time_us() >= commands[key_index].when))
    {
        int key = commands[key_index++].key;

        if (('S' == key) && (!stop_received))
        {
            stop_received = true;
            first_stop_us = host_time_us();
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

uint32_t i2c_init (i2c_inst_t * bus, uint32_t baudrate)
{
    assert((bus == i2c0) && (100000 == baudrate));
    memset(bus, 0, sizeof(*bus));
    bus->reg[0x0f] = 0x49;
    bus->reg[0x27] = 8;
    /* Recorded healthy stationary vector. */
    const int16_t raw[] = {901, 456, -14521};

    for (uint32_t idx = 0; idx < 3; idx++)
    {
        bus->reg[0x28 + 2 * idx] = (uint16_t)raw[idx] & 255;
        bus->reg[0x29 + 2 * idx] = (uint16_t)raw[idx] >> 8;
    }

    return baudrate;
}

static bool io_error (void)
{
    return (3 == scenario) && (host_time_us() >= 3000000)
            && (host_time_us() < 4000000);
}

int i2c_write_timeout_us (i2c_inst_t * bus, uint8_t address, const uint8_t * p,
                          size_t n, bool no_stop, uint32_t timeout)
{
    assert((bus == i2c0) && (0x1d == address)
           && (BUDDY4_I2C_TIMEOUT_US == timeout));

    if (io_error())
    {
        return -1;
    }

    if (1 == n)
    {
        assert(no_stop);
        bus->pointer = p[0];
    }
    else
    {
        assert((2 == n) && (!no_stop));
        bus->reg[p[0]] = p[1];
    }

    return (int)n;
}

int i2c_read_timeout_us (i2c_inst_t * bus, uint8_t address, uint8_t * p,
                         size_t n, bool no_stop, uint32_t timeout)
{
    assert((bus == i2c0) && (0x1d == address) && (!no_stop)
           && (BUDDY4_I2C_TIMEOUT_US == timeout));

    if (io_error())
    {
        return -1;
    }

    for (size_t idx = 0; idx < n; idx++)
    {
        p[idx] = bus->reg[(bus->pointer & 0x7f) + idx];
    }

    return (int)n;
}

static int capture_printf (const char * format, ...)
{
    char text[4096];
    va_list ap;
    va_start(ap, format);
    int count = vsnprintf(text, sizeof(text), format, ap);
    va_end(ap);
    assert((count >= 0) && ((size_t)count < sizeof(text)));
    attempted++;

    if ((1 == scenario) && (host_time_us() >= 1500000)
        && (host_time_us() < 4000000))
    {
        dropped++;
        sim_advance_ms(1); /* Model finite TX wait/drop while RX works. */
        return count;
    }

    if (!strncmp(text, "seq,", 4))
    {
        header_count++;
        return count;
    }

    if (strncmp(format, "%lu,%llu,", 9))
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
    assert((!have_seq) || ((seq > previous_seq) && (row_us > last_row_us)));
    previous_seq = seq;
    last_row_us = row_us;
    have_seq = true;

    if (atoi(field[34]))
    {
        driven = true;
    }

    if (stop_received)
    {
        rows_after++;
        assert(!atoi(field[34]));
        assert((!control.running) && (!control.armed));

        if (row_us > first_stop_us + 20000)
        {
            assert(MOTION_STATE_ERROR == atoi(field[45]));
            assert(MOTION_FAULT_ESTOP == atoi(field[46]));
            seen_error = true;
        }

        if (!atoi(field[3]))
        {
            invalid_rows++;
        }
    }
    else
    {
        rows_before++;
    }

    if (row_us >= 4000000)
    {
        received_after_resume++;
    }

    return count;
}

void tight_loop_contents (void)
{
    if (stop_received)
    {
        assert((!control.running) && (!control.armed));
        assert((sim_is_braked(MOTION_SIDE_LEFT))
               && (sim_is_braked(MOTION_SIDE_RIGHT)));
    }

    if (host_time_us() >= 6500000)
    {
        longjmp(finished, 1);
    }

    sim_advance_ms(1);
}

int main (int argc, char ** argv)
{
    assert(2 == argc);
    scenario = atoi(argv[1]);
    assert((scenario >= 0) && (scenario <= 3));

    if (!setjmp(finished))
    {
        actual_sdk_main();
    }

    assert(driven && stop_received && seen_error);
    assert((rows_before > 20) && (rows_after > 100)
           && (received_after_resume > 100));
    assert((last_row_us > 6400000) && (sim_is_braked(MOTION_SIDE_LEFT))
           && (sim_is_braked(MOTION_SIDE_RIGHT)));
    assert(header_count == (2 == scenario ? 2u : 1u));

    if (1 == scenario)
    {
        assert((dropped > 50) && (attempted > rows_before + rows_after));
    }

    if (3 == scenario)
    {
        assert(invalid_rows > 20);
    }

    printf("Actual SDK main-loop scenario %d: %u received rows before S, %u "
           "after S, %u dropped writes, motors remain latched stopped\n",
           scenario, rows_before, rows_after, dropped);
    return 0;
}

/*** end of file ***/
