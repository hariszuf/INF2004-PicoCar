#include "buddy4_device.h"
#ifdef BUDDY4_MTK_HOST_TEST
#include "tests/mtk_stubs/device_api.h"
#else
/* Use the port's documented size_t coexistence guard, local to this TU. */
#define PROHIBIT_DEF_SIZE_T
#include <tk/tkernel.h>
#include "../../device/include/dev_i2c.h"
#undef PROHIBIT_DEF_SIZE_T
#endif

static int write_bus(void *context, uint8_t address, const uint8_t *data,
                     size_t count, bool no_stop)
{
    buddy4_device_t *d = context;
    d->pending = false;
    if (!data || d->descriptor <= 0 || count == 0 || count > 2) return -1;
    if (no_stop) {
        if (count != 1) return -1;
        d->register_address = data[0];
        d->slave_address = address;
        d->pending = true;
        return 1; /* Actual address ACK belongs to the combined request below. */
    }
    SZ actual = 0;
    ER e = tk_swri_dev(d->descriptor, address, data, (SZ)count, &actual);
    d->last_error = e >= E_OK && actual != (SZ)count ? E_IO : (int)e;
    return e >= E_OK && actual == (SZ)count ? (int)count : -1;
}

static int read_bus(void *context, uint8_t address, uint8_t *data,
                    size_t count, bool no_stop)
{
    buddy4_device_t *d = context;
    bool pending = d->pending;
    d->pending = false;
    if (!pending || no_stop || address != d->slave_address || !data ||
        count == 0 || count > 6 || d->descriptor <= 0) return -1;
    T_I2C_EXEC request = {0};
    request.sadr = address;
    request.snd_size = 1;
    request.snd_data = &d->register_address;
    request.rcv_size = (SZ)count;
    request.rcv_data = data;
    SZ actual = 0;
    ER e = tk_swri_dev(d->descriptor, TDN_I2C_EXEC, &request,
                       sizeof(request), &actual);
    d->last_error = e >= E_OK && actual != (SZ)sizeof(request) ? E_IO : (int)e;
    return e >= E_OK && actual == (SZ)sizeof(request) ? (int)count : -1;
}

buddy4_bus_t buddy4_device_bus(buddy4_device_t *device)
{
    return (buddy4_bus_t){device, write_bus, read_bus};
}
