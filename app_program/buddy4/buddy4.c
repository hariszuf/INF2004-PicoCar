#include <stdint.h>
#include <string.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../../device/include/dev_i2c.h"
#include "buddy4.h"

#define MPU_ADDR 0x68U  /* MPU6050, AD0 connected low */
#define MPU_WHOAMI 0x75U
#define MPU_POWER 0x6BU
#define MPU_ACCEL_CONFIG 0x1CU
#define MPU_GYRO_CONFIG 0x1BU
#define MPU_ACCEL_DATA 0x3BU
#define SAMPLE_PERIOD_MS 20

static ID imu_device = 0;
static ID imu_task_id = 0;
static ID status_mutex = 0;
static buddy4_status_t latest;
static imu_detector_t detector;

static ER read_regs(uint8_t reg, uint8_t *data, SZ len)
{
    T_I2C_EXEC xfer = {0};
    SZ actual = 0;
    UB address = reg;
    xfer.sadr = MPU_ADDR;
    xfer.snd_size = 1;
    xfer.snd_data = &address;
    xfer.rcv_size = len;
    xfer.rcv_data = data;
    return tk_swri_dev(imu_device, TDN_I2C_EXEC, &xfer, sizeof(xfer), &actual);
}

static ER write_reg(uint8_t reg, uint8_t value)
{
    SZ actual = 0;
    UB buffer[2] = {reg, value};
    return tk_swri_dev(imu_device, MPU_ADDR, buffer, sizeof(buffer), &actual);
}

static int16_t be16(const uint8_t *b)
{
    return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

static ER read_imu(imu_raw_t *r)
{
    UB bytes[14];
    ER rc = read_regs(MPU_ACCEL_DATA, bytes, sizeof(bytes));
    if (rc < E_OK) return rc;
    r->ax = be16(bytes); r->ay = be16(bytes + 2); r->az = be16(bytes + 4);
    r->gx = be16(bytes + 8); r->gy = be16(bytes + 10); r->gz = be16(bytes + 12);
    return E_OK;
}

static void imu_task(INT stacd, void *exinf)
{
    buddy4_status_t sample = {0};
    imu_terrain_t last_state = IMU_TERRAIN_CALIBRATING;
    unsigned errors = 0;
    (void)stacd; (void)exinf;
    for (;;) {
        ER rc = read_imu(&sample.raw);
        if (rc < E_OK) {
            if (++errors == 3U) {
                imu_detector_reset(&detector);
                sample.result.terrain = IMU_TERRAIN_ERROR;
                sample.result.ready = 0;
                sample.valid = 0;
            }
            if (status_mutex > 0 && tk_loc_mtx(status_mutex, TMO_FEVR) >= E_OK) {
                latest = sample;
                tk_unl_mtx(status_mutex);
            }
            if (errors == 3U) tm_printf((UB *)"[Buddy4] IMU disconnected/read failure: %d\n", rc);
            tk_dly_tsk(200);
            continue;
        }
        errors = 0;
        sample.result = imu_detector_step(&detector, &sample.raw);
        sample.valid = sample.result.ready;
        sample.sequence++;
        if (sample.result.terrain != last_state) {
            tm_printf((UB *)"[Buddy4] terrain: %s; pitch=%d mdeg\n",
                imu_terrain_name(sample.result.terrain), sample.result.pitch_mdeg);
            last_state = sample.result.terrain;
        }
        if (status_mutex > 0 && tk_loc_mtx(status_mutex, TMO_FEVR) >= E_OK) {
            latest = sample;
            tk_unl_mtx(status_mutex);
        }
        tk_dly_tsk(SAMPLE_PERIOD_MS);
    }
}

static T_CTSK task_config = {
    .itskpri = 10, .stksz = 4096, .task = imu_task, .tskatr = TA_HLNG | TA_RNG3
};

ER buddy4_init(void)
{
    UB who = 0;
    ER rc;
    T_CMTX mtx = {0};
    memset(&latest, 0, sizeof(latest));
    imu_detector_reset(&detector);
    mtx.mtxatr = TA_TFIFO;
    status_mutex = tk_cre_mtx(&mtx);
    if (status_mutex < E_OK) return status_mutex;
    imu_device = tk_opn_dev((const UB *)"iica", TD_READ | TD_WRITE);
    if (imu_device < E_OK) return imu_device;
    rc = read_regs(MPU_WHOAMI, &who, 1);
    if (rc < E_OK) return rc;
    if (who != 0x68U) {
        tm_printf((UB *)"[Buddy4] Unexpected IMU chip ID 0x%x (expected MPU6050)\n", who);
        return E_OBJ;
    }
    rc = write_reg(MPU_POWER, 0x00); if (rc < E_OK) return rc;
    tk_dly_tsk(100);
    rc = write_reg(MPU_ACCEL_CONFIG, 0x00); if (rc < E_OK) return rc;
    rc = write_reg(MPU_GYRO_CONFIG, 0x00); if (rc < E_OK) return rc;
    tm_printf((UB *)"[Buddy4] MPU6050 initialized; keep robot LEVEL and STILL 2 seconds\n");
    return E_OK;
}

ER buddy4_start(void)
{
    imu_task_id = tk_cre_tsk(&task_config);
    if (imu_task_id < E_OK) return imu_task_id;
    return tk_sta_tsk(imu_task_id, 0);
}

ER buddy4_get_latest(buddy4_status_t *out)
{
    ER rc;
    if (out == 0) return E_PAR;
    if (status_mutex <= 0) return E_OBJ;
    rc = tk_loc_mtx(status_mutex, TMO_FEVR);
    if (rc < E_OK) return rc;
    *out = latest;
    tk_unl_mtx(status_mutex);
    return out->valid ? E_OK : E_OBJ;
}
