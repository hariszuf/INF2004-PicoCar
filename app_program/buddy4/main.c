// Buddy4 GY-511 (probable LSM303D) C/Pico SDK project for VS Code.
// SDA=GP16, SCL=GP17, sensor address=0x1D.
#include <stdio.h>
#include "pico/stdlib.h"
#include "lsm303d.h"
#include "terrain.h"
#if BUDDY4_ENABLE_MQTT
#include "mqtt_telemetry.h"
#endif

#define SAMPLE_DELAY_MS 20
#define CALIBRATION_SAMPLES 100

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // USB serial monitor start-up.
    printf("\nBuddy4 / GY-511: Pico SDK sensor test\n");
    printf("SDA=GP16, SCL=GP17, I2C0, address=0x1D\n");
    lsm303d_bus_init();

    // Always leave the motor off or car wheels raised for this test.
    while (!lsm303d_init()) {
        printf("GY-511 not identified as LSM303D (WHO_AM_I must be 0x49).\n");
        printf("Check I2C, wiring, or the sensor variant. Retrying...\n");
        sleep_ms(1000);
    }
    printf("LSM303D identified. Keep car LEVEL and STILL for 2 seconds.\n");

    float pitch_sum = 0.0f, magnitude_sum = 0.0f;
    unsigned good_samples = 0;
    while (good_samples < CALIBRATION_SAMPLES) {
        accel_g_t a;
        if (lsm303d_read_accel(&a)) {
            pitch_sum += terrain_pitch_degrees(&a);
            magnitude_sum += terrain_magnitude(&a);
            ++good_samples;
        } else {
            printf("IMU calibration read failed; retrying...\n");
        }
        sleep_ms(SAMPLE_DELAY_MS);
    }
    terrain_detector_t detector;
    terrain_init(&detector, pitch_sum/CALIBRATION_SAMPLES,
                 magnitude_sum/CALIBRATION_SAMPLES);
    printf("Calibrated: pitch baseline %.2f deg.\n", detector.baseline_pitch);

#if BUDDY4_ENABLE_MQTT
    // Enable only after Stage 1 works. MQTT requires Pico W and local_config.h.
    mqtt_telemetry_start();
#endif

    uint32_t samples = 0;
    while (true) {
        accel_g_t a;
        float pitch = 0.0f;
        if (lsm303d_read_accel(&a)) {
            terrain_t t = terrain_process(&detector, &a, &pitch);
            if ((++samples % 25) == 0 || t == TERRAIN_POSSIBLE_HUMP) {
                printf("X=%+.2f Y=%+.2f Z=%+.2f g | pitch=%+.1f deg | %s\n",
                    a.x, a.y, a.z, pitch, terrain_name(t));
            }
#if BUDDY4_ENABLE_MQTT
            // Publish at 1 Hz to avoid flooding the broker.
            if ((samples % 50) == 0 || t == TERRAIN_POSSIBLE_HUMP) {
                mqtt_telemetry_publish(terrain_name(t), pitch, a.x, a.y, a.z);
            }
            mqtt_telemetry_poll();
#endif
        } else {
            printf("GY-511 read error\n");
        }
        sleep_ms(SAMPLE_DELAY_MS);
    }
}
