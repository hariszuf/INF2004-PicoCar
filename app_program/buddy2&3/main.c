// IR-controlled motor test based on Buddy 2 speed_control.c
// Main line IR: GP2 | Backup line IR: GP0 | Barcode IR: GP26
// 1 = BLACK. Forward if EITHER line sensor is black; otherwise stop.
// Standalone Pico SDK test ONLY; not a muT-Kernel task or curve follower.

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11
#define LEFT_ENC_A 6
#define RIGHT_ENC_A 4

#define IR_MAIN_PIN 2
#define IR_BACKUP_PIN 0
#define IR_BARCODE_PIN 26
#define IR_BLACK_LEVEL 1

#define CONTROL_MS 50
#define PRINT_MS 250
#define START_DELAY 5
#define TARGET_SPEED 800.0f  // encoder counts/second; start conservatively

// PID values copied from your teammate's code
#define KP 0.02f
#define KI 0.05f
#define KD 0.0f
#define FEEDFORWARD (40.0f / 2000.0f)
#define MAX_PWM 100.0f
#define PWM_WRAP 1000
#define COUNTS_PER_REV 0
#define WHEEL_DIAMETER_CM 6.5f

// ===== Encoder, motor, and PID code from Buddy 2 =====
    // ========================

    volatile int left_count  = 0;
    volatile int right_count = 0;

    // Runs automatically every time an encoder pin changes
    void encoder_pulse(uint gpio, uint32_t events) {
        if (gpio == LEFT_ENC_A)  left_count++;
        if (gpio == RIGHT_ENC_A) right_count++;
    }

    void setup_encoder_pin(uint pin) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }

    // ========================
    // MOTORS
    // ========================

    void setup_motor_pin(uint pin) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_clkdiv(slice, 6.25f);          // 20 kHz PWM (quiet)
        pwm_set_gpio_level(pin, 0);
        pwm_set_enabled(slice, true);
    }

    // Drive one motor forward at 'percent' (0-100)
    void motor_forward(uint pin_a, uint pin_b, float percent) {
        if (percent < 0) percent = 0;
        if (percent > MAX_PWM) percent = MAX_PWM;
        pwm_set_gpio_level(pin_a, (uint16_t)(percent * PWM_WRAP / 100.0f));
        pwm_set_gpio_level(pin_b, 0);
    }

    // Drive one motor backward at 'percent' (0-100)
    void motor_backward(uint pin_a, uint pin_b, float percent) {
        if (percent < 0) percent = 0;
        if (percent > MAX_PWM) percent = MAX_PWM;
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, (uint16_t)(percent * PWM_WRAP / 100.0f));
    }

    // Brake one motor: both pins HIGH = motor driver shorts the motor -> stops fast
    void motor_brake(uint pin_a, uint pin_b) {
        pwm_set_gpio_level(pin_a, PWM_WRAP);
        pwm_set_gpio_level(pin_b, PWM_WRAP);
    }

    void stop_all() {
        pwm_set_gpio_level(M1A, 0);
        pwm_set_gpio_level(M1B, 0);
        pwm_set_gpio_level(M2A, 0);
        pwm_set_gpio_level(M2B, 0);
    }

    // ========================
    // PID CONTROLLER
    // ========================

    typedef struct {
        float integral;
        float last_error;
    } PID;

    // Returns the PWM % the motor should get
    float pid_update(PID *pid, float target, float measured, float dt) {

        float error = target - measured;

        // I: add up error over time
        pid->integral += error * dt;

        // Limit I so it can't grow forever (anti-windup)
        float i_limit = 30.0f / KI;            // I part max +/-30% PWM
        if (pid->integral >  i_limit) pid->integral =  i_limit;
        if (pid->integral < -i_limit) pid->integral = -i_limit;

        // D: how fast the error is changing
        float derivative = (error - pid->last_error) / dt;
        pid->last_error = error;

        // Feedforward (rough guess) + P + I + D
        float output = FEEDFORWARD * target
                    + KP * error
                    + KI * pid->integral
                    + KD * derivative;

        if (output < 0) output = 0;
        if (output > MAX_PWM) output = MAX_PWM;
        return output;
    }

    void pid_reset(PID *pid) {
        pid->integral = 0;
        pid->last_error = 0;
    }

    float counts_to_cm(float counts) {
    #if COUNTS_PER_REV > 0
        return counts * (3.14159f * WHEEL_DIAMETER_CM) / COUNTS_PER_REV;
    #else
        return 0;   // COUNTS_PER_REV not filled in yet
    #endif
    }

    // ========================
// ===== IR-controlled forward/stop test =====
static void init_ir_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
}

int main(void) {
    stdio_init_all();

    setup_motor_pin(M1A);
    setup_motor_pin(M1B);
    setup_motor_pin(M2A);
    setup_motor_pin(M2B);
    stop_all(); // Never start with motors running

    gpio_set_irq_callback(&encoder_pulse);
    irq_set_enabled(IO_IRQ_BANK0, true);
    setup_encoder_pin(LEFT_ENC_A);
    setup_encoder_pin(RIGHT_ENC_A);

    init_ir_pin(IR_MAIN_PIN);
    init_ir_pin(IR_BACKUP_PIN);
    init_ir_pin(IR_BARCODE_PIN);

    printf("\nIR-gated motor test. 1=BLACK; GP2 main, GP0 backup, GP26 barcode.\n");
    for (int i = START_DELAY; i > 0; --i) {
        printf("Starting in %d...\n", i);
        sleep_ms(1000);
    }

    PID left_pid = {0}, right_pid = {0};
    int last_left = left_count, last_right = right_count;
    bool was_moving = false;
    int print_elapsed = PRINT_MS;
    const float dt = CONTROL_MS / 1000.0f;
    absolute_time_t next_tick = get_absolute_time();

    while (true) {
        next_tick = delayed_by_ms(next_tick, CONTROL_MS);
        sleep_until(next_tick);

        bool main_black = gpio_get(IR_MAIN_PIN) == IR_BLACK_LEVEL;
        bool backup_black = gpio_get(IR_BACKUP_PIN) == IR_BLACK_LEVEL;
        bool barcode_black = gpio_get(IR_BARCODE_PIN) == IR_BLACK_LEVEL;
        bool allow_forward = main_black || backup_black;

        int left_now = left_count;
        int right_now = right_count;
        float left_speed = (left_now - last_left) / dt;
        float right_speed = (right_now - last_right) / dt;
        last_left = left_now;
        last_right = right_now;

        if (!allow_forward) {
            // Always stop whenever neither sensor detects black.
            stop_all();
            pid_reset(&left_pid);
            pid_reset(&right_pid);
        } else {
            // Keep your teammate's encoder-based PID for straight-forward driving.
            float left_pwm = pid_update(&left_pid, TARGET_SPEED, left_speed, dt);
            float right_pwm = pid_update(&right_pid, TARGET_SPEED, right_speed, dt);
            motor_forward(M1A, M1B, left_pwm);
            motor_forward(M2A, M2B, right_pwm);
        }

        print_elapsed += CONTROL_MS;
        if (allow_forward != was_moving || print_elapsed >= PRINT_MS) {
            printf("[IR] Main=%s Backup=%s Barcode=%s | [MOTOR] %s | Speed L=%.0f R=%.0f counts/s\n",
                main_black ? "BLACK" : "WHITE",
                backup_black ? "BLACK" : "WHITE",
                barcode_black ? "BLACK" : "WHITE",
                allow_forward ? "FORWARD" : "STOP",
                left_speed, right_speed);
            print_elapsed = 0;
        }
        was_moving = allow_forward;
    }
}
