/*
 * PicoCar Buddy 3: two-IR line follower + Buddy 2 wheel-speed PID.
 *
 * GP0: left module as seen in the user's photo
 * GP2: right module as seen in the user's photo
 * GP26: barcode sensor (monitored only; not decoded)
 * Raw IR level 1 = BLACK, 0 = WHITE (confirmed on this car).
 *
 * IMPORTANT: 'left in photo' might be the CAR'S RIGHT when facing it.
 * The photo looks front-facing, so GP0_IS_CAR_LEFT defaults to 0.
 * Change it to 1 if GP0 is actually on the CAR'S left when moving forward.
 *
 * Intended starting alignment: BOTH line sensors see the 18 mm black line.
 * Mainline test program for the Pico C SDK, not an RTOS task.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// -------- Hardware pins (from Team 08's wiring) --------
#define LEFT_MOTOR_A       8u
#define LEFT_MOTOR_B       9u
#define RIGHT_MOTOR_A     10u
#define RIGHT_MOTOR_B     11u
#define LEFT_ENCODER_A     6u
#define RIGHT_ENCODER_A    4u
#define IR_GP0             0u  // module on LEFT in the photo
#define IR_GP2             2u  // module on RIGHT in the photo
#define IR_BARCODE        26u

// Change to 0 if GP0 is actually on the ROBOT'S right, looking forward.
#define GP0_IS_CAR_LEFT    0
#define BLACK_LEVEL        1

// -------- Tuning --------
#define CONTROL_MS        30     // control loop, ~33 Hz
#define PRINT_MS         300     // terminal output interval
#define START_DELAY_S      5     // time to position the car

// Target speeds in encoder counts per second; tune on your hardware.
#define STRAIGHT_SPEED   300.0f
#define TURN_OUTER_SPEED 300.0f
#define TURN_INNER_SPEED 120.0f

// From Buddy 2's original motor-speed controller.
#define KP                0.02f
#define KI                0.05f
#define KD                0.00f
#define FEEDFORWARD       (40.0f / 2000.0f)
#define MAX_PWM           60.0f  // cap for initial low-speed tests
#define PWM_WRAP         1000u

typedef enum {
    ACTION_STOP = 0,
    ACTION_STRAIGHT,
    ACTION_LEFT,
    ACTION_RIGHT
} action_t;

typedef struct {
    float integral;
    float previous_error;
} wheel_pid_t;

static volatile uint32_t left_encoder_count = 0;
static volatile uint32_t right_encoder_count = 0;

static void encoder_irq(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == LEFT_ENCODER_A)  ++left_encoder_count;
    if (gpio == RIGHT_ENCODER_A) ++right_encoder_count;
}

static void encoder_init(void)
{
    gpio_init(LEFT_ENCODER_A);
    gpio_set_dir(LEFT_ENCODER_A, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_A);

    gpio_init(RIGHT_ENCODER_A);
    gpio_set_dir(RIGHT_ENCODER_A, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_A);

    // Register one shared callback for both encoder GPIOs.
    gpio_set_irq_enabled_with_callback(
        LEFT_ENCODER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, encoder_irq);
    gpio_set_irq_enabled(
        RIGHT_ENCODER_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

static void motor_pin_init(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_clkdiv(slice, 6.25f);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

static void motors_stop(void)
{
    // PWM=0 on both driver inputs: coast/stop drive, not active braking.
    pwm_set_gpio_level(LEFT_MOTOR_A, 0);
    pwm_set_gpio_level(LEFT_MOTOR_B, 0);
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0);
}

static void motor_forward(uint pin_a, uint pin_b, float pwm_percent)
{
    if (pwm_percent < 0.0f) pwm_percent = 0.0f;
    if (pwm_percent > MAX_PWM) pwm_percent = MAX_PWM;
    pwm_set_gpio_level(pin_a, (uint16_t)(pwm_percent * (float)PWM_WRAP / 100.0f));
    pwm_set_gpio_level(pin_b, 0);
}

static void ir_init(void)
{
    const uint pins[] = {IR_GP0, IR_GP2, IR_BARCODE};
    for (unsigned i = 0; i < 3u; ++i) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
    }
}

static void pid_reset(wheel_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

static float pid_update(wheel_pid_t *pid, float target, float measured, float dt)
{
    // PID and feedforward based on Buddy 2's original speed_control.c.
    float error = target - measured;
    pid->integral += error * dt;

    const float limit = 30.0f / KI;
    if (pid->integral > limit) pid->integral = limit;
    if (pid->integral < -limit) pid->integral = -limit;

    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    float output = FEEDFORWARD * target
                 + KP * error
                 + KI * pid->integral
                 + KD * derivative;
    if (output < 0.0f) output = 0.0f;
    if (output > MAX_PWM) output = MAX_PWM;
    return output;
}

static action_t decide_action(bool gp0_black, bool gp2_black)
{
    if (gp0_black && gp2_black) return ACTION_STRAIGHT;
    if (!gp0_black && !gp2_black) return ACTION_STOP;

    // The side that still sees the line is the side to steer towards.
    if (gp0_black) {
        return GP0_IS_CAR_LEFT ? ACTION_LEFT : ACTION_RIGHT;
    }
    return GP0_IS_CAR_LEFT ? ACTION_RIGHT : ACTION_LEFT;
}

int main(void)
{
    stdio_init_all();

    motor_pin_init(LEFT_MOTOR_A);
    motor_pin_init(LEFT_MOTOR_B);
    motor_pin_init(RIGHT_MOTOR_A);
    motor_pin_init(RIGHT_MOTOR_B);
    motors_stop();

    encoder_init();
    ir_init();

    // Prints may be missed until USB Serial Monitor connects.
    printf("\nPicoCar two-sensor line test (1=BLACK).\n");
    printf("GP0=photo-left | GP2=photo-right | GP26=barcode\n");
    printf("GP0_IS_CAR_LEFT=%d\n", GP0_IS_CAR_LEFT);
    for (int s = START_DELAY_S; s > 0; --s) {
        printf("Starting in %d s (wheels raised for first test)...\n", s);
        sleep_ms(1000);
    }

    wheel_pid_t pid_left = {0}, pid_right = {0};
    uint32_t last_left = left_encoder_count;
    uint32_t last_right = right_encoder_count;
    action_t last_action = (action_t)-1;
    int print_elapsed = PRINT_MS;
    const float dt = CONTROL_MS / 1000.0f;
    absolute_time_t next_tick = get_absolute_time();

    while (true) {
        next_tick = delayed_by_ms(next_tick, CONTROL_MS);
        sleep_until(next_tick);

        bool gp0_black = (gpio_get(IR_GP0) == BLACK_LEVEL);
        bool gp2_black = (gpio_get(IR_GP2) == BLACK_LEVEL);
        bool barcode_black = (gpio_get(IR_BARCODE) == BLACK_LEVEL);
        action_t action = decide_action(gp0_black, gp2_black);

        uint32_t now_left = left_encoder_count;
        uint32_t now_right = right_encoder_count;
        float speed_left = (now_left - last_left) / dt;
        float speed_right = (now_right - last_right) / dt;
        last_left = now_left;
        last_right = now_right;

        float target_left = 0.0f;
        float target_right = 0.0f;
        if (action == ACTION_STRAIGHT) {
            target_left = STRAIGHT_SPEED;
            target_right = STRAIGHT_SPEED;
        } else if (action == ACTION_LEFT) {
            target_left = TURN_INNER_SPEED;
            target_right = TURN_OUTER_SPEED;
        } else if (action == ACTION_RIGHT) {
            target_left = TURN_OUTER_SPEED;
            target_right = TURN_INNER_SPEED;
        }

        if (action == ACTION_STOP) {
            // No line: never drive blindly. A later version can add controlled recovery.
            motors_stop();
            pid_reset(&pid_left);
            pid_reset(&pid_right);
        } else {
            float pwm_left = pid_update(&pid_left, target_left, speed_left, dt);
            float pwm_right = pid_update(&pid_right, target_right, speed_right, dt);
            motor_forward(LEFT_MOTOR_A, LEFT_MOTOR_B, pwm_left);
            motor_forward(RIGHT_MOTOR_A, RIGHT_MOTOR_B, pwm_right);
        }

        print_elapsed += CONTROL_MS;
        if (action != last_action || print_elapsed >= PRINT_MS) {
            static const char *labels[] = {"STOP", "STRAIGHT", "STEER LEFT", "STEER RIGHT"};
            printf("[IR] GP0=%s GP2=%s Barcode=%s | %s | "
                   "target L/R=%.0f/%.0f measured L/R=%.0f/%.0f\n",
                   gp0_black ? "BLACK" : "WHITE",
                   gp2_black ? "BLACK" : "WHITE",
                   barcode_black ? "BLACK" : "WHITE",
                   labels[action], target_left, target_right,
                   speed_left, speed_right);
            print_elapsed = 0;
        }
        last_action = action;
    }
}
