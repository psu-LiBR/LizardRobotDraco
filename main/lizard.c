#include "lizard.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"


static const char *TAG = "lizard";

// ---------------- Servo / pin map ----------------
// Xiao ESP32-S3 pin -> GPIO, per board silkscreen
#define SERVO1_GPIO   8   // D9  - front spine
#define SERVO2_GPIO   4   // D3  - mid spine
#define SERVO3_GPIO   9   // D10 - front-left leg
#define SERVO4_GPIO   7   // D8  - front-right leg
#define SERVO5_GPIO   3   // D2  - rear-left leg
#define SERVO6_GPIO   2   // D1  - rear-right leg
#define SERVO7_GPIO   1   // D0  - tail

#define NUM_SERVOS    7

static const int servo_gpio[NUM_SERVOS] = {
    SERVO1_GPIO, SERVO2_GPIO, SERVO3_GPIO, SERVO4_GPIO,
    SERVO5_GPIO, SERVO6_GPIO, SERVO7_GPIO
};

static const ledc_channel_t servo_channel[NUM_SERVOS] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
    LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6
};

// ---------------- PWM config ----------------
// Standard RC servo: 50Hz, pulse 500-2500us (0-180 deg).
// ESP32-S3 LEDC max duty resolution is 14-bit (0-16383).
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ        50
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT
#define LEDC_RES_MAX        16383
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_PERIOD_US     (1000000 / LEDC_FREQ_HZ)   // 20000us @ 50Hz

// ---------------- Gait parameters (ported from Python/Dynamixel version) ----------------
#define CENTER_DEG       90.0f
#define AMP_BODY_DEG      30.0f   // was ampBody = 1.0 rad in original -> scaled to servo degrees
#define AMP_LEG_DEG       30.0f   // was ampLeg = pi/4 rad
#define FREQ_BODY          1.0f
#define FREQ_LEG           1.0f
#define DUTY               0.5f
#define TAIL_ACTUATION      1
#define PHI_BODY            0.0f   // rad, was phiBody in original -- shifts m2 vs m1, and m7
#define PHI_LEG             0.0f   // rad, was phiLeg in original -- shifts m4/m5 vs m3/m6
#define DT                  0.2f    // time step, matches original t += 0.2
#define STEP_DELAY_MS      50      // matches original time.sleep(0.05)
#define TWO_PI              (2.0f * (float)M_PI)

// Turning bias, ported from the old Dynamixel/MATLAB code's "turn_amp":
// added directly to the body/tail joints (m1, m2, m7) only. Legs are
// left completely untouched, matching the original behavior -- the
// robot arcs left/right by biasing the body curve while still walking.
#define TURN_AMP_DEG        20.0f

typedef enum {
    LIZARD_DIR_NONE = 0,
    LIZARD_DIR_FORWARD,
    LIZARD_DIR_BACKWARD,
    LIZARD_DIR_LEFT,
    LIZARD_DIR_RIGHT,
} lizard_dir_t;

static SemaphoreHandle_t s_start_sem = NULL;
static volatile lizard_dir_t s_dir = LIZARD_DIR_NONE;

// ---------------- Helpers ----------------

// Convert a servo angle (0-180 deg) into a LEDC duty value
static uint32_t angle_to_duty(float angle_deg)
{
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    float pulse_us = SERVO_MIN_US + (angle_deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    uint32_t duty = (uint32_t)((pulse_us / (float)SERVO_PERIOD_US) * LEDC_RES_MAX);
    return duty;
}

static void servo_write_deg(int idx, float angle_deg)
{
    // angle_to_duty() already clamps to 0-180, so this is safe even
    // when turn_amp pushes a body joint past its nominal travel.
    ledc_set_duty(LEDC_MODE, servo_channel[idx], angle_to_duty(angle_deg));
    ledc_update_duty(LEDC_MODE, servo_channel[idx]);
}

// Equivalent of Python duty_function(): returns 1 for a fraction `duty` of
// each period, 0 otherwise, phase-shifted.
static int duty_function(float t, float freq, float duty, float phase)
{
    float two_pi = 2.0f * (float)M_PI;
    float ph = fmodf(freq * t + phase, two_pi);
    if (ph < 0) ph += two_pi; // keep positive like Python's modulo
    ph /= two_pi;
    return (ph < duty) ? 1 : 0;
}

// ---------------- Gait task ----------------
static void lizard_task(void *arg)
{
    while (1) {
        // Wait here until tx_task signals a D-pad press (forward/left/right)
        if (xSemaphoreTake(s_start_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Gait start");
            float t = 0.0f;

            // Loop the gait cycle continuously as long as any D-pad
            // direction is held. Check the direction every step (~50ms)
            // so release is picked up almost immediately instead of
            // finishing a fixed number of cycles.
            lizard_dir_t dir;
            while ((dir = s_dir) != LIZARD_DIR_NONE) {
                // Turn bias (ported from the old Dynamixel/MATLAB
                // "turn_amp"): added only to the body/tail joints
                // (m1, m2, m7). Legs (m3-m6) are left untouched in all
                // cases, including turns -- matches the original logic.
                float turn_amp = 0.0f;
                if (dir == LIZARD_DIR_LEFT)  turn_amp = TURN_AMP_DEG;
                if (dir == LIZARD_DIR_RIGHT) turn_amp = -TURN_AMP_DEG;

                // Leg phase set: FORWARD (and turning, which is forward
                // walking with a body bias) uses the working phases you
                // confirmed drive the robot forward. BACKWARD swaps the
                // m3/m6 vs m4/m5 phase assignment -- this is the exact
                // half-cycle-reversed leg timing that was identified
                // earlier as making the robot walk backward, so it's
                // reused deliberately here instead of being a bug.
                float leg_phase_36, leg_phase_45;
                if (dir == LIZARD_DIR_BACKWARD) {
                    leg_phase_36 = 0.0f + PHI_LEG;
                    leg_phase_45 = -(float)M_PI + PHI_LEG;
                } else {
                    leg_phase_36 = -(float)M_PI + PHI_LEG;
                    leg_phase_45 = 0.0f + PHI_LEG;
                }

                float m1 = CENTER_DEG + AMP_BODY_DEG * cosf(FREQ_BODY * t) + turn_amp;
                float m2 = CENTER_DEG + AMP_BODY_DEG * cosf(FREQ_BODY * t + PHI_BODY) + turn_amp;
                float m3 = CENTER_DEG + AMP_LEG_DEG  * duty_function(t, FREQ_LEG, DUTY, leg_phase_36);
                float m4 = CENTER_DEG + AMP_LEG_DEG  * duty_function(t, FREQ_LEG, DUTY, leg_phase_45);
                float m5 = CENTER_DEG + AMP_LEG_DEG  * duty_function(t, FREQ_LEG, DUTY, leg_phase_45);
                float m6 = CENTER_DEG + AMP_LEG_DEG  * duty_function(t, FREQ_LEG, DUTY, leg_phase_36);
                float m7 = CENTER_DEG + TAIL_ACTUATION * AMP_BODY_DEG * cosf(t + 2.0f * PHI_BODY) + turn_amp;

                servo_write_deg(0, m1);
                servo_write_deg(1, m2);
                servo_write_deg(2, m3);
                servo_write_deg(3, 180.0f - m4); // D8 / front-right leg - mechanically mirrored
                servo_write_deg(4, 180.0f - m5); // D2 / rear-left leg - mechanically mirrored
                servo_write_deg(5, m6);
                servo_write_deg(6, m7);

                vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
                t += DT;
                if (t > TWO_PI) t -= TWO_PI; // wrap, avoid unbounded growth
            }

            // D-pad released: snap all servos back to center/zero immediately
            for (int i = 0; i < NUM_SERVOS; i++) {
                servo_write_deg(i, CENTER_DEG);
            }
            ESP_LOGI(TAG, "D-pad released, parked");
        }
    }
}

// ---------------- Public API ----------------
void lizard_init(void)
{
    // Timer config, shared by all 7 channels
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t ch_conf = {
            .gpio_num   = servo_gpio[i],
            .speed_mode = LEDC_MODE,
            .channel    = servo_channel[i],
            .timer_sel  = LEDC_TIMER,
            .duty       = angle_to_duty(CENTER_DEG),
            .hpoint     = 0,
        };
        ledc_channel_config(&ch_conf);
    }

    // Move everything to center pose on boot
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_write_deg(i, CENTER_DEG);
    }

    s_start_sem = xSemaphoreCreateBinary();
    if (s_start_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create start semaphore");
        return;
    }

    xTaskCreate(lizard_task, "lizard_task", 1024 * 3, NULL, configMAX_PRIORITIES - 3, NULL);
    ESP_LOGI(TAG, "Lizard servo task initialized, centered, waiting for D-pad");
}

// Shared implementation: sets the current direction and wakes the gait
// task on a NONE -> non-NONE transition. Any direction change while
// already running (e.g. forward -> left) just updates s_dir in place --
// the task picks up the new turn_amp on its very next ~50ms step.
static void lizard_set_dir(bool held, lizard_dir_t dir)
{
    if (s_start_sem == NULL) {
        return; // lizard_init() not called yet
    }

    lizard_dir_t was_dir = s_dir;
    s_dir = held ? dir : LIZARD_DIR_NONE;

    // Rising edge from idle: wake the gait task. (No-op if it's already
    // running for a different direction -- the semaphore only needs to
    // be given once to get the task out of its initial wait.)
    if (held && was_dir == LIZARD_DIR_NONE) {
        xSemaphoreGive(s_start_sem);
    }
    // Falling edge needs no action here -- lizard_task polls s_dir
    // itself every ~50ms step and will park the servos as soon as it
    // sees it go back to LIZARD_DIR_NONE.
}

void lizard_set_forward_held(bool held)
{
    lizard_set_dir(held, LIZARD_DIR_FORWARD);
}

void lizard_set_left_held(bool held)
{
    lizard_set_dir(held, LIZARD_DIR_LEFT);
}

void lizard_set_right_held(bool held)
{
    lizard_set_dir(held, LIZARD_DIR_RIGHT);
}

void lizard_set_backward_held(bool held)
{
    lizard_set_dir(held, LIZARD_DIR_BACKWARD);
}
