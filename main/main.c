/* 
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "queue_handler.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// bluepad32
#include <stdlib.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include <uni.h>

#include "sdkconfig.h"

// crsf
#include "crsf.h"

// lizard servo gait (runs on the same chip, own GPIOs via LEDC)
#include "lizard.h"

// uncomment for official baud rate 416666 Kbit/s
// #define USE_CRSF_OFFICIAL_SPEC

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

static const char *TAG = "main";

// Defined in gamepad.c
struct uni_platform* get_my_platform(void);

static const int RX_BUF_SIZE = 1024;

// CRSF link to flight controller -- matches actual wiring (D4/D5)
#define TXD_PIN (GPIO_NUM_5)
#define RXD_PIN (GPIO_NUM_6)

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(uni_gamepad_t)

QueueHandle_t gamepad_queue;
uni_gamepad_t gamepad_data;

struct misc_btn_state_hold {
    bool sys;
    bool sel;
    bool sta;
};

struct misc_btn_state_hold state_hold = {false};

struct controller_state {
    bool armed;
    bool angle;
};

struct controller_state ctrl_state = {false, false};

typedef enum {
    GP_LX,
    GP_LY,
    GP_RX,
    GP_RY,
    GP_LT,
    GP_RT
} joystick_axis_t;

typedef struct {
    int16_t neg_lx;
    int16_t pos_lx;
    int16_t neg_ly;
    int16_t pos_ly;
    int16_t neg_rx;
    int16_t pos_rx;
    int16_t neg_ry;
    int16_t pos_ry;
    int16_t neg_lt;
    int16_t pos_lt;
    int16_t neg_rt;
    int16_t pos_rt;
} __attribute__ ((__packed__)) joystick_drift_t;

typedef union {
    joystick_drift_t drift;
    int16_t array[sizeof(joystick_drift_t) / sizeof(int16_t)];
} joystick_drift_union_t;

// set drift values
joystick_drift_union_t joystick_drift = {
    .drift = {
        .neg_lx = -40,
        .pos_lx = 40,
        .neg_ly = -40,
        .pos_ly = 40,
        .neg_rx = -10,
        .pos_rx = 60,
        .neg_ry = -20,
        .pos_ry = 20,
        .neg_lt = 0,
        .pos_lt = 0,
        .neg_rt = 0,
        .pos_rt = 0,
    }
};


void init_gamepad_queue() {
    gamepad_queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (gamepad_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create gamepad queue");
    } else {
        ESP_LOGI(TAG, "Gamepad queue created successfully");
    }
}

void init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // --- Lizard servo PWM + gait task init (lizard.c) ---
    lizard_init();
}

int sendData(const char* logName, const uint8_t* data, const int len)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

/**
 * Accept gamepad analog stick value and apply drift value to it, 
 * then normalize / rescale to fill range
 * 
 * @param input: gamepad value
 * @param AXIS: enum gamepad axis constant
 * @return compensated scaled value
 */
int32_t compensate_drift(const int32_t input, const joystick_axis_t AXIS) {
    int16_t neg_drift_thr = joystick_drift.array[AXIS * 2];
    int16_t pos_drift_thr = joystick_drift.array[AXIS * 2 + 1];
    if (input > neg_drift_thr && input < pos_drift_thr) {
        return 0;
    }
    if (input >= pos_drift_thr) {
        return ((input - pos_drift_thr) * 511) / (511 - pos_drift_thr);
    }
    if (input <= neg_drift_thr) {
        return ((input - neg_drift_thr) * 512) / (512 + neg_drift_thr);
    }
    return input; // this line should never reached
}

/**
 * Hold toggle state of buttons, change state only after crossing through zero value
 * so duplicate events can't flip toggle
 * 
 * @param misc_buttons bit mask coded states of misc buttons
 * @param btn_mask bit mask of specific button to deal with
 * @return state of toggle associated with button 1 | 0
 */
uint8_t get_misc_state(uint8_t misc_buttons, uint8_t btn_mask) {
    static uint8_t prev_state = 0;
    static uint8_t state = 0;
    if ((misc_buttons & btn_mask) && !(prev_state & btn_mask)) { // check is prev was cleared
        state ^= btn_mask;
        prev_state |= btn_mask; // set bit
    } else if (!(misc_buttons & btn_mask) && (prev_state & btn_mask)) { // check is prev was set
        prev_state &= ~btn_mask; // clear bit
    }

    return (state & btn_mask) ? 1 : 0 ;
}

int32_t invert_axis(int32_t axis) {
    return -axis;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    static uint8_t rc_channels_buf[CRSF_FRAME_SIZE_RC] = {0};
    static bool dpad_up_prev    = false;   // tracks D-pad edges for lizard triggers
    static bool dpad_down_prev  = false;
    static bool dpad_left_prev  = false;
    static bool dpad_right_prev = false;

    /**
     * init channels to default values: AETR1234, AER - MID, T - MIN
     * Roll[A] - MID
     * Pitch[E] - MID
     * Yaw[R] - MID
     * Throttle[T] - MIN
     * 1234 - doesn't matter
     */
    static rcChannelsUnpacked_t channel = {CRSF_CHANNEL_MIN};
    channel[0] = CRSF_CHANNEL_MID; // A
    channel[1] = CRSF_CHANNEL_MID; // E
    channel[2] = CRSF_CHANNEL_MIN; // T
    channel[3] = CRSF_CHANNEL_MID; // R

    while (1) {
        if (xQueueReceive(gamepad_queue, &gamepad_data, 0) == pdPASS) {
            // Virtual Gamepad layout:
            //
            //  Left             Center            Right
            //
            //  brake: 0-1023    Menu button       throttle: 0-1023
            //  L-shoulder button                  R-shoulder button
            //  L-trigger button                   R-trigger button
            //  d-pad                              buttons: A,B,X,Y,
            //  L-joypad (axis: -512, 511)         R-joypad (axis: -512, 511)
            //  axis-L button                      axis-R button
            //  Gyro: is measured in degress/second
            //  Accelerometer: is measured in "G"s
            // #define CRSF_CHANNEL_MIN    172
            // #define CRSF_CHANNEL_MID    992
            // #define CRSF_CHANNEL_MAX    1811
            // Transform signed values in range -512..511 to 0..1023
            // AETR1234, AER - MID, T - MIN

            /**
             * compensate center stick drift 
             */

            uint32_t stick_lx  = (((compensate_drift(gamepad_data.axis_x, GP_LX) + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            // int16_t stick_ly =  gamepad_data.axis_y + 512;
            uint32_t stick_rx  = (((compensate_drift(gamepad_data.axis_rx, GP_RX) + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            uint32_t stick_ry  = (((compensate_drift(invert_axis(gamepad_data.axis_ry), GP_RY) + 512) * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            uint32_t throttle = ((gamepad_data.throttle * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            
            ctrl_state.armed = get_misc_state(gamepad_data.misc_buttons, MISC_BUTTON_SYSTEM);
            ctrl_state.angle = get_misc_state(gamepad_data.misc_buttons, MISC_BUTTON_SELECT);
            // uint32_t brake =    ((gamepad_data.brake    * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)) / 1023) + CRSF_CHANNEL_MIN;
            channel[0] = stick_rx; // Roll    [A]
            channel[1] = stick_ry; // Pitch   [E]
            channel[2] = throttle; // Throttle[T]
            channel[3] = stick_lx; // Yaw     [R]
            channel[4] = ctrl_state.armed ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN; // AUX 1
            channel[5] = ctrl_state.angle ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN; // AUX 2
            /**
             * add new channels here
             */
            #if 0
                printf("Gamepad: rx: %03ld, ry: %03ld, sys: %u, sel: %u, all: %u\n",
                gamepad_data.axis_rx, gamepad_data.axis_ry, gamepad_data.misc_buttons & MISC_BUTTON_SYSTEM, gamepad_data.misc_buttons & MISC_BUTTON_SELECT,
                gamepad_data.misc_buttons);
            #endif
            
            /*
            Add additional channels if needed
            */
            build_frame(channel, rc_channels_buf, CRSF_FRAME_SIZE_RC);

            // --- D-pad drives the lizard gait (lizard.c) ---
            // Forward walks straight; left/right walk while arcing via
            // a body-joint turn bias (ported from the old Dynamixel
            // "turn_amp" logic -- legs untouched, bias only applied to
            // the body/tail joints). Gait runs continuously while held,
            // and snaps back to center/zero the instant it's released.
            // Bit values -- verify against uni_gamepad.h if a direction
            // seems swapped: Bluepad32 standard is DPAD_UP=0x01,
            // DPAD_DOWN=0x02, DPAD_RIGHT=0x04, DPAD_LEFT=0x08.
            bool dpad_up_now    = (gamepad_data.dpad & 0x01) != 0;
            bool dpad_down_now  = (gamepad_data.dpad & 0x02) != 0;
            bool dpad_right_now = (gamepad_data.dpad & 0x04) != 0;
            bool dpad_left_now  = (gamepad_data.dpad & 0x08) != 0;

            // TEMP DIAGNOSTIC: prints the raw dpad byte whenever it changes,
            // so we can confirm the real bit values for down/left/right.
            // Remove once directions are confirmed working.
            static uint8_t dpad_raw_prev = 0xFF;
            if (gamepad_data.dpad != dpad_raw_prev) {
                ESP_LOGI(TX_TASK_TAG, "RAW dpad = 0x%02X", gamepad_data.dpad);
                dpad_raw_prev = gamepad_data.dpad;
            }

            if (dpad_up_now != dpad_up_prev) {
                lizard_set_forward_held(dpad_up_now);
                ESP_LOGI(TX_TASK_TAG, "%s", dpad_up_now ? "D-pad forward pressed" : "D-pad forward released");
            }
            if (dpad_down_now != dpad_down_prev) {
                lizard_set_backward_held(dpad_down_now);
                ESP_LOGI(TX_TASK_TAG, "%s", dpad_down_now ? "D-pad backward pressed" : "D-pad backward released");
            }
            if (dpad_left_now != dpad_left_prev) {
                lizard_set_left_held(dpad_left_now);
                ESP_LOGI(TX_TASK_TAG, "%s", dpad_left_now ? "D-pad left pressed" : "D-pad left released");
            }
            if (dpad_right_now != dpad_right_prev) {
                lizard_set_right_held(dpad_right_now);
                ESP_LOGI(TX_TASK_TAG, "%s", dpad_right_now ? "D-pad right pressed" : "D-pad right released");
            }
            dpad_up_prev    = dpad_up_now;
            dpad_down_prev  = dpad_down_now;
            dpad_left_prev  = dpad_left_now;
            dpad_right_prev = dpad_right_now;
        }
        sendData(TX_TASK_TAG, rc_channels_buf, CRSF_FRAME_SIZE_RC);
        vTaskDelay(5 / portTICK_PERIOD_MS); // 5ms delay for 200HZ
    }
}
#if 0
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}
#endif
static void bt_task(void *arg) {
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

    // Don't use BTstack buffered UART. It conflicts with the console.
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif  // CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif  // CONFIG_ESP_CONSOLE_UART

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Init Bluepad32.
    uni_init(0 /* argc */, NULL /* argv */);

    // Does not return.
    btstack_run_loop_execute();

}


void app_main(void)
{
    init();
    init_gamepad_queue(); // Ensure the queue is initialized before creating tasks
    // xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(bt_task, "bt_gpad_task", 1024 * 5, NULL, configMAX_PRIORITIES - 2, NULL);
}
