#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"
#include "nvs.h"
#include "nvs_flash.h"

// ESP-IDF log tag used by ESP_LOGI / ESP_LOGW / ESP_LOGE messages.
static const char *TAG = "GPS_MAP_TFT";

// Set these to 0 when the LIS3DH accelerometer and VEML7700 light sensor are not connected.
// This lets the firmware upload/run without trying to talk to missing I2C sensors.
#define USE_ACCELEROMETER 0
#define USE_VEML7700 1

// Embedded map image data. This header should contain map_rgb565[] and map_rgb565_len.
#include "map_rgb565.h"

// DMA-capable drawing buffers used by the LCD driver.
// These are allocated after the LCD is initialized.
static uint16_t *g_fill_buf = nullptr;
static uint16_t *g_char_buf = nullptr;
static uint16_t *g_patch_buf = nullptr;
static esp_lcd_panel_handle_t panel_handle = nullptr;

// =========================
// LCD PIN CONFIGURATION
// =========================
// These GPIOs connect the ESP32 SPI bus to the ST7796 LCD.
static constexpr gpio_num_t PIN_LCD_MOSI = GPIO_NUM_38;
static constexpr gpio_num_t PIN_LCD_SCLK = GPIO_NUM_48;
static constexpr gpio_num_t PIN_LCD_CS   = GPIO_NUM_21;
static constexpr gpio_num_t PIN_LCD_DC   = GPIO_NUM_18;
static constexpr gpio_num_t PIN_LCD_RST  = GPIO_NUM_17;

// LCD driver settings. DRAW_LINES controls how many rows are sent per SPI transfer.
static constexpr spi_host_device_t LCD_HOST = SPI2_HOST;
static constexpr int LCD_H_RES = 320;
static constexpr int LCD_V_RES = 480;
static constexpr int DRAW_LINES = 20;

// =========================
// GPS UART CONFIGURATION
// =========================
// NEO-6M GPS modules usually output NMEA sentences at 9600 baud.
static constexpr uart_port_t GPS_UART = UART_NUM_1;
static constexpr int GPS_RX_GPIO = 5;
static constexpr int GPS_TX_GPIO = UART_PIN_NO_CHANGE;

// =========================
// BUTTON AND BUZZER PINS
// =========================
// Buttons use internal pull-ups, so pressing the button pulls the pin LOW.
// Arduino Nano ESP32 pin map: D7 -> GPIO10, D6 -> GPIO9, D5 -> GPIO8
static constexpr gpio_num_t PIN_HAZARD_ADD   = GPIO_NUM_10;
static constexpr gpio_num_t PIN_HAZARD_CLEAR = GPIO_NUM_9;
static constexpr gpio_num_t PIN_BUZZER       = GPIO_NUM_8;

// =========================
// MAP IMAGE SETTINGS
// =========================
// Pixel size of the full exported map image from QGIS.
static constexpr int MAP_W = 1234;
static constexpr int MAP_H = 662;

// Geographic bounds of the exported map.
// These values are used to convert GPS coordinates into image pixels.
static constexpr double MAP_TOP_LAT    = 38.580167742;
static constexpr double MAP_BOTTOM_LAT = 38.513537271;
static constexpr double MAP_LEFT_LON   = -121.797720852;
static constexpr double MAP_RIGHT_LON  = -121.673518434;

// Fine-tuning offsets for marker placement.
// Negative Y moves the marker up. Positive Y moves it down.
// Your current value fixes the user dot being about 20 pixels too low.
static constexpr int MAP_CAL_OFFSET_X = -13;
static constexpr int MAP_CAL_OFFSET_Y = -20;

// LCD bottom status bar dimensions.
// MAP_VIEW_H is the map area above the status bar.
static constexpr int STATUS_BAR_H = 48;
static constexpr int STATUS_BAR_Y = LCD_V_RES - STATUS_BAR_H;
static constexpr int MAP_VIEW_H = LCD_V_RES - STATUS_BAR_H;

// Zoom factor for the map viewport.
// 5/4 = 1.25x zoom, 3/2 = 1.5x zoom.
static constexpr int ZOOM_NUM = 5;
static constexpr int ZOOM_DEN = 5;
static constexpr int VIEW_SRC_W = (LCD_H_RES * ZOOM_DEN + (ZOOM_NUM / 2)) / ZOOM_NUM;
static constexpr int VIEW_SRC_H = (MAP_VIEW_H * ZOOM_DEN + (ZOOM_NUM / 2)) / ZOOM_NUM;

// RGB565 color constants.
// RGB565 uses 16 bits per pixel: 5 bits red, 6 bits green, 5 bits blue.
static constexpr uint16_t COLOR_BLACK  = 0x0000;
static constexpr uint16_t COLOR_WHITE  = 0xFFFF;
static constexpr uint16_t COLOR_GREEN  = 0x07E0;
static constexpr uint16_t COLOR_YELLOW = 0xFFE0;
static constexpr uint16_t COLOR_CYAN   = 0x07FF;
static constexpr uint16_t COLOR_RED    = 0xF800;
static constexpr uint16_t COLOR_HAZARD = 0x07E0;
static constexpr uint16_t COLOR_USER_BOX = 0xFFFF;   // change this for user marker square
static constexpr uint16_t COLOR_HAZARD_BOX = 0xFFFF; // change this for hazard marker square

// =========================
// HAZARD MARKER STORAGE / RULES
// =========================
// Hazards are saved into ESP32 NVS flash memory so they survive power cycles.
static constexpr int MAX_HAZARDS = 64;
static constexpr uint32_t HAZARD_STORE_MAGIC = 0x48415A31; // HAZ1
static constexpr const char *HAZARD_NVS_NAMESPACE = "hazards";
static constexpr const char *HAZARD_NVS_KEY = "markers";
static constexpr double HAZARD_DUPLICATE_RADIUS_M = 8.0;
static constexpr double HAZARD_CLEAR_RADIUS_M = 50.0;
static constexpr double HAZARD_NEAR_RADIUS_M = 50.0;
static constexpr int HAZARD_NEAR_REPEAT_MS = 3000;

// =========================
// PASSIVE BUZZER PWM SETTINGS
// =========================
// LEDC is the ESP32 PWM peripheral used to generate buzzer tones.
static constexpr ledc_mode_t BUZZER_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t BUZZER_LEDC_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t BUZZER_LEDC_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_timer_bit_t BUZZER_LEDC_RES = LEDC_TIMER_10_BIT;
static constexpr uint32_t BUZZER_BASE_FREQ_HZ = 2000;
static constexpr uint32_t BUZZER_DUTY = 512;

// =========================
// BIKE LIGHT GPIO CONFIGURATION
// =========================
// These GPIOs drive the bike headlight, turn signals, and PWM brake light.
static constexpr gpio_num_t HEADLIGHT_LED_GPIO    = GPIO_NUM_2;
static constexpr gpio_num_t LEFT_SIGNAL_LED_GPIO  = GPIO_NUM_3;
static constexpr gpio_num_t RIGHT_SIGNAL_LED_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t BRAKE_LED_GPIO        = GPIO_NUM_13;

// Buttons use internal pull-ups, so pressing the button pulls the pin LOW.
static constexpr gpio_num_t HEADLIGHT_BUTTON_GPIO = GPIO_NUM_43;
static constexpr gpio_num_t LEFT_BUTTON_GPIO      = GPIO_NUM_44;
static constexpr gpio_num_t RIGHT_BUTTON_GPIO     = GPIO_NUM_6;
static constexpr gpio_num_t BRAKE_BUTTON_GPIO     = GPIO_NUM_7;

// =========================
// LIS3DH ACCELEROMETER CONFIGURATION
// =========================
// The LIS3DH is used to detect braking motion for the brake light.
static constexpr gpio_num_t SCL_PIN = GPIO_NUM_11;
static constexpr gpio_num_t SDA_PIN = GPIO_NUM_12;
static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
static constexpr uint32_t I2C_FREQ_HZ = 100000;
static constexpr uint8_t LIS3DH_ADDR = 0x18;

static constexpr uint8_t CTRL_REG1 = 0x20;
static constexpr uint8_t CTRL_REG4 = 0x23;
static constexpr uint8_t OUT_X_L   = 0x28;

// =========================
// HALL SPEEDOMETER CONFIGURATION
// =========================
static constexpr gpio_num_t HALL_SENSOR_GPIO = GPIO_NUM_1;

static constexpr float WHEEL_CIRCUMFERENCE_M = 1.954f;
static constexpr int MAGNETS_PER_REV = 1;
static constexpr int64_t STOP_TIMEOUT_US = 3000000;
static constexpr int64_t MIN_PULSE_US = 40000; // 40 ms
static constexpr int HALL_ACTIVE_LEVEL = 0;
static QueueHandle_t hall_pulse_queue = nullptr;

// =========================
// BRAKE DETECTION SETTINGS
// =========================
// More negative thresholds make the brake light harder to trigger.
static constexpr float FAST_BRAKE_THRESHOLD_G = -0.25f;
static constexpr float SLOW_BRAKE_THRESHOLD_G = -0.15f;
static constexpr int REQUIRED_FAST_SAMPLES = 4;
static constexpr int REQUIRED_SLOW_SAMPLES = 15;
static constexpr float SPEED_INCREASING_LIMIT = 0.20f;
static constexpr float SPEED_DECREASING_LIMIT = -0.40f;
static constexpr float GRAVITY = 0.998f;
static constexpr int64_t BRAKE_HOLD_TIME_MS = 400;
static constexpr int CALIBRATION_SAMPLES = 100;

// =========================
// HALL-SPEED DECELERATION BRAKE SETTINGS
// =========================
// The Hall speedometer calculates speed_accel_mps2.
// Negative acceleration means the bike is slowing down.
// More negative = harder braking required before the brake light turns on.
static constexpr float DECEL_BRAKE_TRIGGER_MPS2 = -0.35f;

// Ignore tiny speed changes near 0 mph so the brake light does not flicker while stopped.
static constexpr float DECEL_BRAKE_MIN_SPEED_MPH = 0.8f;

// Keep the brake light on briefly after deceleration is detected.
static constexpr int64_t DECEL_BRAKE_HOLD_TIME_MS = 700;

// Used to scale brake-light brightness from moderate braking to hard braking.
// At DECEL_BRAKE_TRIGGER_MPS2, the brake is brighter than the button dim level.
// At DECEL_BRAKE_STRONG_MPS2 or below, the brake reaches full brightness.
static constexpr float DECEL_BRAKE_STRONG_MPS2 = -2.00f;
static constexpr uint32_t DECEL_BRAKE_MIN_DUTY = 450;

// =========================
// BRAKE LIGHT PWM SETTINGS
// =========================
// Uses a different LEDC timer/channel than the buzzer so they do not fight each other.
static constexpr ledc_timer_t BRAKE_LEDC_TIMER = LEDC_TIMER_1;
static constexpr ledc_mode_t BRAKE_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_channel_t BRAKE_LEDC_CHANNEL = LEDC_CHANNEL_1;
static constexpr ledc_timer_bit_t BRAKE_LEDC_RES = LEDC_TIMER_10_BIT;
static constexpr uint32_t BRAKE_LEDC_FREQ_HZ = 5000;

static constexpr uint32_t BRAKE_OFF_DUTY  = 0;
static constexpr uint32_t BRAKE_DIM_DUTY  = 200;
static constexpr uint32_t BRAKE_FULL_DUTY = 1023;

// =========================
// VEML7700 AMBIENT LIGHT SENSOR SETTINGS
// =========================
// The VEML7700 shares the same I2C bus as the LIS3DH.
// VEML7700 default I2C address is 0x10.
static constexpr uint8_t VEML7700_ADDR = 0x10;

static constexpr uint8_t VEML7700_REG_ALS_CONF = 0x00;
static constexpr uint8_t VEML7700_REG_ALS      = 0x04;

// ALS enabled, gain x1, integration time 100 ms.
static constexpr uint16_t VEML7700_ALS_CONF_VALUE = 0x0000;

// For gain x1 and 100 ms integration time, raw ALS count to lux is about 0.0576 lux/count.
static constexpr float VEML7700_LUX_PER_COUNT = 0.0576f;

// Below this value, the headlight goes full bright.
static constexpr float HEADLIGHT_DARK_THRESHOLD_LUX = 80.0f;

// Above this value, the headlight goes dim.
// This higher threshold prevents flicker around the dark threshold.
static constexpr float HEADLIGHT_BRIGHT_THRESHOLD_LUX = 120.0f;

// Read the VEML7700 slowly so I2C reads do not slow the main loop too much.
static constexpr int64_t AMBIENT_READ_INTERVAL_MS = 200;

// =========================
// HEADLIGHT PWM SETTINGS
// =========================
// Uses a different LEDC timer/channel than the buzzer and brake light.
static constexpr ledc_timer_t HEADLIGHT_LEDC_TIMER = LEDC_TIMER_2;
static constexpr ledc_mode_t HEADLIGHT_LEDC_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_channel_t HEADLIGHT_LEDC_CHANNEL = LEDC_CHANNEL_2;
static constexpr ledc_timer_bit_t HEADLIGHT_LEDC_RES = LEDC_TIMER_10_BIT;
static constexpr uint32_t HEADLIGHT_LEDC_FREQ_HZ = 20000;

static constexpr uint32_t HEADLIGHT_OFF_DUTY  = 0;
static constexpr uint32_t HEADLIGHT_DIM_DUTY  = 100; // Was 250 before; keep adjusting
static constexpr uint32_t HEADLIGHT_FULL_DUTY = 1023;

// =========================
// BIKE LIGHT TIMING SETTINGS
// =========================
static constexpr int64_t BIKE_DEBOUNCE_TIME_MS = 50;
static constexpr int64_t BLINK_INTERVAL_MS = 500;
static constexpr int64_t BIKE_PRINT_INTERVAL_MS = 300;

// Stores one hazard location as latitude and longitude.
struct HazardMarker {
    double latitude;
    double longitude;
};

// Data format written to NVS flash.
// The magic number helps detect whether saved data is valid.
struct HazardStoreBlob {
    uint32_t magic;
    uint32_t count;
    HazardMarker hazards[MAX_HAZARDS];
};

// Runtime hazard list loaded from / saved to flash.
static HazardMarker g_hazards[MAX_HAZARDS] = {};
static int g_hazard_count = 0;

// Button state and timing variables for debouncing and repeat prevention.
static bool g_prev_add_pressed = false;
static bool g_prev_clear_pressed = false;
static TickType_t g_last_add_tick = 0;
static TickType_t g_last_clear_tick = 0;
static bool g_prev_near_hazard = false;
static TickType_t g_last_near_buzz_tick = 0;



// Initializes the LEDC PWM timer/channel that drives the passive buzzer.
static void buzzer_init()
{
    // Configure the PWM timer frequency and resolution.
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = BUZZER_LEDC_MODE;
    timer_conf.timer_num = BUZZER_LEDC_TIMER;
    timer_conf.duty_resolution = BUZZER_LEDC_RES;
    timer_conf.freq_hz = BUZZER_BASE_FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Connect the PWM channel to the buzzer GPIO pin.
    ledc_channel_config_t channel_conf = {};
    channel_conf.gpio_num = PIN_BUZZER;
    channel_conf.speed_mode = BUZZER_LEDC_MODE;
    channel_conf.channel = BUZZER_LEDC_CHANNEL;
    channel_conf.intr_type = LEDC_INTR_DISABLE;
    channel_conf.timer_sel = BUZZER_LEDC_TIMER;
    channel_conf.duty = 0;
    channel_conf.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
}

// Stops the buzzer. Optional delay lets this function create pauses between notes.
static void buzzer_silence(int duration_ms = 0)
{
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    if (duration_ms > 0) vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

// Plays one buzzer tone at the requested frequency for the requested duration.
static void buzzer_play_tone(uint32_t freq_hz, int duration_ms)
{
    if (freq_hz == 0 || duration_ms <= 0) {
        buzzer_silence(duration_ms);
        return;
    }

    // Change PWM frequency to create the requested musical tone.
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
}

// Sound pattern played after a hazard is successfully dropped.
static void play_drop_hazard_sound()
{
    buzzer_play_tone(1700, 90);
    buzzer_silence(10);
    buzzer_play_tone(1900, 90);
    buzzer_silence(10);
    buzzer_play_tone(1700, 140);
    buzzer_silence(10);
}

// Sound pattern played after a nearby hazard is successfully cleared.
static void play_clear_hazard_sound()
{
    buzzer_play_tone(2093, 90);
    buzzer_silence(35);
    buzzer_play_tone(1568, 110);
    buzzer_silence(30);
}

// Repeating warning sound played when the rider is near a saved hazard.
static void play_near_hazard_sound()
{
    buzzer_play_tone(1000, 80);
    buzzer_silence(10);
    buzzer_play_tone(1200, 80);
    buzzer_silence(10);
    buzzer_play_tone(1000, 80);
    buzzer_silence(10);
}

// =========================
// BIKE LIGHT STATE VARIABLES
// =========================
static bool headlight_on = false;
static bool brake_system_on = false;
static bool last_brake_system_on = false;

static bool left_signal_on = false;
static bool right_signal_on = false;
static bool blink_state = false;

// =========================
// AMBIENT LIGHT / HEADLIGHT STATE
// =========================
static float ambient_lux = 0.0f;
static bool veml7700_ok = false;
static bool headlight_auto_dark = false;
static int64_t last_ambient_read_ms = 0;

// =========================
// ACCELEROMETER AXIS STATE
// =========================
enum class AccelAxis {
    X,
    Y,
    Z
};

static AccelAxis forward_axis = AccelAxis::X;
static bool accel_calibrated = false;
static float slow_forward_g = 0.0f;

static int fast_brake_count = 0;
static int slow_brake_count = 0;
static int64_t brake_hold_until_ms = 0;
static int64_t decel_brake_hold_until_ms = 0;
static uint32_t decel_brake_duty = BRAKE_OFF_DUTY;
static int64_t last_print_time_ms = 0;

// =========================
// SPEEDOMETER STATE VARIABLES
// =========================
static int64_t last_pulse_us = 0;
static int64_t last_speed_update_us = 0;

static float speed_mps = 0.0f;
static volatile float hall_speed_mph = 0.0f;
static float prev_speed_mps = 0.0f;
static float speed_accel_mps2 = 0.0f;

// =========================
// DEBOUNCED BUTTON
// =========================
class Button {
public:
    Button() = default;

    void init(gpio_num_t gpio_pin)
    {
        pin = gpio_pin;
        last_raw_state = 1;
        stable_state = 1;
        last_change_time_ms = 0;
    }

    bool pressed()
    {
        const int raw = gpio_get_level(pin);
        const int64_t now = get_time_ms();

        if (raw != last_raw_state) {
            last_change_time_ms = now;
            last_raw_state = raw;
        }

        if ((now - last_change_time_ms) >= BIKE_DEBOUNCE_TIME_MS) {
            if (raw != stable_state) {
                stable_state = raw;

                // Active-low button: 0 means pressed
                if (stable_state == 0) {
                    return true;
                }
            }
        }

        return false;
    }

private:
    gpio_num_t pin = GPIO_NUM_NC;
    int last_raw_state = 1;
    int stable_state = 1;
    int64_t last_change_time_ms = 0;

    static int64_t get_time_ms()
    {
        return esp_timer_get_time() / 1000;
    }
};

// =========================
// BIKE LIGHT TIMER
// =========================
static int64_t get_time_ms()
{
    return esp_timer_get_time() / 1000;
}

// =========================
// BIKE LIGHT GPIO INITIALIZATION
// =========================
static void bike_gpio_init()
{
    gpio_config_t led_config = {};
    // Headlight GPIO is controlled by LEDC PWM, so only the turn signals are normal GPIO outputs.
    led_config.pin_bit_mask = (1ULL << LEFT_SIGNAL_LED_GPIO) |
                              (1ULL << RIGHT_SIGNAL_LED_GPIO);
    led_config.mode = GPIO_MODE_OUTPUT;
    led_config.pull_up_en = GPIO_PULLUP_DISABLE;
    led_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&led_config);

    gpio_config_t button_config = {};
    button_config.pin_bit_mask = (1ULL << HEADLIGHT_BUTTON_GPIO) |
                                 (1ULL << LEFT_BUTTON_GPIO) |
                                 (1ULL << RIGHT_BUTTON_GPIO) |
                                 (1ULL << BRAKE_BUTTON_GPIO);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&button_config);

    gpio_set_level(LEFT_SIGNAL_LED_GPIO, 0);
    gpio_set_level(RIGHT_SIGNAL_LED_GPIO, 0);
}

// =========================
// LIS3DH ACCELEROMETER FUNCTIONS
// =========================
static esp_err_t lis3dh_write_reg(uint8_t reg, uint8_t data)
{
    const uint8_t buffer[2] = {reg, data};

    return i2c_master_write_to_device(
        I2C_PORT,
        LIS3DH_ADDR,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t lis3dh_read_accel(float &ax_g, float &ay_g, float &az_g)
{
    uint8_t reg = OUT_X_L | 0x80;
    uint8_t data[6] = {};

    const esp_err_t ret = i2c_master_write_read_device(
        I2C_PORT,
        LIS3DH_ADDR,
        &reg,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    int16_t raw_x = static_cast<int16_t>((data[1] << 8) | data[0]);
    int16_t raw_y = static_cast<int16_t>((data[3] << 8) | data[2]);
    int16_t raw_z = static_cast<int16_t>((data[5] << 8) | data[4]);

    raw_x >>= 4;
    raw_y >>= 4;
    raw_z >>= 4;

    ax_g = raw_x * 0.001f;
    ay_g = raw_y * 0.001f;
    az_g = raw_z * 0.001f;

    return ESP_OK;
}

static void i2c_bus_init()
{
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = SDA_PIN;
    config.scl_io_num = SCL_PIN;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, config.mode, 0, 0, 0));
}

static void i2c_lis3dh_init()
{
    // 100 Hz, X/Y/Z enabled
    ESP_ERROR_CHECK(lis3dh_write_reg(CTRL_REG1, 0x57));

    // High-resolution mode, ±2g
    ESP_ERROR_CHECK(lis3dh_write_reg(CTRL_REG4, 0x08));
}

// =========================
// VEML7700 AMBIENT LIGHT SENSOR FUNCTIONS
// =========================
static esp_err_t veml7700_write_reg16(uint8_t reg, uint16_t value)
{
    const uint8_t buffer[3] = {
        reg,
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };

    return i2c_master_write_to_device(
        I2C_PORT,
        VEML7700_ADDR,
        buffer,
        sizeof(buffer),
        pdMS_TO_TICKS(100)
    );
}

static esp_err_t veml7700_read_reg16(uint8_t reg, uint16_t &value)
{
    uint8_t data[2] = {};

    const esp_err_t ret = i2c_master_write_read_device(
        I2C_PORT,
        VEML7700_ADDR,
        &reg,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    value = static_cast<uint16_t>(data[0] | (data[1] << 8));
    return ESP_OK;
}

static void veml7700_init()
{
    const esp_err_t ret = veml7700_write_reg16(VEML7700_REG_ALS_CONF, VEML7700_ALS_CONF_VALUE);

    if (ret == ESP_OK) {
        veml7700_ok = true;
        ESP_LOGI(TAG, "VEML7700 initialized");
    } else {
        veml7700_ok = false;
        ESP_LOGW(TAG, "VEML7700 not found: %s", esp_err_to_name(ret));
    }
}

static esp_err_t veml7700_read_lux(float &lux)
{
    uint16_t raw_als = 0;

    const esp_err_t ret = veml7700_read_reg16(VEML7700_REG_ALS, raw_als);
    if (ret != ESP_OK) {
        return ret;
    }

    lux = raw_als * VEML7700_LUX_PER_COUNT;
    return ESP_OK;
}

// =========================
// PWM HEADLIGHT FUNCTIONS
// =========================
static void headlight_pwm_init()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = HEADLIGHT_LEDC_MODE;
    timer.timer_num = HEADLIGHT_LEDC_TIMER;
    timer.duty_resolution = HEADLIGHT_LEDC_RES;
    timer.freq_hz = HEADLIGHT_LEDC_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {};
    channel.gpio_num = HEADLIGHT_LED_GPIO;
    channel.speed_mode = HEADLIGHT_LEDC_MODE;
    channel.channel = HEADLIGHT_LEDC_CHANNEL;
    channel.timer_sel = HEADLIGHT_LEDC_TIMER;
    channel.duty = HEADLIGHT_OFF_DUTY;
    channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static void set_headlight_brightness(uint32_t duty)
{
    ledc_set_duty(HEADLIGHT_LEDC_MODE, HEADLIGHT_LEDC_CHANNEL, duty);
    ledc_update_duty(HEADLIGHT_LEDC_MODE, HEADLIGHT_LEDC_CHANNEL);
}

// =========================
// PWM BRAKE LIGHT FUNCTIONS
// =========================
static void brake_pwm_init()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = BRAKE_LEDC_MODE;
    timer.timer_num = BRAKE_LEDC_TIMER;
    timer.duty_resolution = BRAKE_LEDC_RES;
    timer.freq_hz = BRAKE_LEDC_FREQ_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {};
    channel.gpio_num = BRAKE_LED_GPIO;
    channel.speed_mode = BRAKE_LEDC_MODE;
    channel.channel = BRAKE_LEDC_CHANNEL;
    channel.timer_sel = BRAKE_LEDC_TIMER;
    channel.duty = BRAKE_OFF_DUTY;
    channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static void set_brake_brightness(uint32_t duty)
{
    ledc_set_duty(BRAKE_LEDC_MODE, BRAKE_LEDC_CHANNEL, duty);
    ledc_update_duty(BRAKE_LEDC_MODE, BRAKE_LEDC_CHANNEL);
}

// =========================
// ACCELEROMETER AXIS
// =========================
static float get_axis_value(float ax, float ay, float az, AccelAxis axis)
{
    switch (axis) {
        case AccelAxis::X:
            return ax;
        case AccelAxis::Y:
            return ay;
        case AccelAxis::Z:
        default:
            return az;
    }
}

static const char* axis_name(AccelAxis axis)
{
    switch (axis) {
        case AccelAxis::X:
            return "X";
        case AccelAxis::Y:
            return "Y";
        case AccelAxis::Z:
        default:
            return "Z";
    }
}

// =========================
// ACCELEROMETER CALIBRATION
// =========================
// Detemines which Axis corresponds to forward motion based on accelerometer set position
static void calibrate_forward_axis()
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    int good_samples = 0;

    ESP_LOGI(TAG, "Calibrating LIS3DH... keep bike still");

    // Averages Accelerometer readings to reduce noise and set a stable baseline
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        if (lis3dh_read_accel(ax, ay, az) == ESP_OK) {
            sum_x += ax;
            sum_y += ay;
            sum_z += az;
            good_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));      // Dealy to reduce noisy sampling
    }
    // Defaults Forward Axis to X-Axis if No Good Samples / Or Accelerometer doesn't properly read
    if (good_samples == 0) {
        forward_axis = AccelAxis::X;
        slow_forward_g = 0.0f;
        accel_calibrated = true;
        return;
    }
    // Calculates avg acceleration on each axis
    const float avg_x = sum_x / good_samples;
    const float avg_y = sum_y / good_samples;
    const float avg_z = sum_z / good_samples;

    const float abs_x = std::fabs(avg_x);
    const float abs_y = std::fabs(avg_y);
    const float abs_z = std::fabs(avg_z);

    // Set Axis of least magniude(least affected by gravity) to Forward Axis
    if (abs_x <= abs_y && abs_x <= abs_z) {
        forward_axis = AccelAxis::X;
        slow_forward_g = avg_x;
    } else if (abs_y <= abs_x && abs_y <= abs_z) {
        forward_axis = AccelAxis::Y;
        slow_forward_g = avg_y;
    } else {
        forward_axis = AccelAxis::Z;
        slow_forward_g = avg_z;
    }

    accel_calibrated = true;

    ESP_LOGI(TAG, "Forward axis estimated as %s", axis_name(forward_axis));
    ESP_LOGI(TAG, "avg_x=%.3f avg_y=%.3f avg_z=%.3f", avg_x, avg_y, avg_z);
}

// =========================
// HALL GPIO INTERRUPT HANDLER
// =========================
static void IRAM_ATTR hall_isr_handler(void *arg)
{
    const int64_t now_us = esp_timer_get_time();

    BaseType_t higher_priority_task_woken = pdFALSE;

    xQueueSendFromISR(
        hall_pulse_queue,
        &now_us,
        &higher_priority_task_woken
    );

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

// =========================
// HALL SPEEDOMETER INITIALIZATION
// =========================
static void hall_speedometer_init()
{
    hall_pulse_queue = xQueueCreate(10, sizeof(int64_t));
    if (hall_pulse_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create Hall pulse queue");
        return;
    }

    gpio_config_t hall_config = {};
    hall_config.pin_bit_mask = (1ULL << HALL_SENSOR_GPIO);
    hall_config.mode = GPIO_MODE_INPUT;
    hall_config.pull_up_en = GPIO_PULLUP_ENABLE;
    hall_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    hall_config.intr_type = GPIO_INTR_NEGEDGE;

    ESP_ERROR_CHECK(gpio_config(&hall_config));

    // Install GPIO ISR service.
    // ESP_ERR_INVALID_STATE means it was already installed somewhere else.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        HALL_SENSOR_GPIO,
        hall_isr_handler,
        nullptr
    ));

    ESP_LOGI(TAG, "Digital Hall interrupt initialized on GPIO %d", HALL_SENSOR_GPIO);
}

// Converts wheel-speed deceleration into a PWM brightness value.
// Larger negative acceleration gives a brighter brake light.
static uint32_t brake_duty_from_deceleration(float accel_mps2)
{
    if (accel_mps2 > DECEL_BRAKE_TRIGGER_MPS2) {
        return BRAKE_OFF_DUTY;
    }

    float t = (DECEL_BRAKE_TRIGGER_MPS2 - accel_mps2) /
              (DECEL_BRAKE_TRIGGER_MPS2 - DECEL_BRAKE_STRONG_MPS2);

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float duty = DECEL_BRAKE_MIN_DUTY +
        t * (BRAKE_FULL_DUTY - DECEL_BRAKE_MIN_DUTY);

    return static_cast<uint32_t>(duty);
}

// =========================
// HALL SPEEDOMETER UPDATE
// =========================
static void process_hall_pulse(int64_t now_us)
{
    // Reject fake pulses that happen too close together.
    if (last_pulse_us != 0 && (now_us - last_pulse_us) < MIN_PULSE_US) {
        return;
    }

    // Need two pulses to calculate speed.
    if (last_pulse_us != 0) {
        const int64_t dt_us = now_us - last_pulse_us;
        const float dt_s = dt_us / 1000000.0f;

        if (dt_s > 0.0f) {
            const float distance_per_pulse =
                WHEEL_CIRCUMFERENCE_M / MAGNETS_PER_REV;

            const float new_speed_mps = distance_per_pulse / dt_s;

            if (last_speed_update_us != 0) {
                const float speed_dt_s =
                    (now_us - last_speed_update_us) / 1000000.0f;

                if (speed_dt_s > 0.0f) {
                    speed_accel_mps2 =
                        (new_speed_mps - prev_speed_mps) / speed_dt_s;

                    const float new_speed_mph = new_speed_mps * 2.23694f;

                    // Wheel-speed deceleration controls how bright the brake light gets.
                    // The button can turn on dim brake light, but deceleration overrides it brighter.
                    if (new_speed_mph >= DECEL_BRAKE_MIN_SPEED_MPH &&
                        speed_accel_mps2 <= DECEL_BRAKE_TRIGGER_MPS2) {
                        decel_brake_duty = brake_duty_from_deceleration(speed_accel_mps2);
                        decel_brake_hold_until_ms =
                            (esp_timer_get_time() / 1000) + DECEL_BRAKE_HOLD_TIME_MS;
                    }
                }
            }

            prev_speed_mps = new_speed_mps;
            speed_mps = new_speed_mps;
            hall_speed_mph = speed_mps * 2.23694f;
            last_speed_update_us = now_us;
        }
    }

    last_pulse_us = now_us;
}

static void speedometer_task(void *arg)
{
    int64_t pulse_time_us = 0;

    while (1) {
        // Wait for a Hall pulse from the GPIO interrupt.
        // It wakes every 100 ms even if no pulse happens,
        if (xQueueReceive(hall_pulse_queue, &pulse_time_us, pdMS_TO_TICKS(100)) == pdTRUE) {
            process_hall_pulse(pulse_time_us);
        }

        // If no pulse has happened recently, set speed to 0.
        if (last_pulse_us != 0) {
            const int64_t now_us = esp_timer_get_time();

            if ((now_us - last_pulse_us) > STOP_TIMEOUT_US) {
                speed_mps = 0.0f;
                hall_speed_mph = 0.0f;
                prev_speed_mps = 0.0f;
                speed_accel_mps2 = 0.0f;
                decel_brake_duty = BRAKE_OFF_DUTY;
                decel_brake_hold_until_ms = 0;
                last_speed_update_us = now_us;
            }
        }
    }
}

// Updates the headlight, turn signals, speedometer, and brake light.
// All four light controls use momentary pushbuttons with internal pull-ups.
// Each new press toggles the matching light/state ON or OFF, like the turn signals.
static void bike_lights_update(Button &headlight_button,
                               Button &left_button,
                               Button &right_button,
                               Button &brake_button,
                               int64_t *last_blink_time_ms)
{
    const int64_t now = get_time_ms();

    // Headlight Pushbutton: press once ON, press again OFF.
    if (headlight_button.pressed()) {
        headlight_on = !headlight_on;
    }

    // Brake Pushbutton: press once ON, press again OFF.
    if (brake_button.pressed()) {
        brake_system_on = !brake_system_on;
    }

    // If the brake sysem was just turned OFF, reset all braking states
    if (last_brake_system_on && !brake_system_on) {
        set_brake_brightness(BRAKE_OFF_DUTY);
        accel_calibrated = false;
        fast_brake_count = 0;
        slow_brake_count = 0;
        brake_hold_until_ms = 0;
    }

    last_brake_system_on = brake_system_on;


    // Turn Signal Buttons: Only One Turn Signal on at a time
    if (left_button.pressed()) {
        if (left_signal_on) {
            left_signal_on = false;
        } else if (!right_signal_on) {
            left_signal_on = true;
        }
    }

    if (right_button.pressed()) {
        if (right_signal_on) {
            right_signal_on = false;
        } else if (!left_signal_on) {
            right_signal_on = true;
        }
    }

    if ((now - *last_blink_time_ms) >= BLINK_INTERVAL_MS) {
        blink_state = !blink_state;
        *last_blink_time_ms = now;
    }

    // Headlight control.
    // When USE_VEML7700 is 0, the missing light sensor is ignored and the switch simply turns
    // the headlight on at dim brightness.
    if (!headlight_on) {
        set_headlight_brightness(HEADLIGHT_OFF_DUTY);
    } else {
#if USE_VEML7700
        if ((now - last_ambient_read_ms) >= AMBIENT_READ_INTERVAL_MS) {
            float new_lux = 0.0f;

            if (veml7700_ok && veml7700_read_lux(new_lux) == ESP_OK) {
                ambient_lux = new_lux;

                // Buffer to prevent flicker if the lux value is near the threshold.
                if (ambient_lux < HEADLIGHT_DARK_THRESHOLD_LUX) {
                    headlight_auto_dark = true;
                } else if (ambient_lux > HEADLIGHT_BRIGHT_THRESHOLD_LUX) {
                    headlight_auto_dark = false;
                }
            } else {
                // Sensor fallback: keep the headlight dim instead of completely off.
                headlight_auto_dark = false;
            }

            last_ambient_read_ms = now;
        }

        if (headlight_auto_dark) {
            set_headlight_brightness(HEADLIGHT_FULL_DUTY);
        } else {
            set_headlight_brightness(HEADLIGHT_DIM_DUTY);
        }
#else
        // No VEML7700 connected: use manual dim headlight only.
        set_headlight_brightness(HEADLIGHT_DIM_DUTY);
#endif
    }

    // Brake Light Control
    // Brake button = dim steady light.
    // Speedometer deceleration = brighter light, scaled by how hard the bike slows down.
    const bool speed_decreasing_brake = (now <= decel_brake_hold_until_ms);
    uint32_t brake_duty = BRAKE_OFF_DUTY;

    if (brake_system_on) {
        brake_duty = BRAKE_DIM_DUTY;
    }

    if (speed_decreasing_brake && decel_brake_duty > brake_duty) {
        brake_duty = decel_brake_duty;
    }

    if (!speed_decreasing_brake) {
        decel_brake_duty = BRAKE_OFF_DUTY;
    }

    set_brake_brightness(brake_duty);

    if ((now - last_print_time_ms) >= BIKE_PRINT_INTERVAL_MS) {
        ESP_LOGI(TAG,
                 "lights headlight=%d left=%d right=%d brake_button=%d decel_brake=%d brake_duty=%lu speed=%.2f mph accel=%.2f m/s^2 lux=%.1f",
                 headlight_on,
                 left_signal_on,
                 right_signal_on,
                 brake_system_on,
                 speed_decreasing_brake,
                 static_cast<unsigned long>(brake_duty),
                 hall_speed_mph,
                 speed_accel_mps2,
                 ambient_lux);
        last_print_time_ms = now;
    }

    // Turn Signal Outputs: Blinking
    gpio_set_level(LEFT_SIGNAL_LED_GPIO, left_signal_on && blink_state ? 1 : 0);
    gpio_set_level(RIGHT_SIGNAL_LED_GPIO, right_signal_on && blink_state ? 1 : 0);
}

// FONT
static const uint8_t FONT_SPACE[5] = {0x00,0x00,0x00,0x00,0x00};
static const uint8_t FONT_DASH [5] = {0x08,0x08,0x08,0x08,0x08};
static const uint8_t FONT_DOT  [5] = {0x00,0x60,0x60,0x00,0x00};
static const uint8_t FONT_COLON[5] = {0x00,0x36,0x36,0x00,0x00};
static const uint8_t FONT_SLASH[5] = {0x20,0x10,0x08,0x04,0x02};

static const uint8_t FONT_0[5] = {0x3E,0x51,0x49,0x45,0x3E};
static const uint8_t FONT_1[5] = {0x00,0x42,0x7F,0x40,0x00};
static const uint8_t FONT_2[5] = {0x62,0x51,0x49,0x49,0x46};
static const uint8_t FONT_3[5] = {0x22,0x41,0x49,0x49,0x36};
static const uint8_t FONT_4[5] = {0x18,0x14,0x12,0x7F,0x10};
static const uint8_t FONT_5[5] = {0x2F,0x49,0x49,0x49,0x31};
static const uint8_t FONT_6[5] = {0x3E,0x49,0x49,0x49,0x32};
static const uint8_t FONT_7[5] = {0x01,0x71,0x09,0x05,0x03};
static const uint8_t FONT_8[5] = {0x36,0x49,0x49,0x49,0x36};
static const uint8_t FONT_9[5] = {0x26,0x49,0x49,0x49,0x3E};

static const uint8_t FONT_A[5] = {0x7E,0x11,0x11,0x11,0x7E};
static const uint8_t FONT_B[5] = {0x7F,0x49,0x49,0x49,0x36};
static const uint8_t FONT_C[5] = {0x3E,0x41,0x41,0x41,0x22};
static const uint8_t FONT_D[5] = {0x7F,0x41,0x41,0x22,0x1C};
static const uint8_t FONT_E[5] = {0x7F,0x49,0x49,0x49,0x41};
static const uint8_t FONT_F[5] = {0x7F,0x09,0x09,0x09,0x01};
static const uint8_t FONT_G[5] = {0x3E,0x41,0x49,0x49,0x7A};
static const uint8_t FONT_H[5] = {0x7F,0x08,0x08,0x08,0x7F};
static const uint8_t FONT_I[5] = {0x00,0x41,0x7F,0x41,0x00};
static const uint8_t FONT_J[5] = {0x20,0x40,0x41,0x3F,0x01};
static const uint8_t FONT_K[5] = {0x7F,0x08,0x14,0x22,0x41};
static const uint8_t FONT_L[5] = {0x7F,0x40,0x40,0x40,0x40};
static const uint8_t FONT_M[5] = {0x7F,0x02,0x0C,0x02,0x7F};
static const uint8_t FONT_N[5] = {0x7F,0x04,0x08,0x10,0x7F};
static const uint8_t FONT_O[5] = {0x3E,0x41,0x41,0x41,0x3E};
static const uint8_t FONT_P[5] = {0x7F,0x09,0x09,0x09,0x06};
static const uint8_t FONT_Q[5] = {0x3E,0x41,0x51,0x21,0x5E};
static const uint8_t FONT_R[5] = {0x7F,0x09,0x19,0x29,0x46};
static const uint8_t FONT_S[5] = {0x46,0x49,0x49,0x49,0x31};
static const uint8_t FONT_T[5] = {0x01,0x01,0x7F,0x01,0x01};
static const uint8_t FONT_U[5] = {0x3F,0x40,0x40,0x40,0x3F};
static const uint8_t FONT_V[5] = {0x1F,0x20,0x40,0x20,0x1F};
static const uint8_t FONT_W[5] = {0x7F,0x20,0x18,0x20,0x7F};
static const uint8_t FONT_X[5] = {0x63,0x14,0x08,0x14,0x63};
static const uint8_t FONT_Y[5] = {0x07,0x08,0x70,0x08,0x07};
static const uint8_t FONT_Z[5] = {0x61,0x51,0x49,0x45,0x43};

// Returns the 5-column bitmap pattern for one printable character.
static const uint8_t *glyph_for(char c)
{
    c = (char)toupper((unsigned char)c);
    switch (c) {
        case ' ': return FONT_SPACE;
        case '-': return FONT_DASH;
        case '.': return FONT_DOT;
        case ':': return FONT_COLON;
        case '/': return FONT_SLASH;
        case '0': return FONT_0; case '1': return FONT_1; case '2': return FONT_2;
        case '3': return FONT_3; case '4': return FONT_4; case '5': return FONT_5;
        case '6': return FONT_6; case '7': return FONT_7; case '8': return FONT_8;
        case '9': return FONT_9;
        case 'A': return FONT_A; case 'B': return FONT_B; case 'C': return FONT_C;
        case 'D': return FONT_D; case 'E': return FONT_E; case 'F': return FONT_F;
        case 'G': return FONT_G; case 'H': return FONT_H; case 'I': return FONT_I;
        case 'J': return FONT_J; case 'K': return FONT_K; case 'L': return FONT_L;
        case 'M': return FONT_M; case 'N': return FONT_N; case 'O': return FONT_O;
        case 'P': return FONT_P; case 'Q': return FONT_Q; case 'R': return FONT_R;
        case 'S': return FONT_S; case 'T': return FONT_T; case 'U': return FONT_U;
        case 'V': return FONT_V; case 'W': return FONT_W; case 'X': return FONT_X;
        case 'Y': return FONT_Y; case 'Z': return FONT_Z;
        default: return FONT_SPACE;
    }
}

// Holds the most recent GPS information parsed from NMEA sentences.
struct GpsData {
    bool fix_valid = false;
    bool location_valid = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double speed_mph = 0.0;
    double altitude_m = 0.0;
    int satellites_used = 0;
    int satellites_in_view = 0;
    int fix_quality = 0;
    char utc_time[16] = "--:--:--";
    char utc_date[16] = "----/--/--";
};

static GpsData gps;
static int g_prev_map_px = -1;
static int g_prev_map_py = -1;

// Sets the local timezone to Pacific Time with daylight-saving rules.
static void set_pacific_time_zone()
{
    setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();
}

// Converts GPS UTC date/time strings into Pacific local date/time strings.
static bool gps_utc_to_pacific(const char *utc_date,
                               const char *utc_time,
                               char *date_out,
                               size_t date_out_size,
                               char *time_out,
                               size_t time_out_size)
{
    int year = 0, month = 0, day = 0;
    int hour = 0, minute = 0, second = 0;
    if (sscanf(utc_date, "%d/%d/%d", &year, &month, &day) != 3) return false;
    if (sscanf(utc_time, "%d:%d:%d", &hour, &minute, &second) != 3) return false;

    struct tm tm_utc = {};
    tm_utc.tm_year = year - 1900;
    tm_utc.tm_mon  = month - 1;
    tm_utc.tm_mday = day;
    tm_utc.tm_hour = hour;
    tm_utc.tm_min  = minute;
    tm_utc.tm_sec  = second;
    tm_utc.tm_isdst = 0;

    setenv("TZ", "UTC0", 1);
    tzset();
    time_t epoch = mktime(&tm_utc);
    if (epoch == (time_t)-1) {
        set_pacific_time_zone();
        return false;
    }

    set_pacific_time_zone();
    struct tm tm_local = {};
    localtime_r(&epoch, &tm_local);
    strftime(date_out, date_out_size, "%Y/%m/%d", &tm_local);
    strftime(time_out, time_out_size, "%H:%M:%S", &tm_local);
    return true;
}

// Estimates distance between two GPS coordinates in meters.
// This uses a local flat-earth approximation, which is fine for short hazard distances.
static double distance_meters(double lat1, double lon1, double lat2, double lon2)
{
    double lat_mid_rad = ((lat1 + lat2) * 0.5) * (M_PI / 180.0);
    double dx = (lon2 - lon1) * 111320.0 * cos(lat_mid_rad);
    double dy = (lat2 - lat1) * 110540.0;
    return sqrt(dx * dx + dy * dy);
}

// Saves the current hazard list to ESP32 NVS flash memory.
static esp_err_t hazards_save_to_nvs()
{
    // Pack all hazards into one blob before writing it to flash.
    HazardStoreBlob blob = {};
    blob.magic = HAZARD_STORE_MAGIC;
    blob.count = (uint32_t)g_hazard_count;
    for (int i = 0; i < g_hazard_count; i++) {
        blob.hazards[i] = g_hazards[i];
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(HAZARD_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, HAZARD_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

// Loads saved hazards from NVS flash memory during startup.
static void hazards_load_from_nvs()
{
    g_hazard_count = 0;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(HAZARD_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved hazards yet");
        return;
    }

    HazardStoreBlob blob = {};
    size_t size = sizeof(blob);
    err = nvs_get_blob(handle, HAZARD_NVS_KEY, &blob, &size);
    nvs_close(handle);

    if (err != ESP_OK || size != sizeof(blob) || blob.magic != HAZARD_STORE_MAGIC) {
        ESP_LOGW(TAG, "Hazard storage invalid or empty");
        return;
    }

    if (blob.count > MAX_HAZARDS) {
        ESP_LOGW(TAG, "Hazard count invalid: %u", (unsigned)blob.count);
        return;
    }

    g_hazard_count = (int)blob.count;
    for (int i = 0; i < g_hazard_count; i++) {
        g_hazards[i] = blob.hazards[i];
    }

    ESP_LOGI(TAG, "Loaded %d saved hazards", g_hazard_count);
}

// Adds the current GPS position as a hazard if GPS is valid and it is not a duplicate.
static bool hazards_add_current_location()
{
    if (!gps.fix_valid || !gps.location_valid) {
        ESP_LOGW(TAG, "Cannot add hazard: no valid GPS fix");
        return false;
    }

    // Reject duplicate hazards that are already close to the current GPS location.
    for (int i = 0; i < g_hazard_count; i++) {
        double d = distance_meters(gps.latitude, gps.longitude,
                                   g_hazards[i].latitude, g_hazards[i].longitude);
        if (d <= HAZARD_DUPLICATE_RADIUS_M) {
            ESP_LOGI(TAG, "Hazard already exists nearby (%.1f m)", d);
            return false;
        }
    }

    if (g_hazard_count >= MAX_HAZARDS) {
        ESP_LOGW(TAG, "Hazard list full");
        return false;
    }

    // Store the new hazard at the current GPS coordinates.
    g_hazards[g_hazard_count].latitude = gps.latitude;
    g_hazards[g_hazard_count].longitude = gps.longitude;
    g_hazard_count++;

    esp_err_t err = hazards_save_to_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save hazards: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Hazard saved at lat=%.6f lon=%.6f (count=%d)",
             gps.latitude, gps.longitude, g_hazard_count);
    return true;
}


// Returns the distance to the closest saved hazard from the current GPS position.
static double hazards_nearest_distance_current_location()
{
    if (g_hazard_count <= 0 || !gps.fix_valid || !gps.location_valid) {
        return 1e30;
    }

    double best_distance = 1e30;
    for (int i = 0; i < g_hazard_count; i++) {
        double d = distance_meters(gps.latitude, gps.longitude,
                                   g_hazards[i].latitude, g_hazards[i].longitude);
        if (d < best_distance) best_distance = d;
    }
    return best_distance;
}

// Removes the closest hazard only if it is within HAZARD_CLEAR_RADIUS_M.
static bool hazards_remove_nearest_current_location()
{
    if (g_hazard_count <= 0) {
        ESP_LOGI(TAG, "No hazards to clear");
        return false;
    }
    if (!gps.fix_valid || !gps.location_valid) {
        ESP_LOGW(TAG, "Cannot clear hazard: no valid GPS fix");
        return false;
    }

    // Search for the nearest saved hazard.
    int best_index = -1;
    double best_distance = 1e30;
    for (int i = 0; i < g_hazard_count; i++) {
        double d = distance_meters(gps.latitude, gps.longitude,
                                   g_hazards[i].latitude, g_hazards[i].longitude);
        if (d < best_distance) {
            best_distance = d;
            best_index = i;
        }
    }

    if (best_index < 0 || best_distance > HAZARD_CLEAR_RADIUS_M) {
        ESP_LOGI(TAG, "No saved hazard within %.1f m to clear (nearest %.1f m)",
                 HAZARD_CLEAR_RADIUS_M, best_distance);
        return false;
    }

    // Shift later hazards down to remove the selected hazard from the array.
    for (int i = best_index; i < g_hazard_count - 1; i++) {
        g_hazards[i] = g_hazards[i + 1];
    }
    g_hazard_count--;

    esp_err_t err = hazards_save_to_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save hazards after clear: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Cleared nearest hazard (%.1f m away). Remaining=%d", best_distance, g_hazard_count);
    return true;
}

// Configures the hazard add/clear buttons as pull-up inputs.
static void buttons_init()
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_HAZARD_ADD) | (1ULL << PIN_HAZARD_CLEAR);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

// Debounces one button and returns true only once per new press.
static bool consume_button_press(gpio_num_t pin, bool *prev_pressed, TickType_t *last_tick)
{
    // Because pull-ups are enabled, LOW means the physical button is pressed.
    bool pressed_now = (gpio_get_level(pin) == 0);
    TickType_t now = xTaskGetTickCount();
    bool fired = false;

    if (pressed_now && !(*prev_pressed)) {
        if ((now - *last_tick) >= pdMS_TO_TICKS(250)) {
            fired = true;
            *last_tick = now;
        }
    }

    *prev_pressed = pressed_now;
    return fired;
}

// Fills the entire LCD with one color. Used for startup/error screens.
static void fill_screen(uint16_t color)
{
    for (int i = 0; i < LCD_H_RES * DRAW_LINES; i++) g_fill_buf[i] = color;
    // Send the screen in chunks instead of one huge transfer.
    for (int y = 0; y < LCD_V_RES; y += DRAW_LINES) {
        int y_end = std::min(y + DRAW_LINES, LCD_V_RES);
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y_end, g_fill_buf);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

// Fills a rectangular area on the LCD with one color.
static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) return;
    if (x < 0 || y < 0) return;
    if (x + w > LCD_H_RES || y + h > LCD_V_RES) return;

    for (int i = 0; i < w * DRAW_LINES; i++) g_fill_buf[i] = color;
    for (int row = y; row < y + h; row += DRAW_LINES) {
        int lines = std::min(DRAW_LINES, (y + h) - row);
        esp_lcd_panel_draw_bitmap(panel_handle, x, row, x + w, row + lines, g_fill_buf);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Draws one scaled text character using the small built-in bitmap font.
static void draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    const uint8_t *glyph = glyph_for(c);
    const int char_w = 6 * scale;
    const int char_h = 8 * scale;

    for (int i = 0; i < char_w * char_h; i++) g_char_buf[i] = bg;

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (!((glyph[col] >> row) & 0x01)) continue;
            for (int dx = 0; dx < scale; dx++) {
                for (int dy = 0; dy < scale; dy++) {
                    int px = col * scale + dx;
                    int py = row * scale + dy;
                    g_char_buf[py * char_w + px] = fg;
                }
            }
        }
    }

    esp_lcd_panel_draw_bitmap(panel_handle, x, y, x + char_w, y + char_h, g_char_buf);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Draws a string by drawing one character at a time.
static void draw_text(int x, int y, const char *text, uint16_t fg, uint16_t bg, int scale)
{
    int cursor_x = x;
    while (*text) {
        draw_char(cursor_x, y, *text, fg, bg, scale);
        cursor_x += 6 * scale;
        text++;
    }
}

// Helper functions used to verify the embedded map has the expected byte size.
static size_t map_size_bytes() { return (size_t)map_rgb565_len; }
static size_t expected_map_bytes() { return (size_t)MAP_W * (size_t)MAP_H * 2u; }
static bool map_is_valid() { return map_size_bytes() >= expected_map_bytes(); }

// Keeps a value inside a minimum and maximum range.
static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Converts longitude into an X pixel location on the full map image.
static int lon_to_map_x(double lon)
{
    // Normalize longitude into a 0.0 to 1.0 position across the map width.
    double t = (lon - MAP_LEFT_LON) / (MAP_RIGHT_LON - MAP_LEFT_LON);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    int x = (int)(t * (MAP_W - 1)) + MAP_CAL_OFFSET_X;
    return clampi(x, 0, MAP_W - 1);
}

// Converts latitude into a Y pixel location on the full map image.
static int lat_to_map_y(double lat)
{
    // Normalize latitude into a 0.0 to 1.0 position down the map height.
    // Top of the image is larger latitude, so the subtraction is reversed.
    double t = (MAP_TOP_LAT - lat) / (MAP_TOP_LAT - MAP_BOTTOM_LAT);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    int y = (int)(t * (MAP_H - 1)) + MAP_CAL_OFFSET_Y;
    return clampi(y, 0, MAP_H - 1);
}

// Draws the rider marker on top of the map.
// The patch buffer prevents overwriting more of the map than necessary.
static void draw_user_marker_patch_from_source(int screen_x, int screen_y, int view_x, int view_y)
{
    static constexpr int R = 6;
    static constexpr int W = 2 * R + 1;
    static constexpr int H = 2 * R + 1;

    (void)view_x;
    (void)view_y;

    if (!map_is_valid()) return;
    if (screen_x < R || screen_y < R || screen_x >= (LCD_H_RES - R) || screen_y >= (MAP_VIEW_H - R)) return;

    for (int i = 0; i < W * H; i++) {
        g_patch_buf[i] = COLOR_USER_BOX;
    }

    for (int y = -R; y <= R; y++) {
        for (int x = -R; x <= R; x++) {
            int d2 = x * x + y * y;
            if (d2 <= (R * R) && d2 >= ((R - 2) * (R - 2))) {
                g_patch_buf[(y + R) * W + (x + R)] = COLOR_YELLOW;
            }
        }
    }
    for (int y = -2; y <= 2; y++) {
        for (int x = -2; x <= 2; x++) {
            if ((x * x + y * y) <= 4) {
                g_patch_buf[(y + R) * W + (x + R)] = COLOR_RED;
            }
        }
    }

    esp_lcd_panel_draw_bitmap(panel_handle, screen_x - R, screen_y - R, screen_x + R + 1, screen_y + R + 1, g_patch_buf);
}

// Draws one hazard marker on top of the map.
static void draw_hazard_marker_patch_from_source(int screen_x, int screen_y, int view_x, int view_y)
{
    static constexpr int R = 4;
    static constexpr int W = 2 * R + 1;
    static constexpr int H = 2 * R + 1;

    (void)view_x;
    (void)view_y;

    if (!map_is_valid()) return;
    if (screen_x < R || screen_y < R || screen_x >= (LCD_H_RES - R) || screen_y >= (MAP_VIEW_H - R)) return;

    for (int i = 0; i < W * H; i++) {
        g_patch_buf[i] = COLOR_HAZARD_BOX;
    }

    for (int i = -R; i <= R; i++) {
        g_patch_buf[(R) * W + (i + R)] = COLOR_HAZARD;
        g_patch_buf[(i + R) * W + (R)] = COLOR_HAZARD;
    }
    g_patch_buf[R * W + R] = COLOR_GREEN;

    esp_lcd_panel_draw_bitmap(panel_handle, screen_x - R, screen_y - R, screen_x + R + 1, screen_y + R + 1, g_patch_buf);
}

// Draws a map window centered around the given full-map pixel coordinate.
// Then it overlays hazards and the rider marker.
static void draw_centered_view(int map_px, int map_py)
{
    if (!map_is_valid()) {
        fill_screen(COLOR_BLACK);
        draw_text(10, 10, "MAP FILE ERROR", COLOR_YELLOW, COLOR_BLACK, 2);
        return;
    }

    // For zoom > 1.0, we sample a smaller source window from the big map
    // and scale it up to the LCD map area.
    int view_x = clampi(map_px - (VIEW_SRC_W / 2), 0, MAP_W - VIEW_SRC_W);
    int view_y = clampi(map_py - (VIEW_SRC_H / 2), 0, MAP_H - VIEW_SRC_H);

    const uint16_t *map16 = reinterpret_cast<const uint16_t *>(map_rgb565);

    for (int y = 0; y < MAP_VIEW_H; y += DRAW_LINES) {
        int lines = std::min(DRAW_LINES, MAP_VIEW_H - y);
        for (int row = 0; row < lines; row++) {
            int screen_y = y + row;
            int src_y = view_y + (screen_y * VIEW_SRC_H) / MAP_VIEW_H;
            const uint16_t *src_row = map16 + (src_y * MAP_W);

            uint16_t *dst_row = &g_fill_buf[row * LCD_H_RES];
            for (int x = 0; x < LCD_H_RES; x++) {
                int src_x = view_x + (x * VIEW_SRC_W) / LCD_H_RES;
                dst_row[x] = src_row[src_x];
            }
        }
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, LCD_H_RES, y + lines, g_fill_buf);
    }

    // Draw all saved hazards that fall inside the current zoomed view.
    for (int i = 0; i < g_hazard_count; i++) {
        int hx = lon_to_map_x(g_hazards[i].longitude);
        int hy = lat_to_map_y(g_hazards[i].latitude);
        int sx = (int)lround(((double)(hx - view_x) * LCD_H_RES) / VIEW_SRC_W);
        int sy = (int)lround(((double)(hy - view_y) * MAP_VIEW_H) / VIEW_SRC_H);
        draw_hazard_marker_patch_from_source(sx, sy, view_x, view_y);
    }

    // Draw the rider marker last so it appears above the map and hazards.
    int screen_x = (int)lround(((double)(map_px - view_x) * LCD_H_RES) / VIEW_SRC_W);
    int screen_y = (int)lround(((double)(map_py - view_y) * MAP_VIEW_H) / VIEW_SRC_H);
    draw_user_marker_patch_from_source(screen_x, screen_y, view_x, view_y);
}

// Updates the bottom status bar with speed, hazard count, and GPS/time status.
static void draw_status_bar()
{
    char line1[32];
    char line2[32];
    char local_time[16] = "--:--:--";
    char local_date[16] = "----/--/--";

    bool time_ok = gps_utc_to_pacific(gps.utc_date, gps.utc_time,
                                      local_date, sizeof(local_date),
                                      local_time, sizeof(local_time));

    // First status line shows speed and number of saved hazards.
    snprintf(line1, sizeof(line1), "SPD %.1fMPH HZ %d", hall_speed_mph, g_hazard_count);
    // Second status line shows GPS fix state and local Pacific time when available.
    if (gps.fix_valid && time_ok) snprintf(line2, sizeof(line2), "FIX %s", local_time);
    else if (gps.fix_valid) snprintf(line2, sizeof(line2), "FIX ACQUIRED");
    else snprintf(line2, sizeof(line2), "WAITING FOR FIX");

    fill_rect(0, STATUS_BAR_Y, LCD_H_RES, STATUS_BAR_H, COLOR_BLACK);
    draw_text(6, STATUS_BAR_Y + 6,  line1, COLOR_GREEN, COLOR_BLACK, 2);
    draw_text(6, STATUS_BAR_Y + 26, line2, COLOR_CYAN,  COLOR_BLACK, 2);
}

// Redraws the current map view after hazards are added/cleared.
static void refresh_current_view()
{
    if (!map_is_valid()) return;

    int map_px = (g_prev_map_px >= 0) ? g_prev_map_px : (MAP_W / 2);
    int map_py = (g_prev_map_py >= 0) ? g_prev_map_py : (MAP_H / 2);

    if (gps.fix_valid && gps.location_valid) {
        map_px = lon_to_map_x(gps.longitude);
        map_py = lat_to_map_y(gps.latitude);
    }

    draw_centered_view(map_px, map_py);
    draw_status_bar();
    g_prev_map_px = map_px;
    g_prev_map_py = map_py;
}

// Initializes the SPI bus, LCD panel driver, screen orientation, and DMA drawing buffers.
static void tft_init()
{
    // Set up the SPI bus used by the LCD. Only MOSI/SCLK are needed for writing pixels.
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = PIN_LCD_SCLK;
    buscfg.mosi_io_num = PIN_LCD_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * DRAW_LINES * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = PIN_LCD_CS;
    io_config.dc_gpio_num = PIN_LCD_DC;
    io_config.spi_mode = 0;
    // LCD SPI clock. If the display looks smeary/unstable, lower this value.
    io_config.pclk_hz = 20 * 1000 * 1000;
    io_config.trans_queue_depth = 1;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = PIN_LCD_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
    panel_config.bits_per_pixel = 16;

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Allocate DMA-capable buffers. LCD transfers require memory the SPI DMA engine can access.
    g_fill_buf = (uint16_t *)heap_caps_malloc(LCD_H_RES * DRAW_LINES * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_char_buf = (uint16_t *)heap_caps_malloc((6 * 5) * (8 * 5) * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    g_patch_buf = (uint16_t *)heap_caps_malloc((13 * 13) * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!g_fill_buf || !g_char_buf || !g_patch_buf) {
        ESP_LOGE(TAG, "DMA buffer alloc failed");
        abort();
    }
}

// Converts one hexadecimal character into an integer value. Used for NMEA checksum parsing.
static int hex_to_int(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)toupper((unsigned char)c);
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

// Verifies the NMEA checksum at the end of a GPS sentence.
static bool nmea_checksum_ok(const char *s)
{
    if (!s || s[0] != '$') return false;
    const char *star = strchr(s, '*');
    if (!star || !star[1] || !star[2]) return false;
    // NMEA checksum is XOR of all characters between $ and *.
    unsigned char sum = 0;
    for (const char *p = s + 1; p < star; ++p) sum ^= (unsigned char)(*p);
    int hi = hex_to_int(star[1]);
    int lo = hex_to_int(star[2]);
    if (hi < 0 || lo < 0) return false;
    return sum == (unsigned char)((hi << 4) | lo);
}

// Splits an NMEA sentence into comma-separated fields in-place.
static int split_nmea_fields(char *s, char *fields[], int max_fields)
{
    int count = 0;
    if (max_fields <= 0) return 0;
    fields[count++] = s;
    while (*s && count < max_fields) {
        if (*s == ',') {
            *s = '\0';
            fields[count++] = s + 1;
        } else if (*s == '*') {
            *s = '\0';
            break;
        }
        s++;
    }
    return count;
}

// Checks whether a GPS sentence header ends with a specific 3-letter type like RMC/GGA/GSV.
static bool sentence_type_is(const char *header, const char *type3)
{
    size_t n = strlen(header);
    return n >= 3 && strcmp(header + n - 3, type3) == 0;
}

// Converts NMEA coordinates from ddmm.mmmm format into normal decimal degrees.
static double nmea_to_decimal(const char *val_str, const char *dir_str)
{
    if (!val_str || !dir_str || val_str[0] == '\0' || dir_str[0] == '\0') return 0.0;
    // Example: 3828.1234 means 38 degrees + 28.1234 minutes.
    double raw = atof(val_str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    char dir = dir_str[0];
    if (dir == 'S' || dir == 'W') decimal = -decimal;
    return decimal;
}

// Formats raw GPS time hhmmss into hh:mm:ss.
static void format_utc_time(const char *raw, char *out, size_t out_size)
{
    if (!raw || strlen(raw) < 6) { snprintf(out, out_size, "--:--:--"); return; }
    snprintf(out, out_size, "%c%c:%c%c:%c%c", raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
}

// Formats raw GPS date ddmmyy into yyyy/mm/dd.
static void format_utc_date(const char *raw, char *out, size_t out_size)
{
    if (!raw || strlen(raw) < 6) { snprintf(out, out_size, "----/--/--"); return; }
    int year = 2000 + ((raw[4] - '0') * 10 + (raw[5] - '0'));
    snprintf(out, out_size, "%04d/%c%c/%c%c", year, raw[2], raw[3], raw[0], raw[1]);
}

// Parses RMC sentences: fix status, date/time, position, and speed.
static void parse_rmc(char *fields[], int count)
{
    if (count < 10) return;
    format_utc_time(fields[1], gps.utc_time, sizeof(gps.utc_time));
    format_utc_date(fields[9], gps.utc_date, sizeof(gps.utc_date));
    gps.fix_valid = (fields[2][0] == 'A');
    if (fields[3][0] && fields[4][0] && fields[5][0] && fields[6][0]) {
        gps.latitude = nmea_to_decimal(fields[3], fields[4]);
        gps.longitude = nmea_to_decimal(fields[5], fields[6]);
        gps.location_valid = true;
    }
    if (fields[7][0]) gps.speed_mph = atof(fields[7]) * 1.15078;
}

// Parses GGA sentences: fix quality, satellites used, position, and altitude.
static void parse_gga(char *fields[], int count)
{
    if (count < 10) return;
    format_utc_time(fields[1], gps.utc_time, sizeof(gps.utc_time));
    if (fields[2][0] && fields[3][0] && fields[4][0] && fields[5][0]) {
        gps.latitude = nmea_to_decimal(fields[2], fields[3]);
        gps.longitude = nmea_to_decimal(fields[4], fields[5]);
        gps.location_valid = true;
    }
    gps.fix_quality = atoi(fields[6]);
    gps.fix_valid = (gps.fix_quality > 0);
    if (fields[7][0]) gps.satellites_used = atoi(fields[7]);
    if (fields[9][0]) gps.altitude_m = atof(fields[9]);
}

// Parses GSV sentences: satellites currently in view.
static void parse_gsv(char *fields[], int count)
{
    if (count >= 4 && fields[3][0]) gps.satellites_in_view = atoi(fields[3]);
}

// Receives one complete NMEA sentence, validates it, then sends it to the correct parser.
static void process_nmea_sentence(const char *sentence)
{
    if (!nmea_checksum_ok(sentence)) return;
    char temp[160];
    strncpy(temp, sentence, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    char *fields[24];
    int count = split_nmea_fields(temp, fields, 24);
    if (count <= 0) return;
    if (sentence_type_is(fields[0], "RMC")) parse_rmc(fields, count);
    else if (sentence_type_is(fields[0], "GGA")) parse_gga(fields, count);
    else if (sentence_type_is(fields[0], "GSV")) parse_gsv(fields, count);
}

// Configures UART1 to receive NMEA text from the GPS module.
static void gps_uart_init()
{
    // UART settings for typical NEO-6M GPS output.
    uart_config_t uart_config = {};
    uart_config.baud_rate = 9600;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_TX_GPIO, GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_flush_input(GPS_UART);
}

// Initializes NVS flash and loads previously saved hazard markers.
static void persistent_storage_init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    hazards_load_from_nvs();
}

// ESP-IDF entry point. This is where setup happens, then the infinite firmware loop runs.
extern "C" void app_main(void)
{
    // Reduce log noise so Serial Monitor mainly shows useful status prints.
    esp_log_level_set("*", ESP_LOG_WARN);
    set_pacific_time_zone();

    // Initialize each subsystem before entering the main loop.
    persistent_storage_init();
    tft_init();
    gps_uart_init();
    buttons_init();
    buzzer_init();
    bike_gpio_init();
#if USE_ACCELEROMETER || USE_VEML7700
    // LIS3DH and VEML7700 share this I2C bus, but each sensor is initialized separately.
    i2c_bus_init();
#endif

#if USE_ACCELEROMETER
    // Only configure the LIS3DH if it is physically connected.
    i2c_lis3dh_init();
#endif

#if USE_VEML7700
    veml7700_init();
#else
    veml7700_ok = false;
#endif
    headlight_pwm_init();
    brake_pwm_init();
    hall_speedometer_init();

    xTaskCreate(
        speedometer_task,
        "speedometer_task",
        4096,
        nullptr,
        1,
        nullptr
    );
    
    printf("\nGPS MAP TFT started\n");
    printf("map_rgb565_len=%u\n", (unsigned)map_size_bytes());
    printf("expected_len=%u\n", (unsigned)expected_map_bytes());
    printf("Loaded hazard count=%d\n", g_hazard_count);

    if (!map_is_valid()) {
        fill_screen(COLOR_BLACK);
        draw_text(10, 10, "MAP FILE ERROR", COLOR_YELLOW, COLOR_BLACK, 2);
    } else {
        int startup_px = MAP_W / 2;
        int startup_py = MAP_H / 2;
        draw_centered_view(startup_px, startup_py);
        draw_status_bar();
    }

    // Buffer used to collect one complete NMEA sentence from the GPS.
    char line[160];
    int line_idx = 0;
    uint8_t ch = 0;
    TickType_t last_lcd_update = xTaskGetTickCount();

    Button headlight_button;
    Button left_button;
    Button right_button;
    Button brake_button;

    headlight_button.init(HEADLIGHT_BUTTON_GPIO);
    left_button.init(LEFT_BUTTON_GPIO);
    right_button.init(RIGHT_BUTTON_GPIO);
    brake_button.init(BRAKE_BUTTON_GPIO);

    int64_t last_blink_time_ms = get_time_ms();

    while (true) {
        // Update bike lights and speedometer every loop
        bike_lights_update(headlight_button, left_button, right_button, brake_button, &last_blink_time_ms);

        // Read GPS one byte at a time and assemble full NMEA sentences.
        int len = uart_read_bytes(GPS_UART, &ch, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            if (ch == '$') {
                line_idx = 0;
                line[line_idx++] = '$';
            } else if (line_idx > 0) {
                if (ch == '\n') {
                    line[line_idx] = '\0';
                    process_nmea_sentence(line);
                    line_idx = 0;
                } else if (ch != '\r') {
                    if (line_idx < (int)sizeof(line) - 1) line[line_idx++] = (char)ch;
                    else line_idx = 0;
                }
            }
        }

        // Set this when a button changes the hazard list and the map needs a redraw.
        bool view_needs_refresh = false;

        // Drop hazard button: save current GPS location as a new marker.
        if (consume_button_press(PIN_HAZARD_ADD, &g_prev_add_pressed, &g_last_add_tick)) {
            if (hazards_add_current_location()) {
                play_drop_hazard_sound();
                view_needs_refresh = true;
            }
        }

        // Clear hazard button: remove the nearest saved hazard if close enough.
        if (consume_button_press(PIN_HAZARD_CLEAR, &g_prev_clear_pressed, &g_last_clear_tick)) {
            if (hazards_remove_nearest_current_location()) {
                play_clear_hazard_sound();
                view_needs_refresh = true;
            }
        }

        // Check whether the rider is near any saved hazard and play a warning sound.
        double nearest_hazard_m = hazards_nearest_distance_current_location();
        bool near_hazard_now = (nearest_hazard_m <= HAZARD_NEAR_RADIUS_M);
        TickType_t now_tick = xTaskGetTickCount();
        if (near_hazard_now) {
            if (!g_prev_near_hazard || (now_tick - g_last_near_buzz_tick) >= pdMS_TO_TICKS(HAZARD_NEAR_REPEAT_MS)) {
                play_near_hazard_sound();
                g_last_near_buzz_tick = now_tick;
            }
        }
        g_prev_near_hazard = near_hazard_now;

        // Refresh LCD/status and print Serial Monitor data roughly once per second.
        if ((xTaskGetTickCount() - last_lcd_update) >= pdMS_TO_TICKS(1000)) {
            if (gps.fix_valid && gps.location_valid) {
                int map_px = lon_to_map_x(gps.longitude);
                int map_py = lat_to_map_y(gps.latitude);
                if (map_px != g_prev_map_px || map_py != g_prev_map_py) {
                    draw_centered_view(map_px, map_py);
                    g_prev_map_px = map_px;
                    g_prev_map_py = map_py;
                }
            }
            draw_status_bar();

            char pacific_date[16] = "----/--/--";
            char pacific_time[16] = "--:--:--";
            gps_utc_to_pacific(gps.utc_date, gps.utc_time, pacific_date, sizeof(pacific_date), pacific_time, sizeof(pacific_time));
            printf("FIX:%s PACIFIC:%s %s LAT:%.6f LON:%.6f SAT:%d/%d GPS_SPD:%.1fMPH WHEEL:%.1fMPH HZ:%d\n",
                   gps.fix_valid ? "YES" : "NO",
                   pacific_date,
                   pacific_time,
                   gps.latitude,
                   gps.longitude,
                   gps.satellites_used,
                   gps.satellites_in_view,
                   gps.speed_mph,
                   hall_speed_mph,
                   g_hazard_count);
            last_lcd_update = xTaskGetTickCount();
        }

        // Redraw immediately after add/clear so marker changes show without waiting.
        if (view_needs_refresh) {
            refresh_current_view();
        }
    }
}
