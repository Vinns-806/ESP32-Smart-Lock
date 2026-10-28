#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"

static const char* TAG = "SMART_LOCK";

// ==================== 配置区 ====================
#define LED_GPIO            7   // 绿灯（开锁指示），原 GPIO10 疑似损坏
#define RED_LED_GPIO        6   // 红灯（错误指示）
#define TARGET_NAME         "MY_PHONE_BLE" // 目标蓝牙名

// I2C OLED 引脚
#define I2C_MASTER_SCL_IO   5
#define I2C_MASTER_SDA_IO   15
#define I2C_MASTER_NUM      I2C_NUM_0
#define OLED_ADDR           0x3C

// 矩阵键盘引脚
#define KEYPAD_ROW_NUM      4
#define KEYPAD_COL_NUM      4
static const gpio_num_t KEYPAD_ROW_PINS[KEYPAD_ROW_NUM] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};
static const gpio_num_t KEYPAD_COL_PINS[KEYPAD_COL_NUM] = {GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18, GPIO_NUM_21};

// 键盘映射表
static const char KEYPAD_MAP[KEYPAD_ROW_NUM][KEYPAD_COL_NUM] = {
    {'4', '8', 'B', '#'},
    {'3', '7', 'A', 'D'},
    {'2', '6', '0', 'C'},
    {'1', '5', '9', '*'}
};

// 舵机 PWM 引脚
#define SERVO_GPIO       14
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_CHANNEL     LEDC_CHANNEL_0
#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_FREQ        50
#define LEDC_RESOLUTION  10

#define PASSWORD        "123456"
#define PASSWORD_LEN    6
#define LOCKOUT_TIME_MS 30000
#define UNLOCK_TIME_MS  3000

// ==================== 核心阈值定义 ====================
#define BLE_OPEN_RSSI    -55  // 滤波均值达到此值开门（先设 -55，实测再调）
#define BLE_CLOSE_RSSI   -65  // 瞬时原始值跌破此值，强制关门
#define BLE_LOST_MS      800  // 超过该毫秒数没收到广播包即判定离开

// ==================== 系统状态机定义 ====================
typedef enum {
    SYS_LOCKED = 0,
    SYS_UNLOCKED,
    SYS_LOCKOUT
} sys_state_t;

static volatile sys_state_t g_sys_state = SYS_LOCKED;
static volatile TickType_t g_state_deadline = 0; 
static volatile sys_state_t g_last_drawn_state = -1;

// ==================== 全局变量 ====================
static volatile int g_last_rssi = -100;     // 滤波后的平滑信号
static volatile int g_raw_rssi = -100;      // 单次原始信号
static volatile bool g_found = false;
static volatile TickType_t g_last_seen_time = 0; // 最后一次扫到目标的时间
static volatile TickType_t g_ble_hold_until = 0; // 键盘开锁后屏蔽 BLE 关门

static char g_input_buf[PASSWORD_LEN + 1] = {0};
static uint8_t g_input_len = 0;
static uint8_t g_error_count = 0;
static bool g_need_update_oled = false; 

// ==================== 前置声明 ====================
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

// ==================== 内置 OLED 驱动 ====================
static void i2c_bus_recover(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SDA_IO) | (1ULL << I2C_MASTER_SCL_IO),
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_MASTER_SCL_IO, 1);
        esp_rom_delay_us(5);
    }
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    esp_rom_delay_us(5);
}

static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void ssd1306_write(uint8_t type, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, type, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static void ssd1306_clear() {
    for (uint8_t i = 0; i < 8; i++) {
        ssd1306_write(0x00, 0xB0 + i);
        ssd1306_write(0x00, 0x00);
        ssd1306_write(0x00, 0x10);
        for (uint8_t j = 0; j < 128; j++) {
            ssd1306_write(0x40, 0x00);
        }
    }
}

static void ssd1306_init() {
    uint8_t init_cmds[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF
    };
    for (int i = 0; i < sizeof(init_cmds); i++) {
        ssd1306_write(0x00, init_cmds[i]);
    }
    ssd1306_clear();
}

// 字库
static const uint8_t font6x8[][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00}, {0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00}, {0x23, 0x13, 0x08, 0x64, 0x62, 0x00},
    {0x36, 0x49, 0x55, 0x22, 0x50, 0x00}, {0x00, 0x05, 0x03, 0x00, 0x00, 0x00}, {0x00, 0x1C, 0x22, 0x41, 0x00, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00, 0x00}, {0x14, 0x08, 0x3E, 0x08, 0x14, 0x00}, {0x08, 0x08, 0x3E, 0x08, 0x08, 0x00},
    {0x00, 0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08, 0x00}, {0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02, 0x00}, {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00}, {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00}, {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00}, {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00}, {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00}, {0x01, 0x71, 0x09, 0x05, 0x03, 0x00},
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00}, {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00}, {0x00, 0x36, 0x36, 0x00, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00, 0x00}, {0x08, 0x14, 0x22, 0x41, 0x00, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14, 0x00},
    {0x00, 0x41, 0x22, 0x14, 0x08, 0x00}, {0x02, 0x01, 0x51, 0x09, 0x06, 0x00}, {0x32, 0x49, 0x79, 0x41, 0x3E, 0x00},
    {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00},
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00}, {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00}, {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00},
    {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00}, {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00}, {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00}, {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00}, {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00},
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00}, {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00}, {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00},
    {0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00},
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00}, {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00}, {0x63, 0x14, 0x08, 0x14, 0x63, 0x00},
    {0x07, 0x08, 0x70, 0x08, 0x07, 0x00}, {0x61, 0x51, 0x49, 0x45, 0x43, 0x00},
};

static void ssd1306_draw_char(uint8_t x, uint8_t page, char c) {
    if (c < 32 || c > 122) return; 
    uint8_t index = c - 32;
    ssd1306_write(0x00, 0xB0 + page);
    ssd1306_write(0x00, 0x00 + (x & 0x0F));
    ssd1306_write(0x00, 0x10 + (x >> 4));
    for (int i = 0; i < 6; i++) {
        ssd1306_write(0x40, font6x8[index][i]);
    }
}

static void ssd1306_draw_string(uint8_t x, uint8_t page, const char *str) {
    while (*str) {
        ssd1306_draw_char(x, page, *str++);
        x += 6;
    }
}

// ==================== 舵机 PWM 驱动 ====================
static void servo_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION, .freq_hz = LEDC_FREQ, .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL, .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_GPIO, .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&ch_conf);
}

static uint32_t angle_to_duty(int angle) {
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    return 26 + (uint32_t)(angle * 102.0 / 180.0);
}

static void set_angle(int angle) {
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    ESP_LOGI(TAG, "servo: %d", angle);
}

// ==================== 矩阵键盘驱动 ====================
static void keypad_init(void) {
    for (int i = 0; i < KEYPAD_ROW_NUM; i++) {
        gpio_set_direction(KEYPAD_ROW_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(KEYPAD_ROW_PINS[i], 1);
    }
    for (int i = 0; i < KEYPAD_COL_NUM; i++) {
        gpio_set_direction(KEYPAD_COL_PINS[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(KEYPAD_COL_PINS[i], GPIO_PULLUP_ONLY);
    }
}

static char keypad_scan(void) {
    for (int r = 0; r < KEYPAD_ROW_NUM; r++) {
        gpio_set_level(KEYPAD_ROW_PINS[r], 0);
        for (int c = 0; c < KEYPAD_COL_NUM; c++) {
            if (gpio_get_level(KEYPAD_COL_PINS[c]) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
                if (gpio_get_level(KEYPAD_COL_PINS[c]) == 0) {
                    while (gpio_get_level(KEYPAD_COL_PINS[c]) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    gpio_set_level(KEYPAD_ROW_PINS[r], 1);
                    return KEYPAD_MAP[r][c];
                }
            }
        }
        gpio_set_level(KEYPAD_ROW_PINS[r], 1);
    }
    return 0;
}

// ==================== 蓝牙回调 ====================
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = ESP_BLE_GAP_SCAN_ITVL_MS(50),
    .scan_window            = ESP_BLE_GAP_SCAN_WIN_MS(30),
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    esp_err_t err;
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            esp_ble_gap_start_scanning(0);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "BLE scan fail: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "BLE scan started.");
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            if (scan_result->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;
            uint8_t adv_name_len = 0;
            uint8_t *adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name == NULL) {
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_SHORT, &adv_name_len);
            }

            if (adv_name == NULL || adv_name_len != strlen(TARGET_NAME)) break;
            if (strncmp((char *)adv_name, TARGET_NAME, adv_name_len) != 0) break;

            g_raw_rssi = scan_result->scan_rst.rssi; 
            g_last_seen_time = xTaskGetTickCount();  

            static TickType_t last_print_time = 0;
            TickType_t now_time = xTaskGetTickCount();
            if (now_time - last_print_time >= pdMS_TO_TICKS(1000)) {
                last_print_time = now_time;
                ESP_LOGI(TAG, "RSSI raw:%d | flt:%d", g_raw_rssi, g_last_rssi);
            }

            g_found = true;
            break;
        }
        default: break;
    }
}

// ==================== 状态机管理器任务 ====================
static void state_manager_task(void *pv) {
    while (1) {
        TickType_t now = xTaskGetTickCount();
        bool is_unlocked_by_ble = false;

        // 靠近开锁：使用滤波后的平滑值
        if (g_found && (now - g_last_seen_time) < pdMS_TO_TICKS(1000) && g_last_rssi > BLE_OPEN_RSSI) {
            is_unlocked_by_ble = true;
        }

        if (is_unlocked_by_ble && g_sys_state != SYS_UNLOCKED) {
            ESP_LOGI(TAG, "BLE proximity unlock.");
            g_sys_state = SYS_UNLOCKED;
            g_state_deadline = now + pdMS_TO_TICKS(UNLOCK_TIME_MS);
            g_error_count = 0;
            gpio_set_level(RED_LED_GPIO, 0);
            gpio_set_level(LED_GPIO, 1);
            set_angle(90);
            
            ssd1306_clear();
            ssd1306_draw_string(24, 3, "UNLOCKED");
            g_last_drawn_state = SYS_UNLOCKED;
            g_need_update_oled = false;
        }

        switch (g_sys_state) {
            case SYS_LOCKED:
                gpio_set_level(RED_LED_GPIO, 0);
                gpio_set_level(LED_GPIO, 0);
                
                if (g_input_len > 0 || g_need_update_oled) {
                    ssd1306_clear();
                    ssd1306_draw_string(20, 2, "INPUT PASS:");
                    ssd1306_draw_string(40, 4, g_input_buf); 
                    g_need_update_oled = false;
                    g_last_drawn_state = -1;
                } else if (g_last_drawn_state != SYS_LOCKED) {
                    ssd1306_clear();
                    ssd1306_draw_string(30, 3, "LOCKED");
                    g_last_drawn_state = SYS_LOCKED;
                }
                break;

            case SYS_UNLOCKED:
                // 进入开锁态：统一处理点灯/舵机/屏幕（BLE 与键盘共用）
                if (g_last_drawn_state != SYS_UNLOCKED) {
                    gpio_set_level(RED_LED_GPIO, 0);
                    gpio_set_level(LED_GPIO, 1);
                    set_angle(90);
                    ssd1306_clear();
                    ssd1306_draw_string(24, 3, "UNLOCKED");
                    g_last_drawn_state = SYS_UNLOCKED;
                }
                if (now < g_ble_hold_until) break; // 键盘刚开锁，暂不响应 BLE 关门
                // 关门：使用单次原始信号极速判断
                if (g_raw_rssi < BLE_CLOSE_RSSI || (now - g_last_seen_time) > pdMS_TO_TICKS(BLE_LOST_MS)) {
                    ESP_LOGW(TAG, "lock: raw=%d, dt=%d ms", g_raw_rssi, (int)((now - g_last_seen_time) * portTICK_PERIOD_MS));
                    g_sys_state = SYS_LOCKED;
                    
                    g_found = false;
                    g_raw_rssi = -100;
                    g_last_rssi = -100;
                    
                    g_last_drawn_state = -1; 
                    g_input_len = 0;
                    memset(g_input_buf, 0, sizeof(g_input_buf));
                    gpio_set_level(LED_GPIO, 0);
                    set_angle(0);
                } 
                else if (now >= g_state_deadline) {
                    ESP_LOGI(TAG, "Unlock timeout. Relocking.");
                    g_sys_state = SYS_LOCKED;
                    g_last_drawn_state = -1; 
                    g_input_len = 0;
                    memset(g_input_buf, 0, sizeof(g_input_buf));
                    gpio_set_level(LED_GPIO, 0);
                    set_angle(0);
                } else {
                    gpio_set_level(LED_GPIO, 1);
                }
                break;

            case SYS_LOCKOUT:
                if (now >= g_state_deadline) {
                    ESP_LOGI(TAG, "Lockout timeout. Unlocked.");
                    g_sys_state = SYS_LOCKED;
                    g_last_drawn_state = -1;
                    g_error_count = 0;
                    gpio_set_level(RED_LED_GPIO, 0);
                } else {
                    gpio_set_level(RED_LED_GPIO, (now / pdMS_TO_TICKS(500)) % 2);
                    
                    int remain_sec = (g_state_deadline - now) / pdMS_TO_TICKS(1000);
                    
                    static int last_remain_sec = -1;
                    if (g_last_drawn_state != SYS_LOCKOUT || last_remain_sec != remain_sec) {
                        last_remain_sec = remain_sec;
                        g_last_drawn_state = SYS_LOCKOUT;
                        
                        ssd1306_clear();
                        ssd1306_draw_string(18, 2, "LOCKOUT!");
                        
                        char countdown_str[20];
                        sprintf(countdown_str, "WAIT %02ds", remain_sec);
                        ssd1306_draw_string(30, 4, countdown_str);
                    }
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== 矩阵键盘任务 ====================
static void keypad_task(void *pv) {
    char key;
    while (1) {
        if (g_sys_state == SYS_LOCKOUT || g_sys_state == SYS_UNLOCKED) {
            memset(g_input_buf, 0, sizeof(g_input_buf));
            g_input_len = 0;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        key = keypad_scan();
        if (key != 0) {
            ESP_LOGI(TAG, "Key pressed: %c", key);

            if (key == '#') {
                if (g_input_len == PASSWORD_LEN && strncmp(g_input_buf, PASSWORD, PASSWORD_LEN) == 0) {
                    ESP_LOGI(TAG, "Auth passed. Unlocking.");
                    g_error_count = 0;
                    g_sys_state = SYS_UNLOCKED;
                    g_state_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(UNLOCK_TIME_MS);
                    
                    // 防止键盘开锁后被蓝牙逻辑秒关
                    g_last_seen_time = xTaskGetTickCount(); 
                    g_ble_hold_until = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                } else {
                    ESP_LOGW(TAG, "Auth failed. Attempt: %d", g_error_count + 1);
                    g_error_count++;

                    if (g_error_count >= 3) {
                        ESP_LOGE(TAG, "Max attempts. Lockout!");
                        g_sys_state = SYS_LOCKOUT;
                        g_state_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LOCKOUT_TIME_MS);
                    }
                }
                memset(g_input_buf, 0, sizeof(g_input_buf));
                g_input_len = 0;
                g_need_update_oled = true;
                
            } else if (key == '*') {
                if (g_input_len > 0) {
                    g_input_len--;
                    g_input_buf[g_input_len] = '\0';
                    g_need_update_oled = true;
                }
            } else if (key >= '0' && key <= '9') {
                if (g_input_len < PASSWORD_LEN) {
                    g_input_buf[g_input_len++] = key;
                    g_input_buf[g_input_len] = '\0';
                } else {
                    for (int i = 0; i < PASSWORD_LEN - 1; i++) {
                        g_input_buf[i] = g_input_buf[i + 1];
                    }
                    g_input_buf[PASSWORD_LEN - 1] = key;
                }
                g_need_update_oled = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==================== RSSI 滤波任务 ====================
static void rssi_filter_task(void *pv) {
    int rssi_history[3] = {-100, -100, -100};
    int history_index = 0;

    while (1) {
        bool fresh = g_found && (xTaskGetTickCount() - g_last_seen_time) < pdMS_TO_TICKS(1000);
        if (fresh) {
            rssi_history[history_index] = g_raw_rssi;
            history_index = (history_index + 1) % 3;
            int sum = 0;
            for (int i = 0; i < 3; i++) sum += rssi_history[i];
            g_last_rssi = sum / 3;
        } else {
            for (int i = 0; i < 3; i++) rssi_history[i] = -100;
            g_last_rssi = -100;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ==================== 入口函数 ====================
void app_main(void) {
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_drive_capability(LED_GPIO, GPIO_DRIVE_CAP_3); 
    gpio_set_level(LED_GPIO, 0);
    
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RED_LED_GPIO, 0);

    i2c_bus_recover();
    i2c_master_init();
    ssd1306_init();
    keypad_init();
    servo_init();
    set_angle(0);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gap_set_scan_params(&ble_scan_params);

    xTaskCreate(rssi_filter_task, "rssi_filter", 4096, NULL, 5, NULL);
    xTaskCreate(state_manager_task, "state_mgr", 4096, NULL, 4, NULL);
    xTaskCreate(keypad_task, "keypad_task", 4096, NULL, 3, NULL);
}