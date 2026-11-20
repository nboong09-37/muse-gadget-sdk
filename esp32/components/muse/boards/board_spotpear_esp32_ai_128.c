/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * PetBot ESP32-S3:
 * 1. Loa MAX98357A (I2S1 TX): BCLK 15, WS 16, DIN/DOUT 7 - 16 kHz 16-bit Stereo.
 * 2. Mic INMP441 (I2S0 RX): BCLK 5, WS 4, SD/DIN 6 - 16 kHz 32-bit slot, top 16 bits.
 * 3. Nút bấm BOOT (GPIO 0): Nhấn giữ để nói, thả ra để gửi.
 * 4. Màn hình tròn GC9A01 240x240 SPI (DC 38, CS 17, SCLK 8, MOSI 9, RST 41, BL 3).
 *    DRAW_BUF_LINES = 16 để tiết kiệm RAM nội (tránh cạn kiệt heap gây đứt kết nối).
 */
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_mem.h"

static const char *TAG = "board";

#define LCD_RES 240
#define LCD_HOST SPI2_HOST
#define LCD_DC GPIO_NUM_38
#define LCD_CS GPIO_NUM_17
#define LCD_SCLK GPIO_NUM_8
#define LCD_MOSI GPIO_NUM_9
#define LCD_RST GPIO_NUM_41
#define LCD_BL GPIO_NUM_3
#define DRAW_BUF_LINES 16

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static muse_gpio_button_t s_talk;

static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;

static esp_err_t init(void)
{
    ESP_RETURN_ON_ERROR(muse_gpio_button_init(&s_talk, GPIO_NUM_0), TAG, "boot button");
    s_talk.pressed = gpio_get_level(GPIO_NUM_0) == 0;
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    const ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    if (ledc_timer_config(&bl_timer) != ESP_OK || ledc_channel_config(&bl_ch) != ESP_OK) {
        ESP_LOGE(TAG, "backlight pwm");
        return NULL;
    }

    const spi_bus_config_t bus = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = LCD_RES * DRAW_BUF_LINES * 2,
    };
    if (spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
        ESP_LOGE(TAG, "spi bus");
        return NULL;
    }
    esp_lcd_panel_io_spi_config_t io_cfg = GC9A01_PANEL_IO_SPI_CONFIG(LCD_CS, LCD_DC, NULL, NULL);
    io_cfg.pclk_hz = 40000000;
    io_cfg.spi_mode = 0;
    if (esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &s_io) != ESP_OK) {
        ESP_LOGE(TAG, "panel io");
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_gc9a01(s_io, &panel_cfg, &s_panel) != ESP_OK) {
        ESP_LOGE(TAG, "gc9a01");
        return NULL;
    }
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = s_io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_RES,
            .ver_res = LCD_RES,
            .buffer_height = DRAW_BUF_LINES,
            .use_psram = false,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        return NULL;
    }

    *touch = NULL;
    if (esp_lv_adapter_start() != ESP_OK) {
        return NULL;
    }
    return disp;
}

static bool display_lock(int timeout_ms)
{
    return esp_lv_adapter_lock(timeout_ms) == ESP_OK;
}

static void set_brightness(int pct)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pct * 1023 / 100);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void panel_sleep(bool sleep)
{
    esp_lcd_panel_io_tx_param(s_io, sleep ? 0x10 : 0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
}

// ===== XỬ LÝ NÚT BẤM BOOT (GPIO 0) CHUẨN XÁC VÀ BỀN BỈ =====
static unsigned poll_buttons(void)
{
    return muse_gpio_button_poll(&s_talk);
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_button_t *buttons[] = { &s_talk };
    muse_gpio_buttons_wait(buttons, 1, timeout_ms);
}

// ===== THU ÂM TỪ MIC INMP441 (STEREO 32-BIT SLOT, DC TRACKING & GAIN) =====
static int s_mic_gain_q8 = 256;
static int32_t s_mic_dc[2];

static int mic_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, bool on)
{
    (void)h;
    (void)type;
    (void)on;
    return ESP_CODEC_DEV_OK;
}

/* A MEMS mic's slot has a small DC offset, which the gain would turn into
 * clipping: each slot's zero level is tracked and taken off first. */
static int mic_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    size_t got = 0;
    if (i2s_channel_read(s_rx, data, size, &got, pdMS_TO_TICKS(1000)) != ESP_OK || got != (size_t)size) {
        return ESP_CODEC_DEV_READ_FAIL;
    }
    int16_t *s = (int16_t *)data;
    for (int i = 0; i < size / 2; i++) {
        int32_t *dc = &s_mic_dc[i & 1];
        *dc += (s[i] * 256 - *dc) >> 8;
        int v = (s[i] - (*dc >> 8)) * s_mic_gain_q8 >> 8;
        s[i] = v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v;
    }
    return ESP_CODEC_DEV_OK;
}

static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    (void)mic;
    s_mic_gain_q8 = (int)(256.0f * powf(10.0f, (float)db / 20.0f));
}

// ===== KHỞI TẠO ÂM THANH: LOA MAX98357A VÀ MIC INMP441 =====
static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    *spk = NULL;
    *mic = NULL;

    // 1. MIC INMP441 (I2S0 RX): BCLK 5, WS 4, DIN 6
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    rx_chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&rx_chan_cfg, NULL, &s_rx), TAG, "mic I2S channel");

    i2s_std_config_t rx_std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_5,
            .ws = GPIO_NUM_4,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_6,
        },
    };
    /* 32-bit slots (64 clocks a frame), which an INMP441 needs. Each sample is
     * the slot's top 16 bits. */
    rx_std.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    rx_std.slot_cfg.ws_width = 32;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_rx, &rx_std), TAG, "mic I2S init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_rx), TAG, "mic I2S enable");

    // 2. LOA MAX98357A (I2S1 TX): BCLK 15, WS 16, DOUT 7
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&tx_chan_cfg, &s_tx, NULL), TAG, "spk I2S channel");

    const i2s_std_config_t tx_std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE), // 16000 Hz
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_15,
            .ws = GPIO_NUM_16,
            .dout = GPIO_NUM_7,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_tx, &tx_std), TAG, "spk I2S init");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_tx), TAG, "spk I2S enable");

    // Phát âm thanh khởi động nhẹ (0.2s) để xác nhận loa MAX98357A kêu ngay khi bật nguồn
    int16_t beep[320 * 2];
    for (int i = 0; i < 320; i++) {
        int16_t val = (int16_t)(8000.0f * sinf(2.0f * 3.14159f * 1000.0f * i / MUSE_AUDIO_RATE));
        beep[2 * i] = val;
        beep[2 * i + 1] = val;
    }
    size_t wrote = 0;
    for (int rep = 0; rep < 10; rep++) {
        i2s_channel_write(s_tx, beep, sizeof(beep), &wrote, 100);
    }

    // 3. CODEC DATA INTERFACE CHUẨN CỦA ESP-IDF CHO MAX98357A
    audio_codec_i2s_cfg_t tx_data_cfg = { .port = I2S_NUM_1, .tx_handle = s_tx };
    const audio_codec_data_if_t *tx_data = audio_codec_new_i2s_data(&tx_data_cfg);
    ESP_RETURN_ON_FALSE(tx_data, ESP_ERR_NO_MEM, TAG, "speaker data interface");

    esp_codec_dev_cfg_t tx_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = NULL,
        .data_if = tx_data,
    };
    *spk = esp_codec_dev_new(&tx_dev_cfg);

    // 4. CODEC DATA INTERFACE CHO MIC INMP441 (VỚI DC FILTER & SOFTWARE GAIN)
    static const audio_codec_data_if_t mic_data = { .enable = mic_enable, .read = mic_read };
    esp_codec_dev_cfg_t rx_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = NULL,
        .data_if = &mic_data,
    };
    *mic = esp_codec_dev_new(&rx_dev_cfg);

    ESP_LOGI(TAG, "Audio ready: MAX98357A (I2S1 Stereo) & INMP441 (I2S0 Stereo slot with DC filter)");
    return (*spk && *mic) ? ESP_OK : ESP_ERR_NO_MEM;
}

static const muse_board_t s_board = {
    .name = "PetBot ESP32-AI 1.28",
    .width = LCD_RES,
    .height = LCD_RES,
    .round = true,
    .diagonal_in = 1.28f,
    .talk_button = "boot",
    .talk_hint = { LV_ALIGN_BOTTOM_MID, 0, -12 },
    .frame_ms = 40,
    .avatar_px = 128,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .audio_init = audio_init,
    .mic_read = NULL,
    .mic_slot = 0,
    .set_mic_gain = set_mic_gain,
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
};

const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
