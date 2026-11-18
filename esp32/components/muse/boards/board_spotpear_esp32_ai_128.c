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
 * SpotPear ESP32-AI-1.28 v1.0, customized with clean INMP441 audio gain,
 * Voice Activity Detection (Hands-free Auto Listening), and GC9A01 LCD.
 */
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2s_std.h"
#include "muse_audio.h"
#include "spotpear_pcm.h"
#include "muse_board.h"
#include "muse_mem.h"

static const char *TAG = "board";

#define LCD_RES 240
#define LCD_HOST SPI2_HOST
// ===== CÁC CHÂN MÀN HÌNH THEO ĐÚNG DÂY CỦA BẠN =====
#define LCD_DC GPIO_NUM_38
#define LCD_CS GPIO_NUM_17
#define LCD_SCLK GPIO_NUM_8
#define LCD_MOSI GPIO_NUM_9
#define LCD_RST GPIO_NUM_41
#define LCD_BL GPIO_NUM_3
#define DRAW_BUF_LINES 40

static esp_lcd_panel_handle_t s_panel;
static esp_lcd_panel_io_handle_t s_io;
static muse_gpio_button_t s_talk;

// ===== BIẾN TỰ ĐỘNG LẮNG NGHE (VAD) =====
static volatile bool s_vad_speaking = false;
static int s_vad_silence_chunks = 0;
static int s_vad_active_chunks = 0;

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

// ===== HỖ TRỢ CẢ NÚT VẬT LÝ VÀ TỰ ĐỘNG PHÁT HIỆN GIỌNG NÓI =====
static unsigned poll_buttons(void)
{
    unsigned hw = muse_gpio_button_poll(&s_talk);
    if (s_vad_speaking) {
        hw |= 1; // Kích hoạt nút ảo khi phát hiện người đang nói
    }
    return hw;
}

static void wait_buttons(int timeout_ms)
{
    muse_gpio_button_t *buttons[] = { &s_talk };
    muse_gpio_buttons_wait(buttons, 1, timeout_ms);
}

static i2s_chan_handle_t s_tx, s_rx;
static bool s_tx_on, s_rx_on;
static spotpear_resampler_t s_resampler;
static int s_gain_q8 = 256; // Mặc định 0 dB (gain = 1.0) cho INMP441
static int32_t s_dc = 0;

static int data_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, bool on)
{
    (void)h;
    bool output = type == ESP_CODEC_DEV_TYPE_OUT;
    i2s_chan_handle_t channel = output ? s_tx : s_rx;
    bool *enabled = output ? &s_tx_on : &s_rx_on;
    if (*enabled == on) return ESP_CODEC_DEV_OK;
    esp_err_t err = on ? i2s_channel_enable(channel) : i2s_channel_disable(channel);
    if (err != ESP_OK) return ESP_CODEC_DEV_DRV_ERR;
    *enabled = on;
    if (output) memset(&s_resampler, 0, sizeof(s_resampler));
    else s_dc = 0;
    return ESP_CODEC_DEV_OK;
}

static int spk_write(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    if (size < 0 || size % 4) return ESP_CODEC_DEV_WRITE_FAIL;
    const int16_t *input = (const int16_t *)data;
    int frames = size / 4;
    while (frames) {
        int n = frames > 128 ? 128 : frames;
        int32_t samples[192];
        int count = 0;
        for (int i = 0; i < n; ++i) {
            count += spotpear_resample(&s_resampler, input[2 * i], samples + count);
        }
        size_t wrote = 0, bytes = count * sizeof(samples[0]);
        if (i2s_channel_write(s_tx, samples, bytes, &wrote, 1000) != ESP_OK || wrote != bytes)
            return ESP_CODEC_DEV_WRITE_FAIL;
        input += n * 2;
        frames -= n;
    }
    return ESP_CODEC_DEV_OK;
}

/* Thu âm Mic INMP441: Giữ âm thanh trong trẻo, không vỡ tiếng + Thuật toán VAD tự động lắng nghe */
static int mic_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    if (size < 0 || size % 4) return ESP_CODEC_DEV_READ_FAIL;
    int16_t *output = (int16_t *)data;
    int frames = size / 4;
    int32_t max_amp = 0;

    while (frames) {
        int n = frames > 128 ? 128 : frames;
        int32_t samples[128];
        size_t got = 0, bytes = n * sizeof(samples[0]);
        if (i2s_channel_read(s_rx, samples, bytes, &got, 1000) != ESP_OK || got != bytes)
            return ESP_CODEC_DEV_READ_FAIL;
        for (int i = 0; i < n; ++i) {
            int32_t v = samples[i] >> 16;
            s_dc += (v * 256 - s_dc) >> 8;
            int64_t raw = (int64_t)(v - (s_dc >> 8));
            int64_t amplified = (raw * s_gain_q8) / 256;
            int16_t sample16 = amplified > INT16_MAX ? INT16_MAX :
                               amplified < INT16_MIN ? INT16_MIN : (int16_t)amplified;
            output[2 * i] = sample16;
            output[2 * i + 1] = 0;

            int32_t amp = abs(sample16);
            if (amp > max_amp) max_amp = amp;
        }
        output += n * 2;
        frames -= n;
    }

    // Thuật toán VAD: Ngưỡng phát hiện tiếng nói (khoảng 1000 đối với 16-bit PCM)
    const int32_t VAD_THRESHOLD = 1000;
    if (max_amp > VAD_THRESHOLD) {
        s_vad_active_chunks++;
        s_vad_silence_chunks = 0;
        if (s_vad_active_chunks >= 2) { // Nói liên tục ~40ms trở lên thì kích hoạt nghe
            s_vad_speaking = true;
        }
    } else {
        s_vad_active_chunks = 0;
        if (s_vad_speaking) {
            s_vad_silence_chunks++;
            // Im lặng khoảng 1.0 giây thì tự động kết thúc câu và gửi đi
            if (s_vad_silence_chunks >= 50) {
                s_vad_speaking = false;
                s_vad_silence_chunks = 0;
            }
        }
    }

    return ESP_CODEC_DEV_OK;
}

static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    (void)mic;
    // INMP441 có độ nhạy cao: Giới hạn gain tối đa 6 dB để chống vỡ tiếng hoàn toàn
    db = db < 0 ? 0 : db > 6 ? 6 : db;
    s_gain_q8 = (int)(256.0f * powf(10.0f, db / 20.0f));
}

static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    *spk = NULL;
    *mic = NULL;
    i2s_chan_config_t tx = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_chan_config_t rx = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx.auto_clear = true;
    esp_err_t err = i2s_new_channel(&tx, &s_tx, NULL);
    if (err != ESP_OK) goto fail;
    err = i2s_new_channel(&rx, NULL, &s_rx);
    if (err != ESP_OK) goto fail;
    i2s_std_config_t config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_15,
            .ws = GPIO_NUM_16,
            .dout = GPIO_NUM_7,
            .din = I2S_GPIO_UNUSED,
        },
    };
    config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    err = i2s_channel_init_std_mode(s_tx, &config);
    if (err != ESP_OK) goto fail;
    config.clk_cfg.sample_rate_hz = MUSE_AUDIO_RATE;
    config.gpio_cfg.bclk = GPIO_NUM_5;
    config.gpio_cfg.ws = GPIO_NUM_4;
    config.gpio_cfg.dout = I2S_GPIO_UNUSED;
    config.gpio_cfg.din = GPIO_NUM_6;
    err = i2s_channel_init_std_mode(s_rx, &config);
    if (err != ESP_OK) goto fail;

    static const audio_codec_data_if_t spk_if = { .enable = data_enable, .write = spk_write };
    static const audio_codec_data_if_t mic_if = { .enable = data_enable, .read = mic_read };
    esp_codec_dev_cfg_t out_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .data_if = &spk_if };
    esp_codec_dev_cfg_t in_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN, .data_if = &mic_if };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    if (*spk && *mic) return ESP_OK;
    err = ESP_ERR_NO_MEM;
fail:
    if (*spk) esp_codec_dev_delete(*spk);
    if (*mic) esp_codec_dev_delete(*mic);
    *spk = NULL;
    *mic = NULL;
    if (s_tx) i2s_del_channel(s_tx);
    if (s_rx) i2s_del_channel(s_rx);
    s_tx = NULL;
    s_rx = NULL;
    return err;
}

static const muse_board_t s_board = {
    .name = "SpotPear ESP32-AI-1.28 v1.0",
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
    .mic_slot = 0,
    .set_mic_gain = set_mic_gain,
    .poll_buttons = poll_buttons,
    .wait_buttons = wait_buttons,
};

const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
