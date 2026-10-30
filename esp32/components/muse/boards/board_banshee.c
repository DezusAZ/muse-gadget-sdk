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
 * Wired Hatters Banshee (ESP32-C5 module). 240x320 ILI9341 LCD, BOOT button on
 * GPIO28, active-high LEDC backlight on GPIO1. Audio (the S3's TLV320 codec)
 * and the capacitive touch bar are not wired up here yet: this is a
 * display-and-button board for now (avatar + Link status on screen; voice
 * comes later).
 *
 * The panel is an ordinary ILI9341, but the Banshee does NOT drive it over the
 * SPI peripheral — the stock GhostESP firmware clocks the single data line out
 * through the ESP32-C5 PARLIO peripheral, and a plain esp_lcd SPI panel draws
 * nothing on this board (verified: blank/white, no response). So we reuse the
 * proven esp_lcd_ili9341 driver on top of a custom esp_lcd panel IO whose
 * transport is PARLIO, matching the stock firmware's banshee_c5_parlio.c.
 *
 * Pin map (reverse-engineered from the stock Banshee C5 firmware):
 *   DATA0/MOSI = GPIO23, CLK = GPIO27, CS/valid = GPIO26, DC = GPIO24,
 *   RST tied to the module reset (none), backlight = GPIO1 (active high).
 */
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/parlio_tx.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_codec_dev.h"
#include "esp_timer.h"
#include "tlv320dac3100.h"

#include "muse_board.h"
#include "muse_mem.h"
#include "muse_settings.h"   /* mute toggle = muse_settings_set_speaker_on() */

static const char *TAG = "board";

#define LCD_H_RES 240
#define LCD_V_RES 320
#define LCD_DATA  GPIO_NUM_23   /* single data line (MOSI/DATA0) */
#define LCD_CLK   GPIO_NUM_27
#define LCD_CS    GPIO_NUM_26   /* PARLIO "valid" signal, used as CS */
#define LCD_DC    GPIO_NUM_24
#define LCD_RST   GPIO_NUM_25   /* hardware reset — the stock firmware pulses this */
#define LCD_BL    GPIO_NUM_1    /* active high, LEDC PWM */
/* The stock firmware defaults to 80 MHz, but that is ~5x over the ILI9341's
 * rated write speed: under the avatar's partial-refresh animation it produced
 * accumulating pixel corruption (trails), and the fast edges on the display
 * pins disturbed the C5's Wi-Fi. 20 MHz is within spec and clean. */
#define PARLIO_CLK_HZ (20 * 1000 * 1000)
#define DRAW_BUF_LINES 32

#define TALK_GPIO GPIO_NUM_28   /* BOOT button; also the pairing-confirm press */

/* ---- PARLIO-backed esp_lcd panel IO (the wire transport) ------------------ */

#define PARLIO_ALIGN 64
#define BOUNCE_BYTES (LCD_H_RES * (DRAW_BUF_LINES + 8) * 2)

typedef struct {
    esp_lcd_panel_io_t base;
    parlio_tx_unit_handle_t tx;
    gpio_num_t dc;
    uint8_t *scratch;   /* aligned DMA buffer for command + parameter bytes */
    uint8_t *bounce;    /* aligned DMA buffer for pixel data */
    volatile bool color_pending;   /* true only while a pixel transfer is in flight */
    esp_lcd_panel_io_color_trans_done_cb_t on_done;
    void *user_ctx;
} parlio_io_t;

static esp_lcd_panel_io_event_data_t s_trans_evt; /* zeroed; adapter ignores contents */

static bool IRAM_ATTR parlio_on_trans_done(parlio_tx_unit_handle_t unit,
                                           const parlio_tx_done_event_data_t *edata,
                                           void *user_ctx)
{
    (void)unit;
    (void)edata;
    parlio_io_t *io = (parlio_io_t *)user_ctx;
    /* Only a pixel (color) transfer signals LVGL flush-ready; command and
     * parameter transfers also complete here but must be ignored. */
    if (io->color_pending) {
        io->color_pending = false;
        if (io->on_done) {
            return io->on_done(&io->base, &s_trans_evt, io->user_ctx);
        }
    }
    return false;
}

static esp_err_t parlio_send_blocking(parlio_io_t *io, const void *data, size_t bytes)
{
    const parlio_transmit_config_t cfg = { .idle_value = 0 };
    esp_err_t ret = parlio_tx_unit_transmit(io->tx, data, bytes * 8, &cfg);
    if (ret == ESP_OK) {
        ret = parlio_tx_unit_wait_all_done(io->tx, 2000);   /* finite: never hang */
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "parlio wait (%u bytes): %s", (unsigned)bytes, esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "parlio transmit (%u bytes): %s", (unsigned)bytes, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t parlio_io_tx_param(esp_lcd_panel_io_t *base, int lcd_cmd,
                                    const void *param, size_t param_size)
{
    parlio_io_t *io = (parlio_io_t *)base;
    parlio_tx_unit_wait_all_done(io->tx, 2000);
    io->scratch[0] = (uint8_t)lcd_cmd;
    gpio_set_level(io->dc, 0);                 /* command */
    esp_err_t ret = parlio_send_blocking(io, io->scratch, 1);
    if (ret == ESP_OK && param && param_size) {
        if (param_size > 60) {
            param_size = 60;
        }
        memcpy(io->scratch, param, param_size);
        gpio_set_level(io->dc, 1);             /* data */
        ret = parlio_send_blocking(io, io->scratch, param_size);
    }
    return ret;
}

static esp_err_t parlio_io_tx_color(esp_lcd_panel_io_t *base, int lcd_cmd,
                                    const void *color, size_t color_size)
{
    parlio_io_t *io = (parlio_io_t *)base;
    parlio_tx_unit_wait_all_done(io->tx, 2000);
    io->scratch[0] = (uint8_t)lcd_cmd;
    gpio_set_level(io->dc, 0);                 /* RAMWR command */
    esp_err_t ret = parlio_send_blocking(io, io->scratch, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    gpio_set_level(io->dc, 1);                 /* pixel data */
    /* Send the pixels through a 64-byte-aligned DMA bounce buffer (LVGL's
     * buffers are not guaranteed aligned to the PARLIO DMA constraint). A flush
     * can be larger than one bounce buffer, so chunk it: every chunk but the
     * last is blocking; the final chunk is async and its completion signals
     * LVGL flush-ready (via the parlio done ISR). Truncating instead (the old
     * behaviour) left regions undrawn and caused smears/trails. The adapter
     * waits for flush-ready before the next flush, so one bounce buffer is
     * safe. */
    const parlio_transmit_config_t cfg = { .idle_value = 0 };
    const uint8_t *src = color;
    size_t remaining = color_size;
    while (remaining > 0) {
        size_t chunk = remaining > BOUNCE_BYTES ? BOUNCE_BYTES : remaining;
        memcpy(io->bounce, src, chunk);
        if (chunk == remaining) {
            /* last chunk: async, fires the completion callback */
            io->color_pending = true;
            esp_err_t r = parlio_tx_unit_transmit(io->tx, io->bounce, chunk * 8, &cfg);
            if (r != ESP_OK) {
                io->color_pending = false;
                ESP_LOGE(TAG, "parlio color transmit: %s", esp_err_to_name(r));
                return r;
            }
        } else {
            esp_err_t r = parlio_send_blocking(io, io->bounce, chunk);
            if (r != ESP_OK) {
                return r;
            }
        }
        src += chunk;
        remaining -= chunk;
    }
    return ESP_OK;
}

static esp_err_t parlio_io_rx_param(esp_lcd_panel_io_t *base, int lcd_cmd,
                                    void *param, size_t param_size)
{
    (void)base; (void)lcd_cmd; (void)param; (void)param_size;
    return ESP_ERR_NOT_SUPPORTED;   /* single data line, no read-back */
}

static esp_err_t parlio_io_register_cbs(esp_lcd_panel_io_t *base,
                                        const esp_lcd_panel_io_callbacks_t *cbs,
                                        void *user_ctx)
{
    parlio_io_t *io = (parlio_io_t *)base;
    io->on_done = cbs ? cbs->on_color_trans_done : NULL;
    io->user_ctx = user_ctx;
    return ESP_OK;
}

static esp_err_t parlio_io_del(esp_lcd_panel_io_t *base)
{
    parlio_io_t *io = (parlio_io_t *)base;
    if (io->tx) {
        parlio_tx_unit_disable(io->tx);
        parlio_del_tx_unit(io->tx);
    }
    free(io->scratch);
    free(io->bounce);
    free(io);
    return ESP_OK;
}

static esp_err_t parlio_io_new(esp_lcd_panel_io_handle_t *out_io)
{
    parlio_io_t *io = calloc(1, sizeof(parlio_io_t));
    if (!io) {
        return ESP_ERR_NO_MEM;
    }
    io->dc = LCD_DC;
    io->scratch = heap_caps_aligned_alloc(PARLIO_ALIGN, PARLIO_ALIGN,
                                          MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    io->bounce = heap_caps_aligned_alloc(PARLIO_ALIGN, BOUNCE_BYTES,
                                         MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!io->scratch || !io->bounce) {
        free(io->scratch);
        free(io->bounce);
        free(io);
        return ESP_ERR_NO_MEM;
    }

    const gpio_config_t dc_cfg = {
        .pin_bit_mask = 1ULL << LCD_DC,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&dc_cfg);
    gpio_set_level(LCD_DC, 0);

    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .data_width = 1,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = PARLIO_CLK_HZ,
        .data_gpio_nums = { LCD_DATA },
        .clk_out_gpio_num = LCD_CLK,
        .valid_gpio_num = LCD_CS,
        .trans_queue_depth = 4,
        .max_transfer_size = LCD_H_RES * (DRAW_BUF_LINES + 8) * 2,
        .dma_burst_size = 32,
        .shift_edge = PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_MSB,
        .flags = { .invert_valid_out = true },
    };
    esp_err_t ret = parlio_new_tx_unit(&cfg, &io->tx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "parlio_new_tx_unit: %s", esp_err_to_name(ret));
        free(io->scratch);
        free(io);
        return ret;
    }
    const parlio_tx_event_callbacks_t pcbs = { .on_trans_done = parlio_on_trans_done };
    parlio_tx_unit_register_event_callbacks(io->tx, &pcbs, io);
    parlio_tx_unit_enable(io->tx);

    /* The stock driver boosts drive strength on these lines (CLK=27/CS=26 are
     * C5 strapping pins); default drive left the panel blank. */
    gpio_set_drive_capability(LCD_DATA, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(LCD_CLK, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(LCD_CS, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(LCD_DC, GPIO_DRIVE_CAP_3);

    io->base.rx_param = parlio_io_rx_param;
    io->base.tx_param = parlio_io_tx_param;
    io->base.tx_color = parlio_io_tx_color;
    io->base.del = parlio_io_del;
    io->base.register_event_callbacks = parlio_io_register_cbs;

    ESP_LOGI(TAG, "PARLIO LCD IO ready: DATA0=%d CLK=%d CS=%d DC=%d @%dHz",
             LCD_DATA, LCD_CLK, LCD_CS, LCD_DC, PARLIO_CLK_HZ);
    *out_io = &io->base;
    return ESP_OK;
}

/* ---- panel init: the stock GhostESP Banshee ILI9341 sequence --------------
 * esp_lcd's generic ILI9341 init did not light this panel; the power/VCOM/gamma
 * registers below are the exact ones the stock firmware (ili9341.c) sends, which
 * are known to drive it. Sent through the PARLIO IO. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t len;   /* bit7 = 120ms delay after; low bits = data byte count */
} banshee_init_cmd_t;

static const banshee_init_cmd_t s_init_seq[] = {
    {0xCF, {0x00, 0x83, 0x30}, 3},
    {0xED, {0x64, 0x03, 0x12, 0x81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x48}, 1},   /* MADCTL: MX+BGR, portrait, no MV transpose (stock set_orientation value) */
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0x87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0xE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3F}, 4},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},   /* sleep out + delay */
    {0x29, {0}, 0x80},   /* display on + delay */
    {0xFF, {0}, 0xFF},   /* end */
};

static void banshee_panel_init_seq(esp_lcd_panel_io_handle_t io)
{
    for (int i = 0; s_init_seq[i].len != 0xFF; i++) {
        esp_lcd_panel_io_tx_param(io, s_init_seq[i].cmd, s_init_seq[i].data,
                                  s_init_seq[i].len & 0x1F);
        if (s_init_seq[i].len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }
    esp_lcd_panel_io_tx_param(io, 0x20, NULL, 0);   /* INVOFF (no colour inversion) */
}

/* ---- board interface ------------------------------------------------------ */

static esp_lcd_panel_handle_t s_panel;
static muse_gpio_button_t s_talk;
static i2c_master_dev_handle_t s_expander;   /* 0x74 expander, read for the front buttons */
static bool s_front_down;                     /* debounced: any front button held */

/* Self-heal: internal RAM on the C5 is tight, and a flaky cloud connection can
 * slowly leak it (via the HTTP/TLS retry path) until nothing can allocate and
 * the gadget looks dead until it's power-cycled. If free internal RAM stays
 * critically low for ~30 s (a genuinely stuck, starved state, not a blip),
 * restart cleanly so it reconnects on its own instead of hanging. */
static void ram_watchdog_task(void *arg)
{
    (void)arg;
    int low = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        size_t freeb = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        if (freeb < 5000) {
            if (++low >= 3) {
                ESP_LOGE(TAG, "internal RAM critically low (%u B) for 30s; restarting to recover", (unsigned)freeb);
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        } else {
            low = 0;
        }
    }
}

static esp_err_t init(void)
{
    xTaskCreate(ram_watchdog_task, "ram_wd", 3072, NULL, 1, NULL);
    return muse_gpio_button_init(&s_talk, TALK_GPIO);
}

/* --- Resistive touch: TSC2007 @ I2C 0x4B on the shared port-0 bus, for
 * drag-scrolling the reply. The device handle is opened in audio_init (which
 * owns that bus); until then tp_read reports no touch. Calibration is a
 * placeholder -- tune the TP_* values from the "touch raw" log lines. --- */
static i2c_master_dev_handle_t s_touch;
#define TP_W 240
#define TP_H 320
/* Calibrated 2026-10-07 from live corner/center touches: idle z1 <= 6, a real
 * press reads z1 160..2400. Raw X grows left->right (= screen X directly), raw
 * Y grows bottom->top (= screen Y, inverted); no axis swap. */
#define TP_Z_MIN 25          /* z1 press threshold (idle floor ~6) */
#define TP_X_MIN 396
#define TP_X_MAX 3570
#define TP_Y_MIN 286
#define TP_Y_MAX 3712
#define TP_INV_X 0
#define TP_INV_Y 1

static bool tsc_meas(uint8_t cmd, uint16_t *out)
{
    uint8_t v[2];
    if (i2c_master_transmit_receive(s_touch, &cmd, 1, v, 2, 20) != ESP_OK) {
        return false;
    }
    *out = ((uint16_t)v[0] << 4) | (v[1] >> 4);   /* 12-bit, left-justified */
    return true;
}

static int tp_map(int raw, int lo, int hi, int span, int inv)
{
    if (hi <= lo) {
        return 0;
    }
    int v = (raw - lo) * span / (hi - lo);
    if (v < 0) {
        v = 0;
    } else if (v > span - 1) {
        v = span - 1;
    }
    return inv ? (span - 1 - v) : v;
}

static void tp_read(lv_indev_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    static int32_t lastx, lasty;
    static bool was_pressed;
    data->point.x = lastx;
    data->point.y = lasty;
    data->state = LV_INDEV_STATE_RELEASED;
    if (!s_touch) {
        return;
    }
    uint16_t z1 = 0, x = 0, y = 0;
    if (!tsc_meas(0xE4, &z1) || z1 < TP_Z_MIN) {   /* MEASURE_Z1 | ADC on */
        if (was_pressed) {
            ESP_LOGI(TAG, "touch up");
            was_pressed = false;
        }
        return;                               /* no / too-light touch */
    }
    if (!tsc_meas(0xC4, &x) || !tsc_meas(0xD4, &y)) {   /* MEASURE_X, MEASURE_Y */
        return;
    }
    lastx = tp_map(x, TP_X_MIN, TP_X_MAX, TP_W, TP_INV_X);
    lasty = tp_map(y, TP_Y_MIN, TP_Y_MAX, TP_H, TP_INV_Y);
    data->point.x = lastx;
    data->point.y = lasty;
    data->state = LV_INDEV_STATE_PRESSED;
    if (!was_pressed) {       /* log the press edge only (raw + mapped), to verify */
        ESP_LOGI(TAG, "touch down raw(%u,%u) z1=%u -> screen(%ld,%ld)",
                 x, y, z1, (long)lastx, (long)lasty);
        was_pressed = true;
    }
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
        return NULL;
    }

    esp_lcd_panel_io_handle_t io;
    if (parlio_io_new(&io) != ESP_OK) {
        return NULL;
    }
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_ili9341(io, &panel_cfg, &s_panel) != ESP_OK) {
        return NULL;
    }
    /* HARDWARE reset on GPIO25, exactly as the stock firmware does it (low
     * 100 ms, high 100 ms). Without this physical reset the ILI9341 comes up
     * backlit-but-blank (white) and silently accepts commands without ever
     * displaying — which is what we were seeing. */
    const gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    banshee_panel_init_seq(io);            /* the stock power/VCOM/gamma init */
    esp_lcd_panel_disp_on_off(s_panel, true);

    esp_lv_adapter_config_t adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_cfg.task_core_id = MUSE_UI_CORE;
    adapter_cfg.task_priority = MUSE_UI_PRIORITY;
    if (esp_lv_adapter_init(&adapter_cfg) != ESP_OK) {
        return NULL;
    }
    const esp_lv_adapter_display_config_t disp_cfg = {
        .panel = s_panel,
        .panel_io = io,
        .profile = {
            .interface = ESP_LV_ADAPTER_PANEL_IF_OTHER,
            .rotation = ESP_LV_ADAPTER_ROTATE_0,
            .hor_res = LCD_H_RES,
            .ver_res = LCD_V_RES,
            .buffer_height = DRAW_BUF_LINES,
            /* LVGL draw buffers in PSRAM: the flush bounce-copies them into an
             * internal DMA buffer anyway, so the display tolerates PSRAM
             * latency fine. This frees internal RAM for Wi-Fi, which does NOT
             * tolerate latency (beacons/ACKs) and must live in internal RAM. */
            .use_psram = true,
            .require_double_buffer = true,
        },
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE,
    };
    lv_display_t *disp = esp_lv_adapter_register_display(&disp_cfg);
    if (!disp) {
        return NULL;
    }

    /* TSC2007 resistive touch (drag-scroll). The indev reports no touch until
     * audio_init opens the 0x4B handle on the shared bus. */
    *touch = lv_indev_create();
    if (*touch) {
        lv_indev_set_type(*touch, LV_INDEV_TYPE_POINTER);
        lv_indev_set_display(*touch, disp);
        lv_indev_set_read_cb(*touch, tp_read);
    }

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
    esp_lcd_panel_disp_sleep(s_panel, sleep);
}

/* ---- Cross-chip audio bridge ------------------------------------------
 * The mic and speaker are physically on the S3. The S3 runs a bridge that
 * streams mic PCM to us over the inter-chip UART and plays back PCM we send.
 * We present that link to Muse as two esp_codec_dev handles (a shared data_if
 * whose read() pulls mic frames, write() pushes speaker frames), and configure
 * the TLV320 speaker codec over our own I2C. Muse runs 16 kHz / 16-bit /
 * stereo; the S3 link carries mono, so we expand on read and downmix on write.
 * Frame: 0xA5 0x5A | seq | nsamp_lo nsamp_hi (uint16 LE) | pcm int16 LE | sum8.
 */
#define LINK_UART     UART_NUM_1
#define LINK_TX_GPIO  13
#define LINK_RX_GPIO  14
#define LINK_BAUD     921600
#define LINK_NS_TX    160            /* samples per speaker frame (10 ms) */
#define LINK_PREROLL_BYTES (150 * 16 * 2)   /* ~150 ms cushion before playback drains, so the first word doesn't stutter */
#define MIC_SB_BYTES  3072           /* ~96 ms of mono int16 before we drop */

static StreamBufferHandle_t s_mic_sb;      /* RX task -> mic read(): mono int16 */
static StreamBufferHandle_t s_spk_sb;      /* link_write -> spk_tx_task: mono int16 */
static bool s_audio_opened;

/* Reassemble mic frames from the S3 and feed mono samples to the mic buffer. */
static void link_rx_task(void *arg)
{
    static int16_t pcm[1024];
    for (;;) {
        uint8_t b;
        if (uart_read_bytes(LINK_UART, &b, 1, pdMS_TO_TICKS(1000)) <= 0) continue;
        if (b != 0xA5) continue;
        if (uart_read_bytes(LINK_UART, &b, 1, pdMS_TO_TICKS(50)) <= 0 || b != 0x5A) continue;
        uint8_t hdr[3];
        if (uart_read_bytes(LINK_UART, hdr, 3, pdMS_TO_TICKS(50)) < 3) continue;
        uint16_t nsamp = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
        if (nsamp == 0 || nsamp > 1024) continue;
        int plen = nsamp * 2, got = 0;
        while (got < plen) {
            int n = uart_read_bytes(LINK_UART, (uint8_t *)pcm + got, plen - got, pdMS_TO_TICKS(100));
            if (n <= 0) break;
            got += n;
        }
        if (got < plen) continue;
        if (uart_read_bytes(LINK_UART, &b, 1, pdMS_TO_TICKS(50)) <= 0) continue;
        uint8_t sum = 0;
        for (int i = 0; i < plen; i++) sum += ((uint8_t *)pcm)[i];
        if (sum != b) continue;
        /* Drop when full (Muse not draining) so latency stays bounded. */
        if (xStreamBufferSpacesAvailable(s_mic_sb) >= (size_t)plen) {
            xStreamBufferSend(s_mic_sb, pcm, plen, 0);
        }
    }
}

static int link_open(const audio_codec_data_if_t *h, void *cfg, int sz) { (void)h; (void)cfg; (void)sz; return ESP_CODEC_DEV_OK; }
static bool link_is_open(const audio_codec_data_if_t *h) { (void)h; return true; }
static int link_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t t, bool en) { (void)h; (void)t; (void)en; return ESP_CODEC_DEV_OK; }
static int link_set_fmt(const audio_codec_data_if_t *h, esp_codec_dev_type_t t, esp_codec_dev_sample_info_t *fs) { (void)h; (void)t; (void)fs; return ESP_CODEC_DEV_OK; }
static int link_close(const audio_codec_data_if_t *h) { (void)h; return ESP_CODEC_DEV_OK; }

/* Muse reads stereo 16-bit; the link is mono -> duplicate each sample L=R. */
static int link_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    int frames = size / 4;                 /* stereo int16 frames requested */
    int16_t *out = (int16_t *)data;
    static int16_t mono[512];

    /* Cap latency: the S3 streams mic at a real 16 kHz, so if more than the
     * request plus a small margin has piled up (e.g. before a voice session
     * started), drop the excess so capture stays real-time instead of draining
     * a backlog faster than real time (which skews the rate). */
    size_t keep = (size_t)(frames + 160) * sizeof(int16_t);
    size_t avail = xStreamBufferBytesAvailable(s_mic_sb);
    while (avail > keep) {
        size_t drop = avail - keep;
        if (drop > sizeof(mono)) drop = sizeof(mono);
        size_t g = xStreamBufferReceive(s_mic_sb, mono, drop, 0);
        if (!g) break;
        avail -= g;
    }

    int done = 0;
    while (done < frames) {
        int want = frames - done;
        if (want > 512) want = 512;
        /* Block until real mic samples arrive (the S3 feeds at a true 16 kHz,
         * so this paces Muse's capture to real time). Only pad+bail if the S3
         * genuinely goes quiet for 300 ms, to avoid a deadlock. A partial
         * return is NOT underflow — keep accumulating, don't inject silence. */
        size_t got = xStreamBufferReceive(s_mic_sb, mono, want * sizeof(int16_t), pdMS_TO_TICKS(300));
        int ns = got / sizeof(int16_t);
        for (int i = 0; i < ns; i++) { out[(done + i) * 2] = mono[i]; out[(done + i) * 2 + 1] = mono[i]; }
        done += ns;
        if (ns == 0) {                     /* link starved for 300 ms: pad rest, don't stall */
            for (int i = done; i < frames; i++) { out[i * 2] = 0; out[i * 2 + 1] = 0; }
            break;
        }
    }
    return ESP_CODEC_DEV_OK;               /* data_if read reports status, not byte count */
}

/* Muse writes stereo 16-bit; downmix to mono and hand it to the TX buffer.
 * Blocking send paces Muse's playback to the TX task's steady 16 kHz drain
 * (so the reply caption scrolls at real speed), while spk_tx_task sends a
 * continuous, gap-free frame stream to the S3 (no silence-dilution). */
static int link_write(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    int frames = size / 4;
    const int16_t *in = (const int16_t *)data;
    static int16_t mono[512];
    int off = 0;
    while (off < frames) {
        int m = 0;
        while (m < 512 && off < frames) {
            /* Downmix (L+R)/2 at unity. Loudness comes from the TLV320 class-D
             * analog gain (24 dB); real TTS audio is hot enough that digital
             * boost here would only clip. Clipped guard kept for safety. */
            int32_t s = ((int32_t)in[off * 2] + in[off * 2 + 1]) / 2;
            if (s > 32767) s = 32767; else if (s < -32768) s = -32768;
            mono[m++] = (int16_t)s;
            off++;
        }
        if (s_spk_sb) {
            xStreamBufferSend(s_spk_sb, mono, m * sizeof(int16_t), pdMS_TO_TICKS(500));
        }
    }
    return ESP_CODEC_DEV_OK;               /* data_if write reports status, not byte count */
}

/* Steady 16 kHz feeder: drains the TX buffer and sends a continuous frame
 * stream to the S3 (silence when idle), so the S3 never has to insert its own
 * silence between bursts. Paced by a sample clock. */
static void spk_tx_task(void *arg)
{
    (void)arg;
    static int16_t mono[LINK_NS_TX];
    int64_t next = esp_timer_get_time();
    uint8_t seq = 0;
    bool primed = false;          /* false = building the pre-roll cushion (sending silence) */
    int64_t fill_start = 0;       /* when audio first appeared, for the short-reply fallback */
    int empty_run = 0;            /* consecutive empty frames while primed, to detect idle */
    for (;;) {
        int n = 0;
        if (s_spk_sb) {
            if (!primed) {
                /* Pre-roll: hold off draining until a cushion builds, so the first
                 * word doesn't stutter while the decode pipeline ramps up. Prime
                 * on the cushion, or shortly after any audio arrives so a reply
                 * shorter than the cushion still plays. */
                size_t avail = xStreamBufferBytesAvailable(s_spk_sb);
                if (avail == 0) {
                    fill_start = 0;
                } else if (fill_start == 0) {
                    fill_start = esp_timer_get_time();
                }
                if (avail >= LINK_PREROLL_BYTES ||
                    (fill_start && esp_timer_get_time() - fill_start > 250000)) {
                    primed = true;
                    empty_run = 0;
                }
            }
            if (primed) {
                size_t got = xStreamBufferReceive(s_spk_sb, mono, LINK_NS_TX * sizeof(int16_t), 0);
                n = got / sizeof(int16_t);
                /* A brief gap mid-reply just pads one silent frame; a sustained
                 * gap (reply ended / idle) re-arms the pre-roll for the next one. */
                if (n == 0) {
                    if (++empty_run > 50) { primed = false; fill_start = 0; }
                } else {
                    empty_run = 0;
                }
            }
        }
        for (int i = n; i < LINK_NS_TX; i++) mono[i] = 0;   /* pad with silence */
        uint8_t hdr[5] = { 0xA5, 0x5A, seq++, LINK_NS_TX & 0xFF, LINK_NS_TX >> 8 };
        uint8_t sum = 0; uint8_t *pb = (uint8_t *)mono;
        for (int i = 0; i < LINK_NS_TX * 2; i++) sum += pb[i];
        uart_write_bytes(LINK_UART, (const char *)hdr, 5);
        uart_write_bytes(LINK_UART, (const char *)mono, LINK_NS_TX * 2);
        uart_write_bytes(LINK_UART, (const char *)&sum, 1);
        next += (int64_t)LINK_NS_TX * 1000000 / 16000;      /* 10 ms per frame */
        int64_t wait = next - esp_timer_get_time();
        if (wait > 2000) vTaskDelay(pdMS_TO_TICKS(wait / 1000));
        else if (wait < -100000) next = esp_timer_get_time();   /* resync if far behind */
    }
}

static const audio_codec_data_if_t s_link_data_if = {
    .open = link_open, .is_open = link_is_open, .enable = link_enable,
    .set_fmt = link_set_fmt, .read = link_read, .write = link_write, .close = link_close,
};

/* Minimal speaker codec_if. Its presence (a non-NULL set_vol) makes esp_codec_dev
 * SKIP its software volume (sw_vol), which was zeroing the reply. Volume/mute are
 * handled by the TLV320 hardware instead. */
static int spk_if_open(const audio_codec_if_t *h, void *cfg, int sz) { (void)h; (void)cfg; (void)sz; return ESP_CODEC_DEV_OK; }
static bool spk_if_is_open(const audio_codec_if_t *h) { (void)h; return true; }
static int spk_if_enable(const audio_codec_if_t *h, bool en) { (void)h; (void)en; return ESP_CODEC_DEV_OK; }
static int spk_if_set_fs(const audio_codec_if_t *h, esp_codec_dev_sample_info_t *fs) { (void)h; (void)fs; return ESP_CODEC_DEV_OK; }
static int spk_if_mute(const audio_codec_if_t *h, bool mute) { (void)h; tlv320dac3100_set_mute(mute); return ESP_CODEC_DEV_OK; }
static int spk_if_set_vol(const audio_codec_if_t *h, float db) { (void)h; (void)db; return ESP_CODEC_DEV_OK; }  /* vol via TLV320 hw; disables sw_vol */
static int spk_if_close(const audio_codec_if_t *h) { (void)h; return ESP_CODEC_DEV_OK; }
static const audio_codec_if_t s_spk_codec_if = {
    .open = spk_if_open, .is_open = spk_if_is_open, .enable = spk_if_enable, .set_fs = spk_if_set_fs,
    .mute = spk_if_mute, .set_vol = spk_if_set_vol, .close = spk_if_close,
};

static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    if (s_audio_opened) { return ESP_OK; }

    /* Inter-chip UART to the S3 audio engine (C5 TX13/RX14 <-> S3 TX9/RX10). */
    const uart_config_t uc = {
        .baud_rate = LINK_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(LINK_UART, 2048, 2048, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(LINK_UART, &uc), TAG, "uart cfg");
    ESP_RETURN_ON_ERROR(uart_set_pin(LINK_UART, LINK_TX_GPIO, LINK_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart pins");

    s_mic_sb = xStreamBufferCreate(MIC_SB_BYTES, 2);
    ESP_RETURN_ON_FALSE(s_mic_sb, ESP_ERR_NO_MEM, TAG, "mic streambuffer");
    xTaskCreate(link_rx_task, "link_rx", 3072, NULL, 6, NULL);

    s_spk_sb = xStreamBufferCreate(8192, 1);   /* link_write -> spk_tx_task */
    ESP_RETURN_ON_FALSE(s_spk_sb, ESP_ERR_NO_MEM, TAG, "spk streambuffer");
    xTaskCreate(spk_tx_task, "spk_tx", 3072, NULL, 6, NULL);

    /* Speaker codec: the S3 clocks the I2S into the TLV320; we control it over
     * I2C. The S3 bridge is already clocking BCLK by now, so the PLL can lock. */
    /* Power the audio rail: the Banshee gates the TLV320's power behind a
     * PCA9539/TCA9539 IO expander at I2C 0x74 (SDA6/SCL7). Pin P1_3 (bit 11)
     * high enables it (reverse-engineered by sweeping the expander). GhostESP
     * sets this; Muse must too. The PCA9539 latches its output register, so it
     * holds until power loss. (config reg 0x07: 1=input/0=output; data reg 0x03) */
    {
        i2c_master_bus_config_t bc = {
            .i2c_port = 0, .sda_io_num = 6, .scl_io_num = 7,
            .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        };
        bc.flags.enable_internal_pullup = true;
        i2c_master_bus_handle_t bus = NULL;
        if (i2c_new_master_bus(&bc, &bus) == ESP_OK) {
            i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                       .device_address = 0x74, .scl_speed_hz = 100000 };
            i2c_master_dev_handle_t ex = NULL;
            if (i2c_master_bus_add_device(bus, &dc, &ex) == ESP_OK) {
                #define PW(reg, val) do { uint8_t b_[2] = {(reg), (val)}; i2c_master_transmit(ex, b_, 2, 100); } while (0)
                PW(0x03, 0x08);   /* P1 data: P1_3 high (codec power) */
                PW(0x07, 0xF7);   /* P1 config: P1_3 output, rest input */
                #undef PW
                i2c_master_bus_rm_device(ex);
                ESP_LOGI(TAG, "audio rail enabled via PCA9539@0x74 P1_3");
                vTaskDelay(pdMS_TO_TICKS(80));   /* let the TLV320 power up */
            }
            i2c_del_master_bus(bus);
        }
    }

    tlv320dac3100_config_t tc = TLV320DAC3100_DEFAULT_CONFIG();   /* port0 sda6 scl7 0x18 */
    esp_err_t r = ESP_FAIL;
    for (int attempt = 0; attempt < 6; attempt++) {              /* the I2C probe is flaky at cold start */
        r = tlv320dac3100_init(&tc);
        if (r == ESP_OK) break;
        tlv320dac3100_deinit();
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    if (r == ESP_OK) {
        tlv320dac3100_set_sample_rate(16000);
        tlv320dac3100_set_output_route(DAC_ROUTE_SPK);
        tlv320dac3100_set_volume(100);
        tlv320dac3100_set_mute(false);
    } else {
        ESP_LOGW(TAG, "TLV320 init failed (%s); speaker may be silent", esp_err_to_name(r));
    }

    /* Front-panel buttons share the 0x74 expander (Port1 input bits 0-2, the
     * three front buttons, active-low). Grab a persistent handle on the TLV320's
     * I2C bus (port 0) so poll_buttons() can read them — this lets the front
     * buttons act as talk, off the BOOT button (which sits next to RESET). */
    {
        i2c_master_bus_handle_t bus = NULL;
        if (i2c_master_get_bus_handle(0, &bus) == ESP_OK && bus) {
            i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                       .device_address = 0x74, .scl_speed_hz = 100000 };
            if (i2c_master_bus_add_device(bus, &dc, &s_expander) == ESP_OK) {
                ESP_LOGI(TAG, "front buttons: 0x74 expander handle ready");
            } else {
                s_expander = NULL;
            }

            /* TSC2007 resistive touch controller on the same bus (drag-scroll). */
            i2c_device_config_t td = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                       .device_address = 0x4B, .scl_speed_hz = 100000 };
            if (i2c_master_bus_add_device(bus, &td, &s_touch) == ESP_OK) {
                ESP_LOGI(TAG, "touch: TSC2007@0x4B handle ready");
            } else {
                s_touch = NULL;
            }
        }
    }

    esp_codec_dev_cfg_t out_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_OUT, .codec_if = &s_spk_codec_if, .data_if = &s_link_data_if };
    esp_codec_dev_cfg_t in_cfg  = { .dev_type = ESP_CODEC_DEV_TYPE_IN,  .codec_if = NULL, .data_if = &s_link_data_if };
    *spk = esp_codec_dev_new(&out_cfg);
    *mic = esp_codec_dev_new(&in_cfg);
    ESP_RETURN_ON_FALSE(*spk && *mic, ESP_FAIL, TAG, "codec_dev_new");

    s_audio_opened = true;
    ESP_LOGI(TAG, "audio bridge up: mic/speaker over S3 link (UART1 TX%d/RX%d @%d), TLV320 %s",
             LINK_TX_GPIO, LINK_RX_GPIO, LINK_BAUD, r == ESP_OK ? "on" : "FAILED");
    return ESP_OK;
}

/* The three front buttons are expander Port1 (input reg 0x01) bits 0-2,
 * active-low (idle 0x0F). Identified live 2026-10-07 by logging the raw byte on
 * each press:
 *   bit 0 (0x0E) = centre, bit 2 (0x0B) = left  -> talk (push-to-talk)
 *   bit 1 (0x0D) = bottom-right                 -> mute toggle (speaker on/off)
 * Talk sits on two buttons so the user needn't reach the BOOT button next to
 * RESET. Read ~every 50 ms (poll_buttons runs at 10 ms) to keep I2C light. */
#define FRONT_MASK 0x07                        /* P10|P11|P12 = bits 0,1,2 */
#define MUTE_BIT   0x02                        /* bottom-right button */
#define TALK_MASK  (FRONT_MASK & ~MUTE_BIT)    /* centre + left = 0x05 */
static unsigned poll_front(void)
{
    static unsigned tick;
    static uint8_t last = 0xFF;
    static bool mute_down;
    if (!s_expander || tick++ % 5) {
        return 0;
    }
    uint8_t reg = 0x01, val = 0xFF;
    if (i2c_master_transmit_receive(s_expander, &reg, 1, &val, 1, 20) != ESP_OK) {
        return 0;
    }
    if ((val & FRONT_MASK) != (last & FRONT_MASK)) {
        ESP_LOGI(TAG, "front buttons: P1 in = 0x%02X", val);
        last = val;
    }
    /* Bottom-right button: toggle the speaker on its press edge (mute). The UI
     * shows a muted-speaker icon whenever the speaker is off. */
    bool mdown = !(val & MUTE_BIT);
    if (mdown && !mute_down) {
        mute_down = true;
        bool on = !muse_settings_speaker_on();
        muse_settings_set_speaker_on(on);
        ESP_LOGI(TAG, "mute button: speaker %s", on ? "on" : "off");
    } else if (!mdown && mute_down) {
        mute_down = false;
    }
    /* Centre + left buttons: push-to-talk. */
    bool down = (val & TALK_MASK) != TALK_MASK;
    if (down && !s_front_down) { s_front_down = true;  return MUSE_BTN_TALK_PRESS; }
    if (!down && s_front_down) { s_front_down = false; return MUSE_BTN_TALK_RELEASE; }
    return 0;
}

static unsigned poll_buttons(void)
{
    unsigned front = poll_front();
    if (front) {
        return front;
    }
    return muse_gpio_button_poll(&s_talk);   /* BOOT still works as a fallback */
}

static esp_err_t power_off(void)
{
    set_brightness(0);
    if (s_panel) {
        esp_lcd_panel_disp_on_off(s_panel, false);
    }
    esp_deep_sleep_start();   /* no RTC power latch; wakes on reset */
    return ESP_FAIL;
}

static const muse_board_t s_board = {
    .name = "Wired Hatters Banshee",
    .width = LCD_H_RES,
    .height = LCD_V_RES,
    .round = false,
    /* The panel has a resistive touch controller and we register an LVGL
     * pointer indev for it (display_start), which is enough to drag-scroll the
     * reply. We leave this flag false so the full touch UI (the swipe-to-
     * settings tileview and the on-screen speaker button) stays off: that UI
     * wants a calibrated panel, and setup still happens over BLE as before. */
    .touch = false,
    .diagonal_in = 2.4f,
    .talk_button = "boot",
    .talk_hint = { LV_ALIGN_TOP_LEFT, 8, 8 },
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = esp_lv_adapter_unlock,
    .set_brightness = set_brightness,
    .panel_sleep = panel_sleep,
    .audio_init = audio_init,
    .mic_slot = -1,
    .poll_buttons = poll_buttons,
    .power_off = power_off,
};

const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
