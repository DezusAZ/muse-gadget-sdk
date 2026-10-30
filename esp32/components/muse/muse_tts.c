// Google Translate TTS fetcher. See muse_tts.h for the contract.
//
// Uses the free, key-less translate_tts endpoint: it returns an MP3 of the
// text in one voice, capped at ~200 characters per request, so long replies
// are split on word boundaries and fetched as back-to-back MP3s (minimp3 picks
// the stream back up at each boundary). It's an unofficial endpoint and can
// change, which is why the URL is built in one place and the HTTP status and
// content type are logged: swapping in OpenAI/ElevenLabs/Piper means editing
// build_tts_url() and (for keyed services) the request headers in fetch_chunk.

#include "muse_tts.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "muse_tts";

#define TTS_TEXT_MAX    1024                 /* one reply message */
#define TTS_CHUNK_CHARS 180                  /* under the endpoint's ~200-char cap */
#define TTS_MP3_BUF     (32 * 1024)          /* fetch task -> session handoff */
#define TTS_LANG        "en"
#define TTS_UA          "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 " \
                        "(KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
#define TTS_HTTP_TIMEOUT_MS 10000

static StreamBufferHandle_t s_mp3;           /* MP3 bytes, fetch task -> session */
static TaskHandle_t         s_task;
static SemaphoreHandle_t    s_lock;          /* guards s_text */
static char                 s_text[TTS_TEXT_MAX];
static volatile bool        s_busy;
static volatile uint32_t    s_req;           /* bumped to abort the in-flight fetch */

// Percent-encode src[0..n) into dst (cap incl. NUL). Leaves RFC 3986 unreserved
// bytes as-is, encodes the rest (UTF-8 bytes included). Returns dst length.
static size_t url_encode(const char *src, size_t n, char *dst, size_t cap)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t o = 0;
    for (size_t i = 0; i < n && o + 4 < cap; i++) {
        unsigned char c = (unsigned char)src[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            dst[o++] = (char)c;
        } else {
            dst[o++] = '%';
            dst[o++] = hex[c >> 4];
            dst[o++] = hex[c & 0xF];
        }
    }
    dst[o] = '\0';
    return o;
}

// Build the TTS request URL for one text chunk into `url`. Swap this (and, for
// keyed providers, the headers in fetch_chunk) to change TTS backends.
static void build_tts_url(const char *chunk, size_t len, char *url, size_t cap)
{
    char enc[TTS_CHUNK_CHARS * 3 + 1];
    url_encode(chunk, len, enc, sizeof(enc));
    snprintf(url, cap,
             "https://translate.google.com/translate_tts"
             "?ie=UTF-8&client=tw-ob&tl=" TTS_LANG "&total=1&idx=0&q=%s",
             enc);
}

// True if the first bytes of a response look like an MP3 (ID3 tag or frame sync).
static bool looks_like_mp3(const uint8_t *b, int n)
{
    if (n >= 3 && b[0] == 'I' && b[1] == 'D' && b[2] == '3') return true;
    if (n >= 1 && b[0] == 0xFF) return true;
    return false;
}

// Fetch one chunk's MP3 and stream it into s_mp3. Stops early if the request
// generation changed (aborted). Returns false on HTTP/transport failure.
static bool fetch_chunk(const char *chunk, size_t len, uint32_t req)
{
    char url[TTS_CHUNK_CHARS * 3 + 128];
    uint8_t buf[1536];
    bool ok = false, checked = false, aborted = false;
    size_t total = 0;

    build_tts_url(chunk, len, url, sizeof(url));

    esp_http_client_config_t cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = TTS_HTTP_TIMEOUT_MS,
        .buffer_size = 1536,
        .disable_auto_redirect = true,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        ESP_LOGE(TAG, "http init failed");
        return false;
    }
    esp_http_client_set_header(c, "User-Agent", TTS_UA);
    esp_http_client_set_header(c, "Referer", "https://translate.google.com/");

    esp_err_t err = esp_http_client_open(c, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: %s", esp_err_to_name(err));
    } else {
        esp_http_client_fetch_headers(c);
        int status = esp_http_client_get_status_code(c);
        if (status != 200) {
            ESP_LOGW(TAG, "TTS HTTP %d (%u-char chunk) — endpoint may have changed",
                     status, (unsigned)len);
        } else {
            while (s_req == req) {
                int r = esp_http_client_read(c, (char *)buf, sizeof(buf));
                if (r < 0) {
                    ESP_LOGW(TAG, "read error after %u bytes", (unsigned)total);
                    break;
                }
                if (r == 0) {
                    ok = esp_http_client_is_complete_data_received(c);
                    break;
                }
                if (!checked) {
                    checked = true;
                    if (!looks_like_mp3(buf, r)) {
                        ESP_LOGW(TAG, "not MP3 (first bytes %02X %02X) — wrong endpoint or consent page",
                                 buf[0], r > 1 ? buf[1] : 0);
                        break;
                    }
                }
                size_t off = 0;
                while (off < (size_t)r) {
                    /* Non-blocking send under the lock so a concurrent reset in
                     * muse_tts_request/abort can't race the write; re-check the
                     * generation under the lock before writing. */
                    size_t sent = 0;
                    xSemaphoreTake(s_lock, portMAX_DELAY);
                    if (s_req == req) {
                        sent = xStreamBufferSend(s_mp3, buf + off, (size_t)r - off, 0);
                    }
                    xSemaphoreGive(s_lock);
                    if (s_req != req) {
                        aborted = true;
                        break;
                    }
                    off += sent;
                    if (off < (size_t)r) {
                        vTaskDelay(pdMS_TO_TICKS(5));   /* buffer full: let the reader drain */
                    }
                }
                if (aborted) {
                    break;
                }
                total += (size_t)r;
            }
        }
    }

    if (ok) {
        ESP_LOGI(TAG, "TTS chunk ok: %u MP3 bytes", (unsigned)total);
    }
    esp_http_client_close(c);
    esp_http_client_cleanup(c);
    return ok;
}

// Split `text` into <=TTS_CHUNK_CHARS pieces on spaces and fetch each in order.
static void fetch_all(const char *text, uint32_t req)
{
    size_t n = strlen(text), i = 0;
    while (i < n && s_req == req) {
        size_t end = i + TTS_CHUNK_CHARS;
        if (end >= n) {
            end = n;
        } else {
            size_t sp = end;                  /* back up to the last space */
            while (sp > i && text[sp] != ' ') sp--;
            if (sp > i) end = sp;
        }
        fetch_chunk(text + i, end - i, req);
        i = end;
        while (i < n && text[i] == ' ') i++;   /* skip the split space */
    }
}

static void tts_task(void *arg)
{
    (void)arg;
    static char text[TTS_TEXT_MAX];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        uint32_t req = s_req;                  /* the request we're serving */
        strlcpy(text, s_text, sizeof(text));
        xSemaphoreGive(s_lock);
        if (text[0] && s_req == req) {
            fetch_all(text, req);
        }
        if (s_req == req) {
            s_busy = false;                    /* only clear for the request we served */
        }
        ESP_LOGI(TAG, "fetch done (req %u), stack free %u",
                 (unsigned)req, (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }
}

void muse_tts_init(void)
{
    if (s_task) {
        return;
    }
    s_mp3 = xStreamBufferCreateWithCaps(TTS_MP3_BUF, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_lock = xSemaphoreCreateMutex();
    if (!s_mp3 || !s_lock ||
        xTaskCreatePinnedToCoreWithCaps(tts_task, "muse_tts", 16 * 1024, NULL, 5, &s_task, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "init failed (no PSRAM?)");
        s_task = NULL;
        return;
    }
    ESP_LOGI(TAG, "ready (Google TTS, %s)", TTS_LANG);
}

bool muse_tts_ready(void)
{
    return s_task != NULL;
}

void muse_tts_request(const char *text)
{
    if (!s_task || !text) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_req++;                                   /* abort any in-flight fetch (checked under lock) */
    strlcpy(s_text, text, sizeof(s_text));
    xStreamBufferReset(s_mp3);                 /* clear any bytes the old fetch left */
    xSemaphoreGive(s_lock);
    s_busy = true;
    xTaskNotifyGive(s_task);
}

void muse_tts_abort(void)
{
    if (!s_task) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_req++;
    xStreamBufferReset(s_mp3);
    xSemaphoreGive(s_lock);
    s_busy = false;
}

bool muse_tts_busy(void)
{
    return s_busy;
}

size_t muse_tts_available(void)
{
    if (!s_mp3) {
        return 0;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = xStreamBufferBytesAvailable(s_mp3);
    xSemaphoreGive(s_lock);
    return n;
}

size_t muse_tts_read(uint8_t *dst, size_t max)
{
    if (!s_mp3 || !max) {
        return 0;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = xStreamBufferReceive(s_mp3, dst, max, 0);
    xSemaphoreGive(s_lock);
    return n;
}
