#include "hcsr05.h"
#include <stdbool.h>
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

static bool s_isr_service = false;

/*
 * Ngắt chân ECHO (chạy mỗi khi ECHO đổi mức):
 *   - cạnh lên:  ghi thời điểm bắt đầu xung
 *   - cạnh xuống: ghi thời điểm kết thúc và báo cho task đang chờ là đo xong
 */
static void echo_isr(void *arg)
{
    hcsr05_t *s = (hcsr05_t *)arg;
    int64_t now = esp_timer_get_time();

    if (gpio_get_level(s->echo)) {
        s->t_rise = now;
    } else if (s->t_rise != 0 && s->t_fall == 0) {
        s->t_fall = now;
        TaskHandle_t w = s->waiter;
        if (w) {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(w, &woken);
            portYIELD_FROM_ISR(woken);
        }
    }
}

/* Cấu hình chân TRIG / ECHO và gắn hàm ngắt cho ECHO */
void hcsr05_init(hcsr05_t *s, gpio_num_t trig, gpio_num_t echo)
{
    s->status = HCSR05_NO_ECHO;
    s->trig = trig;
    s->echo = echo;
    s->t_rise = 0;
    s->t_fall = 0;
    s->waiter = NULL;

    gpio_config_t out = { .pin_bit_mask = 1ULL << trig, .mode = GPIO_MODE_OUTPUT };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(trig, 0);

    // Kéo xuống để chân ECHO không bị thả nổi khi chưa cắm cảm biến (tránh báo vật cản giả)
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << echo,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    if (!s_isr_service) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
        s_isr_service = true;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(echo, echo_isr, s));
}

/*
 * Đo 1 lần:
 *   1. phát xung 10 µs ở TRIG -> cảm biến bắn 8 xung siêu âm 40 kHz
 *   2. ECHO lên 1 trong thời gian sóng đi và về
 *   3. khoảng cách (cm) = thời gian (µs) / 58
 */
float hcsr05_read_cm(hcsr05_t *s, int timeout_ms)
{
    // ECHO vẫn đang cao từ lần đo trước (không có tiếng vọng) -> bỏ lần này
    if (gpio_get_level(s->echo)) { s->status = HCSR05_ECHO_STUCK_HIGH; return -1; }

    s->t_rise = 0;
    s->t_fall = 0;
    s->waiter = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);                 // xóa thông báo cũ (nếu có)

    gpio_set_level(s->trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s->trig, 0);

    bool done = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
    s->waiter = NULL;

    if (s->t_rise == 0) { s->status = HCSR05_NO_ECHO; return -1; }
    if (!done || s->t_fall <= s->t_rise) { s->status = HCSR05_NO_FALL; return -1; }

    s->last_us = (int32_t)(s->t_fall - s->t_rise);
    float cm = (float)s->last_us / 58.0f;
    if (cm < 2.0f || cm > 400.0f) { s->status = HCSR05_OUT_OF_RANGE; return -1; }   // ngoài tầm đo
    s->status = HCSR05_OK;
    return cm;
}
