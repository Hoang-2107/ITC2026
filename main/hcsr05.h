#pragma once
/*
 * Driver cảm biến siêu âm HC-SR05 (dùng được cho HC-SR04).
 * Đo độ rộng xung ECHO bằng ngắt GPIO (cạnh lên / cạnh xuống), không chờ bận CPU.
 * Mỗi cảm biến là một biến hcsr05_t riêng -> dùng được nhiều cảm biến cùng lúc.
 *
 * ⚠ ECHO của HC-SR05 ra mức 5V: phải qua cầu phân áp (vd 1k + 2k) trước khi vào ESP32.
 */
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Kết quả lần đo gần nhất – dùng để chẩn đoán lỗi đấu dây */
typedef enum {
    HCSR05_OK = 0,          // đo được
    HCSR05_ECHO_STUCK_HIGH, // ECHO luôn ở mức 1 trước khi đo
    HCSR05_NO_ECHO,         // phát TRIG nhưng ECHO không hề lên mức 1
    HCSR05_NO_FALL,         // ECHO lên 1 nhưng không xuống trong thời gian chờ (không có vật / quá xa)
    HCSR05_OUT_OF_RANGE,    // đo được nhưng ngoài 2..400 cm
} hcsr05_status_t;

typedef struct {
    volatile int status;             // hcsr05_status_t của lần đo gần nhất
    volatile int32_t last_us;        // độ dài xung ECHO lần đo gần nhất (µs), để chẩn đoán
    gpio_num_t trig;                 // chân phát xung kích (đầu ra)
    gpio_num_t echo;                 // chân nhận xung phản hồi (đầu vào, có ngắt)
    volatile int64_t t_rise;         // thời điểm ECHO lên mức 1 (µs)
    volatile int64_t t_fall;         // thời điểm ECHO xuống mức 0 (µs)
    volatile TaskHandle_t waiter;    // task đang chờ kết quả đo
} hcsr05_t;

/* Khởi tạo 1 cảm biến: TRIG làm đầu ra, ECHO làm đầu vào có ngắt 2 cạnh. Gọi 1 lần lúc khởi động. */
void hcsr05_init(hcsr05_t *s, gpio_num_t trig, gpio_num_t echo);

/*
 * Đo khoảng cách (cm). Chặn task gọi tối đa timeout_ms.
 * Trả về -1 nếu không đo được (không có vật trong tầm, chưa cắm cảm biến, nhiễu).
 */
float hcsr05_read_cm(hcsr05_t *s, int timeout_ms);
