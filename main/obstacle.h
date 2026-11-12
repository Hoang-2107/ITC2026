#pragma once
/*
 * Xử lý vật cản bằng 4 cảm biến HC-SR05 (H1..H4), chạy trong task riêng trên lõi 0.
 *
 *   TH1: H1 VÀ H2 cùng < stop_cm  -> phát tín hiệu DỪNG (giữ tới khi vật cản rời đi)
 *   TH2: H3 HOẶC H4 < turn_cm     -> phát tín hiệu ĐẢO CHIỀU (1 lần mỗi lần gặp vật cản)
 *
 * Task điều khiển đọc tín hiệu bằng obstacle_stop_active() / obstacle_take_reverse().
 */
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

typedef struct {
    int32_t enable;     // 0 = bỏ qua vật cản (khi chưa lắp cảm biến)
    int32_t stop_cm;    // ngưỡng TH1
    int32_t turn_cm;    // ngưỡng TH2
} obstacle_cfg_t;

/* Khởi tạo 4 cảm biến và chạy task đo. cfg phải tồn tại suốt chương trình (đổi được lúc chạy). */
void obstacle_init(const obstacle_cfg_t *cfg);

/* TH1: true trong suốt thời gian H1 và H2 cùng bị chắn */
bool obstacle_stop_active(void);

/* TH2: trả về true đúng 1 lần mỗi khi H3 hoặc H4 gặp vật cản (đọc xong tự xóa) */
bool obstacle_take_reverse(void);

/* Xóa tín hiệu đảo chiều đang chờ (dùng khi đang dừng vì TH1) */
void obstacle_clear_reverse(void);

/* Khoảng cách đo gần nhất của H1..H4 (cm, -1 = không đo được) */
void obstacle_get_cm(float cm[NUM_SONARS]);

/* Trạng thái lần đo gần nhất của cảm biến i (hcsr05_status_t) – dùng để chẩn đoán đấu dây */
int obstacle_get_status(int i);

/* Độ dài xung ECHO lần đo gần nhất của cảm biến i (µs) – để chẩn đoán */
int obstacle_get_pulse_us(int i);
