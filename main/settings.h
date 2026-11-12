#pragma once
/*
 * Thông số chỉnh được lúc chạy (PID, tốc độ, ngưỡng) và lưu / nạp từ flash (NVS).
 * Giá trị mặc định lấy từ config.h; lệnh "save" trong Monitor ghi lại để tắt nguồn không mất.
 */
#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
#include "obstacle.h"

typedef struct {
    uint32_t magic;                  // đổi cấu trúc thì đổi magic để không nạp nhầm dữ liệu cũ
    pid_gains_t pid;
    int32_t base, rbase, max, search, noise, lost;
    int32_t lost_ms, ramp_ms;
    obstacle_cfg_t obs;
} params_t;

extern params_t g_params;

/* Khởi tạo flash, nạp thông số và hiệu chuẩn đã lưu (nếu có). Gọi 1 lần lúc khởi động. */
void settings_load(void);

/* Lưu thông số hiện tại + hiệu chuẩn cảm biến vào flash */
void settings_save(void);

/* Xóa mọi thứ đã lưu (về mặc định config.h sau khi reset board) */
void settings_erase(void);

/* In toàn bộ thông số ra Monitor */
void settings_print(bool running);
