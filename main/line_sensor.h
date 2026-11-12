#pragma once
/*
 * Cảm biến dò line: 8 LDR qua ADC1.
 * Đọc giá trị thô, hiệu chuẩn min/max, chuẩn hóa về 0..1000 và tính vị trí line.
 */
#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#define LINE_CENTER 3500                     // vị trí chính giữa (LDR1 = 0, LDR8 = 7000)

typedef struct {
    uint32_t magic;
    int16_t min[NUM_SENSORS];
    int16_t max[NUM_SENSORS];
} line_calib_t;

void line_sensor_init(void);

/* Đọc giá trị ADC thô (0..4095), càng tối càng cao */
void line_sensor_read_raw(int raw[NUM_SENSORS]);

/* Đọc và chuẩn hóa về 0..1000 theo hiệu chuẩn, 1000 = nằm trên line */
void line_sensor_read_norm(int raw[NUM_SENSORS], int norm[NUM_SENSORS]);

/*
 * Vị trí line 0..7000 từ giá trị chuẩn hóa.
 * noise: bỏ qua cảm biến có norm < noise; lost_sum: tổng < mức này coi là mất line.
 * Trả về false nếu mất line.
 */
bool line_sensor_position(const int norm[NUM_SENSORS], int noise, int lost_sum, int *pos);

/* ---- Hiệu chuẩn ---- */
void line_sensor_calib_begin(void);              // xóa min/max để bắt đầu hiệu chuẩn
bool line_sensor_calib_sample(void);             // lấy 1 mẫu; false nếu bỏ qua (robot bị nhấc)
void line_sensor_calib_end(void);                // đánh dấu đã hiệu chuẩn
void line_sensor_calib_print(void);
bool line_sensor_is_calibrated(void);

/* Dùng để lưu / nạp hiệu chuẩn từ flash */
const line_calib_t *line_sensor_get_calib(void);
bool line_sensor_set_calib(const line_calib_t *c);   // false nếu dữ liệu không hợp lệ
