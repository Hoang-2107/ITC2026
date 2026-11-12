#pragma once
/*
 * Bộ điều khiển PID dùng chung, có lọc thông thấp cho khâu D và giới hạn khâu I.
 * Hệ số (pid_gains_t) tách riêng với trạng thái (pid_state_t) để chỉnh / lưu flash dễ dàng.
 */

typedef struct {
    float kp, ki, kd;
    float dfilt;        // lọc khâu D: 0 = không lọc .. 0.95 = lọc mạnh
    float i_limit;      // giới hạn tổng tích phân (chống bão hòa)
} pid_gains_t;

typedef struct {
    float integral;
    float deriv;        // khâu D đã lọc
    float last_error;
} pid_state_t;

/* Xóa trạng thái PID, gọi khi bắt đầu chạy */
void pid_reset(pid_state_t *s);

/*
 * Tính đầu ra điều khiển từ sai số, gọi đều đặn mỗi chu kỳ.
 *   s:     trạng thái của bộ PID này (mỗi bộ PID một biến riêng)
 *   g:     hệ số kp, ki, kd, dfilt, i_limit
 *   error: sai số hiện tại (dò line: vị trí line - 3500)
 * Trả về lượng điều chỉnh (dò line: cộng vào bánh trái, trừ ở bánh phải).
 */
float pid_update(pid_state_t *s, const pid_gains_t *g, float error);
