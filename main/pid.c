#include "pid.h"

/*
 * Xóa trạng thái PID (tích phân, đạo hàm, sai số trước).
 * Gọi mỗi lần robot bắt đầu chạy để dữ liệu cũ không làm robot giật.
 */
void pid_reset(pid_state_t *s)
{
    s->integral = 0;
    s->deriv = 0;
    s->last_error = 0;
}

/*
 * Tính giá trị điều khiển từ sai số hiện tại.
 *   P = kp * error             -> lệch càng nhiều, bẻ lái càng mạnh
 *   I = ki * tổng các error    -> bù sai lệch kéo dài (thường để ki = 0)
 *   D = kd * tốc độ đổi error  -> hãm dao động, giúp robot bớt lắc
 * Gọi đều đặn mỗi chu kỳ (LOOP_PERIOD_MS). Trả về P + I + D.
 */
float pid_update(pid_state_t *s, const pid_gains_t *g, float error)
{
    // Khâu I: cộng dồn sai số, giới hạn để không tăng mãi (chống bão hòa)
    s->integral += error;
    if (s->integral > g->i_limit) s->integral = g->i_limit;
    if (s->integral < -g->i_limit) s->integral = -g->i_limit;

    // Khâu D qua bộ lọc thông thấp để không khuếch đại nhiễu cảm biến
    s->deriv = g->dfilt * s->deriv + (1.0f - g->dfilt) * (error - s->last_error);
    s->last_error = error;

    // Tổng 3 khâu = lượng bẻ lái
    return g->kp * error + g->ki * s->integral + g->kd * s->deriv;
}
