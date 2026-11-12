#pragma once
/*
 * Điều khiển 2 động cơ DC qua driver L298N (ENA/ENB = PWM, IN1..IN4 = chiều quay).
 * Tốc độ: -PWM_MAX..PWM_MAX, dương = tiến, âm = lùi, 0 = thả trôi.
 * Chân và chiều quay cấu hình trong config.h.
 */

/* Cấu hình PWM (LEDC, tần số PWM_FREQ_HZ, 10 bit) và các chân IN1..IN4. Gọi 1 lần lúc khởi động. */
void motor_init(void);

/*
 * Đặt tốc độ 2 bánh cùng lúc.
 *   left, right: -1023..1023 (dương = tiến, âm = lùi, 0 = thả trôi)
 * Ví dụ: motor_drive(400, 400) đi thẳng; motor_drive(-300, 300) xoay trái tại chỗ.
 */
void motor_drive(int left, int right);

/* Dừng cả 2 động cơ (thả trôi) */
void motor_stop(void);
