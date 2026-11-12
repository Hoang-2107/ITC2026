#pragma once
/*
 * Lệnh gõ trong Monitor qua cổng USB của ESP32-S3 (gõ "help" để xem danh sách).
 */
#include <stdbool.h>

/* Cài driver USB-Serial-JTAG để vừa in log vừa đọc được lệnh. Gọi đầu tiên trong app_main. */
void console_init(void);

/* Đọc lệnh (không chờ), gọi đều đặn trong vòng lặp chính */
void console_poll(void);

/* ---- Các hàm main.c cung cấp để console điều khiển robot ---- */
void app_set_running(bool run);
bool app_is_running(void);
void app_calibrate(void);
void app_set_print(bool on);
