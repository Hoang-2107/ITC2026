#pragma once
/*
 * Cấu hình phần cứng robot dò line – ESP32-S3
 *
 * Mạch cảm biến (Doc/): 8 LDR, mỗi LDR kéo lên 3V3 qua 10k (RN1), chân còn lại xuống GND.
 *   -> Vạch đen: ít ánh sáng phản xạ -> LDR trở cao -> điện áp CAO.
 *   -> Nền trắng: nhiều ánh sáng      -> LDR trở thấp -> điện áp THẤP.
 * Header 10 chân: 1 = 3V3, 2 = GND, 3..10 = LDR1..LDR8 (kiểm tra lại với board thật).
 *
 * Cảm biến nối vào ADC1 (GPIO1..GPIO8 = ADC1_CH0..CH7), không dùng ADC2 vì xung đột Wi-Fi.
 * LDR1 đặt bên TRÁI robot (nhìn theo hướng chạy), LDR8 bên PHẢI.
 */

#include "hal/adc_types.h"
#include "driver/gpio.h"

/* ---------- Cảm biến ---------- */
#define NUM_SENSORS 8
// LDR1..LDR8 -> GPIO1..GPIO8 (ADC1_CH0..CH7)
#define SENSOR_CHANNEL_LIST { \
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, \
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, \
}

#define LINE_IS_DARK      1     // 1: line đen trên nền trắng; 0: line trắng trên nền đen
#define SENSOR_SAMPLES    4     // số lần đọc ADC để lấy trung bình
#define SENSOR_NOISE      150   // giá trị chuẩn hóa (0..1000) dưới mức này coi như không thấy line
#define LINE_LOST_SUM     400   // tổng giá trị < mức này -> mất line
#define AMBIENT_REJECT    1     // 1: trừ phần ánh sáng môi trường làm cả dãy cùng tăng/giảm
#define CALIB_LIFT_RAW    3800  // khi hiệu chuẩn, cả 8 cảm biến > mức này = robot bị nhấc -> bỏ qua

/* ---------- Driver động cơ ----------
 * MOTOR_DRIVER = 1: L298N (có ENA, ENB)
 *   ⚠ Tháo 2 jumper ENA, ENB, rồi nối:
 *   ENA -> GPIO9 (PWM)   IN1 -> GPIO10   IN2 -> GPIO11
 *   ENB -> GPIO12 (PWM)  IN3 -> GPIO13   IN4 -> GPIO14
 *
 * MOTOR_DRIVER = 2: driver MOSFET chỉ có IN1..IN4 (board đỏ "7A dual motor driver")
 *   PWM thẳng vào IN, KHÔNG có ENA/ENB (GPIO9, GPIO12 để trống):
 *   IN1 -> GPIO10   IN2 -> GPIO11   (động cơ TRÁI, cổng OUT1/OUT2)
 *   IN3 -> GPIO13   IN4 -> GPIO14   (động cơ PHẢI, cổng OUT3/OUT4)
 *   +5V của header -> 5V,  GND của header -> GND chung với ESP32
 *
 * Cả 2 loại: GND driver nối chung GND ESP32; pin động cơ cắm vào cầu đấu nguồn của driver.
 */
#define MOTOR_DRIVER  2             // 1 = L298N, 2 = driver MOSFET IN1..IN4

#define MOTOR_L_PWM   GPIO_NUM_9    // ENA
#define MOTOR_L_IN1   GPIO_NUM_10   // IN1
#define MOTOR_L_IN2   GPIO_NUM_11   // IN2
#define MOTOR_R_PWM   GPIO_NUM_12   // ENB
#define MOTOR_R_IN1   GPIO_NUM_13   // IN3
#define MOTOR_R_IN2   GPIO_NUM_14   // IN4
#define MOTOR_STBY    GPIO_NUM_NC   // L298N không có chân STBY (GPIO15 để trống)

#define MOTOR_L_INVERT 0            // đổi thành 1 nếu bánh trái quay ngược
#define MOTOR_R_INVERT 0            // đổi thành 1 nếu bánh phải quay ngược

// L298N đóng cắt chậm -> 2 kHz. Driver MOSFET cách ly quang -> 5 kHz (quang trở chậm, không nên quá 10 kHz).
// Động cơ có thể kêu rít nhẹ, bình thường.
#if MOTOR_DRIVER == 1
#define PWM_FREQ_HZ   2000
#else
#define PWM_FREQ_HZ   5000
#endif
#define PWM_MAX       1023          // độ phân giải 10 bit

// PWM nhỏ nhất để bánh bắt đầu quay (L298N sụt ~2 V nên động cơ cần PWM khá cao mới chạy).
// Tìm bằng lệnh "motor N N", tăng dần N tới khi bánh vừa quay, rồi đặt giá trị đó vào đây.
// Mọi tốc độ khác 0 sẽ được dời lên trong khoảng MOTOR_MIN_DUTY..PWM_MAX.
#define MOTOR_MIN_DUTY 0

/* ---------- Cảm biến siêu âm HC-SR05 (H1..H4) ---------- */
// ⚠ ECHO ra 5V -> qua cầu phân áp (1k nối tiếp, 2k xuống GND) trước khi vào ESP32
#define NUM_SONARS 4
//                         H1           H2           H3           H4
#define SONAR_TRIG_LIST { GPIO_NUM_16, GPIO_NUM_18, GPIO_NUM_38, GPIO_NUM_40 }
#define SONAR_ECHO_LIST { GPIO_NUM_17, GPIO_NUM_21, GPIO_NUM_39, GPIO_NUM_41 }

// Chế độ test cảm biến siêu âm (không đọc line):
//   0 = TẮT, robot dò line bình thường
//   1 = chỉ test HC-SR05,  động cơ đứng yên
//   2 = test HC-SR05 KÈM ĐỘNG CƠ L298N: nhấn BOOT cho 2 bánh chạy / dừng,
//       TH2 -> đảo chiều, TH1 -> dừng. ⚠ Kê bánh xe lên không chạm đất.
#define TEST_SONAR_ONLY   2
#define TEST_MOTOR_SPEED  450   // tốc độ 2 bánh khi test chế độ 2 (0..1023)

// Chọn cặp cảm biến khi debug (cặp còn lại không đo, coi như không có vật):
//   1 = chỉ H1, H2 (TH1)   2 = chỉ H3, H4 (TH2)   0 = đo cả 4 (khi đã lắp đủ)
#define SONAR_DEBUG_PAIR  2

#define SONAR_ENABLE      0     // 0 = bỏ qua vật cản, chỉ dò line (lệnh "sonar 0/1"). Lắp xong HC-SR05 thì đổi 1
#define STOP_DIST_CM      10    // TH1: H1 VÀ H2 cùng < mức này -> dừng (lệnh "stopcm N")
#define STOP_HYST_CM      3     // vật cản phải ra xa hơn STOP_DIST_CM + mức này mới chạy lại
#define TURN_DIST_CM      10    // TH2: H3 HOẶC H4 < mức này -> đảo chiều chạy (lệnh "turncm N")
#define SONAR_CONFIRM     2     // số lần đo liên tiếp phải đúng mới phát tín hiệu (chống nhiễu)
#define SONAR_TIMEOUT_MS  25    // chờ tiếng vọng tối đa (25 ms ~ 4 m)
#define SONAR_GAP_MS      5     // nghỉ giữa 2 cảm biến để sóng không lẫn nhau

/* ---------- Đảo chiều chạy (TH2) ---------- */
#define REVERSE_SPEED     350   // tốc độ cơ bản khi chạy lùi (lệnh "rbase N"), nên chậm hơn BASE_SPEED
#define REVERSE_STEER     1     // đổi thành -1 nếu khi chạy lùi robot bẻ lái ra xa line thay vì về line
#define DIR_COOLDOWN_MS   1500  // sau khi đảo chiều, bỏ qua TH2 trong thời gian này (tránh đảo liên tục)

/* ---------- Nút nhấn ---------- */
#define BUTTON_GPIO   GPIO_NUM_0    // nút BOOT trên board DevKit

/* ---------- Điều khiển ---------- */
#define LOOP_PERIOD_MS 5
#define BASE_SPEED     450          // tốc độ cơ bản (0..PWM_MAX)
#define MAX_SPEED      800
#define SEARCH_SPEED   400          // tốc độ xoay tìm lại line khi mất line
#define CALIB_SPEED    300          // tốc độ xoay tại chỗ khi hiệu chuẩn
#define CALIB_TIME_MS  4000

// Giá trị mặc định. Có thể chỉnh lúc chạy bằng lệnh trong Monitor (kp 0.3, kd 2.5...) rồi "save".
#define KP 0.25f
#define KI 0.0f
#define KD 2.0f

#define LOST_TIMEOUT_MS 1500        // mất line quá lâu -> tự dừng (lệnh "lostms N", 0 = không dừng)
#define D_FILTER        0.7f        // lọc khâu D: 0 = không lọc, 0.9 = lọc mạnh (lệnh "dfilt X")
#define RAMP_MS         500         // thời gian tăng tốc từ 0 lên tốc độ đặt khi bắt đầu chạy (lệnh "ramp N")

#define DEBUG_PRINT   0           // 1: in giá trị cảm biến ra Monitor mỗi 200 ms (lệnh "print 0/1")
