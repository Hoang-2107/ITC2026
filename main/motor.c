#include "motor.h"
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "config.h"

/*
 * Hỗ trợ 2 loại driver (chọn MOTOR_DRIVER trong config.h):
 *
 *   1 = L298N: PWM ở chân ENA/ENB (độ lớn), IN1..IN4 chọn chiều (mức 0/1).
 *   2 = Driver MOSFET chỉ có IN1..IN4 (vd board đỏ "7A dual motor driver"):
 *       PWM thẳng vào chân IN.  Tiến: IN1 = PWM, IN2 = 0.  Lùi: IN1 = 0, IN2 = PWM.  Dừng: cả 2 = 0.
 */

#if MOTOR_DRIVER == 1
#define CH_LEFT  LEDC_CHANNEL_0
#define CH_RIGHT LEDC_CHANNEL_1
#else
#define CH_L_IN1 LEDC_CHANNEL_0
#define CH_L_IN2 LEDC_CHANNEL_1
#define CH_R_IN1 LEDC_CHANNEL_2
#define CH_R_IN2 LEDC_CHANNEL_3
#endif

/* Gắn 1 chân GPIO vào 1 kênh PWM (duty ban đầu = 0) */
static void pwm_pin(gpio_num_t pin, ledc_channel_t ch)
{
    ledc_channel_config_t c = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = ch,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void pwm_set(ledc_channel_t ch, int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

/*
 * Khởi tạo phần cứng điều khiển động cơ:
 *   - 1 timer PWM tần số PWM_FREQ_HZ, độ phân giải 10 bit (0..1023)
 *   - L298N: 2 kênh PWM cho ENA/ENB, IN1..IN4 là đầu ra số
 *   - Driver MOSFET: 4 kênh PWM cho IN1..IN4
 */
void motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

#if MOTOR_DRIVER == 1
    pwm_pin(MOTOR_L_PWM, CH_LEFT);
    pwm_pin(MOTOR_R_PWM, CH_RIGHT);
    uint64_t mask = (1ULL << MOTOR_L_IN1) | (1ULL << MOTOR_L_IN2) |
                    (1ULL << MOTOR_R_IN1) | (1ULL << MOTOR_R_IN2);
    int stby = MOTOR_STBY;
    if (stby >= 0) mask |= (1ULL << stby);
    gpio_config_t io = { .pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT };
    ESP_ERROR_CHECK(gpio_config(&io));
    if (stby >= 0) gpio_set_level((gpio_num_t)stby, 1);
#else
    pwm_pin(MOTOR_L_IN1, CH_L_IN1);
    pwm_pin(MOTOR_L_IN2, CH_L_IN2);
    pwm_pin(MOTOR_R_IN1, CH_R_IN1);
    pwm_pin(MOTOR_R_IN2, CH_R_IN2);
#endif

    motor_stop();
}

/* Đổi tốc độ -PWM_MAX..PWM_MAX thành độ rộng xung, có bù vùng chết MOTOR_MIN_DUTY */
static int to_duty(int speed)
{
    int duty = speed < 0 ? -speed : speed;
    if (duty > PWM_MAX) duty = PWM_MAX;
    if (duty > 0 && MOTOR_MIN_DUTY > 0) {
        duty = MOTOR_MIN_DUTY + duty * (PWM_MAX - MOTOR_MIN_DUTY) / PWM_MAX;
    }
    return duty;
}

#if MOTOR_DRIVER == 1
/*
 * L298N – điều khiển 1 động cơ.
 *   ch: kênh PWM ở chân EN;  in1, in2: chiều (1-0 tiến, 0-1 lùi, 0-0 thả trôi)
 */
static void motor_set(ledc_channel_t ch, gpio_num_t in1, gpio_num_t in2, bool invert, int speed)
{
    if (invert) speed = -speed;
    gpio_set_level(in1, speed > 0);
    gpio_set_level(in2, speed < 0);
    pwm_set(ch, to_duty(speed));
}
#else
/*
 * Driver MOSFET – điều khiển 1 động cơ bằng 2 kênh PWM ở IN1, IN2.
 *   Tiến: IN1 = PWM, IN2 = 0.  Lùi: IN1 = 0, IN2 = PWM.  Dừng: cả 2 = 0 (thả trôi).
 */
static void motor_set(ledc_channel_t ch1, ledc_channel_t ch2, bool invert, int speed)
{
    if (invert) speed = -speed;
    int duty = to_duty(speed);
    pwm_set(ch1, speed > 0 ? duty : 0);
    pwm_set(ch2, speed < 0 ? duty : 0);
}
#endif

/* Đặt tốc độ bánh trái và phải (-1023..1023) */
void motor_drive(int left, int right)
{
#if MOTOR_DRIVER == 1
    motor_set(CH_LEFT, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_INVERT, left);
    motor_set(CH_RIGHT, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_INVERT, right);
#else
    motor_set(CH_L_IN1, CH_L_IN2, MOTOR_L_INVERT, left);
    motor_set(CH_R_IN1, CH_R_IN2, MOTOR_R_INVERT, right);
#endif
}

/* Dừng cả 2 bánh */
void motor_stop(void)
{
    motor_drive(0, 0);
}
