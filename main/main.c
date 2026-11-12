/*
 * Robot AMHS – xe dò line 8 cảm biến LDR, PID, tránh vật cản HC-SR05. ESP32-S3, ESP-IDF v6.x
 *
 *   config.h        chân cắm + thông số mặc định
 *   main.c          luồng điều khiển chính (file này)
 *   line_sensor.c   đọc cảm biến line, hiệu chuẩn, tính vị trí line
 *   motor.c         driver động cơ L298N
 *   pid.c           bộ điều khiển PID
 *   obstacle.c      4 cảm biến HC-SR05, phát tín hiệu TH1 / TH2   (hcsr05.c: driver cảm biến)
 *   settings.c      thông số chỉnh được + lưu flash
 *   console.c       lệnh gõ trong Monitor ("help")
 *
 * Nút BOOT: chưa hiệu chuẩn -> hiệu chuẩn; đã hiệu chuẩn -> chạy / dừng.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "config.h"
#include "line_sensor.h"
#include "motor.h"
#include "pid.h"
#include "obstacle.h"
#include "hcsr05.h"
#include "settings.h"
#include "console.h"

#define P g_params                  // viết tắt cho gọn

/* ======================= Trạng thái ======================= */

static bool s_running = false;
static bool s_obst_stop = false;    // TH1: đang dừng vì vật cản
static int s_dir = 1;               // 1 = tiến, -1 = lùi (TH2 đảo hướng)
static int64_t s_dir_changed = 0;   // lần đảo chiều gần nhất (µs), 0 = chưa đảo
static bool s_print = DEBUG_PRINT;
static pid_state_t s_pid;
static int64_t s_run_start = 0;     // mốc tăng tốc mềm (µs)
static int64_t s_lost_since = 0;    // bắt đầu mất line (µs), 0 = đang thấy line

/* Bắt đầu lại dò line từ trạng thái sạch: xóa PID, tăng tốc mềm lại từ 0 */
static void resume_follow(int64_t now)
{
    s_obst_stop = false;
    pid_reset(&s_pid);
    s_lost_since = 0;
    s_run_start = now;
}

/* ======================= Hàm cho console.c ======================= */

void app_set_running(bool run)
{
    s_running = run;
    s_dir = 1;                      // mỗi lần bắt đầu chạy luôn đi tiến
    s_dir_changed = 0;
    obstacle_clear_reverse();
    resume_follow(esp_timer_get_time());
    motor_stop();
    printf("%s\n", run ? "CHAY" : "DUNG");
}

bool app_is_running(void) { return s_running; }
void app_set_print(bool on) { s_print = on; }

/*
 * Hiệu chuẩn: robot xoay trái rồi xoay phải tại chỗ trong CALIB_TIME_MS để cả 8 cảm biến
 * quét qua line và nền. Chưa lắp động cơ thì dùng tay kéo cảm biến qua lại trên line.
 */
void app_calibrate(void)
{
    s_running = false;
    printf("Hieu chuan %d ms... (khong cham vao robot)\n", CALIB_TIME_MS);
    line_sensor_calib_begin();

    int skipped = 0;
    int64_t half = esp_timer_get_time() + (int64_t)CALIB_TIME_MS * 500;
    int64_t end = half + (int64_t)CALIB_TIME_MS * 500;
    while (esp_timer_get_time() < end) {
        if (esp_timer_get_time() < half) motor_drive(-CALIB_SPEED, CALIB_SPEED);
        else motor_drive(CALIB_SPEED, -CALIB_SPEED);
        if (!line_sensor_calib_sample()) skipped++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motor_stop();
    line_sensor_calib_end();

    if (skipped) printf("Bo qua %d mau vi robot bi nhac len\n", skipped);
    line_sensor_calib_print();
    printf("Go \"save\" de luu hieu chuan, nhan BOOT hoac go \"run\" de chay\n");
}

/* ======================= Nút BOOT ======================= */

static void button_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

/* true đúng 1 lần mỗi khi nút vừa được nhấn (chống dội 50 ms) */
static bool button_pressed(void)
{
    static bool last = false;
    static int64_t last_change = 0;
    bool now = gpio_get_level(BUTTON_GPIO) == 0;
    int64_t t = esp_timer_get_time();
    if (now != last && t - last_change > 50000) {
        last = now;
        last_change = t;
        return now;
    }
    return false;
}

/* ======================= Điều khiển ======================= */

static int clamp(int v, int lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/*
 * Dò line 1 chu kỳ:
 *   - Thấy line: corr = PID(pos - 3500); bánh trái = base + corr, bánh phải = base - corr.
 *     Line lệch phải (corr > 0) -> bánh trái nhanh hơn -> rẽ phải về line. Chạy lùi: base = -rbase.
 *   - Mất line: xoay về phía thấy line lần cuối, quá lost_ms thì tự dừng.
 *   - ramp_ms đầu tiên: nhân tốc độ với hệ số 0..1 để tăng tốc mềm.
 */
static void control_step(bool on_line, int pos, int64_t now)
{
    float ramp = 1.0f;
    if (P.ramp_ms > 0) {
        ramp = (float)(now - s_run_start) / (P.ramp_ms * 1000.0f);
        if (ramp > 1.0f) ramp = 1.0f;
    }

    if (on_line) {
        s_lost_since = 0;
        float corr = pid_update(&s_pid, &P.pid, (float)(pos - LINE_CENTER));
        int base = P.base;
        if (s_dir < 0) {
            base = -P.rbase;
            corr *= REVERSE_STEER;
        }
        motor_drive(clamp((int)(ramp * (base + corr)), P.max), clamp((int)(ramp * (base - corr)), P.max));
        return;
    }

    if (s_lost_since == 0) s_lost_since = now;
    if (P.lost_ms > 0 && now - s_lost_since > (int64_t)P.lost_ms * 1000) {
        printf("Mat line qua %ld ms -> TU DUNG\n", (long)P.lost_ms);
        app_set_running(false);
        return;
    }
    int s = (int)(ramp * P.search);
    if (s_pid.last_error > 0) motor_drive(s, -s);
    else motor_drive(-s, s);
}

/*
 * Một chu kỳ khi đang chạy, theo thứ tự ưu tiên:
 *   1. TH1 (H1 và H2 cùng gần): dừng, chờ vật cản rời đi rồi chạy tiếp theo hướng cũ.
 *   2. TH2 (H3 hoặc H4 gần): đảo chiều, dò line theo hướng ngược lại.
 *   3. Bình thường: dò line.
 */
static void run_step(bool on_line, int pos, int64_t now)
{
    if (obstacle_stop_active()) {
        if (!s_obst_stop) {
            s_obst_stop = true;
            printf("TH1: H1 va H2 gap vat can < %ld cm -> DUNG\n", (long)P.obs.stop_cm);
        }
        motor_stop();
        obstacle_clear_reverse();
        return;
    }
    if (s_obst_stop) {
        printf("TH1: het vat can -> chay tiep\n");
        resume_follow(now);
    }

    if (obstacle_take_reverse() &&
        (s_dir_changed == 0 || now - s_dir_changed > (int64_t)DIR_COOLDOWN_MS * 1000)) {
        s_dir = -s_dir;
        s_dir_changed = now;
        printf("TH2: H3/H4 gap vat can -> DAO CHIEU, chay %s\n", s_dir > 0 ? "TIEN" : "LUI");
        resume_follow(now);
    }

    control_step(on_line, pos, now);
}

/* 1 dòng log: raw | norm | pos, err | khoảng cách H1..H4 */
static void print_sensors(const int raw[], const int norm[], bool on_line, int pos)
{
    float cm[NUM_SONARS];
    obstacle_get_cm(cm);

    printf("raw:");
    for (int i = 0; i < NUM_SENSORS; i++) printf(" %4d", raw[i]);
    printf(" | norm:");
    for (int i = 0; i < NUM_SENSORS; i++) printf(" %4d", norm[i]);
    if (!line_sensor_is_calibrated()) printf(" | CHUA HIEU CHUAN");
    else if (on_line) printf(" | pos=%4d err=%+5d", pos, pos - LINE_CENTER);
    else printf(" | MAT LINE");
    printf(" | H:");
    for (int i = 0; i < NUM_SONARS; i++) {
        if (cm[i] > 0) printf(" %3d", (int)cm[i]);
        else printf("  --");
    }
    if (s_running && s_obst_stop) printf(" [DUNG VAT CAN]");
    if (s_running && s_dir < 0) printf(" [LUI]");
    printf("\n");
}

/* Vòng lặp chính, 5 ms/lần: lệnh Monitor -> nút BOOT -> đọc line -> điều khiển -> in log */
static void line_follow_task(void *arg)
{
    int raw[NUM_SENSORS], norm[NUM_SENSORS];
    int64_t last_print = 0;
    TickType_t wake = xTaskGetTickCount();

    printf("\n=== Robot AMHS do line === Go \"help\" de xem lenh\n");
    printf(line_sensor_is_calibrated() ? "Nhan BOOT hoac go \"run\" de chay\n"
                                       : "Chua hieu chuan: dat robot len line, nhan BOOT hoac go \"cal\"\n");
    settings_print(s_running);

    while (1) {
        console_poll();

        if (button_pressed()) {
            if (!line_sensor_is_calibrated()) app_calibrate();
            else app_set_running(!s_running);
        }

        int pos = 0;
        bool on_line = false;
        if (line_sensor_is_calibrated()) {
            line_sensor_read_norm(raw, norm);
            on_line = line_sensor_position(norm, P.noise, P.lost, &pos);
        } else {
            line_sensor_read_raw(raw);
            memset(norm, 0, sizeof(norm));
        }

        int64_t now = esp_timer_get_time();
        if (s_running) run_step(on_line, pos, now);

        if (s_print && now - last_print > 200000) {
            last_print = now;
            print_sensors(raw, norm, on_line, pos);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

/* ======================= Chế độ test riêng HC-SR05 (TEST_SONAR_ONLY = 1) ======================= */

static void print_cm(const char *name, float cm)
{
    if (cm > 0) printf("%s=%5.1f cm  ", name, cm);
    else printf("%s=   --     ", name);          // không đo được: không có vật / chưa cắm
}

static void sonar_test_task(void *arg)
{
    printf("\n=== TEST HC-SR05 ===\n"
           "TH1: che CA H1 VA H2 < %ld cm  -> [TH1: DUNG]\n"
           "TH2: che H3 HOAC H4 < %ld cm   -> [TH2: DAO CHIEU] (1 lan moi lan che)\n\n",
           (long)P.obs.stop_cm, (long)P.obs.turn_cm);
    // Cặp đang debug: H1+H2 (chỉ số 0, 1) hoặc H3+H4 (chỉ số 2, 3)
    const int a = (SONAR_DEBUG_PAIR == 2) ? 2 : 0;
    const bool is_th2 = (SONAR_DEBUG_PAIR == 2);
    if (SONAR_DEBUG_PAIR) printf("Dang debug H%d va H%d. In 1 dong moi khi khoang cach thay doi >= 1 cm. Dung yen thi khong in.\n\n",
                                a + 1, a + 2);

    if (TEST_SONAR_ONLY == 2)
        printf(">>> TEST KEM DONG CO: ke banh xe len khong cham dat, nhan BOOT de cho dong co chay / dung.\n"
               "    TH2 (H3/H4 < %ld cm): dong co DAO CHIEU.  TH1 (H1+H2 < %ld cm): dong co DUNG.\n\n",
               (long)P.obs.turn_cm, (long)P.obs.stop_cm);

    float last_cm[2] = { -100, -100 };
    int last_st[2] = { -1, -1 };
    int th2_count = 0;
    bool motor_on = false, motor_held = false;
    int mdir = 1;
    while (1) {
        float cm[NUM_SONARS];
        obstacle_get_cm(cm);

        // TH2 phát 1 lần mỗi lần gặp vật -> lấy ngay mỗi vòng, nếu không sẽ bị bỏ lỡ
        bool th2_now = obstacle_take_reverse();

        /* ---- Test kèm động cơ L298N (TEST_SONAR_ONLY = 2) ---- */
        if (TEST_SONAR_ONLY == 2) {
            if (button_pressed()) {
                motor_on = !motor_on;
                mdir = 1;
                printf("=== DONG CO: %s ===\n", motor_on ? "CHAY TIEN" : "DUNG");
            }
            if (motor_on && th2_now) {
                mdir = -mdir;
                printf("=== DONG CO DAO CHIEU -> %s ===\n", mdir > 0 ? "TIEN" : "LUI");
            }
            bool hold = motor_on && obstacle_stop_active();
            if (hold != motor_held) {
                motor_held = hold;
                if (motor_on) printf("=== DONG CO: %s ===\n", hold ? "DUNG VI TH1" : "CHAY TIEP");
            }
            if (motor_on && !hold) motor_drive(mdir * TEST_MOTOR_SPEED, mdir * TEST_MOTOR_SPEED);
            else motor_stop();
        }

        if (SONAR_DEBUG_PAIR) {
            if (!is_th2) th2_now = false;       // đang debug H1/H2 thì không in TH2
            if (th2_now) th2_count++;

            // In khi khoảng cách đổi >= 1 cm, khi trạng thái (đo được / lỗi) đổi, hoặc khi vừa có TH2
            int st[2] = { obstacle_get_status(a), obstacle_get_status(a + 1) };
            bool changed = th2_now;
            for (int k = 0; k < 2; k++) {
                if (st[k] != last_st[k]) changed = true;
                else if (cm[a + k] > 0 && fabsf(cm[a + k] - last_cm[k]) >= 1.0f) changed = true;
            }
            if (changed) {
                bool warn_wire = false;
                for (int k = 0; k < 2; k++) {
                    int i = a + k;
                    last_cm[k] = cm[i];
                    last_st[k] = st[k];
                    if (cm[i] > 0) printf("H%d=%5.1f cm    ", i + 1, cm[i]);
                    else if (st[k] == HCSR05_NO_FALL) printf("H%d= xa/trong   ", i + 1);
                    else if (st[k] == HCSR05_OUT_OF_RANGE) printf("H%d= ngoai tam (%d us)  ", i + 1, obstacle_get_pulse_us(i));
                    else if (st[k] == HCSR05_ECHO_STUCK_HIGH) { printf("H%d= LOI ECHO ket 1  ", i + 1); warn_wire = true; }
                    else { printf("H%d= LOI khong ECHO  ", i + 1); warn_wire = true; }
                }
                long lim = is_th2 ? (long)P.obs.turn_cm : (long)P.obs.stop_cm;
                bool n0 = cm[a] > 0 && cm[a] < lim;
                bool n1 = cm[a + 1] > 0 && cm[a + 1] < lim;
                if (is_th2) {
                    // TH2: chỉ cần 1 trong 2 cảm biến gần là đảo chiều
                    if (th2_now) printf("[TH2: DAO CHIEU] (lan thu %d)", th2_count);
                    else if (n0 || n1) printf("(%s < %ld cm, da phat TH2 hoac dang xac nhan...)", n0 ? "H3" : "H4", lim);
                } else {
                    // TH1: phải cả 2 cảm biến cùng gần mới dừng
                    if (obstacle_stop_active()) printf("[TH1: DUNG]");
                    else if (n0 && n1) printf("(ca 2 < %ld cm, dang xac nhan TH1...)", lim);
                    else if (n0 || n1) printf("(chi %s < %ld cm -> chua du TH1)", n0 ? "H1" : "H2", lim);
                }
                if (warn_wire) printf("  -> kiem tra VCC 5V, GND chung, cau phan ap ECHO, chan OUT de trong");
                printf("\n");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        } else {
            print_cm("H1", cm[0]);
            print_cm("H2", cm[1]);
            print_cm("H3", cm[2]);
            print_cm("H4", cm[3]);
            if (obstacle_stop_active()) printf("[TH1: DUNG] ");
            if (th2_now) printf("[TH2: DAO CHIEU]");
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    console_init();
    settings_load();

    if (TEST_SONAR_ONLY) {
        motor_init();                       // động cơ đứng yên (chế độ 2: chạy khi nhấn BOOT)
        button_init();
        g_params.obs.enable = 1;            // luôn bật xử lý vật cản khi test
        obstacle_init(&g_params.obs);
        xTaskCreate(sonar_test_task, "sonar_test", 4096, NULL, 5, NULL);
        return;
    }

    line_sensor_init();
    motor_init();
    button_init();
    obstacle_init(&g_params.obs);

    xTaskCreatePinnedToCore(line_follow_task, "line", 6144, NULL, 10, NULL, 1);
}
