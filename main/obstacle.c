#include "obstacle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "hcsr05.h"
#include <stdio.h>
/* Các "tín hiệu ngắt" gửi sang task điều khiển */
#define SIG_STOP     BIT0      // TH1: đang có vật cản phía trước -> dừng
#define SIG_REVERSE  BIT1      // TH2: cần đảo chiều chạy

static const gpio_num_t TRIG_PINS[NUM_SONARS] = SONAR_TRIG_LIST;
static const gpio_num_t ECHO_PINS[NUM_SONARS] = SONAR_ECHO_LIST;

static hcsr05_t s_sonar[NUM_SONARS];
static volatile float s_cm[NUM_SONARS];
static EventGroupHandle_t s_sig;
static const obstacle_cfg_t *s_cfg;

/* true nếu đo được và khoảng cách nhỏ hơn ngưỡng (-1 = không có vật -> false) */
static bool is_near(float cm, float limit)
{
    return cm > 0 && cm < limit;
}

/*
 * Task đo vật cản: lần lượt đo H1 -> H4 (đo lần lượt để sóng không lẫn nhau),
 * rồi xét TH1, TH2. Mỗi trường hợp phải đúng SONAR_CONFIRM lần liên tiếp mới phát
 * tín hiệu, để 1 lần đo nhiễu không làm robot dừng / quay đầu nhầm.
 */
static void obstacle_task(void *arg)
{
    int stop_cnt = 0, clear_cnt = 0, turn_cnt = 0;

    while (1) {
        // SONAR_DEBUG_PAIR: 1 = chỉ đo H1, H2; 2 = chỉ đo H3, H4; 0 = đo cả 4
        for (int i = 0; i < NUM_SONARS; i++) {
            bool skip = (SONAR_DEBUG_PAIR == 1 && i >= 2) || (SONAR_DEBUG_PAIR == 2 && i < 2);
            if (skip) { s_cm[i] = -1; continue; }
            s_cm[i] = hcsr05_read_cm(&s_sonar[i], SONAR_TIMEOUT_MS);
            vTaskDelay(pdMS_TO_TICKS(SONAR_GAP_MS));
        }
        // Debug: bỏ // ở 2 dòng dưới để in khoảng cách 4 cảm biến mỗi vòng đo
        // printf("[SONAR] H1=%5.1f H2=%5.1f H3=%5.1f H4=%5.1f | dem TH1=%d TH2=%d\n",
        //        s_cm[0], s_cm[1], s_cm[2], s_cm[3], stop_cnt, turn_cnt);
        if (!s_cfg->enable) {
            xEventGroupClearBits(s_sig, SIG_STOP | SIG_REVERSE);
            stop_cnt = clear_cnt = turn_cnt = 0;
            continue;
        }

        /* ---- TH1: H1 và H2 cùng gần -> dừng ---- */
        bool stopping = (xEventGroupGetBits(s_sig) & SIG_STOP) != 0;
        if (!stopping) {
            bool both_near = is_near(s_cm[0], s_cfg->stop_cm) && is_near(s_cm[1], s_cfg->stop_cm);
            stop_cnt = both_near ? stop_cnt + 1 : 0;
            if (stop_cnt >= SONAR_CONFIRM) {
                printf(">>> [TH1] H1=%.1f va H2=%.1f < %ld cm -> PHAT TIN HIEU DUNG\n",
                       s_cm[0], s_cm[1], (long)s_cfg->stop_cm);
                xEventGroupSetBits(s_sig, SIG_STOP);
                clear_cnt = 0;
            }
        } else {
            // Nhả dừng khi vật cản lùi xa hơn ngưỡng + khoảng trễ (tránh dừng / chạy liên tục ở sát ngưỡng)
            float release = s_cfg->stop_cm + STOP_HYST_CM;
            bool still_near = is_near(s_cm[0], release) && is_near(s_cm[1], release);
            clear_cnt = still_near ? 0 : clear_cnt + 1;
            if (clear_cnt >= SONAR_CONFIRM) {
                printf("<<< [TH1] het vat can (H1=%.1f H2=%.1f) -> NHA DUNG\n", s_cm[0], s_cm[1]);
                xEventGroupClearBits(s_sig, SIG_STOP);
                stop_cnt = 0;
            }
        }

        /* ---- TH2: H3 hoặc H4 gần -> đảo chiều (phát 1 lần mỗi lần gặp vật cản) ---- */
        bool side_near = is_near(s_cm[2], s_cfg->turn_cm) || is_near(s_cm[3], s_cfg->turn_cm);
        turn_cnt = side_near ? turn_cnt + 1 : 0;
        if (turn_cnt == SONAR_CONFIRM) {
            printf(">>> [TH2] H3=%.1f H4=%.1f < %ld cm -> PHAT TIN HIEU DAO CHIEU\n",
                   s_cm[2], s_cm[3], (long)s_cfg->turn_cm);
            xEventGroupSetBits(s_sig, SIG_REVERSE);
        }
    }
}

void obstacle_init(const obstacle_cfg_t *cfg)
{
    s_cfg = cfg;
    s_sig = xEventGroupCreate();
    for (int i = 0; i < NUM_SONARS; i++) {
        s_cm[i] = -1;
        hcsr05_init(&s_sonar[i], TRIG_PINS[i], ECHO_PINS[i]);
    }
    // Lõi 0, ưu tiên thấp hơn task dò line (lõi 1) để không ảnh hưởng vòng điều khiển
    xTaskCreatePinnedToCore(obstacle_task, "obstacle", 3072, NULL, 5, NULL, 0);
}

bool obstacle_stop_active(void)
{
    return (xEventGroupGetBits(s_sig) & SIG_STOP) != 0;
}

bool obstacle_take_reverse(void)
{
    return (xEventGroupClearBits(s_sig, SIG_REVERSE) & SIG_REVERSE) != 0;
}

void obstacle_clear_reverse(void)
{
    xEventGroupClearBits(s_sig, SIG_REVERSE);
}

void obstacle_get_cm(float cm[NUM_SONARS])
{
    for (int i = 0; i < NUM_SONARS; i++) cm[i] = s_cm[i];
}

int obstacle_get_status(int i)
{
    return s_sonar[i].status;
}

int obstacle_get_pulse_us(int i)
{
    return s_sonar[i].last_us;
}
