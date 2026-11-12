#include "settings.h"
#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"
#include "line_sensor.h"

#define PARAMS_MAGIC 0x4C494E06u
#define NVS_NS       "line"

static const char *TAG = "SETTINGS";

params_t g_params = {
    .magic = PARAMS_MAGIC,
    .pid = { .kp = KP, .ki = KI, .kd = KD, .dfilt = D_FILTER, .i_limit = 100000 },
    .base = BASE_SPEED, .rbase = REVERSE_SPEED, .max = MAX_SPEED, .search = SEARCH_SPEED,
    .noise = SENSOR_NOISE, .lost = LINE_LOST_SUM,
    .lost_ms = LOST_TIMEOUT_MS, .ramp_ms = RAMP_MS,
    .obs = { .enable = SONAR_ENABLE, .stop_cm = STOP_DIST_CM, .turn_cm = TURN_DIST_CM },
};

void settings_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;

    params_t p;
    size_t len = sizeof(p);
    if (nvs_get_blob(h, "params", &p, &len) == ESP_OK && len == sizeof(p) && p.magic == PARAMS_MAGIC) {
        g_params = p;
        ESP_LOGI(TAG, "Đã nạp thông số từ flash");
    }
    line_calib_t c;
    len = sizeof(c);
    if (nvs_get_blob(h, "calib", &c, &len) == ESP_OK && len == sizeof(c) && line_sensor_set_calib(&c)) {
        ESP_LOGI(TAG, "Đã nạp hiệu chuẩn từ flash (gõ \"cal\" để hiệu chuẩn lại)");
    }
    nvs_close(h);
}

void settings_save(void)
{
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NS, NVS_READWRITE, &h));
    nvs_set_blob(h, "params", &g_params, sizeof(g_params));
    bool cal = line_sensor_is_calibrated();
    if (cal) nvs_set_blob(h, "calib", line_sensor_get_calib(), sizeof(line_calib_t));
    nvs_commit(h);
    nvs_close(h);
    printf("Da luu thong so%s vao flash\n", cal ? " + hieu chuan" : "");
}

void settings_erase(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    printf("Da xoa. Nhan RESET tren board de ve mac dinh\n");
}

void settings_print(bool running)
{
    const params_t *p = &g_params;
    printf("kp=%.3f ki=%.4f kd=%.3f dfilt=%.2f base=%ld rbase=%ld max=%ld search=%ld ramp=%ld\n"
           "noise=%ld lost=%ld lostms=%ld | sonar=%ld stopcm=%ld turncm=%ld | %s\n",
           p->pid.kp, p->pid.ki, p->pid.kd, p->pid.dfilt, (long)p->base, (long)p->rbase, (long)p->max,
           (long)p->search, (long)p->ramp_ms, (long)p->noise, (long)p->lost, (long)p->lost_ms,
           (long)p->obs.enable, (long)p->obs.stop_cm, (long)p->obs.turn_cm,
           running ? "DANG CHAY" : "DUNG");
}
