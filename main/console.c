#include "console.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "settings.h"
#include "line_sensor.h"
#include "motor.h"

static void print_help(void)
{
    printf(
        "\n--- LENH ---\n"
        "run / stop        chay / dung robot\n"
        "cal               hieu chuan lai\n"
        "show              xem thong so + hieu chuan\n"
        "print 0|1         tat / bat in gia tri cam bien\n"
        "kp X  ki X  kd X  he so PID (vd: kp 0.3)\n"
        "dfilt X           loc khau D: 0 = khong loc .. 0.95 = loc manh\n"
        "base N  max N     toc do co ban / toi da (0..1023)\n"
        "rbase N           toc do co ban khi chay lui (0..1023)\n"
        "search N          toc do xoay tim line khi mat line\n"
        "ramp N            tang toc mem trong N ms khi bat dau chay\n"
        "noise N  lost N   nguong nhieu / nguong mat line (thang 0..1000)\n"
        "lostms N          mat line qua N ms thi tu dung (0 = khong dung)\n"
        "sonar 0|1         tat / bat xu ly vat can HC-SR05\n"
        "stopcm N          TH1: H1 va H2 cung < N cm -> dung\n"
        "turncm N          TH2: H3 hoac H4 < N cm -> dao chieu chay\n"
        "motor L R         chay thu dong co 1 giay (vd: motor 400 400)\n"
        "save              luu thong so vao flash\n"
        "reset             xoa thong so da luu, ve mac dinh trong config.h\n\n");
}

/* Xử lý 1 dòng lệnh dạng "lenh [so1] [so2]", vd "kp 0.3", "motor 400 -400" */
static void handle_command(char *line)
{
    params_t *p = &g_params;
    char cmd[16] = {0};
    float a = 0, b = 0;
    int n = sscanf(line, "%15s %f %f", cmd, &a, &b);
    if (n < 1) return;

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) print_help();
    else if (!strcmp(cmd, "show")) {
        settings_print(app_is_running());
        if (line_sensor_is_calibrated()) line_sensor_calib_print();
    }
    else if (!strcmp(cmd, "run")) {
        if (!line_sensor_is_calibrated()) printf("Chua hieu chuan! Go \"cal\" truoc\n");
        else app_set_running(true);
    }
    else if (!strcmp(cmd, "stop")) app_set_running(false);
    else if (!strcmp(cmd, "cal")) app_calibrate();
    else if (!strcmp(cmd, "save")) settings_save();
    else if (!strcmp(cmd, "reset")) settings_erase();
    else if (!strcmp(cmd, "print") && n >= 2) app_set_print(a != 0);
    else if (!strcmp(cmd, "motor") && n >= 3) {
        app_set_running(false);
        printf("Dong co trai=%d phai=%d trong 1 giay\n", (int)a, (int)b);
        motor_drive((int)a, (int)b);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_stop();
    }
    else if (n >= 2 && !strcmp(cmd, "kp")) p->pid.kp = a;
    else if (n >= 2 && !strcmp(cmd, "ki")) p->pid.ki = a;
    else if (n >= 2 && !strcmp(cmd, "kd")) p->pid.kd = a;
    else if (n >= 2 && !strcmp(cmd, "dfilt")) p->pid.dfilt = a < 0 ? 0 : (a > 0.95f ? 0.95f : a);
    else if (n >= 2 && !strcmp(cmd, "base")) p->base = (int)a;
    else if (n >= 2 && !strcmp(cmd, "rbase")) p->rbase = (int)a;
    else if (n >= 2 && !strcmp(cmd, "max")) p->max = (int)a;
    else if (n >= 2 && !strcmp(cmd, "search")) p->search = (int)a;
    else if (n >= 2 && !strcmp(cmd, "ramp")) p->ramp_ms = (int)a;
    else if (n >= 2 && !strcmp(cmd, "noise")) p->noise = (int)a;
    else if (n >= 2 && !strcmp(cmd, "lost")) p->lost = (int)a;
    else if (n >= 2 && !strcmp(cmd, "lostms")) p->lost_ms = (int)a;
    else if (n >= 2 && !strcmp(cmd, "sonar")) p->obs.enable = a != 0;
    else if (n >= 2 && !strcmp(cmd, "stopcm")) p->obs.stop_cm = (int)a;
    else if (n >= 2 && !strcmp(cmd, "turncm")) p->obs.turn_cm = (int)a;
    else { printf("Khong hieu lenh \"%s\". Go help\n", line); return; }

    // Đổi thông số xong thì in lại để thấy giá trị mới
    if (n >= 2 && strcmp(cmd, "print") && strcmp(cmd, "motor")) settings_print(app_is_running());
}

void console_poll(void)
{
    static char line[48];
    static int len = 0;
    uint8_t buf[32];
    int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = 0;
                handle_command(line);
                len = 0;
            }
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

void console_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_use_driver();
    setvbuf(stdout, NULL, _IONBF, 0);       // in log ra ngay, không chờ đầy bộ đệm
}
