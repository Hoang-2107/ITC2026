#include "line_sensor.h"
#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#define CALIB_MAGIC 0x43414C01u

static const adc_channel_t SENSOR_CHANNELS[NUM_SENSORS] = SENSOR_CHANNEL_LIST;

static adc_oneshot_unit_handle_t s_adc;   // bộ ADC1
static line_calib_t s_cal;                 // min/max của từng cảm biến
static bool s_calibrated = false;

/*
 * Khởi tạo ADC1 cho 8 kênh cảm biến.
 * Suy giảm 12 dB để đo được toàn dải 0..~3.1 V, độ phân giải 12 bit (0..4095).
 */
void line_sensor_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,          // đo được 0..~3.1 V
        .bitwidth = ADC_BITWIDTH_12,
    };
    for (int i = 0; i < NUM_SENSORS; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, SENSOR_CHANNELS[i], &ch_cfg));
    }
}

/*
 * Đọc giá trị thô của 8 cảm biến.
 * Mỗi kênh đọc SENSOR_SAMPLES lần rồi lấy trung bình để giảm nhiễu.
 * Kết quả 0..4095: càng tối (trên vạch đen) càng cao.
 */
void line_sensor_read_raw(int raw[NUM_SENSORS])
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int sum = 0, v = 0;
        for (int s = 0; s < SENSOR_SAMPLES; s++) {
            adc_oneshot_read(s_adc, SENSOR_CHANNELS[i], &v);
            sum += v;
        }
        raw[i] = sum / SENSOR_SAMPLES;
    }
}

/*
 * Đọc cảm biến và chuẩn hóa về cùng thang 0..1000:
 *   norm = (raw - min) / (max - min) * 1000
 * 0 = nền, 1000 = chắc chắn trên line. Nhờ vậy 8 cảm biến lệch nhau vẫn so sánh được.
 * raw[] trả về luôn giá trị thô để in ra Monitor.
 */
void line_sensor_read_norm(int raw[NUM_SENSORS], int norm[NUM_SENSORS])
{
    line_sensor_read_raw(raw);
    int lowest = 1000;
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = s_cal.max[i] - s_cal.min[i];
        int v = 0;
        if (range > 50) {
            v = (raw[i] - s_cal.min[i]) * 1000 / range;
            if (v < 0) v = 0;
            if (v > 1000) v = 1000;
        }
        if (!LINE_IS_DARK) v = 1000 - v;
        norm[i] = v;
        if (v < lowest) lowest = v;
    }
#if AMBIENT_REJECT
    // Ánh sáng môi trường làm cả dãy cùng tăng/giảm -> trừ mức chung (cảm biến thấp nhất)
    if (lowest < 1000) {
        for (int i = 0; i < NUM_SENSORS; i++) {
            norm[i] = (norm[i] - lowest) * 1000 / (1000 - lowest);
        }
    }
#endif
}

/*
 * Tính vị trí line bằng trung bình có trọng số:
 *   pos = Σ(norm[i] * i * 1000) / Σ(norm[i])
 * Ví dụ line nằm giữa LDR4 và LDR5 (i = 3 và 4) -> pos ≈ 3500.
 * Cảm biến có norm < noise bị bỏ qua để nhiễu nền không kéo lệch vị trí.
 * Trả về false (mất line) nếu tổng norm < lost_sum.
 */
bool line_sensor_position(const int norm[NUM_SENSORS], int noise, int lost_sum, int *pos)
{
    long weighted = 0;
    long sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        int v = norm[i];
        if (v < noise) continue;
        weighted += (long)v * i * 1000;
        sum += v;
    }
    if (sum < lost_sum || sum == 0) return false;
    *pos = (int)(weighted / sum);
    return true;
}

/* ======================= Hiệu chuẩn ======================= */

/* Bắt đầu hiệu chuẩn: đặt min = 4095, max = 0 để mẫu đầu tiên ghi đè được */
void line_sensor_calib_begin(void)
{
    s_cal.magic = CALIB_MAGIC;
    for (int i = 0; i < NUM_SENSORS; i++) {
        s_cal.min[i] = 4095;
        s_cal.max[i] = 0;
    }
}

/*
 * Lấy 1 mẫu hiệu chuẩn: cập nhật min (sáng nhất) và max (tối nhất) của từng cảm biến.
 * Gọi liên tục trong lúc robot xoay qua line.
 * Trả về false nếu mẫu bị bỏ (cả 8 cảm biến đều rất tối = robot đang bị nhấc lên).
 */
bool line_sensor_calib_sample(void)
{
    int raw[NUM_SENSORS];
    line_sensor_read_raw(raw);

    // Cả dãy cùng rất tối -> robot đang bị nhấc lên, bỏ mẫu này
    int dark = 0;
    for (int i = 0; i < NUM_SENSORS; i++) if (raw[i] > CALIB_LIFT_RAW) dark++;
    if (dark == NUM_SENSORS) return false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] < s_cal.min[i]) s_cal.min[i] = raw[i];
        if (raw[i] > s_cal.max[i]) s_cal.max[i] = raw[i];
    }
    return true;
}

/* Kết thúc hiệu chuẩn, từ giờ line_sensor_read_norm() dùng min/max mới */
void line_sensor_calib_end(void)
{
    s_calibrated = true;
}

/* In min, max, độ chênh của 8 cảm biến; cảnh báo cảm biến có độ chênh < 300 */
void line_sensor_calib_print(void)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        int range = s_cal.max[i] - s_cal.min[i];
        printf("LDR%d: min=%4d max=%4d  chenh=%4d%s\n", i + 1, s_cal.min[i], s_cal.max[i], range,
               range < 300 ? "  <-- QUA NHO: cam bien khong quet qua line / bi lech" : "");
    }
}

/* true nếu đã hiệu chuẩn (bằng lệnh cal hoặc nạp từ flash) */
bool line_sensor_is_calibrated(void)
{
    return s_calibrated;
}

/* Lấy dữ liệu hiệu chuẩn hiện tại để lưu vào flash */
const line_calib_t *line_sensor_get_calib(void)
{
    return &s_cal;
}

/* Nạp dữ liệu hiệu chuẩn đọc từ flash; false nếu dữ liệu sai định dạng (magic không khớp) */
bool line_sensor_set_calib(const line_calib_t *c)
{
    if (c->magic != CALIB_MAGIC) return false;
    s_cal = *c;
    s_calibrated = true;
    return true;
}
