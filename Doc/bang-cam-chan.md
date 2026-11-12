# Bảng cắm chân – Robot AMHS (ESP32-S3-DevKitC-1)

Nguồn chân: `main/config.h`. Nhìn board từ trên, 2 cổng USB ở dưới.
Nạp code và xem Monitor qua cổng **USB** (bên phải), không phải cổng UART.

## 1. Cảm biến dò line (header 10 chân) – ĐANG DÙNG

| Header 10P | Tín hiệu | ESP32-S3 | Vị trí trên board |
|---|---|---|---|
| Chân 1 | 3V3 | **3V3** | Cột trái, trên cùng |
| Chân 2 | GND | **GND** | Cột phải, trên cùng |
| Chân 3 | LDR1 (trái) | GPIO1 | Cột phải, chân thứ 4 |
| Chân 4 | LDR2 | GPIO2 | Cột phải, chân thứ 5 |
| Chân 5 | LDR3 | GPIO3 | Cột trái, ngay dưới GPIO8 |
| Chân 6 | LDR4 | GPIO4 | Cột trái, chân thứ 4 |
| Chân 7 | LDR5 | GPIO5 | Cột trái, chân thứ 5 |
| Chân 8 | LDR6 | GPIO6 | Cột trái, chân thứ 6 |
| Chân 9 | LDR7 | GPIO7 | Cột trái, chân thứ 7 |
| Chân 10 | LDR8 (phải) | GPIO8 | Cột trái, ngay dưới GPIO18 |

⚠ Chân 1 bắt buộc là **3V3**, không được nối 5V (hỏng chân ADC).

## 2. Đèn LED chiếu sáng (2 header 2 chân) – ĐANG DÙNG

| Header LED | Nối vào |
|---|---|
| +5 (cả 2 header) | Cực + nguồn 5V ngoài |
| GND (cả 2 header) | Cực − nguồn 5V ngoài **và GND của ESP32** |

⚠ Cực + của nguồn ngoài **không** được chạm chân nào của ESP32.

## 3. Driver động cơ L298N – ĐANG DÙNG

| L298N | ESP32-S3 | Ghi chú |
|---|---|---|
| ENA | GPIO9 | **Tháo jumper ENA** |
| IN1 | GPIO10 | |
| IN2 | GPIO11 | |
| ENB | GPIO12 | **Tháo jumper ENB** |
| IN3 | GPIO13 | |
| IN4 | GPIO14 | |
| GND | **GND** | Bắt buộc nối chung |
| 12V (VS) | — | Cực + pin động cơ (khuyên dùng 2S 7,4 V) |
| OUT1, OUT2 | — | Động cơ TRÁI |
| OUT3, OUT4 | — | Động cơ PHẢI |
| 5V (ra) | — | **Không** nối vào ESP32 khi đang cắm USB |

## 4. Nút nhấn

| Chức năng | ESP32-S3 |
|---|---|
| Hiệu chuẩn / Chạy / Dừng | GPIO0 (nút BOOT có sẵn trên board) |

## 5. Cảm biến siêu âm HC-SR05 – CHƯA DÙNG (`SONAR_ENABLE 0`)

| Cảm biến | Vị trí | TRIG | ECHO (qua cầu phân áp) |
|---|---|---|---|
| H1 | Tay đòn, nhìn xuống, phía trước | GPIO16 | GPIO17 |
| H2 | Tay đòn, nhìn xuống, phía sau | GPIO18 | GPIO21 |
| H3 | Đầu xe, nhìn về trước | GPIO38 | GPIO39 |
| H4 | Cuối xe, nhìn về sau | GPIO40 | GPIO41 |

VCC = 5V, GND chung. Cầu phân áp cho mỗi ECHO: `ECHO ─[1kΩ]─┬─► GPIO`, `[2kΩ]` từ điểm giữa xuống GND.
Lắp xong thì đổi `SONAR_ENABLE` thành 1 trong `config.h`.

## 6. Chân KHÔNG được dùng

| Chân | Lý do |
|---|---|
| GPIO19, GPIO20 | Đường USB D−/D+, cắm vào sẽ mất kết nối nạp code |
| GPIO43, GPIO44 | UART0 (TX/RX) của cổng UART |
| GPIO45, GPIO46 | Chân strapping, ảnh hưởng lúc khởi động |
| GPIO48 | LED RGB trên board |
| GPIO35, 36, 37 | Dùng cho PSRAM trên module N8R8 / N16R8 |

Chân còn trống để mở rộng: GPIO15, GPIO42, GPIO47.
