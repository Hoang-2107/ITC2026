# Sơ đồ cắm chân – 2 dãy dò line, 4 HC-SR05, driver L298N

Board: **ESP32-S3-DevKitC-1**. Nạp code và xem Monitor qua cổng **USB** (bên phải).
**Mọi GND** (ESP32, 2 board dò line, CD74HC4067, 4 HC-SR05, L298N, nguồn LED, pin động cơ) **phải nối chung**.

---

## 1. Hai dãy dò line qua IC CD74HC4067

ESP32-S3 không đủ chân ADC để cắm thẳng 16 LDR, nên cả 2 dãy đi qua **1 IC chọn kênh CD74HC4067** (16 kênh analog).
ESP32 chọn kênh bằng S0..S3, rồi đọc giá trị kênh đó ở chân SIG.

> ⚠ **Firmware hiện tại chưa hỗ trợ phần này.** Code đang đọc dãy trước cắm thẳng GPIO1–8 (xem mục 1c).
> Khi lắp CD74HC4067 theo bảng 1a, cần sửa `line_sensor.c` trước khi chạy.

### 1a. CD74HC4067 ↔ ESP32-S3

| CD74HC4067 | ESP32-S3 | Ghi chú |
|---|---|---|
| SIG | **GPIO1** (ADC1) | Tín hiệu analog của kênh đang chọn |
| S0 | GPIO4 | Chọn kênh, bit 0 |
| S1 | GPIO5 | Chọn kênh, bit 1 |
| S2 | GPIO6 | Chọn kênh, bit 2 |
| S3 | GPIO7 | Chọn kênh, bit 3 |
| EN | **GND** | Luôn bật |
| VCC | **3V3** | ⚠ Không cấp 5V (tín hiệu vào ADC phải ≤ 3,3 V) |
| GND | GND | |

### 1b. Hai board dò line ↔ CD74HC4067

Mỗi board dò line có header 10 chân: chân 1 = 3V3, chân 2 = GND, chân 3..10 = LDR1..LDR8.

| Header 10P | Dãy TRƯỚC (đầu xe) | Dãy SAU (cuối xe) |
|---|---|---|
| Chân 1 (3V3) | 3V3 của ESP32 | 3V3 của ESP32 |
| Chân 2 (GND) | GND | GND |
| Chân 3 – LDR1 | C0 | C8 |
| Chân 4 – LDR2 | C1 | C9 |
| Chân 5 – LDR3 | C2 | C10 |
| Chân 6 – LDR4 | C3 | C11 |
| Chân 7 – LDR5 | C4 | C12 |
| Chân 8 – LDR6 | C5 | C13 |
| Chân 9 – LDR7 | C6 | C14 |
| Chân 10 – LDR8 | C7 | C15 |

- Dãy trước: LDR1 nằm bên **trái** xe (nhìn từ sau ra trước).
- Dãy sau thường lắp xoay 180°, nên LDR1 của dãy sau nằm bên **phải** xe. Ghi lại hướng lắp thực tế để chỉnh trong firmware.
- ⚠ Chân 1 của header là **3V3**, không bao giờ nối 5V.

### 1c. Đèn LED chiếu sáng (cả 2 board)

| Header LED 2 chân (2 cái trên mỗi board) | Nối vào |
|---|---|
| +5 | Cực + nguồn 5V ngoài |
| GND | Cực − nguồn 5V ngoài **và GND chung** |

⚠ Cực + của nguồn 5V ngoài không được chạm vào bất kỳ chân nào của ESP32.

### 1d. Cách cắm HIỆN TẠI (chỉ 1 dãy, chưa có CD74HC4067) – khớp với firmware đang chạy

| Header 10P dãy trước | ESP32-S3 |
|---|---|
| Chân 1 (3V3) | 3V3 |
| Chân 2 (GND) | GND |
| Chân 3..10 (LDR1..LDR8) | GPIO1..GPIO8 |

---

## 2. Bốn cảm biến siêu âm HC-SR05

| Cảm biến | Vị trí, hướng | Nhiệm vụ | TRIG | ECHO |
|---|---|---|---|---|
| **H1** | Tay đòn, nhìn **xuống**, phía trước | TH1: cùng H2 thấy Load Port → dừng | GPIO16 | GPIO17 |
| **H2** | Tay đòn, nhìn **xuống**, phía sau | TH1 | GPIO18 | GPIO21 |
| **H3** | **Đầu xe**, nhìn về trước | TH2: gặp vật cản → đảo chiều | GPIO38 | GPIO39 |
| **H4** | **Cuối xe**, nhìn về sau | TH2 | GPIO40 | GPIO41 |

Mỗi cảm biến: **VCC = 5V**, **GND = GND chung**.

⚠ **ECHO ra 5V** → mỗi chân ECHO phải qua **cầu phân áp** trước khi vào ESP32:

```
ECHO (HC-SR05) ──[ 1 kΩ ]──┬──► GPIO ESP32
                           │
                        [ 2 kΩ ]
                           │
                          GND
```

Lắp xong 4 cảm biến thì đổi `SONAR_ENABLE` thành `1` trong `main/config.h`.

---

## 3. Driver động cơ L298N

| L298N | ESP32-S3 | Ghi chú |
|---|---|---|
| ENA | GPIO9 | ⚠ **Tháo jumper ENA** trên module |
| IN1 | GPIO10 | |
| IN2 | GPIO11 | |
| ENB | GPIO12 | ⚠ **Tháo jumper ENB** trên module |
| IN3 | GPIO13 | |
| IN4 | GPIO14 | |
| GND | **GND chung** | Bắt buộc |
| 12V (VS) | — | Cực + **pin động cơ riêng** (khuyên dùng 2S 7,4 V) |
| OUT1, OUT2 | — | Động cơ **TRÁI** |
| OUT3, OUT4 | — | Động cơ **PHẢI** |
| 5V (chân ra) | — | **Không** nối vào ESP32 khi ESP32 đang cắm USB |

---

## 4. Tổng hợp chân ESP32-S3

| GPIO | Dùng cho | | GPIO | Dùng cho |
|---|---|---|---|---|
| 0 | Nút BOOT (chạy / dừng / hiệu chuẩn) | | 16 | H1 TRIG |
| 1 | CD74HC4067 SIG *(hiện tại: LDR1)* | | 17 | H1 ECHO |
| 2 | *Trống (hiện tại: LDR2)* | | 18 | H2 TRIG |
| 3 | *Trống (hiện tại: LDR3)* | | 21 | H2 ECHO |
| 4 | CD74HC4067 S0 *(hiện tại: LDR4)* | | 38 | H3 TRIG |
| 5 | CD74HC4067 S1 *(hiện tại: LDR5)* | | 39 | H3 ECHO |
| 6 | CD74HC4067 S2 *(hiện tại: LDR6)* | | 40 | H4 TRIG |
| 7 | CD74HC4067 S3 *(hiện tại: LDR7)* | | 41 | H4 ECHO |
| 8 | *Trống (hiện tại: LDR8)* | | 15, 42, 47 | Trống |
| 9 | L298N ENA | | | |
| 10 | L298N IN1 | | | |
| 11 | L298N IN2 | | | |
| 12 | L298N ENB | | | |
| 13 | L298N IN3 | | | |
| 14 | L298N IN4 | | | |

**Không được dùng:** GPIO19, 20 (USB) · GPIO43, 44 (UART) · GPIO45, 46 (strapping) · GPIO48 (LED RGB) · GPIO35, 36, 37 (PSRAM).
