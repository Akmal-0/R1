# Bot (R1)

Robot 4-Wheel Omniwheel berbasis PJRC Teensy 4.1.

## Fitur & Kontrol Gamepad/Joystick

### 1. Kontrol Manual Base
- **Stik Analog Kiri**: Translasi planar ($V_x, V_y$).
- **Stik Analog Kanan**: Rotasi ($\omega_z$).
- **D-Pad Up**: Toggle mode kecepatan cepat/lambat (*Speed Mode*).
- **Profil S-Curve (Jerk-Limited)**: Menghaluskan pergerakan manual agar responsif tanpa slip roda.

### 2. Fitur Semi-Auto (Maju & Mundur 5 Meter)
Menggunakan **2 Roda Encoder Eksternal (Tracking Wheel X & Y)** bebas slip yang ditekan ke lantai untuk mengukur posisi fisik secara nyata:
- **Tombol Y**: Memicu robot meluncur maju lurus sejauh **5.0 meter** (`SEMI_AUTO_DISTANCE`).
- **Tombol A**: Memicu robot mundur lurus kembali ke titik **0.0 meter**.
- **Tombol B**: Membatalkan (*cancel/stop*) gerak semi-auto seketika.
- **Manual Override (Safety)**: Jika driver menyentuh/menggerakkan stik analog joystick saat semi-auto sedang berjalan, mode semi-auto otomatis dibatalkan dan kontrol kembali 100% ke driver manual.

### 3. Sistem Stabilisasi Semi-Auto
- **Profil S-Curve Y**: Akselerasi lembut ke kecepatan puncak (1.5 m/s) dan deselerasi mulus tepat di target jarak dengan toleransi 2 cm tanpa overshoot.
- **Koreksi Sumbu X (Anti-Hanyut)**: Mengunci posisi lateral $X = 0$ menggunakan Encoder Eksternal X agar robot tidak geser ke kiri/kanan.
- **Koreksi Rotasi Wz (Anti-Miring)**: Mengunci sudut hadap robot menggunakan IMU BNO055 agar tidak berputar/miring.

## Konfigurasi Pin Hardware

- **Motor Base (BTS7960)**:
  - Motor 1: PWM (Pin 23, 22), Encoder (Pin 20, 21)
  - Motor 2: PWM (Pin 15, 14), Encoder (Pin 18, 19)
  - Motor 3: PWM (Pin 37, 36), Encoder (Pin 41, 40)
  - Motor 4: PWM (Pin 29, 33), Encoder (Pin 35, 34)
- **Sensor IMU (BNO055)**: I2C Wire1 (SDA1 = Pin 16, SCL1 = Pin 17)
- **Encoder Eksternal (Tracking Wheel)**:
  - Sumbu X: Pin 2 (Phase A), Pin 3 (Phase B)
  - Sumbu Y: Pin 4 (Phase A), Pin 5 (Phase B)

## Fitur Keselamatan (Safety)
- Output motor disetel LOW saat inisialisasi awal.
- **Watchdog Timeout**: Jika koneksi joystick putus atau tidak ada data > 300 ms, `Emergencystop()` aktif seketika (seluruh motor mati dan filter direset).
