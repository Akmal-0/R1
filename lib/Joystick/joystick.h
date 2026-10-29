#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <USBHost_t36.h>
#include <config.h>

struct ButtonState
{
    int A = 0;
    int B = 0;
    int X = 0;
    int Y = 0;
    int RT = 0;
    int LT = 0;
    int LB = 0;
    int RB = 0;
    int select = 0;
    int start = 0;
    int home = 0;
    int up = 0;
    int down = 0;
    int left = 0;
    int right = 0;
};

struct AxisState
{
    double axis0_x = 0;
    double axis0_y = 0;
    double axis1_x = 0;
    double axis1_y = 0;
    int but_red = 0;
    int but_blue = 0;
    int but_black = 0;
    int but_green = 0;
};

// Typedef alias untuk kompatibilitas kode lama
typedef ButtonState but;
typedef AxisState joy;

class JoystickManager
{
public:
    ButtonState button;
    AxisState axis;

    JoystickManager();

    // Inisialisasi stack USB Host Teensy
    void begin();

    // Memproses background task USB Host (harus dipanggil setiap loop)
    void task();

    // Memeriksa apakah perangkat joystick terhubung secara fisik
    bool isConnected();

    // Status kesiapan joystick (semua sumbu wajib terdeteksi dan stik dalam keadaan netral)
    bool isReady() const { return joystickReady_; }
    void setReady(bool ready) { joystickReady_ = ready; }

    // Memeriksa apakah terjadi timeout transmisi joystick (> 300 ms)
    bool isTimeout() const;

    // Memeriksa apakah ada input pergerakan manual pada stik
    bool isManualInput() const;

    // Status mode kecepatan (Fast / Slow)
    bool isSpeedFast() const { return speedModeFast_; }

    // Pembacaan status tombol Y dari raw buttons
    bool isButtonYPressed();

    // Mengambil timestamp laporan joystick terakhir
    unsigned long getLastReportTime() const { return lastJoystickTime_; }
    void setLastReportTime(unsigned long t) { lastJoystickTime_ = t; }

    // Membaca data terbaru dari joystick. Mengembalikan true jika ada data baru.
    bool update();

    // Memvalidasi apakah joystick siap diaktifkan saat pertama kali terhubung
    bool qualifyReady();

    // Menyimpan dan memulihkan bitmask sumbu yang telah terdeteksi
    uint64_t getAxesSeen() const { return joystickAxesSeen_; }
    void restoreAxesSeen(uint64_t seen) { joystickAxesSeen_ = seen; }

    // Reset seluruh status tombol, axis, dan status kesiapan (digunakan saat Emergencystop)
    void reset();

    // Membersihkan buffer joystick saat terputus
    void handleDisconnect();

    // Fungsi utilitas deadzone
    static int applyDeadzone(int value, int deadzone = JOYSTICK_DEADZONE);

private:
    USBHost usb_joy_;
    USBHub joy_hub_;
    USBHIDParser hid_joy_;
    JoystickController joy_control_[4];

    unsigned long lastJoystickTime_ = 0;
    bool joystickReady_ = false;
    uint64_t joystickAxesSeen_ = 0;

    bool speedModeFast_ = false;
    bool lastButtonUp_ = false;
    unsigned long lastToggleTime_ = 0;

    void parseButtons(uint32_t buttons, int hatAxis);
    void handleSpeedToggle();
};

#endif // JOYSTICK_H
