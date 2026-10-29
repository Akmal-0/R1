#ifndef SCURVE_H
#define SCURVE_H

#include <Arduino.h>

/**
 * @brief Jerk-Limited S-Curve Velocity Profile Filter
 *
 * Menghasilkan profil kurva S secara real-time dengan membatasi nilai
 * akselerasi (a_max) dan jerk (j_max = da/dt). Mencegah roda omni selip
 * dan meminimalkan guncangan mekanik pada aktuator atas.
 */
class SCurveFilter
{
public:
    SCurveFilter();
    SCurveFilter(float max_accel, float max_jerk);

    // Mengatur batas akselerasi dan jerk
    void setLimits(float max_accel, float max_jerk);

    // Menghitung kecepatan berikutnya dengan input target_vel dan selang waktu dt (detik)
    float update(float target_vel, float dt);

    // Reset kecepatan dan akselerasi (misal saat emergency stop)
    void reset(float initial_vel = 0.0f);

    float getVelocity() const { return current_vel_; }
    float getAcceleration() const { return current_acc_; }

private:
    float max_accel_;
    float max_jerk_;
    float current_vel_;
    float current_acc_;
};

#endif // SCURVE_H
