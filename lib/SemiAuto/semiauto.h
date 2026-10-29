#ifndef SEMIAUTO_H
#define SEMIAUTO_H

#include <Arduino.h>
#include <config.h>
#include <scurve.h>

enum SemiAutoState
{
    SEMIAUTO_IDLE = 0,
    SEMIAUTO_FORWARD,
    SEMIAUTO_BACKWARD,
    SEMIAUTO_DONE
};

class SemiAutoController
{
public:
    SemiAutoController();

    // Inisialisasi pin interrupt encoder eksternal dan filter
    void begin();

    // Reset posisi koordinat encoder eksternal ke 0,0
    void resetOdometry();

    // Memicu gerak maju lurus sejauh SEMI_AUTO_DISTANCE (5.0 m)
    void triggerForward(float currentHeadingDeg);

    // Memicu gerak mundur lurus kembali ke titik 0.0 m (atau -5.0 m jika dari 0)
    void triggerBackward(float currentHeadingDeg);

    // Membatalkan gerak semi-auto seketika
    void cancel();

    // Status apakah semi-auto sedang aktif berjalan
    bool isBusy() const;

    SemiAutoState getState() const { return state_; }

    // Dijalankan setiap siklus loop saat semi-auto aktif
    // Mengisi outVx, outVy, outWz (dalam m/s dan rad/s)
    bool update(float deltaT, float currentHeadingDeg, float &outVx, float &outVy, float &outWz);

    // Pembacaan posisi real-time dalam satuan meter
    float getX() const;
    float getY() const;

    // ISR handler static untuk interrupt encoder eksternal
    static void isrEncX();
    static void isrEncY();

private:
    SemiAutoState state_;
    float targetY_;
    float targetHeadingDeg_;

    SCurveFilter scurveY_;
    SCurveFilter scurveX_;

    static volatile long countX_;
    static volatile long countY_;
};

#endif // SEMIAUTO_H
