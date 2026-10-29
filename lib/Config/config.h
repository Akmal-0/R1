#ifndef ROBOT_1_CONFIG_H
#define ROBOT_1_CONFIG_H

#include <Arduino.h>

#define ROBOT_1 OMNI // Omniwheel robot (4 roda)

// PID Roda Base (Kecepatan)
#define K_P 90.0f
#define K_I 366.101694915f
#define K_D 0.0f

// Spesifikasi Fisik Robot & PWM
#define MOTOR_MAX_RPS 8.4f              // Motor max RPS
#define MAX_RPS_RATIO 0.85f             // Max RPS ratio allowed
#define MOTOR_OPERATING_VOLTAGE 24      // Operating voltage
#define MOTOR_POWER_MAX_VOLTAGE 24      // Max voltage of power source
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // Measured power voltage

#define COUNTS_PER_REV1 3840 // Wheel 1 encoder ticks per rev
#define COUNTS_PER_REV2 3840 // Wheel 2 encoder ticks per rev
#define COUNTS_PER_REV3 3840 // Wheel 3 encoder ticks per rev
#define COUNTS_PER_REV4 3840 // Wheel 4 encoder ticks per rev

#define WHEEL_DIAMETER 0.0985f // Wheel diameter in meters
#define ROBOT_DIAMETER 0.80f   // Robot diameter in meters (distance between opposite wheels)

#define PWM_BITS 8
#define PWM_FREQUENCY 25000 // PWM frequency in Hz
#define PWM_MAX (pow(2, PWM_BITS) - 1)
#define PWM_MIN (-PWM_MAX)

// Pin Encoder Roda Base
#define MOTOR1_ENCODER_A 20
#define MOTOR1_ENCODER_B 21

#define MOTOR2_ENCODER_A 18
#define MOTOR2_ENCODER_B 19

#define MOTOR3_ENCODER_A 41
#define MOTOR3_ENCODER_B 40

#define MOTOR4_ENCODER_A 35
#define MOTOR4_ENCODER_B 34

// Pin Motor Base (BTS7960)
#define MOTOR1_IN_A 23
#define MOTOR1_IN_B 22

#define MOTOR2_IN_A 15
#define MOTOR2_IN_B 14

#define MOTOR3_IN_A 37
#define MOTOR3_IN_B 36

#define MOTOR4_IN_A 29
#define MOTOR4_IN_B 33

// Array Encoder & Motor Base
const int enca[4] = {
    MOTOR1_ENCODER_A,
    MOTOR2_ENCODER_A,
    MOTOR3_ENCODER_A,
    MOTOR4_ENCODER_A};

const int encb[4] = {
    MOTOR1_ENCODER_B,
    MOTOR2_ENCODER_B,
    MOTOR3_ENCODER_B,
    MOTOR4_ENCODER_B};

const int cw[4] = {
    MOTOR1_IN_A,
    MOTOR2_IN_A,
    MOTOR3_IN_A,
    MOTOR4_IN_A};

const int ccw[4] = {
    MOTOR1_IN_B,
    MOTOR2_IN_B,
    MOTOR3_IN_B,
    MOTOR4_IN_B};

// Parameter Kendali Base Manual & Kecepatan
#define SPEED_FAST_TOP 2.3f
#define SPEED_FAST_ROT 0.7f
#define SPEED_SLOW_TOP 0.35f
#define SPEED_SLOW_ROT 0.25f

// Parameter S-Curve Pelembut Gerak Base Manual (Jerk-Limited Profile - Mode Responsif)
#define SCURVE_MAX_ACCEL_LINEAR 4.5f  // Akselerasi maksimum translasi X & Y (m/s^2)
#define SCURVE_MAX_JERK_LINEAR 35.0f  // Jerk maksimum translasi X & Y (m/s^3) (reaksi stik lincah)
#define SCURVE_MAX_ACCEL_ANGULAR 5.0f // Akselerasi maksimum rotasi Wz (rad/s^2)
#define SCURVE_MAX_JERK_ANGULAR 40.0f // Jerk maksimum rotasi Wz (rad/s^3)

// Parameter Joystick & Watchdog
#define JOYSTICK_TIMEOUT_MS 300UL
#define JOYSTICK_DEADZONE 20
#define JOYSTICK_SPEED_DEBOUNCE_MS 150UL
#define REQUIRED_JOYSTICK_AXES ((1ULL << 0) | (1ULL << 1) | (1ULL << 2) | (1ULL << 5))

// Pin Encoder Eksternal (Tracking Wheel X & Y)
#define EXT_ENC_X_A 2
#define EXT_ENC_X_B 3
#define EXT_ENC_Y_A 4
#define EXT_ENC_Y_B 5

// Inversi Arah Encoder Eksternal jika perhitungan jarak terbalik
#define EXT_ENC_X_INVERT false
#define EXT_ENC_Y_INVERT false

// Spesifikasi Fisik Roda Pasif Eksternal
#define EXT_ENC_WHEEL_DIAMETER 0.058f // Diameter roda pasif (58 mm)
#define EXT_ENC_PPR 2400.0f           // Total ticks per rotasi (misal encoder 600 CPR x 4 = 2400 ticks)
#define EXT_ENC_METERS_PER_TICK ((PI * EXT_ENC_WHEEL_DIAMETER) / EXT_ENC_PPR)

// Parameter Semi-Auto (Maju & Mundur Lurus)
#define SEMI_AUTO_DISTANCE 5.0f       // Jarak tempuh maju/mundur otomatis (meter)
#define SEMI_AUTO_TOLERANCE_M 0.02f   // Toleransi titik berhenti (2 cm)
#define SEMI_AUTO_MAX_SPEED 1.5f      // Kecepatan puncak semi-auto (m/s)
#define SEMI_AUTO_ACCEL 3.0f          // Akselerasi maksimum S-Curve semi-auto (m/s^2)
#define SEMI_AUTO_JERK 20.0f          // Jerk maksimum S-Curve semi-auto (m/s^3)

#define SEMIAUTO_KP_X 1.5f            // Penguatan koreksi drift lateral sumbu X
#define SEMIAUTO_KP_HEADING 2.0f      // Penguatan koreksi sudut hadap IMU Wz

#endif // ROBOT_1_CONFIG_H
