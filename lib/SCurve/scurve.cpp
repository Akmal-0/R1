#include "scurve.h"
#include <math.h>

SCurveFilter::SCurveFilter()
    : max_accel_(2.0f), max_jerk_(10.0f), current_vel_(0.0f), current_acc_(0.0f)
{
}

SCurveFilter::SCurveFilter(float max_accel, float max_jerk)
    : max_accel_(fabsf(max_accel)), max_jerk_(fabsf(max_jerk)), current_vel_(0.0f), current_acc_(0.0f)
{
}

void SCurveFilter::setLimits(float max_accel, float max_jerk)
{
    max_accel_ = fabsf(max_accel);
    max_jerk_ = fabsf(max_jerk);
}

void SCurveFilter::reset(float initial_vel)
{
    current_vel_ = initial_vel;
    current_acc_ = 0.0f;
}

float SCurveFilter::update(float target_vel, float dt)
{
    if (dt <= 0.0f)
    {
        return current_vel_;
    }

    // Batasi dt ke rentang wajar (maksimal 50 ms) untuk kestabilan numerik
    dt = fminf(dt, 0.05f);

    float vel_err = target_vel - current_vel_;

    // Ambang batas toleransi (deadband) saat sudah sangat mendekati target
    if (fabsf(vel_err) < 1e-4f && fabsf(current_acc_) < 1e-3f)
    {
        current_vel_ = target_vel;
        current_acc_ = 0.0f;
        return current_vel_;
    }

    // Batas akselerasi aman untuk mencegah overshoot saat mendekati target:
    // a_bound = sqrt(2 * j_max * |vel_err|)
    float a_bound = sqrtf(2.0f * max_jerk_ * fabsf(vel_err));
    float target_accel = 0.0f;

    if (vel_err > 0.0f)
    {
        target_accel = fminf(max_accel_, a_bound);
    }
    else
    {
        target_accel = -fminf(max_accel_, a_bound);
    }

    // Batasi laju perubahan akselerasi sesuai batasan Jerk maksimum: |da| <= max_jerk * dt
    float max_delta_accel = max_jerk_ * dt;
    float delta_accel = target_accel - current_acc_;
    delta_accel = fmaxf(-max_delta_accel, fminf(delta_accel, max_delta_accel));
    current_acc_ += delta_accel;

    // Integrasi percepatan menjadi kecepatan (menghasilkan profil lengkung S)
    current_vel_ += current_acc_ * dt;

    return current_vel_;
}
