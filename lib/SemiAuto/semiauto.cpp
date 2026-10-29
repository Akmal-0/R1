#include "semiauto.h"

volatile long SemiAutoController::countX_ = 0;
volatile long SemiAutoController::countY_ = 0;

SemiAutoController::SemiAutoController()
    : state_(SEMIAUTO_IDLE),
      targetY_(0.0f),
      targetHeadingDeg_(0.0f),
      scurveY_(SEMI_AUTO_ACCEL, SEMI_AUTO_JERK),
      scurveX_(SEMI_AUTO_ACCEL, SEMI_AUTO_JERK)
{
}

void SemiAutoController::isrEncX()
{
    if (digitalRead(EXT_ENC_X_B))
        countX_++;
    else
        countX_--;
}

void SemiAutoController::isrEncY()
{
    if (digitalRead(EXT_ENC_Y_B))
        countY_++;
    else
        countY_--;
}

void SemiAutoController::begin()
{
    pinMode(EXT_ENC_X_A, INPUT_PULLUP);
    pinMode(EXT_ENC_X_B, INPUT_PULLUP);
    pinMode(EXT_ENC_Y_A, INPUT_PULLUP);
    pinMode(EXT_ENC_Y_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(EXT_ENC_X_A), isrEncX, RISING);
    attachInterrupt(digitalPinToInterrupt(EXT_ENC_Y_A), isrEncY, RISING);

    scurveY_.setLimits(SEMI_AUTO_ACCEL, SEMI_AUTO_JERK);
    scurveX_.setLimits(SEMI_AUTO_ACCEL, SEMI_AUTO_JERK);

    resetOdometry();
}

void SemiAutoController::resetOdometry()
{
    noInterrupts();
    countX_ = 0;
    countY_ = 0;
    interrupts();
}

float SemiAutoController::getX() const
{
    noInterrupts();
    long c = countX_;
    interrupts();
#if EXT_ENC_X_INVERT
    c = -c;
#endif
    return (float)c * EXT_ENC_METERS_PER_TICK;
}

float SemiAutoController::getY() const
{
    noInterrupts();
    long c = countY_;
    interrupts();
#if EXT_ENC_Y_INVERT
    c = -c;
#endif
    return (float)c * EXT_ENC_METERS_PER_TICK;
}

void SemiAutoController::triggerForward(float currentHeadingDeg)
{
    if (fabsf(getY() - SEMI_AUTO_DISTANCE) < SEMI_AUTO_TOLERANCE_M * 2.0f)
    {
        resetOdometry();
    }
    targetY_ = SEMI_AUTO_DISTANCE;
    targetHeadingDeg_ = currentHeadingDeg;
    scurveY_.reset(0.0f);
    scurveX_.reset(0.0f);
    state_ = SEMIAUTO_FORWARD;
}

void SemiAutoController::triggerBackward(float currentHeadingDeg)
{
    if (getY() > SEMI_AUTO_DISTANCE * 0.5f)
    {
        targetY_ = 0.0f;
    }
    else
    {
        targetY_ = -SEMI_AUTO_DISTANCE;
    }
    targetHeadingDeg_ = currentHeadingDeg;
    scurveY_.reset(0.0f);
    scurveX_.reset(0.0f);
    state_ = SEMIAUTO_BACKWARD;
}

void SemiAutoController::cancel()
{
    state_ = SEMIAUTO_IDLE;
    scurveY_.reset(0.0f);
    scurveX_.reset(0.0f);
}

bool SemiAutoController::isBusy() const
{
    return (state_ == SEMIAUTO_FORWARD || state_ == SEMIAUTO_BACKWARD);
}

bool SemiAutoController::update(float deltaT, float currentHeadingDeg, float &outVx, float &outVy, float &outWz)
{
    if (!isBusy())
    {
        outVx = 0.0f;
        outVy = 0.0f;
        outWz = 0.0f;
        return false;
    }

    float currentX = getX();
    float currentY = getY();

    float errY = targetY_ - currentY;
    float dist = fabsf(errY);

    if (dist <= SEMI_AUTO_TOLERANCE_M)
    {
        state_ = SEMIAUTO_DONE;
        scurveY_.reset(0.0f);
        scurveX_.reset(0.0f);
        outVx = 0.0f;
        outVy = 0.0f;
        outWz = 0.0f;
        return false;
    }

    float vBrake = sqrtf(2.0f * SEMI_AUTO_ACCEL * dist);
    float desiredSpeed = fminf(SEMI_AUTO_MAX_SPEED, vBrake);
    if (errY < 0.0f)
    {
        desiredSpeed = -desiredSpeed;
    }

    outVy = scurveY_.update(desiredSpeed, deltaT);

    float errX = 0.0f - currentX;
    float rawVx = SEMIAUTO_KP_X * errX;
    rawVx = constrain(rawVx, -0.6f, 0.6f);
    outVx = scurveX_.update(rawVx, deltaT);

    float errHeading = targetHeadingDeg_ - currentHeadingDeg;
    while (errHeading > 180.0f)
        errHeading -= 360.0f;
    while (errHeading < -180.0f)
        errHeading += 360.0f;

    float errHeadingRad = errHeading * (PI / 180.0f);
    float rawWz = SEMIAUTO_KP_HEADING * errHeadingRad;
    outWz = constrain(rawWz, -1.5f, 1.5f);

    return true;
}
