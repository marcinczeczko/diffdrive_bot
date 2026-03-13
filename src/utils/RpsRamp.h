#ifndef RPS_RAMP_H_
#define RPS_RAMP_H_

#include "config/Config.h"

#include <Arduino.h>
#include <math.h>

class RpsRamp
{
  public:
    void reset()
    {
        current = 0.0F;
        velocity = 0.0F;
        accel = 0.0F;
    }

    float updateNone(float target, float dt) const
    {
        return target;
    }

    float updateLinear(float target, float dt)
    {
        float maxStep = RPS_RAMP_MAX_ACCEL * dt;
        float diff = target - current;

        if (fabs(diff) <= maxStep)
        {
            current = target;
        }
        else
        {
            current += (diff > 0 ? maxStep : -maxStep);
        }

        return current;
    }

#ifdef RPS_RAMP_SCURVE
    float update(float targetVelocity, float dt)
    {
        // 1. DEAD ZONE (Snap-to-Zero)
        // If the target is 0 and we are already very close, stop immediately
        const float STOP_THRESHOLD = 0.1f; // [rps] ok. 1.2 RPM
        if (targetVelocity == 0.0f && fabs(velocity) < STOP_THRESHOLD)
        {
            velocity = 0.0f;
            accel = 0.0f;
            return 0.0f;
        }

        // 2. COMPUTE DESIRED ACCELERATION
        float desiredAccel = (targetVelocity - velocity) / dt;

        // 3. DYNAMIC JERK
        // If we are braking to zero, we can allow a slightly higher jerk
        // to avoid "floating" around zero.
        float currentMaxJerk = RPS_RAMP_MAX_JERK;
        if (targetVelocity == 0.0f)
        {
            currentMaxJerk *= 1.5f; // More aggressive ramp-down on stop
        }

        // 4. LIMIT ACCELERATION CHANGE (JERK)
        float accelDiff = desiredAccel - accel;
        float maxAccelStep = currentMaxJerk * dt;

        if (fabs(accelDiff) > maxAccelStep)
        {
            accel += (accelDiff > 0 ? maxAccelStep : -maxAccelStep);
        }
        else
        {
            accel = desiredAccel;
        }

        // 5. LIMIT MAX ACCELERATION
        if (accel > RPS_RAMP_MAX_ACCEL)
            accel = RPS_RAMP_MAX_ACCEL;
        if (accel < -RPS_RAMP_MAX_ACCEL)
            accel = -RPS_RAMP_MAX_ACCEL;

        // 6. INTEGRATION TO VELOCITY
        float nextVelocity = velocity + (accel * dt);

        // 7. GUARD AGAINST SIGN CHANGE (oscillations around zero)
        // If we were about to stop and the velocity sign is about to flip
        if (targetVelocity == 0.0f && (nextVelocity * velocity < 0))
        {
            velocity = 0.0f;
            accel = 0.0f;
        }
        else
        {
            velocity = nextVelocity;
        }

        return velocity;
    }
#endif

  private:
    float current = 0.0F;
    float velocity = 0.0F; // used only by S-curve
    float accel = 0.0F;    // used only by S-curve
};

#endif
