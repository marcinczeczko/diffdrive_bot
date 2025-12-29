#ifndef MOTOR_H_
#define MOTOR_H_

#include "config/Config.h"
#include "pwm.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <math.h>

#define TICKS_PER_CM ((long)TICKS_PER_REV / (long)(PI * WHEEL_DIAMETER_CM))

using MotorSide = uint8_t;

constexpr MotorSide MOTOR_SIDE_LEFT = 0x00;
constexpr MotorSide MOTOR_SIDE_RIGHT = 0x01;

#pragma pack(push, 1)
struct PidPayload
{
    uint32_t loopCntr;
    uint8_t motor;
    float setpoint;
    float measurement;
    float error;
    float pTerm;
    float iTerm;
    float rawOutput; // before saturation
    float output;    // after saturacion
};
#pragma pack(pop)

static_assert(sizeof(PidPayload) == 33, "PidPayload ABI mismatch");

class MotorDriver
{
  private:
    float dtMs; // Pid Period in ms, e.g 50ms
    long lastTicks = 0;

    PwmOut pwm;
    int dirPin;
    MotorSide motorSide;
    uint32_t ticksPerRev;

    float integral = 0.0f;
    float kP;
    float kI;
    float kFF;

    // // Filtered RPS state
    float filteredRps = 0.0F;

    // Alpha parameter (tunable)
    float rpsAlpha = 0.0F;

    Rtp::RtpTelemetry* telemetry;

  public:
    MotorDriver(int pwmPin, int dirPin, uint32_t ticksPerRev, MotorSide side, float periodMs,
                float kp, float ki, float kff, Rtp::RtpTelemetry* tele)
        : dtMs(periodMs), pwm(pwmPin), ticksPerRev(ticksPerRev), motorSide(side), dirPin(dirPin),
          kP(kp), kI(ki), kFF(kff), telemetry(tele), integral(0.0F), rpsAlpha(RPS_ALPHA)
    {
    }

    void begin()
    {
        pinMode(dirPin, OUTPUT);
        bool status = pwm.begin(PWM_FREQ, 0.0f);
        if (status)
        {
            LOG_SIDE_INFO(motorSide, "[SUCCESS] PWM started.");
        }
        else
        {
            LOG_SIDE_ERR(motorSide, "[ERROR] PWM FAILED! Check Pin/Timer conflict.");
        }
    }

    // ==================================================
    // NORMAL PID UPDATE
    // ==================================================
    void update(float targetRps, int32_t currentTicks, uint32_t loopCounter)
    {
        const float dt = SPEED_PID_SAMPLE_TIME_MS / 1000.0f;
        // -------------------------------------------------
        // 1. Encoder differentiation → measured velocity
        // -------------------------------------------------
        int32_t deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;

        // actual RPS is no in the same units as target Rps
        float actualRpsRaw =
            ((float)deltaTicks * 1000.0f) / (ticksPerRev * SPEED_PID_SAMPLE_TIME_MS);

        // Low-pass filter of calculatred Rps
        filteredRps = actualRpsRaw; // alphaFilter(filteredRps, actualRpsRaw, rpsAlpha);

        // -------------------------------------------------
        // 2. Control error (velocity domain)
        // -------------------------------------------------
        float pidError = targetRps - filteredRps;

        // -------------------------------------------------
        // 3. Feed-forward (plant inversion)
        // PWM ≈ targetRps / K
        // -------------------------------------------------
        float ffEffort = targetRps * kFF;

        // -------------------------------------------------
        // 4. PI terms (controller correction)
        // -------------------------------------------------
        float pTerm = kP * pidError;
        float iTerm = kI * integral;
        float pidOutput = ffEffort + pTerm + iTerm;

        // -------------------------------------------------
        // 5. Actuator saturation (physical limits)
        // -------------------------------------------------
        float saturatedOutput = constrain(pidOutput, -PWM_MAX_DUTY, PWM_MAX_DUTY);

        // -------------------------------------------------
        // 6. Standstill hygiene:
        // Reset integrator when target is ~0
        // Prevents "stored torque" & motor twitching
        // -------------------------------------------------
        if (fabsf(targetRps) < 0.01f)
        {
            integral = 0.0f;
        }
        // -------------------------------------------------
        // 7. Anti-windup:
        // Integrate ONLY when actuator is not saturated
        // -------------------------------------------------
        else if (fabsf(pidOutput - saturatedOutput) < 0.001f)
        {
            integral += pidError * dt;
        }

        // -------------------------------------------------
        // 8. Hard safety clamp for numerical robustness
        // (should never trigger in normal operation)
        // -------------------------------------------------
        const float I_LIMIT = PWM_MAX_DUTY / max(kI, 0.001f);
        integral = constrain(integral, -I_LIMIT, I_LIMIT);

        // -------------------------------------------------
        // 9. Direction + magnitude separation
        // -------------------------------------------------
        digitalWrite(dirPin, saturatedOutput >= 0.0f ? LOW : HIGH);

        // Deadband applied LAST, only to magnitude
        pwm.pulse_perc(applyDeadband(fabsf(saturatedOutput), targetRps));

        if (telemetry != nullptr)
        {
            PidPayload payload{};
            payload.loopCntr = loopCounter;
            payload.motor = motorSide;
            payload.setpoint = targetRps;
            payload.measurement = filteredRps;
            payload.error = pidError;
            payload.pTerm = pTerm;
            payload.iTerm = iTerm;
            payload.rawOutput = pidOutput;
            payload.output = saturatedOutput;

            telemetry->publish(Rtp::RTP_PID, payload);
        }
    }

    inline float alphaFilter(float prev, float current, float alpha)
    {
        return alpha * current + (1.0f - alpha) * prev;
    }

    auto applyDeadband(float pwm, float targetRps) -> float
    {
        // 1. Jeśli cel to 0 i sygnał z PID jest mały - odetnij wszystko.
        // To zapobiega "pukaniu" silników, gdy robot już stoi.
        if (fabs(targetRps) < 0.01f && fabs(pwm) < (PWM_MIN_DUTY * 0.5f))
        {
            return 0.0f;
        }

        // 2. Jeśli PID wyliczył absolutne zero - nie dodawaj deadbandu.
        if (fabs(pwm) < 0.1f)
            return 0.0f;

        float sign = (pwm > 0) ? 1.0f : -1.0f;

        // 3. Mapowanie - stosuj tylko jeśli wynik PID sugeruje, że pokonujemy tarcie
        return sign * (PWM_MIN_DUTY + (fabs(pwm) * (PWM_MAX_DUTY - PWM_MIN_DUTY) / PWM_MAX_DUTY));
    }

    // New method for raw control
    void driveMotor(float dutyPercentage, bool isForward)
    {
        if (dutyPercentage <= 0.1F)
        {
            pwm.pulse_perc(0.0F);
            return;
        }
        digitalWrite(dirPin, isForward ? LOW : HIGH);
        pwm.pulse_perc(constrain(dutyPercentage, 0.0f, 100.0f));
    }

    void shutdown()
    {
        pwm.end();
    }
};

#endif