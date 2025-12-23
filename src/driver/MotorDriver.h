#ifndef MOTOR_H_
#define MOTOR_H_

#include "config/Config.h"
#include "pwm.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <math.h>

#define TICKS_PER_CM ((long)TICKS_PER_REV / (long)(PI * WHEEL_DIAMETER_CM))

enum MotorSide : uint8_t
{
    LEFT = 0,
    RIGHT = 1
};

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
    float output;
};
#pragma pack(pop)

extern Rtp::RtpTelemetry telemetry;

class MotorDriver
{
  private:
    float dtMs; // Pid Period in ms, e.g 50ms
    long lastTicks = 0;

    PwmOut pwm;
    int dirPin;
    MotorSide motorSide;

    float kP;
    float kI;
    float kFF;

  public:
    MotorDriver(int pwmPin, int dirPin, MotorSide side, float periodMs, float kp, float ki,
                float kff)
        : dtMs(periodMs), pwm(pwmPin), motorSide(side), dirPin(dirPin), kP(kp), kI(ki), kFF(kff)
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
        int32_t deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;

        // actual RPS is no in the same units as target Rps
        float actualRps =
            ((float)deltaTicks * 1000.0f) / (TICKS_PER_REV * SPEED_PID_SAMPLE_TIME_MS);

        // the PID Math (just P)
        float pidError = targetRps - actualRps;

        static float integral = 0.0f;
        if (fabsf(targetRps) < 0.01f)
        {
            integral = 0.0f;
        }
        else
        {
            integral += (pidError * SPEED_PID_SAMPLE_TIME_MS / 1000.0);
            // clamp integral
            if (integral * kI > PWM_MAX_DUTY)
                integral = PWM_MAX_DUTY / kI;
            if (integral * kI < -PWM_MAX_DUTY)
                integral = -PWM_MAX_DUTY / kI;
        }

        float ffEffort = targetRps * kFF;
        float pidEffort = (kP * pidError) + (kI * integral);
        float pidOutput = ffEffort + pidEffort;

        digitalWrite(dirPin, pidOutput >= 0 ? LOW : HIGH);

        pwm.pulse_perc(applyDeadband(fabs(pidOutput), targetRps));

        if (loopCounter % 10 == 0)
        {
            PidPayload payload{loopCounter, motorSide,       targetRps,       actualRps,
                               pidError,    (kP * pidError), (kI * pidError), pidOutput};
            telemetry.publish(Rtp::RtpType::PID, &payload, sizeof(payload));

            // LOG_PID(loopCounter, side, targetRps, actualRps, pidError, (kP * pidError),
            //         (kI * pidError), pidOutput, pidOutput);
        }
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