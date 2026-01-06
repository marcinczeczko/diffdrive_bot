#ifndef MOTOR_H_
#define MOTOR_H_

#include "config/Config.h"
#include "controller/PidTypes.h"
#include "pwm.h"
#include "tele/RtpTelemetry.h"

#include <Arduino.h>
#include <math.h>

#define TICKS_PER_CM ((long)TICKS_PER_REV / (long)(PI * WHEEL_DIAMETER_CM))

using MotorSide = uint8_t;

constexpr MotorSide MOTOR_SIDE_LEFT = 0x00;
constexpr MotorSide MOTOR_SIDE_RIGHT = 0x01;
constexpr MotorSide MOTOR_SIDE_BOTH = 0x02;

class MotorDriver
{
  private:
    PwmOut pwm;
    int dirPin;
    MotorSide motorSide;
    uint32_t ticksPerRev;

    float kP;
    float kI;
    float kFF;
    float rpsAlpha = 0.0F;

    PidSideState state{};

    Rtp::RtpTelemetry* telemetry;

  public:
    MotorDriver(int pwmPin, int dirPin, uint32_t ticksPerRev, MotorSide side, float kp, float ki,
                float kff, Rtp::RtpTelemetry* tele)
        : pwm(pwmPin), ticksPerRev(ticksPerRev), motorSide(side), dirPin(dirPin), kP(kp), kI(ki),
          kFF(kff), telemetry(tele), rpsAlpha(RPS_ALPHA)
    {
        resetState();
    }

    void copyState(PidSideState& out) const
    {
        out = state;
    }

    void resetState()
    {
        state.setpoint = 0.0f;
        state.measurement = 0.0f;
        state.measurement_raw = 0.0f;
        state.error = 0.0f;
        state.integral = 0.0f;
        state.p_term = 0.0f;
        state.i_term = 0.0f;
        state.raw_output = 0.0f;
        state.output = 0.0f;
    }

    void begin()
    {
        resetState();
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
    void update(float targetRps, int32_t deltaTicks)
    {
        // Fixed control loop time (derived from RTOS scheduling)
        const float dt = SPEED_PID_SAMPLE_TIME_MS / 1000.0f;

        // =================================================
        // STANDSTILL SANITATION
        // =================================================
        if (deltaTicks == 0 && fabsf(targetRps) < 0.001f)
        {
            state.measurement = 0.0f;
            state.measurement_raw = 0.0f;
            state.error = 0.0f;

            state.p_term = 0.0f;
            state.i_term = 0.0f;
            state.raw_output = 0.0f;
            state.output = 0.0f;

            // integrator zerowany TYLKO w pełnym postoju
            state.integral = 0.0f;

            digitalWrite(dirPin, LOW);
            pwm.pulse_perc(0.0f);

            return;
        }

        // =================================================
        // 1. Velocity estimation from encoder ticks
        // =================================================
        // Convert encoder delta to raw rotational speed [RPS]
        // Note: measurement_raw is noisy due to quantization
        state.setpoint = targetRps;
        state.measurement_raw =
            ((float)deltaTicks * 1000.0f) / (ticksPerRev * SPEED_PID_SAMPLE_TIME_MS);

        // Low-pass filter to reduce quantization noise and jitter
        // This represents the "estimated" motor velocity
        state.measurement = alphaFilter(state.measurement, state.measurement_raw, rpsAlpha);

        // =================================================
        // 2. Control error (velocity domain)
        // =================================================
        // Positive error means motor is too slow
        state.error = targetRps - state.measurement;

        // =================================================
        // 3. Proportional and Integral terms
        // =================================================
        // P-term: immediate response to velocity error
        state.p_term = kP * state.error;

        // I-term: accumulated error compensating static friction,
        // load changes and model inaccuracies
        state.i_term = kI * state.integral;

        // =================================================
        // 4. Feed-forward term
        // =================================================
        // Open-loop approximation of required PWM for a given RPS
        // Reduces the burden on the PI controller
        float ffEffort = targetRps * kFF;

        // =================================================
        // 5. Controller output summation
        // =================================================
        // Raw controller output before actuator limits
        state.raw_output = ffEffort + state.p_term + state.i_term;

        // =================================================
        // 6. Actuator saturation
        // =================================================
        // Clamp to physical PWM limits
        state.output = constrain(state.raw_output, -PWM_MAX_DUTY, PWM_MAX_DUTY);

        // =================================================
        // 7. Integrator update & anti-windup
        // =================================================
        // Reset integrator only when both:
        //  - target speed is zero
        //  - measured speed is zero
        // This prevents stored torque and motor "twitching" at standstill
        if (fabsf(targetRps) < 0.01f && fabsf(state.measurement) < 0.01f)
        {
            state.integral = 0.0f;
        }
        // Integrate error only when actuator is NOT saturated
        // Prevents integrator windup under hard limits
        else if (state.raw_output == state.output)
        {
            state.integral += state.error * dt;
        }

        // Hard safety clamp for numerical robustness
        const float I_LIMIT = PWM_MAX_DUTY / max(kI, 0.001f);
        state.integral = constrain(state.integral, -I_LIMIT, I_LIMIT);

        // =================================================
        // 8. Actuation
        // =================================================
        // Direction is derived from the sign of the output
        state.pwm_cmd = mapToPwm(state.output);

        digitalWrite(dirPin, state.pwm_cmd >= 0.0f ? LOW : HIGH);

        // Apply deadband as the very last stage (actuator domain)
        // This compensates for motor static friction without
        // interfering with PI dynamics
        pwm.pulse_perc(fabsf(state.pwm_cmd));
    }

    inline float alphaFilter(float prev, float current, float alpha)
    {
        return alpha * current + (1.0f - alpha) * prev;
    }

    inline float mapToPwm(float u)
    {
        if (fabsf(u) < 1e-6f)
            return 0.0f;

        float sign = copysignf(1.0f, u);
        float abs_u = fabsf(u);

        // normalize to [0..1]
        float norm = abs_u / 100.0F;

        // map to [PWM_MIN .. PWM_MAX]
        float pwm = PWM_MIN_DUTY + norm * (PWM_MAX_DUTY - PWM_MIN_DUTY);

        return sign * pwm;
    }

    void setPid(float p, float i, float ff, float alpha)
    {
        kP = p;
        kI = i;
        kFF = ff;
        rpsAlpha = alpha;
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