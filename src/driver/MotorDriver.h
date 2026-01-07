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
    float k_aw; // anti-windup back-calculation gain

    bool use_pi = true;

    // Feed forward
    float k1; // static friction (Coulomb)
    float k2; // linear velocity gain
    float k3; // high-speed nonlinearity

    float rpsAlpha = 0.0F;

    PidSideState state{};

    Rtp::RtpTelemetry* telemetry;

  public:
    MotorDriver(int pwmPin, int dirPin, uint32_t ticksPerRev, MotorSide side, float kp, float ki,
                float k1, float k2, float k3, float k_aw, Rtp::RtpTelemetry* tele)
        : pwm(pwmPin), ticksPerRev(ticksPerRev), motorSide(side), dirPin(dirPin), kP(kp), kI(ki),
          k1(k1), k2(k2), k3(k3), k_aw(k_aw), telemetry(tele), rpsAlpha(RPS_ALPHA)
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
        state.u_ff = 0.0f;
        state.u_pi = 0.0f;
        state.u_virtual = 0.0f;
        state.u_sat = 0.0f;
        state.pwm_cmd = 0.0f;
        state.aw_term = 0.0f;
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
            resetState();
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
        // 3. PI observer (ALWAYS computed)
        // =================================================
        // P-term: immediate response to velocity error
        state.p_term = kP * state.error;

        // I-term: accumulated error compensating static friction,
        // load changes and model inaccuracies
        state.i_term = kI * state.integral;

        state.u_pi = state.p_term + state.i_term;

        // =================================================
        // 4. Feed-forward model (k1, k2, k3)
        // =================================================
        float v_ref = targetRps;

        state.u_ff = 0.0f;
        if (fabsf(v_ref) > 1e-4f)
        {
            float sign = copysignf(1.0f, v_ref);
            float abs_vref = fabsf(v_ref);

            state.u_ff = k1 * sign + k2 * v_ref + k3 * v_ref * abs_vref;
        }

        // pi observer
        state.u_pi = state.p_term + state.i_term;
        state.u_virtual = state.u_ff + state.u_pi;

        // =================================================
        // 5. Anti-windup: back-calculation (FULL controller)
        // =================================================
        state.u_sat = constrain(state.u_virtual, -PWM_MAX_DUTY, PWM_MAX_DUTY);

        // Back-calculation term: pushes integrator to feasible value
        state.aw_term = k_aw * (state.u_sat - state.u_virtual);

        // Standstill reset
        if (fabsf(targetRps) < 0.01f)
        {
            state.integral = 0.0f;
        }
        else
        {
            // Integrator update ALWAYS active
            state.integral += (state.error + state.aw_term) * dt;
        }

        // safety clamp
        const float I_LIMIT = PWM_MAX_DUTY / max(kI, 0.001f);
        state.integral = constrain(state.integral, -I_LIMIT, I_LIMIT);

        // =================================================
        // 6. PI gating (ONLY HERE)
        // =================================================
        state.raw_output = use_pi ? state.u_virtual : state.u_ff;
        state.output = constrain(state.raw_output, -PWM_MAX_DUTY, PWM_MAX_DUTY);

        // =================================================
        // 7. Actuation
        // =================================================
        // Direction is derived from the sign of the output
        state.pwm_cmd = mapToPwm(state.output, true);

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

    inline float mapToPwm(float u, float disable)
    {
        if (fabsf(u) < 1e-6f)
            return 0.0f;

        float sign = copysignf(1.0f, u);
        float abs_u = fabsf(u);

        // normalize to [0..1]
        float norm = abs_u / PWM_MAX_DUTY;

        float pwm = 0.0F;
        if (disable)
        {
            pwm = norm * PWM_MAX_DUTY;
        }
        else
        {
            pwm = PWM_MIN_DUTY + norm * (PWM_MAX_DUTY - PWM_MIN_DUTY);
        }
        // map to [PWM_MIN .. PWM_MAX]

        return sign * pwm;
    }

    void setPid(float p, float i, float kff1, float kff2, float kff3, float kaw, float alpha,
                bool usePi)
    {
        kP = p;
        kI = i;
        k1 = kff1;
        k2 = kff2;
        k3 = kff3;
        k_aw = kaw;
        rpsAlpha = alpha;
        use_pi = usePi;
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