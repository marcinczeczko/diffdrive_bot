#ifndef PID_TYPES_H_
#define PID_TYPES_H_

#include <stdint.h>

#pragma pack(push, 1)
/*
===============================================================================
 PID SIDE STATE – CONTROL SIGNAL BREAKDOWN
===============================================================================

This struct captures the COMPLETE internal state of the motor velocity controller
for ONE motor side (left or right).

Control architecture:

            setpoint (v_ref)
                   │
                   ▼
          ┌───────────────────┐
          │  Feed-Forward     │   Motor inverse model
          │  Model            │   u_ff = k1*sign(v)
          │  (k1, k2, k3)     │        + k2*v
          └───────────────────┘        + k3*v*|v|
                   │
                   │ u_ff
                   │
                   ├───────────────┐
                   │               │
                   ▼               ▼
         measurement            PI Observer
              │              (error correction)
              │                     │
              ▼                     │
        error = v_ref − v_meas      │
                                    │
                            u_pi = P + I
                                    │
                   └────-───────────┘
                             │
                             ▼
                 u_virtual = u_ff + u_pi
                             │
                     actuator saturation
                             │
                             ▼
                       u_sat / output
                             │
                     PWM mapping + min PWM
                             │
                             ▼
                          pwm_cmd

Key design principles:
- Feed-forward carries MOST of the work
- PI corrects modeling errors only
- Anti-windup is applied on the FULL controller (u_virtual)
- PI can be observed even when disabled (use_pi = false)
- pwm_cmd is the ONLY value that touches hardware

===============================================================================
*/

struct PidSideState
{
    // ------------------------------------------------------------------------
    // INPUT (REFERENCE)
    // ------------------------------------------------------------------------

    float setpoint;
    // Target wheel velocity [RPS]
    // This is the reference signal v_ref used by both FF and PI.

    // ------------------------------------------------------------------------
    // VELOCITY ESTIMATION
    // ------------------------------------------------------------------------

    float measurement;
    // Filtered wheel velocity estimate [RPS]
    // Used by the controller for error computation.
    // Obtained from measurement_raw via low-pass filtering (alpha).

    float measurement_raw;
    // Raw wheel velocity estimate [RPS] from encoder delta.
    // Noisy due to quantization; used only for diagnostics.

    // ------------------------------------------------------------------------
    // ERROR SIGNAL
    // ------------------------------------------------------------------------

    float error;
    // Velocity error [RPS]
    // error = setpoint − measurement
    // Positive → motor too slow
    // Negative → motor too fast

    // ------------------------------------------------------------------------
    // CONTROLLER MEMORY
    // ------------------------------------------------------------------------

    float integral;
    // Integral accumulator (∫ error dt)
    // Represents required steady-state torque to overcome friction/load.
    // Reset at full standstill, protected by anti-windup.

    // ------------------------------------------------------------------------
    // FEED-FORWARD MODEL OUTPUTS
    // ------------------------------------------------------------------------

    float u_ff;
    // Feed-forward control effort
    // Output of the motor inverse model:
    //   u_ff = k1*sign(v) + k2*v + k3*v*|v|
    // Dominant term during normal operation.

    float u_pi;
    // PI correction term (P + I)
    // Computed even when PI is disabled (useful for observation/tuning).

    float u_virtual;
    // Virtual controller output BEFORE saturation
    // u_virtual = u_ff + u_pi
    // Used for anti-windup logic and model quality assessment.

    float u_sat;
    // Virtual controller output AFTER saturation
    // u_sat = constrain(u_virtual, ±U_MAX)
    // Shows where actuator limits are hit.

    // ------------------------------------------------------------------------
    // CLASSICAL PID TERMS (DIAGNOSTICS)
    // ------------------------------------------------------------------------

    float p_term;
    // Proportional term
    // p_term = kP * error

    float i_term;
    // Integral term contribution
    // i_term = kI * integral

    float aw_term;

    // ------------------------------------------------------------------------
    // CONTROLLER OUTPUTS
    // ------------------------------------------------------------------------

    float raw_output;
    // Controller output AFTER PI gating
    // Equals:
    //   - u_ff when PI is disabled
    //   - u_virtual when PI is enabled

    float output;
    // Controller output AFTER saturation
    // Still in control units (NOT PWM).

    // ------------------------------------------------------------------------
    // ACTUATOR COMMAND
    // ------------------------------------------------------------------------

    float pwm_cmd;
    // Final PWM command sent to hardware
    // Includes:
    //   - saturation
    //   - min-PWM mapping (static friction compensation)
    //   - sign (direction)
};

struct PidLoopSnapshot
{
    uint32_t loop_cntr;

    float left_target_setpoint;
    int32_t left_delta_ticks;
    PidSideState left;

    float right_target_setpoint;
    int32_t right_delta_ticks;
    PidSideState right;
};

struct PidSideTestCommand
{
    uint8_t motor;
    float kp;
    float ki;
    float k1;
    float k2;
    float k3;
    float kaw;
    float alpha;
    float testRps; // e.g. 0.6
    uint8_t rampType;
    uint8_t usePi;
};

struct PidTestAllCommand
{
    float l_kp;
    float l_ki;
    float l_k1;
    float l_k2;
    float l_k3;
    float l_kaw;
    float l_alpha;
    float l_testRps;
    uint8_t l_rampType;
    uint8_t l_usePi;

    float r_kp;
    float r_ki;
    float r_k1;
    float r_k2;
    float r_k3;
    float r_kaw;
    float r_alpha;
    float r_testRps;
    uint8_t r_rampType;
    uint8_t r_usePi;
};

struct OdomPayload
{
    uint32_t loopCntr;
    float x;
    float y;
    float theta;
};

#pragma pack(pop)

enum class PidTestCmdType : uint8_t
{
    SIDE,
    BOTH
};

struct PidTestCommand
{
    PidTestCmdType type;

    union
    {
        PidSideTestCommand side;
        PidTestAllCommand both;
    };
};

static_assert(sizeof(OdomPayload) == 16, "OdomPayload ABI mismatch");

static_assert(sizeof(PidSideState) == 15 * 4, "PidSideState ABI Layout broken");
static_assert(sizeof(PidLoopSnapshot) == 2 * sizeof(PidSideState) + 5 * 4,
              "PidLoopSnapshot ABI Layout broken");

static_assert(sizeof(PidSideTestCommand) == 8 * 4 + 3, "PidTestCommand ABI Layout broken");

#endif