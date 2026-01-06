#ifndef PID_TYPES_H_
#define PID_TYPES_H_

#include <stdint.h>

#pragma pack(push, 1)
struct PidSideState
{
    // Inputs
    float setpoint;

    // Estimation
    float measurement;
    float measurement_raw;

    // Error
    float error;

    // Controller memory
    float integral; // I-term accumulator

    // Output
    float p_term;
    float i_term;
    float raw_output;
    float output;

    float pwm_cmd;
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

struct PidTestCommand
{
    uint8_t motor;
    float kp;
    float ki;
    float kff;
    float alpha;
    float testRps; // e.g. 0.6
    uint8_t rampType;
};
#pragma pack(pop)

static_assert(sizeof(PidSideState) == 10 * 4, "PidSideState ABI Layout broken");
static_assert(sizeof(PidLoopSnapshot) == 2 * sizeof(PidSideState) + 5 * 4,
              "PidLoopSnapshot ABI Layout broken");

static_assert(sizeof(PidTestCommand) == 22, "PidTestCommand ABI Layout broken");

#endif