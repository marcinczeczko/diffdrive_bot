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

struct PidSideTestCommand
{
    uint8_t motor;
    float kp;
    float ki;
    float kff;
    float alpha;
    float testRps; // e.g. 0.6
    uint8_t rampType;
};

struct PidTestAllCommand
{
    float l_kp;
    float l_ki;
    float l_kff;
    float l_alpha;
    float l_testRps;
    uint8_t l_rampType;
    float r_kp;
    float r_ki;
    float r_kff;
    float r_alpha;
    float r_testRps;
    uint8_t r_rampType;
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

static_assert(sizeof(PidSideState) == 10 * 4, "PidSideState ABI Layout broken");
static_assert(sizeof(PidLoopSnapshot) == 2 * sizeof(PidSideState) + 5 * 4,
              "PidLoopSnapshot ABI Layout broken");

static_assert(sizeof(PidSideTestCommand) == 22, "PidTestCommand ABI Layout broken");

#endif