#ifndef CONFIG_H_
#define CONFIG_H_

// ==========================================
//                 PINOUT
// ==========================================
// Left Motor
#define L_PWM 5 // TIMER2 CH-A
#define L_DIR 4

// Right Motor
#define R_PWM 6 // TIMER6 CH-A
#define R_DIR 7

// Left Motor Encoder
#define ENC_L_PIN_A 2 
#define ENC_L_PIN_B 3

// Right Motor Encoder
#define ENC_R_PIN_A 11 // IRQ4
#define ENC_R_PIN_B 8  // IRQ9

// ----- MOTION PROFILING ----
// #define RPS_RAMP_NONE
//  #define RPS_RAMP_LINEAR
// #define RPS_RAMP_SCURVE

// #ifdef RPS_RAMP_LINEAR
#define RPS_RAMP_MAX_ACCEL 0.2F // rps / s
// #endif

#ifdef RPS_RAMP_SCURVE
#define RPS_RAMP_MAX_JERK 5.0F  // [rps/s^3] - how smoothly acceleration changes
#define RPS_RAMP_MAX_ACCEL 2.0F // [rps/s^2] - max acceleration
#endif

// ==========================================
//              BUILD MODE
// ==========================================
// Uncomment EXACTLY ONE

// #define RUN_MODE_CALIB // Step 1: To check proper motor spinning, find minimal PWM to start
//     robot moving & to find exact exact encoder ticks per wheel revolution
#define RUN_MODE_NORMAL // Step 5: Normal driving

// ==========================================
//  CONFIGURATION
// ==========================================
// Physics (measured data)
#define L_TICKS_PER_REV 3800   // Number of slots in the encoder disc
#define R_TICKS_PER_REV 3700   // Number of slots in the encoder disc
#define WHEEL_DIAMETER_CM 6.6F // Wheel diameter in cm
#define TRACK_WIDTH_CM 11.0F   // Distance between center of wheels in cm

#define MAX_RPS 2.0f // physical max wheel speed
#define MAX_CM_PER_S (MAX_RPS * PI * WHEEL_DIAMETER_CM)
#define MAX_DDIST_CM (MAX_CM_PER_S * (SPEED_PID_SAMPLE_TIME_MS / 1000.0f))
#define MAX_DTHETA (MAX_DDIST_CM / TRACK_WIDTH_CM)

// ==========================================
// POST-CALIBRATION CONFIGURATION
// ==========================================

// PID
#define SPEED_PID_SAMPLE_TIME_MS 5 // PID Loop sample rate
// Fill in after PID auto-tune

#define L_MOTOR_PID_KP 8.0f
#define L_MOTOR_PID_KI 1.2f
#define L_MOTOR_PID_KAW 1.2f
// feed forward
#define L_MOTOR_PID_K1 29.0F
#define L_MOTOR_PID_K2 8.5F
#define L_MOTOR_PID_K3 4.5F

#define R_MOTOR_PID_KP 6.0f
#define R_MOTOR_PID_KI 1.5f
#define R_MOTOR_PID_KAW 1.5f
// feed forward
#define R_MOTOR_PID_K1 32.0F
#define R_MOTOR_PID_K2 8.0F
#define R_MOTOR_PID_K3 6.7F

#define MOTOR_MIN_RPS 0.5F
#define MOTOR_MAX_RPS 2.0F

#define RPS_ALPHA 0.2f
//  #define MOTORS_CROSS_COUPLING_GAIN 0.1f

// ==========================================
//                 LIMITS
// ==========================================
#define PWM_FREQ 1000.0f
#define PWM_MAX_DUTY 90.0f
#define PWM_MIN_DUTY 20.0f

#define TELEMETRY_QUEUE_SIZE 8

// ==========================================
//                 DEBUG LEVELS
// ==========================================
#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_INFO

// ==========================================
//                 DEBUG MACROS
// ==========================================
#define DEBUG_LEVEL_NONE 0
#define DEBUG_LEVEL_INFO 1
#define DEBUG_LEVEL_PID 2

#if (CURRENT_DEBUG_LEVEL > DEBUG_LEVEL_NONE)
// Basic Logger
#define LOG_INFO(msg)                                                                              \
    {                                                                                              \
        Serial.print(F("[INFO] "));                                                                \
        Serial.println(F(msg));                                                                    \
    }
#define LOG_DATA(msg, data)                                                                        \
    {                                                                                              \
        Serial.print(F("[INFO] "));                                                                \
        Serial.print(F(msg));                                                                      \
        Serial.print(": ");                                                                        \
        Serial.println((data));                                                                    \
    }
#define LOG_DATA_2(side, msg, d1, d2)                                                              \
    {                                                                                              \
        Serial.print(F("[INFO] "));                                                                \
        Serial.print(F(side));                                                                     \
        Serial.print(F(msg));                                                                      \
        Serial.print(": ");                                                                        \
        Serial.print(d1, 4);                                                                       \
        Serial.print("|");                                                                         \
        Serial.println(d2, 4);                                                                     \
    }
#define LOG_DATA_3(side, msg, d1, d2, d3)                                                          \
    {                                                                                              \
        Serial.print(F("[INFO] "));                                                                \
        Serial.print(F(side));                                                                     \
        Serial.print(F(msg));                                                                      \
        Serial.print(": ");                                                                        \
        Serial.print(d1, 4);                                                                       \
        Serial.print("|");                                                                         \
        Serial.print(d2, 4);                                                                       \
        Serial.print("|");                                                                         \
        Serial.println(d3, 4);                                                                     \
    }
#define LOG_SIDE_INFO(side, msg)                                                                   \
    {                                                                                              \
        Serial.print(F("[INFO] "));                                                                \
        Serial.print(F(side));                                                                     \
        Serial.println(F(msg));                                                                    \
    }
#define LOG_ERR(msg)                                                                               \
    {                                                                                              \
        Serial.print(F("[ERROR] "));                                                               \
        Serial.println(F(msg));                                                                    \
    }
#define LOG_SIDE_ERR(side, msg)                                                                    \
    {                                                                                              \
        Serial.print(F("[ERROR] "));                                                               \
        Serial.print(F(side));                                                                     \
        Serial.println(F(msg));                                                                    \
    }
#else
#define LOG_INFO(msg)
#define LOG_SIDE_INFO(side, msg)
#define LOG_ERR(msg)
#define LOG_SIDE_ERR(side, msg)
#define LOG_DATA(msg, data)
#define LOG_DATA_2(side, msg, d1, d2)
#define LOG_DATA_3(side, msg, d1, d2, d3)
#endif

#if (CURRENT_DEBUG_LEVEL == DEBUG_LEVEL_PID)
// Specialized PID Macro - formatted for Serial Plotter
#define LOG_PID(tick, side, sp, in, err, p, i, out, duty)                                          \
    {                                                                                              \
        Serial.print(F("TICK:"));                                                                  \
        Serial.print(tick);                                                                        \
        Serial.print(F(",SIDE:"));                                                                 \
        Serial.print(F(side));                                                                     \
        Serial.print(F(",SP:"));                                                                   \
        Serial.print(sp, 3);                                                                       \
        Serial.print(F(",IN:"));                                                                   \
        Serial.print(in, 3);                                                                       \
        Serial.print(F(",ERR:"));                                                                  \
        Serial.print(err, 4);                                                                      \
        Serial.print(F(",P:"));                                                                    \
        Serial.print(p, 4);                                                                        \
        Serial.print(F(",I:"));                                                                    \
        Serial.print(i, 4);                                                                        \
        Serial.print(F(",OUT:"));                                                                  \
        Serial.print(out, 4);                                                                      \
        Serial.print(F(",DTY:"));                                                                  \
        Serial.println(duty);                                                                      \
    }
#else
#define LOG_PID(tick, side, set, in, err, p, i, out, duty)
#endif

// ==========================================
//        BUILD MODE SANITY CHECK
// ==========================================

#define RUN_MODE_COUNT (defined(RUN_MODE_CALIB) + defined(RUN_MODE_NORMAL))

#if RUN_MODE_COUNT == 0
#error "No RUN_MODE_* selected. Define exactly ONE run mode."
#elif RUN_MODE_COUNT > 1
#error "Multiple RUN_MODE_* selected. Define exactly ONE run mode."
#endif

// #if (defined(RPS_RAMP_NONE) + defined(RPS_RAMP_LINEAR) + defined(RPS_RAMP_SCURVE)) != 1
// #error "Exactly ONE RPS_RAMP_* must be defined"
// #endif

#endif
