#ifndef CALIB_MODULE_H_
#define CALIB_MODULE_H_

#ifdef RUN_MODE_CALIB

#include "controller/VelocityController.h"
#include "driver/EncoderDriver.h"
#include "driver/MotorDriver.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

// Configuration
constexpr int BASELINE_MS = 1000;    // 1 second of lead-in
constexpr int OBSERVATION_MS = 5000; // 5 seconds of data collection

constexpr float pwmSlow = 30.0F;
constexpr float pwmMedium = 50.0F;
constexpr float pwmHigh = 80.0F;

class VelocityController;

class CalibModule
{
    enum TuneState
    {
        IDLE,
        BASELINE,
        STEP,
        OBSERVATION,
        FINISHED,
        END
    };

    enum MotorSide
    {
        LEFT,
        RIGHT
    };

    enum TuneCollectorSpeed
    {
        SLOW,
        MEDIUM,
        HIGH
    };

    using TelemetryData = struct
    {
        float timestamp;
        float rps;
        float pwm;
    };

    struct MotorIdentification
    {
        float stepPwm = 0.0f;
        float rpsFinal = 0.0f;
        float tau = 0.0f;
        float K = 0.0f;
    };

    struct AutoTuneResult
    {
        float kFF = 0.0f;
        float kP = 0.0f;
        float kI = 0.0f;
    };

  private:
    TuneState currentTuneState = IDLE;
    uint32_t stateTimer = 0;
    uint32_t lastTicks = 0;

    uint32_t idleTimer = 0;
    float outputPWM = 0.0F;

    TaskHandle_t testHandle{nullptr};
    TaskHandle_t teleHandle{nullptr};

    SemaphoreHandle_t xTuneMutex;

    QueueHandle_t m_tele_queue{nullptr};

    // Data collector required robot elements
    MotorDriver* motor;
    SafeEncoder* encoder;
    MotorSide side;

    TuneCollectorSpeed pwmTuneSpeedMode;

    MotorIdentification ident;
    AutoTuneResult tune;

    bool stepDetected = false;
    float stepStartTime = 0.0f;

    // steady-state estimation
    float rpsAccum = 0.0f;
    uint32_t rpsSamples = 0;

    int waitForSelection();

    // Simple test of encoders
    void testMotorsEncoders();

    void testEncoders();

    // Simple test to get the minimum PWM
    void findMinPWM();

    // Simple test to calibrate encoder ticks per wheel revolution
    void calibrateTPR(int targetRotations);

    void calibrateTPR2(int targetRotations);

    float getTestPwm();

    static void taskCollectData(void* pvParameters);

    static void taskTelemetryData(void* pvParameters);

    VelocityController* controller;

  public:
    CalibModule(VelocityController* controller);

    // Initialize system for simple calibrations
    void begin();

    // Initialize data collector tasks for Cohen Calib/CISM calibration method
    void beginDataCollector(MotorSide motorSide, TuneCollectorSpeed speed);
};

#endif

#endif