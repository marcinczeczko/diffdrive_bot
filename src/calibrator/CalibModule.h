#ifndef CALIB_MODULE_H_
#define CALIB_MODULE_H_

#include "controller/VelocityController.h"
#include "driver/EncoderDriver.h"
#include "driver/MotorDriver.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

// Configuration
constexpr float STEP_PWM = 40.0F;    // PWM value for the test
constexpr int BASELINE_MS = 1000;    // 1 second of lead-in
constexpr int OBSERVATION_MS = 4000; // 3 seconds of data collection

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

    TuneCollectorSpeed pwmTuneSpeedMode;

    int waitForSelection();

    // Simple test of encoders
    void testEncoders();

    // Simple test to get the minimum PWM
    void findMinPWM();

    // Simple test to calibrate encoder ticks per wheel revolution
    void calibrateTPR(int targetRotations);

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