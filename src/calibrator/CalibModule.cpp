#include "config/Config.h"

#ifdef RUN_MODE_CALIB

#include "CalibModule.h"
#include "config/Config.h"
#include "driver/EncoderDriver.h"
#include "driver/MotorDriver.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

CalibModule::CalibModule(VelocityController* robot) : controller(robot)
{
}

void CalibModule::begin()
{
    xTuneMutex = xSemaphoreCreateBinary();
    if (xTuneMutex != nullptr)
    {
        xSemaphoreGive(xTuneMutex);
    }

    controller->begin(); // hardware init only, no scheduler

    const int sel = waitForSelection();

    switch (sel)
    {
    case 1:
        testEncoders();
        break;
    case 2:
        findMinPWM();
        break;
    case 3:
        calibrateTPR2(1);
        break;
    case 4:
        beginDataCollector(MotorSide::LEFT, TuneCollectorSpeed::SLOW);
        break;
    case 5:
        beginDataCollector(MotorSide::RIGHT, TuneCollectorSpeed::SLOW);
        break;
    case 6:
        beginDataCollector(MotorSide::LEFT, TuneCollectorSpeed::MEDIUM);
        break;
    case 7:
        beginDataCollector(MotorSide::RIGHT, TuneCollectorSpeed::MEDIUM);
        break;
    case 8:
        beginDataCollector(MotorSide::LEFT, TuneCollectorSpeed::HIGH);
        break;
    case 9:
        beginDataCollector(MotorSide::RIGHT, TuneCollectorSpeed::HIGH);
        break;
    default:
        LOG_ERR("Invalid selection");
    }

    LOG_INFO("Calibration complete. Halting.");
    while (1)
        ; // stop here — no RTOS
}

int CalibModule::waitForSelection()
{
    LOG_INFO("Select calibration:");
    LOG_INFO("1 - Encoder test");
    LOG_INFO("2 - Min PWM");
    LOG_INFO("3 - Ticks per rev");
    LOG_INFO("4 - Collect LEFT Motor Rps -> RPM data - SLOW");
    LOG_INFO("5 - Collect RIGHT Motor Rps -> RPM data - SLOW");
    LOG_INFO("6 - Collect LEFT Motor Rps -> RPM data - MEDIUM");
    LOG_INFO("7 - Collect RIGHT Motor Rps -> RPM data - MEDIUM");
    LOG_INFO("8 - Collect LEFT Motor Rps -> RPM data - HIGH");
    LOG_INFO("9 - Collect RIGHT Motor Rps -> RPM data - HIGH");

    while (!Serial.available())
        ;
    return Serial.read() - '0';
}

void CalibModule::testEncoders()
{
    LOG_INFO("--- ENCODERS TEST (RAW PWM) ---");
    controller->manualPwm(40.0F, 40.0F);

    unsigned long start = millis();
    while (millis() - start < 10000)
    {
        LOG_DATA_2("ALL", "L_Ticks|R_Ticks: ", controller->getLeftTicks(),
                   controller->getRightTicks());

        // Log every 200ms
        delay(200);
    }
    controller->manualPwm(0.0, 0.0);
    LOG_INFO(F("Done."));
}

void CalibModule::findMinPWM()
{
    LOG_INFO("--- Min PWM Calibration ---");

    LOG_INFO(">> LEFT Motor test ...");
    for (int i = (int)PWM_MIN_DUTY; i <= (int)PWM_MAX_DUTY; i += 2)
    {
        controller->manualPwm((float)i, 0.0f);
        LOG_DATA("PWM L: ", i);
        delay(200);
    }
    controller->manualPwm(0, 0);
    delay(2000);

    LOG_INFO(F(">> RIGHT Motor test..."));
    for (int i = (int)PWM_MIN_DUTY; i <= (int)PWM_MAX_DUTY; i += 2)
    {
        controller->manualPwm(0.0f, (float)i);
        LOG_DATA("PWM R: ", i);
        delay(200);
    }
    controller->manualPwm(0, 0);
    LOG_INFO("Done.");
}

// Calibrate Ticks Per Revolution
void CalibModule::calibrateTPR(int targetRotations)
{
    LOG_INFO("--- Ticks Per Revolution CALIBRATION ---");
    LOG_INFO("1. Mark a spot on the wheel and the chassis.");
    LOG_INFO("2. The wheel will spin slowly.");
    LOG_DATA("3. Wheel will be rotated this amount of rotations: ", targetRotations);
    delay(5000);

    // Reset encoder counts to zero
    // You might need to add a resetEncoders() to VelocityController
    controller->resetEncoders();

    // Start spinning slowly (just above your MIN_PWM)
    controller->manualPwm(PWM_MIN_DUTY + 5.0f, PWM_MIN_DUTY + 5.0f);

    LOG_INFO("Spinning... Press any key in Serial Monitor to STOP.");

    while (!Serial.available())
    {
        LOG_DATA("L Ticks: ", controller->getLeftTicks());
        LOG_DATA(" | R Ticks: ", controller->getRightTicks());
        delay(200);
    }

    // Stop and calculate
    controller->manualPwm(0.0f, 0.0f);
    long finalL = controller->getLeftTicks();
    long finalR = controller->getRightTicks();

    LOG_INFO("\n--- RESULTS ---");
    LOG_DATA("Left TPR: ", (float)finalL / targetRotations);
    LOG_DATA("Right TPR: ", (float)finalR / targetRotations);
    LOG_INFO("Update TICKS_PER_REV in Config.h with these values.");

    // Clear the serial buffer
    while (Serial.available())
        Serial.read();
}

void CalibModule::calibrateTPR2(int targetRotations)
{
    LOG_INFO("--- Ticks Per Revolution VERIFICATION ---");
    LOG_INFO("This test uses already calibrated TPR values.");
    LOG_INFO("Wheel will rotate a fixed number of turns and stop automatically.");
    LOG_DATA("Target rotations: ", targetRotations);
    delay(3000);

    // ---- CONFIG ----
    constexpr float TEST_PWM = 28.0f;

    const long targetTicksLeft = (long)(L_TICKS_PER_REV * targetRotations);
    const long targetTicksRight = (long)(R_TICKS_PER_REV * targetRotations);

    // ---- RESET ----
    controller->resetEncoders();
    delay(50);

    // ---- START MOTION ----
    controller->manualPwm(0.0f, TEST_PWM);
    LOG_INFO("Spinning...");

    while (true)
    {
        // long ticksL = labs(controller->getLeftTicks());
        long ticksR = labs(controller->getRightTicks());

        // LOG_DATA("L ticks: ", ticksL);
        LOG_DATA(" | R ticks: ", ticksR);

        // Stop condition: BOTH wheels reached target
        if (ticksR >= targetTicksRight) // && ticksR >= targetTicksRight)
            break;

        delay(50);
    }

    // ---- STOP ----
    controller->stopEmergency();
    delay(200);

    // long finalL = controller->getLeftTicks();
    long finalR = controller->getRightTicks();

    LOG_INFO("\n--- VERIFICATION RESULTS ---");
    // LOG_DATA("Left ticks: ", finalL);
    LOG_DATA("Right ticks: ", finalR);

    // LOG_DATA("Left rotations achieved: ", (float)finalL / L_TICKS_PER_REV);
    LOG_DATA("Right rotations achieved: ", (float)finalR / R_TICKS_PER_REV);

    LOG_INFO("Expected rotations:");
    LOG_DATA("Target: ", targetRotations);

    LOG_INFO("If values differ noticeably → TPR calibration is off.");
}

void CalibModule::beginDataCollector(MotorSide motorSide, TuneCollectorSpeed speedMode)
{
    if (motorSide == MotorSide::LEFT)
    {
        motor = &controller->leftMotor;
        encoder = &controller->leftEnc;
    }
    else
    {
        motor = &controller->rightMotor;
        encoder = &controller->rightEnc;
    }
    pwmTuneSpeedMode = speedMode;
    side = motorSide;

    m_tele_queue = xQueueCreate(100, sizeof(TelemetryData));
    if (m_tele_queue == NULL)
    {
        Serial.println("Not enough memory for Queue");
        while (1)
            ;
    }

    BaseType_t t1 = xTaskCreate(CalibModule::taskCollectData, "Data", 128, this, 3, &testHandle);
    if (t1 != pdPASS)
    {
        Serial.println("FATAL: TaskCollectData creation failed! Check Stack Sizes.");
        while (1)
            ;
    }
    BaseType_t t2 = xTaskCreate(CalibModule::taskTelemetryData, "Tele", 128, this, 1, &teleHandle);
    if (t2 != pdPASS)
    {
        Serial.println("FATAL: vTelemetryTask creation failed! Check Stack Sizes.");
        while (1)
            ;
    }
    vTaskStartScheduler();
}

float CalibModule::getTestPwm()
{
    switch (pwmTuneSpeedMode)
    {
    case SLOW:
        return pwmSlow;

    case MEDIUM:
        return pwmMedium;

    case HIGH:
    default:
        return pwmHigh;
    }
}

void CalibModule::taskCollectData(void* pvParameters)
{
    auto* self = (CalibModule*)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SPEED_PID_SAMPLE_TIME_MS);
    const float dt = SPEED_PID_SAMPLE_TIME_MS / 1000.0f;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t currentTicks = self->encoder->getTicks();
        int32_t deltaTicks = (int32_t)(currentTicks - self->lastTicks);
        self->lastTicks = currentTicks;

        uint32_t ticksPerRev = (self->side == MotorSide::LEFT) ? L_TICKS_PER_REV : R_TICKS_PER_REV;
        float currentRps = (float)deltaTicks / (((float)ticksPerRev) * dt);

        float now = xTaskGetTickCount() * dt;

        switch (self->currentTuneState)
        {
        case IDLE:
        {
            self->stateTimer = 0;
            self->outputPWM = 0.0f;
            self->idleTimer += SPEED_PID_SAMPLE_TIME_MS;

            if (self->idleTimer >= 2000)
            {
                self->idleTimer = 0;
                if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
                {
                    self->currentTuneState = BASELINE;
                    xSemaphoreGive(self->xTuneMutex);
                }
            }
            break;
        }
        case BASELINE:
        {
            self->outputPWM = 0.0f;
            self->stateTimer += SPEED_PID_SAMPLE_TIME_MS;

            // RESET IDENTIFICATION STATE
            self->ident = {};
            self->tune = {};
            self->stepDetected = false;
            self->rpsAccum = 0.0f;
            self->rpsSamples = 0;

            if (self->stateTimer >= BASELINE_MS)
            {
                self->stateTimer = 0;
                if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
                {
                    self->currentTuneState = STEP;
                    xSemaphoreGive(self->xTuneMutex);
                }
            }
            break;
        }
        case STEP:
        {
            self->outputPWM = self->getTestPwm(); // Instant jump

            // Detect step
            self->stepDetected = true;
            self->stepStartTime = now;
            self->ident.stepPwm = fabsf(self->outputPWM);

            self->currentTuneState = OBSERVATION;
            break;
        }
        case OBSERVATION:
        {
            self->stateTimer += SPEED_PID_SAMPLE_TIME_MS;
            // Accumulate for steady-state
            self->rpsAccum += currentRps;
            self->rpsSamples++;

            float rpsAvg = self->rpsAccum / self->rpsSamples;

            // Detect τ (63.2%)
            if (self->ident.tau == 0.0f)
            {
                float rps63 = 0.632f * rpsAvg;
                if (currentRps >= rps63)
                {
                    self->ident.tau = now - self->stepStartTime;
                }
            }

            if (self->stateTimer >= OBSERVATION_MS)
            {
                self->ident.rpsFinal = rpsAvg;
                self->ident.K = self->ident.rpsFinal / self->ident.stepPwm;
                self->currentTuneState = FINISHED;
                self->stateTimer = 0;
            }
            break;
        }
        case FINISHED:
        {
            constexpr float OMEGA_C = 10.0f; // ✔ scoped

            self->tune.kFF = 1.0f / self->ident.K;
            self->tune.kP = (self->ident.tau / self->ident.K) * OMEGA_C;
            self->tune.kI = self->tune.kP / (4.0f * self->ident.tau);

            Serial.println("\n=== AUTO TUNE RESULT ===");
            Serial.print("K     = ");
            Serial.println(self->ident.K, 6);
            Serial.print("tau   = ");
            Serial.println(self->ident.tau, 4);
            Serial.print("kFF   = ");
            Serial.println(self->tune.kFF, 6);
            Serial.print("kP    = ");
            Serial.println(self->tune.kP, 4);
            Serial.print("kI    = ");
            Serial.println(self->tune.kI, 4);

            self->outputPWM = 0.0f;
            self->stateTimer = 0;
            self->currentTuneState = END;
            self->motor->shutdown();
            vTaskDelete(NULL);
        }
        }

        // Apply PWM to motor
        self->motor->driveMotor(self->outputPWM, true);
    }
}

void CalibModule::taskTelemetryData(void* pvParameters)
{
    auto* self = (CalibModule*)pvParameters;

    TelemetryData receivedPacket;
    for (;;)
    {
        // Wait forever (portMAX_DELAY) until a packet arrives in the queue
        if (xQueueReceive(self->m_tele_queue, &receivedPacket, portMAX_DELAY))
        {
            // Print with 5 decimal places for speed
            Serial.print(receivedPacket.timestamp);
            Serial.print(",");
            Serial.print(receivedPacket.rps, 4);
            Serial.print(",");
            Serial.println(receivedPacket.pwm, 3);
        }
    }
}

#endif