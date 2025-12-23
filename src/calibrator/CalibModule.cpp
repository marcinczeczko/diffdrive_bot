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
        calibrateTPR(10);
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

    m_tele_queue = xQueueCreate(100, sizeof(TelemetryData));
    if (m_tele_queue == NULL)
    {
        Serial.println("Not enough memory for Queue");
        while (1)
            ;
    }

    BaseType_t t1 = xTaskCreate(CalibModule::taskCollectData, "Data", 256, this, 3, &testHandle);
    if (t1 != pdPASS)
    {
        Serial.println("FATAL: TaskCollectData creation failed! Check Stack Sizes.");
        while (1)
            ;
    }
    BaseType_t t2 = xTaskCreate(CalibModule::taskTelemetryData, "Tele", 256, this, 1, &teleHandle);
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

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t currentTicks = self->encoder->getTicks();
        uint32_t deltaTicks = currentTicks - self->lastTicks;
        self->lastTicks = currentTicks;
        float currentRps =
            (float)(deltaTicks * 1000.0f) / (TICKS_PER_REV * SPEED_PID_SAMPLE_TIME_MS);

        switch (self->currentTuneState)
        {
        case IDLE:
            self->stateTimer = 0;
            self->outputPWM = 0.0f;
            if (self->idleTimer == 0)
            {
                Serial.println("IDLE....");
            }
            self->idleTimer += SPEED_PID_SAMPLE_TIME_MS;
            if (self->idleTimer >= 2000)
            {
                // Wait for serial command to set state to BASELINE
                if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
                {
                    self->currentTuneState = BASELINE;
                    xSemaphoreGive(self->xTuneMutex);
                }
            }
            else if (self->idleTimer % 500 == 0)
            {
                Serial.println("PREPARE YOURSELF");
            }
            break;

        case BASELINE:
            self->outputPWM = 0.0f;
            if (self->stateTimer == 0)
                Serial.println("START_TEST");
            self->stateTimer += SPEED_PID_SAMPLE_TIME_MS; // Add loop step
            if (self->stateTimer >= BASELINE_MS)
            {
                if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
                {
                    self->currentTuneState = STEP;
                    xSemaphoreGive(self->xTuneMutex);
                }
                self->stateTimer = 0;
            }
            break;

        case STEP:
            self->outputPWM = self->getTestPwm(); // Instant jump
            self->currentTuneState = OBSERVATION;
            break;

        case OBSERVATION:
            self->stateTimer += SPEED_PID_SAMPLE_TIME_MS;
            if (self->stateTimer >= OBSERVATION_MS)
            {
                if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
                {
                    self->currentTuneState = FINISHED;
                    xSemaphoreGive(self->xTuneMutex);
                }
            }
            break;

        case FINISHED:
            self->outputPWM = 0.0;
            Serial.println("END_TEST");
            if (xSemaphoreTake(self->xTuneMutex, portMAX_DELAY))
            {
                self->currentTuneState = END;
                xSemaphoreGive(self->xTuneMutex);
            }
            self->stateTimer = 0;
            break;
        case END:
            self->motor->shutdown();
            Serial.println("By bye");
            vTaskDelete(NULL);
        }

        // Apply PWM to motor
        self->motor->driveMotor(self->outputPWM, true);

        // Send Telemetry (Time, Speed, Power)
        if (self->currentTuneState != IDLE)
        {
            TelemetryData packet;
            packet.timestamp = xTaskGetTickCount();
            packet.rps = currentRps;
            packet.pwm = self->outputPWM;

            // Send to queue (do not wait if queue is full)
            xQueueSend(self->m_tele_queue, &packet, 0);
        }
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