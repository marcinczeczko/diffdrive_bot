#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_

#include "Odometry.h"
#include "calibrator/CalibModule.h"
#include "driver/EncoderDriver.h"
#include "driver/MotorDriver.h"
#include "tele/RtpTelemetry.h"
#include "utils/RpsRamp.h"

#include <Arduino_FreeRTOS.h>
#include <math.h>

extern Rtp::RtpTelemetry telemetry;

enum ControllerMode
{
    AUTO,
    MANUAL
};

class VelocityController
{
    friend class CalibModule;

  private:
    volatile uint32_t loopCounter = 0; // Global sequence counter

    MotorDriver leftMotor, rightMotor;
    SafeEncoder leftEnc, rightEnc;

    float targetL_rps = 0, targetR_rps = 0;
    RpsRamp rampLeft, rampRight;

    int32_t lastLeftTicks = 0, lastRightTicks = 0;
    // cross-coupling gain
    float syncGain;

    // Filtered RPS state
    float filteredLeft_rps = 0.0F;
    float filteredRight_rps = 0.0F;

    // Alpha parameter (tunable)
    float rpsAlpha = 0.0F;

    Odometry odometry;

    SemaphoreHandle_t syncSemaphore;

    ControllerMode runMode;

    // --- PRIVATE WORKER (The "Internal" method) ---
    // This function DOES NOT lock. It assumes the caller handled it.
    void _setTargetRps(float l_rps, float r_rps)
    {
        targetL_rps = l_rps;
        targetR_rps = r_rps;
    }

    auto ticksToCm(int32_t ticks) -> float
    {
        return (float)ticks * (PI * WHEEL_DIAMETER_CM) / TICKS_PER_REV;
    }

  public:
    VelocityController(ControllerMode mode)
        : runMode(mode), leftMotor(L_PWM, L_DIR, MotorSide::LEFT, SPEED_PID_SAMPLE_TIME_MS,
                                   L_MOTOR_PID_KP, L_MOTOR_PID_KI, L_MOTOR_PID_KFF),
          rightMotor(R_PWM, R_DIR, MotorSide::RIGHT, SPEED_PID_SAMPLE_TIME_MS, R_MOTOR_PID_KP,
                     R_MOTOR_PID_KI, R_MOTOR_PID_KFF),
          leftEnc(ENC_L_PIN_A, ENC_L_PIN_B), rightEnc(ENC_R_PIN_A, ENC_R_PIN_B),
          syncGain(MOTORS_CROSS_COUPLING_GAIN), rpsAlpha(RPS_ALPHA)
    {
    }

    Odometry& getOdometry()
    {
        return odometry;
    }

    long getLoopCounter()
    {
        return loopCounter;
    }

    void begin()
    {
        LOG_INFO("--- Starting VelocityController ---");
        odometry.begin();
        // Init Hardware
        leftEnc.begin();
        rightEnc.begin();

        leftMotor.begin();
        rightMotor.begin();

        rampLeft.reset();
        rampRight.reset();

        // Init RTOS Sync
        syncSemaphore = xSemaphoreCreateBinary();
        if (syncSemaphore != nullptr)
        {
            xSemaphoreGive(syncSemaphore); // Binary semaphores start at 0 (locked)
        }

        // For Manual mode (calibraition) do not start main pidloop controlling motors
        if (runMode == AUTO)
        {
            // 3. Start the Task
            BaseType_t xReturned =
                xTaskCreate(VelocityController::vPidLoopTask, "PidLoop", 512, this, 3, nullptr);
            if (xReturned != pdPASS)
            {
                //_tele.logPID('L', localL_rps, vLeftRps, 0);
                LOG_ERR("[CRITICAL] Task Creation Failed! Out of Heap?");
            }
        }
    }

    static void vPidLoopTask(void* pvParameters)
    {
        auto* self = (VelocityController*)pvParameters;
        TickType_t xLastWakeTime = xTaskGetTickCount();

        for (;;)
        {
            // 2. Wait for exactly PID Loop sample Time (e.g. 50ms) since the START of the last
            // execution This function automatically updates xLastWakeTime
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SPEED_PID_SAMPLE_TIME_MS));

            self->loopCounter++;

            constexpr float dt = SPEED_PID_SAMPLE_TIME_MS / 1000.0f;

            // Read target Setpoint
            float localTargetL_rps = 0.0F;
            float localTargetR_rps = 0.0F;
            self->getTargetRps(localTargetL_rps, localTargetR_rps);

            // 2. Apply RPS ramps
            float localL_rps = self->rampLeft.update(localTargetL_rps, dt);
            float localR_rps = self->rampRight.update(localTargetR_rps, dt);

            // Get current encoder ticks
            int32_t currLeftTicks = self->leftEnc.getTicks();
            int32_t currRightTicks = self->rightEnc.getTicks();

            // Calculate delta Ticks
            int32_t dLeftTicks = currLeftTicks - self->lastLeftTicks;
            int32_t dRightTicks = currRightTicks - self->lastRightTicks;

            // Save current ticks for next run delta
            self->lastLeftTicks = currLeftTicks;
            self->lastRightTicks = currRightTicks;

            // --- ODOMETRY
            float deltaLcm = self->ticksToCm(dLeftTicks);
            float deltaRcm = self->ticksToCm(dRightTicks);

            float dDist = (deltaRcm + deltaLcm) / 2.0f;
            float dTheta = (deltaRcm - deltaLcm) / TRACK_WIDTH_CM;

            self->odometry.update(dDist, dTheta);
            // --- ENd of ODOMETRY

            // Calculate RPS for delta ticks
            float rawRps_left = ticksToRps(dLeftTicks);
            float rawRps_right = ticksToRps(dRightTicks);

            // Apply alpha filter (noise filter) to calculated rps
            self->filteredLeft_rps =
                self->alphaFilter(self->filteredLeft_rps, rawRps_left, self->rpsAlpha);

            self->filteredRight_rps =
                self->alphaFilter(self->filteredRight_rps, rawRps_right, self->rpsAlpha);

            // Use filtered values from now on
            float vLeftRps = self->filteredLeft_rps;
            float vRightRps = self->filteredRight_rps;

            // Tracking errors
            float eLeft = localL_rps - vLeftRps;
            float eRight = localR_rps - vRightRps;

            // Cross-coupling error (difference of errors!)
            float syncError = eLeft - eRight;

            // Correct targets (OPPOSITE signs!)
            float corrLeft_rps = localL_rps - self->syncGain * syncError;
            float corrRight_rps = localR_rps + self->syncGain * syncError;

            // Feed motors
            self->leftMotor.update(corrLeft_rps, currLeftTicks, self->loopCounter);
            self->rightMotor.update(corrRight_rps, currRightTicks, self->loopCounter);

            if (self->loopCounter % 10 == 0)
            {
                // LOG_DATA_2("RPS", " rpsL|rpsR", corrLeft_rps, corrRight_rps);
            }
        }
    }

    static float ticksToRps(long ticks)
    {
        return (float)(ticks * 1000.0f) / (TICKS_PER_REV * SPEED_PID_SAMPLE_TIME_MS);
    }

    void setRps(float l_rps, float r_rps)
    {
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY))
        {
            _setTargetRps(l_rps, r_rps);
            xSemaphoreGive(syncSemaphore);
        }
    }

    void setTwist(float linVcms, float angVrads)
    {
        // vL = v - omega * L / 2
        // vR = v + omega * L / 2
        float vL = linVcms - (angVrads * TRACK_WIDTH_CM / 2.0f);
        float vR = linVcms + (angVrads * TRACK_WIDTH_CM / 2.0f);

        // Convert cm/s to RPS (RPS = v / circumference)
        float circ = PI * WHEEL_DIAMETER_CM;

        setRps(vL / circ, vR / circ);
    }

    void stopEmergency()
    {
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY))
        {
            // We call the internal worker to avoid "taking" the semaphore again
            _setTargetRps(0.0f, 0.0f);
            rampLeft.reset();
            rampRight.reset();
            resetEncoders();
            leftMotor.driveMotor(0.0f, true);
            rightMotor.driveMotor(0.0f, true);

            xSemaphoreGive(syncSemaphore);
        }
    }

    // Used by the PID Task
    void getTargetRps(float& l_rps, float& r_rps)
    {
        if (xSemaphoreTake(syncSemaphore, portMAX_DELAY))
        {
            l_rps = targetL_rps;
            r_rps = targetR_rps;
            xSemaphoreGive(syncSemaphore);
        }
    }

    void manualPwm(float l_pwm, float r_pwm)
    {
        leftMotor.driveMotor(fabs(l_pwm), l_pwm >= 0.0f);
        rightMotor.driveMotor(fabs(r_pwm), r_pwm >= 0.0f);
    }

    void resetEncoders()
    {
        leftEnc.reset();
        rightEnc.reset();
    }

    inline float alphaFilter(float prev, float current, float alpha)
    {
        return alpha * current + (1.0f - alpha) * prev;
    }

    // Helper for Calibrator
    long getLeftTicks()
    {
        return leftEnc.getTicks();
    }
    long getRightTicks()
    {
        return rightEnc.getTicks();
    }
};

#endif