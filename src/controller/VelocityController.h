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

#pragma pack(push, 1)
struct ControlLoopPayload
{
    uint32_t loopCntr;
    float setpointL;
    float setpointR;

    float rampSetpointL;
    float rampSsetpointR;

    float measuredRpsL;
    float measuredRpsR;

    float deltaLticks;
    float deltaRticks;
};
#pragma pack(pop)

static_assert(sizeof(ControlLoopPayload) == 36, "ControlLoopPayload ABI mismatch");

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
    // // cross-coupling gain
    // float syncGain;

    Odometry odometry;

    Rtp::RtpTelemetry* telemetry;

    SemaphoreHandle_t syncSemaphore;

    ControllerMode runMode;

    // --- PRIVATE WORKER (The "Internal" method) ---
    // This function DOES NOT lock. It assumes the caller handled it.
    void _setTargetRps(float l_rps, float r_rps)
    {
        targetL_rps = l_rps;
        targetR_rps = r_rps;
    }

  public:
    VelocityController(Rtp::RtpTelemetry* tele, ControllerMode mode)
        : telemetry(tele), runMode(mode),
          leftMotor(L_PWM, L_DIR, L_TICKS_PER_REV, MOTOR_SIDE_LEFT, SPEED_PID_SAMPLE_TIME_MS,
                    L_MOTOR_PID_KP, L_MOTOR_PID_KI, L_MOTOR_PID_KFF, tele),
          rightMotor(R_PWM, R_DIR, R_TICKS_PER_REV, MOTOR_SIDE_RIGHT, SPEED_PID_SAMPLE_TIME_MS,
                     R_MOTOR_PID_KP, R_MOTOR_PID_KI, R_MOTOR_PID_KFF, tele),
          leftEnc(ENC_L_PIN_A, ENC_L_PIN_B), rightEnc(ENC_R_PIN_A, ENC_R_PIN_B)
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

            // ODOMETRY (raw ticks only)
            int32_t dLeftTicks = currLeftTicks - self->lastLeftTicks;
            int32_t dRightTicks = currRightTicks - self->lastRightTicks;

            // Save current ticks for next run delta
            self->lastLeftTicks = currLeftTicks;
            self->lastRightTicks = currRightTicks;

            // --- ODOMETRY
            float deltaLcm = (float)dLeftTicks * (PI * WHEEL_DIAMETER_CM) / L_TICKS_PER_REV;
            float deltaRcm = (float)dRightTicks * (PI * WHEEL_DIAMETER_CM) / R_TICKS_PER_REV;

            float dDist = (deltaRcm + deltaLcm) * 0.5f;
            float dTheta = (deltaRcm - deltaLcm) / TRACK_WIDTH_CM;

            self->odometry.update(dDist, dTheta);
            if (self->loopCounter % 5 == 0 && self->telemetry != nullptr)
            {
                Pose pose = self->odometry.getPose();

                OdomPayload odom{};
                odom.loopCntr = self->loopCounter;
                odom.x = pose.x;
                odom.y = pose.y;
                odom.theta = pose.theta;

                self->telemetry->publish(Rtp::RTP_ODOM, odom);
            }
            // --- ENd of ODOMETRY

            // Feed motors
            self->leftMotor.update(localL_rps, currLeftTicks, self->loopCounter);
            self->rightMotor.update(localR_rps, currRightTicks, self->loopCounter);

            if (self->loopCounter % 5 == 0 && self->telemetry != nullptr)
            {
                ControlLoopPayload payload{};
                payload.loopCntr = self->loopCounter;
                payload.setpointL = localTargetL_rps;
                payload.setpointR = localTargetR_rps;
                payload.rampSetpointL = localL_rps;
                payload.rampSsetpointR = localR_rps;
                payload.deltaLticks = dLeftTicks;
                payload.deltaRticks = dRightTicks;

                self->telemetry->publish(Rtp::RTP_CTRL, payload);
            }
        }
    }

    // static float ticksToRps(long ticks)
    // {
    //     return (float)(ticks * 1000.0f) / (TICKS_PER_REV * SPEED_PID_SAMPLE_TIME_MS);
    // }

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