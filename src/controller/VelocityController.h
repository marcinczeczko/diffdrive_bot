#ifndef VELOCITY_CONTROLLER_H_
#define VELOCITY_CONTROLLER_H_

#include "Odometry.h"
#include "calibrator/CalibModule.h"
#include "controller/PidTypes.h"
#include "driver/EncoderDriver.h"
#include "driver/MotorDriver.h"
#include "tele/RtpTelemetry.h"
#include "utils/RpsRamp.h"

#include <Arduino_FreeRTOS.h>
#include <math.h>

enum ControllerMode
{
    AUTO,
    MANUAL
};

enum Ramp : uint8_t
{
    NONE = 0x00,
    LINEAR = 0x01
};

class VelocityController
{
    friend class CalibModule;

  private:
    uint32_t loopCounter = 0; // Global sequence counter

    MotorDriver leftMotor, rightMotor;
    SafeEncoder leftEnc, rightEnc;

    float targetL_rps = 0, targetR_rps = 0;
    RpsRamp rampLeft, rampRight;

    int32_t lastLeftTicks = 0, lastRightTicks = 0;
    // // cross-coupling gain
    // float syncGain;

    uint8_t rampType = Ramp::NONE;

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
          leftMotor(L_PWM, L_DIR, L_TICKS_PER_REV, MOTOR_SIDE_LEFT, L_MOTOR_PID_KP, L_MOTOR_PID_KI,
                    L_MOTOR_PID_KFF, tele),
          rightMotor(R_PWM, R_DIR, R_TICKS_PER_REV, MOTOR_SIDE_RIGHT, R_MOTOR_PID_KP,
                     R_MOTOR_PID_KI, R_MOTOR_PID_KFF, tele),
          leftEnc(ENC_L_PIN_A, ENC_L_PIN_B), rightEnc(ENC_R_PIN_A, ENC_R_PIN_B)
    {
    }

    Odometry& getOdometry()
    {
        return odometry;
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
            xSemaphoreGive(syncSemaphore);
            // Binary semaphore starts empty; give() makes it available
        }

        // For Manual mode (calibraition) do not start main pidloop controlling motors
        if (runMode == AUTO)
        {
            // 3. Start the Task
            BaseType_t xReturned =
                xTaskCreate(VelocityController::vPidLoopTask, "PidLoop", 512, this, 3, nullptr);
            if (xReturned != pdPASS)
            {
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
            float left_target_rps = 0.0F;
            float right_target_rps = 0.0F;
            self->getTargetRps(left_target_rps, right_target_rps);

            // 2. Apply RPS ramps
            float left_local_rps = left_target_rps;
            float right_local_rps = right_target_rps;
            if (self->rampType == Ramp::LINEAR)
            {
                left_local_rps = self->rampLeft.updateLinear(left_target_rps, dt);
                right_local_rps = self->rampRight.updateLinear(right_target_rps, dt);
            }

            // Get current encoder ticks
            int32_t current_ticks_l = self->leftEnc.getTicks();
            int32_t current_ticks_r = self->rightEnc.getTicks();

            // ODOMETRY (raw ticks only)
            float delta_ticks_l = current_ticks_l - self->lastLeftTicks;
            float delta_ticks_r = current_ticks_r - self->lastRightTicks;
            // Save current ticks for next run delta
            self->lastLeftTicks = current_ticks_l;
            self->lastRightTicks = current_ticks_r;

            // --- ODOMETRY
            float left_delta_cm = (float)delta_ticks_l * (PI * WHEEL_DIAMETER_CM) / L_TICKS_PER_REV;
            float right_delta_cm =
                (float)delta_ticks_r * (PI * WHEEL_DIAMETER_CM) / R_TICKS_PER_REV;

            float dDist = (right_delta_cm + left_delta_cm) * 0.5f;
            float dTheta = (right_delta_cm - left_delta_cm) / TRACK_WIDTH_CM;

            self->odometry.update(dDist, dTheta);

            // Feed motors
            self->leftMotor.update(left_local_rps, delta_ticks_l);
            self->rightMotor.update(right_local_rps, delta_ticks_r);

            if (self->telemetry != nullptr)
            {
                PidLoopSnapshot snapshot;
                memset(&snapshot, 0, sizeof(snapshot));
                snapshot.loop_cntr = self->loopCounter;

                snapshot.left_target_setpoint = left_target_rps;
                snapshot.left_delta_ticks = delta_ticks_l;

                snapshot.right_target_setpoint = right_target_rps;
                snapshot.right_delta_ticks = delta_ticks_r;

                self->leftMotor.copyState(snapshot.left);
                self->rightMotor.copyState(snapshot.right);

                self->telemetry->publish(Rtp::RTP_PID, snapshot);
            }
        }
    }

    void setRampType(uint8_t type)
    {
        rampType = type;
        rampLeft.reset();
        rampRight.reset();
    }

    void updatePids(MotorSide motor, float p, float i, float ff, float alpha)
    {
        if (motor == MOTOR_SIDE_LEFT)
            leftMotor.setPid(p, i, ff, alpha);
        else
            rightMotor.setPid(p, i, ff, alpha);
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

    void hardStop()
    {
        _setTargetRps(0.0F, 0.0F);
        leftMotor.driveMotor(0, true);
        rightMotor.driveMotor(0, true);
    }

    void resetForPidTest()
    {
        rampLeft.reset();
        rampRight.reset();
        resetEncoders();
        leftMotor.resetState();
        rightMotor.resetState();
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

    void resetMotorState(MotorSide m)
    {
        if (m == MOTOR_SIDE_LEFT)
        {
            leftMotor.resetState();
        }
        else if (m == MOTOR_SIDE_RIGHT)
        {
            rightMotor.resetState();
        }
        else
        {
            leftMotor.resetState();
            rightMotor.resetState();
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