#ifndef TEST_DRIVE_TASK_H_
#define TEST_DRIVE_TASK_H_

#include "controller/VelocityController.h"

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

class TestDriveTask
{
  public:
    static void startTask(void* pvParameters)
    {
        auto* vc = (VelocityController*)pvParameters;
        // Wait a moment for everything to stabilize
        vTaskDelay(pdMS_TO_TICKS(2000));

        float startRps = 0.6F;
        float endRps = 2.0F;
        int rampSteps = 100;   // Number of steps in the ramp
        int stepDelayMs = 150; // Time between speed increments (50ms * 100 = 5 seconds)

        LOG_INFO("Drive straight...");
        controller.setRps(1.0F, 1.0F);
        vTaskDelay(pdMS_TO_TICKS(10000));

        // LOG_INFO("Turn right...");
        // robot.setRps(0.4F, -0.4F);
        // vTaskDelay(pdMS_TO_TICKS(1250));

        // LOG_INFO("Go back...");
        // robot.setRps(-0.4F, -0.4F);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // LOG_INFO("Turn 360 degree...");
        // robot.setRps(-0.4F, 0.4F);
        // vTaskDelay(pdMS_TO_TICKS(3000));

        // LOG_INFO("Drive arc...");
        // robot.setRps(0.2F, 0.4F);
        // vTaskDelay(pdMS_TO_TICKS(5000));

        // robot.setRps(0.0f, 0.0f);
        // vTaskDelay(pdMS_TO_TICKS(1000));

        // // 1. RAMP UP: 0.6 -> 2.0
        // LOG_INFO("Ramping Up...");
        // for (int i = 0; i <= rampSteps; i++) {
        //     // Linear Interpolation: current = start + (fraction * total_range)
        //     float currentRps = startRps + (float)i / rampSteps * (endRps - startRps);
        //     robot.setRps(currentRps, currentRps);
        //     vTaskDelay(pdMS_TO_TICKS(stepDelayMs));
        // }

        //  vTaskDelay(pdMS_TO_TICKS(4000));

        // LOG_INFO("Ramping Down...");
        // for (int i = 0; i <= rampSteps; i++) {
        //     // Reverse Interpolation: current = end - (fraction * total_range)
        //     float currentRps = endRps - (float)i / rampSteps * (endRps - startRps);
        //     robot.setRps(-currentRps, currentRps);
        //     vTaskDelay(pdMS_TO_TICKS(stepDelayMs));
        // }

        // 3. Stop
        LOG_INFO("--- Mission Complete: Stopping ---");
        vc->stopEmergency();

        // 4. Delete this task (it's done its job)
        // In FreeRTOS, a task must either loop forever or delete itself.
        // Final check before the task disappears
        vTaskDelete(nullptr);
    }

  private:
};

#endif